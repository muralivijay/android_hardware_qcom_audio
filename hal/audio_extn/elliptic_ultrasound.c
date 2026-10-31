/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "ultrasound_extn"

#include "elliptic_ultrasound.h"
#include <cutils/sockets.h>
#include <errno.h>
#include <log/log.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <cutils/properties.h>
#include <unistd.h>
#include "audio_hw.h"
#include "platform.h"
#include "platform_api.h"

#define US_PARAM_KEY "ultrasound-proximity"
#define US_CAL_MAX_BYTES 4096
#define US_POLL_MS 150               /* conflict re-check interval while ultrasound is wanted */
#define US_START_COOLDOWN_MS 60000   /* pause after a failed start */
#define US_RAMPDOWN_WAIT_US 100000   /* let the DSP ramp the tone down before closing */
#define US_SLOW_TEARDOWN_MS 10000    /* a stop slower than this means the DSP is stuck */

/* Elliptic calibration kcontrol.*/
#define US_CAL_MIXER_CTL "Ultrasound Calibration Data"
#define US_CAL_EXT_MIXER_CTL "Ultrasound Calibration Ext Data"
#define US_CAL_DIR "/mnt/vendor/persist/audio/"

static const char* const us_cal_files[] = {
    US_CAL_DIR "us_cal_v2",
    US_CAL_DIR "elliptic_calibration_v2",
    US_CAL_DIR "elliptic_calibration",
};

enum {
    ULTRASOUND_STATUS_DEFAULT,
    ULTRASOUND_STATUS_STARTED,
    ULTRASOUND_STATUS_STOPPED,
};

// clang-format off
struct pcm_config pcm_config_us_rx = {
    .channels = 1,
    .rate = 96000,
    .period_size = 1024,
    .period_count = 4,
    .format = PCM_FORMAT_S16_LE,
};

struct pcm_config pcm_config_us_tx = {
    .channels = 1,
    .rate = 96000,
    .period_size = 1024,
    .period_count = 4,
    .format = PCM_FORMAT_S16_LE,
};
// clang-format on

struct ultrasound_device {
    struct pcm *rx_pcm, *tx_pcm;
    struct audio_usecase* rx_usecase;
    struct audio_usecase* tx_usecase;
    atomic_int state;
    pthread_mutex_t us_lock;
    struct audio_device* adev;
};

static struct ultrasound_device* us = NULL;
static atomic_bool us_wedged = false; /* DSP stopped answering: no more starts until restart */

/* Worker thread state. */
static pthread_mutex_t cmd_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cmd_cond = PTHREAD_COND_INITIALIZER;
static int pending_cmd = -1; /* -1 none, 0 stop, 1 start (guarded by cmd_lock) */
static bool worker_running = false;
static bool worker_valid = false;
static pthread_t worker_thread;
static long long next_start_ms = 0;  /* worker thread only */
static bool us_wanted = false;       /* worker thread only: sensor wants ultrasound */
static bool us_last_conflict = false;/* worker thread only: for logging */
static bool us_wedge_logged = false; /* worker thread only */

static long long ultrasound_now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

struct elliptic_payload {
    int32_t cmd[2];
    int32_t enable;
} __attribute__((packed));

static pthread_mutex_t us_init_lock = PTHREAD_MUTEX_INITIALIZER;

static atomic_bool server_running = ATOMIC_VAR_INIT(false);
static pthread_t ultrasound_server_thread;
static int sockfd = -1;

static void ultrasound_stop(void) {
    struct audio_usecase *rx_usecase, *tx_usecase;

    ALOGD("%s: enter", __func__);

    pthread_mutex_lock(&us->us_lock);

    if (atomic_load(&us->state) != ULTRASOUND_STATUS_STARTED) {
        pthread_mutex_unlock(&us->us_lock);
        ALOGD("%s: already stopped, exiting", __func__);
        return;
    }

    atomic_store(&us->state, ULTRASOUND_STATUS_STOPPED);

    if (us->tx_pcm) {
        pcm_close(us->tx_pcm);
        us->tx_pcm = NULL;
    }

    if (us->rx_pcm) {
        pcm_close(us->rx_pcm);
        us->rx_pcm = NULL;
    }

    pthread_mutex_lock(&us->adev->lock);

    audio_route_apply_and_update_path(us->adev->audio_route, "ultrasound-rampdown");
    audio_route_reset_and_update_path(us->adev->audio_route, "ultrasound-rampdown");

    audio_route_reset_and_update_path(us->adev->audio_route, "ultrasound-proximity");
    audio_route_reset_and_update_path(us->adev->audio_route, "ultrasound-on");

    audio_route_apply_and_update_path(us->adev->audio_route, "ultrasound-rampdown");
    audio_route_reset_and_update_path(us->adev->audio_route, "ultrasound-rampdown");

    audio_route_apply_and_update_path(us->adev->audio_route, "ultrasound-suspend");

    tx_usecase = us->tx_usecase;
    if (tx_usecase) {
        disable_audio_route(us->adev, tx_usecase);
        disable_snd_device(us->adev, tx_usecase->in_snd_device);
        free(tx_usecase);
        us->tx_usecase = NULL;
    }

    audio_route_apply_and_update_path(us->adev->audio_route, "ultrasound-rampdown");
    audio_route_reset_and_update_path(us->adev->audio_route, "ultrasound-rampdown");

    rx_usecase = us->rx_usecase;
    if (rx_usecase) {
        disable_audio_route(us->adev, rx_usecase);
        disable_snd_device(us->adev, rx_usecase->out_snd_device);
        free(rx_usecase);
        us->rx_usecase = NULL;
    }

    pthread_mutex_unlock(&us->adev->lock);
    pthread_mutex_unlock(&us->us_lock);

    ALOGD("%s: exit", __func__);

    return;
}

static int ultrasound_start(void) {
    int rx_device_id, tx_device_id;
    struct audio_usecase *rx_usecase, *tx_usecase;
    struct mixer_ctl* ctl;

    ALOGD("%s: enter", __func__);

    tx_device_id = platform_get_pcm_device_id(USECASE_AUDIO_ULTRASOUND_INPUT, PCM_CAPTURE);
    rx_device_id = platform_get_pcm_device_id(USECASE_AUDIO_ULTRASOUND_OUTPUT, PCM_PLAYBACK);
    if (rx_device_id < 0 || tx_device_id < 0) {
        ALOGE("%s: Invalid PCM devices (rx: %d tx: %d) for ultrasound usecase", __func__,
              rx_device_id, tx_device_id);
        return -EIO;
    }

    tx_usecase = calloc(1, sizeof(struct audio_usecase));
    rx_usecase = calloc(1, sizeof(struct audio_usecase));
    if (!tx_usecase || !rx_usecase) {
        ALOGE("%s: Out of memory!", __func__);
        if (tx_usecase) free(tx_usecase);
        if (rx_usecase) free(rx_usecase);
        return -ENOMEM;
    }

    pthread_mutex_lock(&us->us_lock);

    if (atomic_load(&us->state) == ULTRASOUND_STATUS_STARTED) {
        pthread_mutex_unlock(&us->us_lock);
        free(tx_usecase);
        free(rx_usecase);
        return -EPERM;
    }

    pthread_mutex_lock(&us->adev->lock);

    audio_route_reset_and_update_path(us->adev->audio_route, "ultrasound-suspend");
    audio_route_apply_and_update_path(us->adev->audio_route, "ultrasound-proximity");
    audio_route_apply_and_update_path(us->adev->audio_route, "ultrasound-on");

    tx_usecase->type = PCM_CAPTURE;
    tx_usecase->in_snd_device = SND_DEVICE_IN_ULTRASOUND;
    tx_usecase->id = USECASE_AUDIO_ULTRASOUND_INPUT;
    list_init(&tx_usecase->device_list);
    us->tx_usecase = tx_usecase;

    rx_usecase->type = PCM_PLAYBACK;
    rx_usecase->out_snd_device = SND_DEVICE_OUT_ULTRASOUND;
    rx_usecase->id = USECASE_AUDIO_ULTRASOUND_OUTPUT;
    list_init(&rx_usecase->device_list);
    us->rx_usecase = rx_usecase;

    enable_snd_device(us->adev, tx_usecase->in_snd_device);
    enable_audio_route(us->adev, tx_usecase);

    enable_snd_device(us->adev, rx_usecase->out_snd_device);
    enable_audio_route(us->adev, rx_usecase);

    pthread_mutex_unlock(&us->adev->lock);

    ALOGD("%s: pcm capture device id %d", __func__, tx_device_id);
    us->tx_pcm = pcm_open(us->adev->snd_card, tx_device_id, PCM_IN, &pcm_config_us_tx);
    if (us->tx_pcm && !pcm_is_ready(us->tx_pcm)) {
        ALOGD("%s: %s", __func__, pcm_get_error(us->tx_pcm));
        goto err;
    }

    ALOGD("%s: pc, playback device id %d", __func__, rx_device_id);
    us->rx_pcm = pcm_open(us->adev->snd_card, rx_device_id, PCM_OUT, &pcm_config_us_rx);
    if (us->rx_pcm && !pcm_is_ready(us->rx_pcm)) {
        ALOGE("%s: %s", __func__, pcm_get_error(us->rx_pcm));
        goto err;
    }

    if (pcm_start(us->tx_pcm) < 0) {
        ALOGE("%s: pcm_start TX failed: %s", __func__, pcm_get_error(us->tx_pcm));
        goto err;
    }
    if (pcm_start(us->rx_pcm) < 0) {
        ALOGE("%s: pcm_start RX failed: %s", __func__, pcm_get_error(us->rx_pcm));
        goto err;
    }

    atomic_store(&us->state, ULTRASOUND_STATUS_STARTED);
    pthread_mutex_unlock(&us->us_lock);

    ALOGD("%s: exit, status(0)", __func__);

    return 0;

err:
    atomic_store(&us->state, ULTRASOUND_STATUS_STARTED);  // For ultrasound_stop to work
    pthread_mutex_unlock(&us->us_lock);

    ultrasound_stop();

    ALOGE("%s: exit: status(%d)", __func__, -EIO);

    return -EIO;
}

static int ultrasound_extn_stop(void) {
    ALOGD("%s: enter", __func__);

    pthread_mutex_lock(&us_init_lock);
    if (!us || atomic_load(&us->state) != ULTRASOUND_STATUS_STARTED) {
        pthread_mutex_unlock(&us_init_lock);
        return -EPERM;
    }
    pthread_mutex_unlock(&us_init_lock);

    ultrasound_stop();

    return 0;
}

static int ultrasound_extn_start(void) {
    int rc = 0;

    ALOGD("%s: enter", __func__);

    pthread_mutex_lock(&us_init_lock);
    if (!us || atomic_load(&us->state) == ULTRASOUND_STATUS_STARTED) {
        pthread_mutex_unlock(&us_init_lock);
        return -EPERM;
    }
    pthread_mutex_unlock(&us_init_lock);

    rc = ultrasound_start();

    return rc;
}

static void ultrasound_worker_stop(void) {
    pthread_mutex_lock(&cmd_lock);
    worker_running = false;
    pthread_cond_signal(&cmd_cond);
    bool join = worker_valid;
    worker_valid = false;
    pthread_mutex_unlock(&cmd_lock);
    if (join) pthread_join(worker_thread, NULL);
}

void audio_extn_ultrasound_deinit(void) {
    ALOGD("%s: enter", __func__);

    ultrasound_worker_stop();
    atomic_store(&server_running, false);

    if (sockfd >= 0) {
        shutdown(sockfd, SHUT_RDWR);
        close(sockfd);
        sockfd = -1;
    }

    if (ultrasound_server_thread) {
        pthread_join(ultrasound_server_thread, NULL);
        ultrasound_server_thread = 0;
    }

    pthread_mutex_lock(&us_init_lock);
    if (us) {
        if (atomic_load(&us->state) == ULTRASOUND_STATUS_STARTED) ultrasound_stop();

        pthread_mutex_destroy(&us->us_lock);
        free(us);
        us = NULL;
    }
    pthread_mutex_unlock(&us_init_lock);

    ALOGD("%s: exit", __func__);
}

/* ------------------------------------------------------------------------ */
/* Supervisor and worker thread                                             */
/* ------------------------------------------------------------------------ */

/*
 * Reconcile "the sensor wants ultrasound" with "nothing conflicting is playing":
 * start when wanted and clear, stop when no longer wanted or when a conflicting
 * stream shows up (it restarts by itself once that is gone).
 */

static void ultrasound_supervise(void) {
    if (!us) return;

    if (!us_wanted) {
        if (atomic_load(&us->state) == ULTRASOUND_STATUS_STARTED) {
            ALOGI("%s: proximity disabled, stopping ultrasound", __func__);
            ultrasound_stop();
        }
        return;
    }

    if (atomic_load(&us_wedged)) {
        if (!us_wedge_logged) {
            ALOGW("%s: not starting, DSP was stuck on a previous stop", __func__);
            us_wedge_logged = true;
        }
        return;
    }

    /* Do not start again if already running */
    if (atomic_load(&us->state) == ULTRASOUND_STATUS_STARTED) {
        return;
    }

    if (ultrasound_now_ms() < next_start_ms) return;

    int rc = ultrasound_start();
    ALOGI("%s: start -> %d", __func__, rc);
    if (rc == -EIO) {
        next_start_ms = ultrasound_now_ms() + US_START_COOLDOWN_MS;
    }
}

static void* ultrasound_worker_func(void* arg __unused) {
    pthread_mutex_lock(&cmd_lock);
    while (worker_running) {
        if (pending_cmd < 0) {
            if (us_wanted) {
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                ts.tv_nsec += (long)US_POLL_MS * 1000000L;
                if (ts.tv_nsec >= 1000000000L) {
                    ts.tv_sec += 1;
                    ts.tv_nsec -= 1000000000L;
                }
                pthread_cond_timedwait(&cmd_cond, &cmd_lock, &ts);
            } else {
                /* When not wanted, sleep indefinitely until next command arrives */
                pthread_cond_wait(&cmd_cond, &cmd_lock);
            }
        }
        if (!worker_running) break;

        int cmd = pending_cmd;
        pending_cmd = -1;
        pthread_mutex_unlock(&cmd_lock);

        if (cmd == 1) {
            us_wanted = true;
            us_wedge_logged = false;
        } else if (cmd == 0) {
            us_wanted = false;
            us_last_conflict = false;
        }

        ultrasound_supervise();

        pthread_mutex_lock(&cmd_lock);
    }
    pthread_mutex_unlock(&cmd_lock);
    return NULL;
}

static void ultrasound_worker_start(void) {
    pthread_mutex_lock(&cmd_lock);
    if (worker_valid) {
        pthread_mutex_unlock(&cmd_lock);
        return;
    }
    worker_running = true;
    pending_cmd = -1;
    if (pthread_create(&worker_thread, NULL, ultrasound_worker_func, NULL) == 0) {
        worker_valid = true;
    } else {
        ALOGE("%s: failed to create worker thread", __func__);
        worker_running = false;
    }
    pthread_mutex_unlock(&cmd_lock);
}

/* Call at the top of adev_set_parameters(), whether or not adev->lock is held. */
void audio_extn_ultrasound_set_parameters(struct audio_device* adev, struct str_parms* parms) {
    char value[32];

    if (!parms || str_parms_get_str(parms, US_PARAM_KEY, value, sizeof(value)) < 0) return;

    int enable = atoi(value);
    ALOGD("%s: %s=%s", __func__, US_PARAM_KEY, value);

    pthread_mutex_lock(&cmd_lock);
    if (worker_valid) {
        pending_cmd = enable ? 1 : 0;
        pthread_cond_signal(&cmd_cond);
    } else {
        ALOGW("%s: worker not running, dropping request", __func__);
    }
    pthread_mutex_unlock(&cmd_lock);
}

static size_t ultrasound_parse_hex_text(const char* text, uint8_t* out, size_t out_max) {
    size_t n = 0;
    const char* p = text;

    while (*p && n < out_max) {
        while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t' || *p == ',') p++;
        if (!*p) break;
        char* e = NULL;
        unsigned long v = strtoul(p, &e, 16); /* base 16 also accepts the 0x prefix */
        if (e == p || v > 0xff) break;
        out[n++] = (uint8_t)v;
        p = e;
    }
    return n;
}

static bool ultrasound_try_cal_ctl(struct audio_device* adev, const char* name,
                                   const uint8_t* data, size_t n) {
    struct mixer_ctl* ctl = mixer_get_ctl_by_name(adev->mixer, name);
    if (!ctl) {
        ALOGW("%s: kcontrol '%s' not present", __func__, name);
        return false;
    }
    if (mixer_ctl_get_type(ctl) != MIXER_CTL_TYPE_BYTE) {
        ALOGW("%s: '%s' is not a byte control", __func__, name);
        return false;
    }
    if (mixer_ctl_get_num_values(ctl) != n) {
        ALOGD("%s: '%s' wants %u bytes, have %zu", __func__, name, mixer_ctl_get_num_values(ctl), n);
        return false;
    }
    if (mixer_ctl_set_array(ctl, data, n) < 0) {
        ALOGE("%s: failed to write '%s'", __func__, name);
        return false;
    }
    ALOGI("%s: wrote %zu bytes to '%s'", __func__, n, name);
    return true;
}

static void ultrasound_load_calibration(struct audio_device* adev) {
    for (size_t i = 0; i < sizeof(us_cal_files) / sizeof(us_cal_files[0]); i++) {
        struct stat st;
        if (stat(us_cal_files[i], &st) != 0) continue;
        if (st.st_size <= 0 || st.st_size > 64 * 1024) {
            ALOGW("%s: %s has unexpected size %ld, skipping", __func__, us_cal_files[i],
                  (long)st.st_size);
            continue;
        }

        char* text = calloc(1, (size_t)st.st_size + 1);
        uint8_t* bytes = calloc(1, US_CAL_MAX_BYTES);
        FILE* fp = fopen(us_cal_files[i], "r");
        if (!text || !bytes || !fp) {
            if (fp) fclose(fp);
            free(text);
            free(bytes);
            continue;
        }
        size_t got = fread(text, 1, (size_t)st.st_size, fp);
        fclose(fp);
        text[got] = '\0';

        size_t n = ultrasound_parse_hex_text(text, bytes, US_CAL_MAX_BYTES);
        ALOGI("%s: %s parsed to %zu bytes", __func__, us_cal_files[i], n);

        bool ok = ultrasound_try_cal_ctl(adev, US_CAL_EXT_MIXER_CTL, bytes, n) ||
                  ultrasound_try_cal_ctl(adev, US_CAL_MIXER_CTL, bytes, n);
        free(text);
        free(bytes);
        if (ok) return;
        ALOGW("%s: %s (%zu bytes) matched no calibration control", __func__, us_cal_files[i], n);
    }
    ALOGW("%s: no calibration applied", __func__);
}

int audio_extn_ultrasound_init(struct audio_device* adev) {
    ALOGD("%s: enter", __func__);

    pthread_mutex_lock(&us_init_lock);
    if (us) {
        ALOGI("%s: ultrasound has been initialized!", __func__);
        pthread_mutex_unlock(&us_init_lock);
        return 0;
    }

    us = calloc(1, sizeof(struct ultrasound_device));
    if (!us) {
        ALOGE("%s: Out of memory!", __func__);
        pthread_mutex_unlock(&us_init_lock);
        return -ENOMEM;
    }

    us->adev = adev;
    pthread_mutex_init(&us->us_lock, NULL);
    atomic_init(&us->state, ULTRASOUND_STATUS_DEFAULT);

    pthread_mutex_unlock(&us_init_lock);

    ultrasound_worker_start();
    ultrasound_load_calibration(us->adev);

    ALOGD("%s: exit, status(0)", __func__);

    return 0;
}
