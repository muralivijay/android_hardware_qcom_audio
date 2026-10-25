/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Elliptic Labs ultrasound (virtual proximity) extension for
 * Redmi Note 11 (spes / SM6225 "bengal").
 *
 * Based on Rocky7842/android_hardware_qcom_audio
 * (hal/audio_extn/elliptic_ultrasound.c), re-targeted using values recovered
 * from the stock audio.primary.bengal.so and mixer_paths:
 *
 *   - RX output goes through a dedicated SND_DEVICE_OUT_ULTRASOUND
 *     (mixer path "ultrasound-output": RX_CDC_DMA_RX_1 @ 96 kHz mono,
 *     earpiece + aw87xxx receiver), NOT SND_DEVICE_OUT_VOICE_HANDSET.
 *   - TX input uses SND_DEVICE_IN_ULTRASOUND ("ultrasound-input":
 *     TX_CDC_DMA_TX_4 @ 96 kHz mono, SWR_MIC4).
 *   - Usecase-level paths are "ultrasound-proximity-output/-input"
 *     (empty in stock mixer); the DSP algorithm controls live in
 *     "ultrasound-proximity", "ultrasound-on", "ultrasound-rampdown" and
 *     "ultrasound-suspend".
 *   - Both PCMs use PCM device 39 (stock pcm_device_table).
 *   - Calibration uses the Elliptic "Ultrasound Calibration Data" kcontrol,
 *     not the Mi_Ultrasound one.
 *
 * Sections marked UNVERIFIED were not confirmed by disassembly of the stock
 * HAL. See SPES_PORT_NOTES.md.
 */

#define LOG_TAG "ultrasound_extn"

#include "elliptic_ultrasound.h"

#include <cutils/list.h>
#include <cutils/properties.h>
#include <cutils/sockets.h>
#include <cutils/str_parms.h>
#include <errno.h>
#include <log/log.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "audio_hw.h"
#include "platform.h"
#include "platform_api.h"

/* ------------------------------------------------------------------------ */
/* Constants                                                                */
/* ------------------------------------------------------------------------ */

#define US_AUDIO_SOCKET_NAME "audio_hw_socket"
#define MAX_CMD_LEN 1024

#define US_CMD_HEAD 17
#define US_CMD_TAIL 1

#define US_PATH_SUSPEND "ultrasound-suspend"
#define US_PATH_PROXIMITY "ultrasound-proximity"
#define US_PATH_ON "ultrasound-on"
#define US_PATH_RAMPDOWN "ultrasound-rampdown"

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

static struct pcm_config pcm_config_us_rx = {
    .channels = 1,
    .rate = 96000,
    .period_size = 1024,
    .period_count = 4,
    .format = PCM_FORMAT_S16_LE,
};

static struct pcm_config pcm_config_us_tx = {
    .channels = 1,
    .rate = 96000,
    .period_size = 1024,
    .period_count = 4,
    .format = PCM_FORMAT_S16_LE,
};

/* ------------------------------------------------------------------------ */
/* State                                                                    */
/* ------------------------------------------------------------------------ */

struct ultrasound_device {
    struct pcm *rx_pcm, *tx_pcm;
    struct audio_usecase* rx_usecase;
    struct audio_usecase* tx_usecase;
    atomic_int state;
    pthread_mutex_t us_lock; /* lock order: us_lock -> adev->lock */
    struct audio_device* adev;
    bool rcv_overridden;
};

struct elliptic_payload {
    int32_t cmd[2];
    int32_t enable;
} __attribute__((packed));

static struct ultrasound_device* us = NULL;
static atomic_bool us_wedged = false;
#define US_SLOW_TEARDOWN_MS 10000
#define US_RAMPDOWN_WAIT_US 100000
#define US_SLOW_WRITE_MS 100
static pthread_mutex_t us_init_lock = PTHREAD_MUTEX_INITIALIZER;
static atomic_bool server_running = ATOMIC_VAR_INIT(false);
static pthread_t ultrasound_server_thread;
static bool server_thread_valid = false;
static atomic_int sockfd = ATOMIC_VAR_INIT(-1);
static atomic_int client_fd = ATOMIC_VAR_INIT(-1);

static void ultrasound_load_calibration(struct audio_device* adev);
static void ultrasound_force_backend_cfg(struct audio_device* adev);
static long long ultrasound_now_ms(void);

/* ------------------------------------------------------------------------ */
/* Helpers                                                                  */
/* ------------------------------------------------------------------------ */

static inline bool is_speaker_out_device(snd_device_t dev) {
    return (dev == SND_DEVICE_OUT_SPEAKER ||
            dev == SND_DEVICE_OUT_SPEAKER_REVERSE ||
            dev == SND_DEVICE_OUT_VOICE_SPEAKER ||
            dev == SND_DEVICE_OUT_VOICE_SPEAKER_STEREO);
}

/*
 * spes coexistence policy:
 * Ultrasound only runs when active playback is directed to the handset/earpiece
 * or when nothing is playing at all. Speaker (mono/stereo) and headphones conflict.
 * Caller must hold adev->lock.
 */
static bool ultrasound_earpiece_busy(struct audio_device* adev) {
    struct listnode* node;
    struct audio_usecase* uc;

    list_for_each(node, &adev->usecase_list) {
        uc = node_to_item(node, struct audio_usecase, list);
        if (uc->out_snd_device == SND_DEVICE_NONE) continue; /* capture only */
        if (uc->out_snd_device == SND_DEVICE_OUT_HANDSET ||
            uc->out_snd_device == SND_DEVICE_OUT_VOICE_HANDSET)
            continue;
        return true;
    }
    return false;
}

static bool ultrasound_conflict_now(struct audio_device* adev) {
    pthread_mutex_lock(&adev->lock);
    bool busy = ultrasound_earpiece_busy(adev);
    pthread_mutex_unlock(&adev->lock);
    return busy;
}

static void ultrasound_force_backend_cfg(struct audio_device* adev) {
    static const struct {
        const char* ctl;
        const char* val;
    } be_cfg[] = {
        {"RX_CDC_DMA_RX_1 SampleRate", "KHZ_96"},
        {"RX_CDC_DMA_RX_1 Channels", "One"},
        {"RX_CDC_DMA_RX_1 Format", "S16_LE"},
        {"TX_CDC_DMA_TX_4 SampleRate", "KHZ_96"},
        {"TX_CDC_DMA_TX_4 Channels", "One"},
        {"TX_CDC_DMA_TX_4 Format", "S16_LE"},
    };

    for (size_t i = 0; i < sizeof(be_cfg) / sizeof(be_cfg[0]); i++) {
        struct mixer_ctl* ctl = mixer_get_ctl_by_name(adev->mixer, be_cfg[i].ctl);
        if (!ctl) {
            ALOGW("%s: control '%s' not found", __func__, be_cfg[i].ctl);
            continue;
        }
        const char* cur = mixer_ctl_get_enum_string(ctl, mixer_ctl_get_value(ctl, 0));
        ALOGD("%s: '%s' = %s (want %s)", __func__, be_cfg[i].ctl, cur ? cur : "?", be_cfg[i].val);
        if (mixer_ctl_set_enum_by_string(ctl, be_cfg[i].val) < 0)
            ALOGE("%s: failed to set '%s' to %s", __func__, be_cfg[i].ctl, be_cfg[i].val);
    }
}

static int ultrasound_elus_set_enum(struct audio_device* adev, const char* name, const char* val) {
    long long t = ultrasound_now_ms();
    struct mixer_ctl* ctl = mixer_get_ctl_by_name(adev->mixer, name);
    int rc = -ENOENT;

    if (ctl) rc = mixer_ctl_set_enum_by_string(ctl, val);
    long long dt = ultrasound_now_ms() - t;

    if (!ctl)
        ALOGW("%s: control '%s' not found", __func__, name);
    else if (rc < 0)
        ALOGE("%s: failed to set '%s' = %s", __func__, name, val);
    if (dt > US_SLOW_WRITE_MS) ALOGW("%s: SLOW write '%s' = %s took %lld ms", __func__, name, val, dt);
    return rc;
}

static int ultrasound_elus_set_int(struct audio_device* adev, const char* name, int val) {
    long long t = ultrasound_now_ms();
    struct mixer_ctl* ctl = mixer_get_ctl_by_name(adev->mixer, name);
    int rc = -ENOENT;

    if (ctl) rc = mixer_ctl_set_value(ctl, 0, val);
    long long dt = ultrasound_now_ms() - t;

    if (!ctl)
        ALOGW("%s: control '%s' not found", __func__, name);
    else if (rc < 0)
        ALOGE("%s: failed to set '%s' = %d", __func__, name, val);
    if (dt > US_SLOW_WRITE_MS) ALOGW("%s: SLOW write '%s' = %d took %lld ms", __func__, name, val, dt);
    return rc;
}

static void ultrasound_elus_apply_start(struct audio_device* adev) {
    ultrasound_elus_set_int(adev, "Ultrasound Suspend", 0);
    ultrasound_elus_set_enum(adev, "Ultrasound Enable", "On");
    ultrasound_elus_set_int(adev, "Ultrasound Mode", 918);
    ultrasound_elus_set_int(adev, "Ultrasound Custom Setting 4", 1);
    ultrasound_elus_set_int(adev, "Ultrasound Debug Mode", 0);
    ultrasound_elus_set_int(adev, "Ultrasound Log Level", 0);
    ultrasound_elus_set_enum(adev, "Ultrasound Rx Port", "On");
    ultrasound_elus_set_int(adev, "Ultrasound Custom Setting 1", 1);
}

static void ultrasound_elus_apply_stop_end(struct audio_device* adev) {
    ultrasound_elus_set_enum(adev, "Ultrasound RampDown", "Off");
    ultrasound_elus_set_enum(adev, "Ultrasound Enable", "Off");
    ultrasound_elus_set_int(adev, "Ultrasound Debug Mode", 3);
    ultrasound_elus_set_enum(adev, "Ultrasound Rx Port", "Off");
    ultrasound_elus_set_int(adev, "Ultrasound Suspend", 1);
}

#define US_AMP_RCV_CTL "aw87xxx_rcv_switch"
#define US_AMP_RCV_ULTRASOUND "Receiver"
#define US_AMP_RCV_IDLE "Off"

static bool ultrasound_any_output_usecase(struct audio_device* adev) {
    struct listnode* node;
    struct audio_usecase* uc;

    list_for_each(node, &adev->usecase_list) {
        uc = node_to_item(node, struct audio_usecase, list);
        if (uc->out_snd_device != SND_DEVICE_NONE) return true;
    }
    return false;
}

static void ultrasound_log_amps(struct audio_device* adev, const char* tag) {
    static const struct {
        const char* ctl;
        const char* label;
    } amps[] = {
        {"aw87xxx_rcv_switch", "rcv"},
        {"aw87xxx_spk_switch", "spk"},
        {"aw87xxx_profile_switch_0", "p0"},
        {"aw87xxx_profile_switch_1", "p1"},
    };
    char buf[160];
    size_t n = 0;

    buf[0] = '\0';
    for (size_t i = 0; i < sizeof(amps) / sizeof(amps[0]) && n < sizeof(buf); i++) {
        struct mixer_ctl* ctl = mixer_get_ctl_by_name(adev->mixer, amps[i].ctl);
        const char* cur = ctl ? mixer_ctl_get_enum_string(ctl, mixer_ctl_get_value(ctl, 0)) : "n/a";
        n += (size_t)snprintf(buf + n, sizeof(buf) - n, "%s=%s ", amps[i].label, cur ? cur : "?");
    }
    ALOGI("amps[%s]: %s", tag, buf);
}

static void ultrasound_log_usecases_locked(struct audio_device* adev) {
    struct listnode* node;
    struct audio_usecase* uc;

    list_for_each(node, &adev->usecase_list) {
        uc = node_to_item(node, struct audio_usecase, list);
        ALOGI("usecase: id=%d type=%d out_dev=%d in_dev=%d", (int)uc->id, (int)uc->type,
              (int)uc->out_snd_device, (int)uc->in_snd_device);
    }
}

#define US_NOAMP_PROP "persist.vendor.audio.us.noamp"
static bool ultrasound_amp_disabled(void) {
    return property_get_bool(US_NOAMP_PROP, false);
}

static void ultrasound_amp_take_locked(struct audio_device* adev) {
    if (ultrasound_amp_disabled()) {
        ultrasound_log_amps(adev, "start-noamp");
        return;
    }
    ultrasound_log_amps(adev, "start-before");
    ultrasound_elus_set_enum(adev, US_AMP_RCV_CTL, US_AMP_RCV_ULTRASOUND);
    us->rcv_overridden = true;
    ultrasound_log_amps(adev, "start-after");
}

static void ultrasound_amp_assert(struct audio_device* adev) {
    if (ultrasound_amp_disabled()) return;
    pthread_mutex_lock(&adev->lock);
    if (!ultrasound_earpiece_busy(adev)) {
        struct mixer_ctl* ctl = mixer_get_ctl_by_name(adev->mixer, US_AMP_RCV_CTL);
        if (ctl) {
            const char* cur = mixer_ctl_get_enum_string(ctl, mixer_ctl_get_value(ctl, 0));
            if (cur && strcmp(cur, US_AMP_RCV_ULTRASOUND) != 0) {
                ALOGI("%s: %s is %s, re-asserting %s", __func__, US_AMP_RCV_CTL, cur,
                      US_AMP_RCV_ULTRASOUND);
                ultrasound_elus_set_enum(adev, US_AMP_RCV_CTL, US_AMP_RCV_ULTRASOUND);
            }
        }
    }
    pthread_mutex_unlock(&adev->lock);
}

static void ultrasound_amp_release(struct audio_device* adev) {
    if (!us->rcv_overridden) return;
    us->rcv_overridden = false;

    pthread_mutex_lock(&adev->lock);

    bool speaker_active = false;
    struct listnode* node;
    struct audio_usecase* uc;

    list_for_each(node, &adev->usecase_list) {
        uc = node_to_item(node, struct audio_usecase, list);
        if (is_speaker_out_device(uc->out_snd_device)) {
            speaker_active = true;
            break;
        }
    }

    if (speaker_active) {
        /*
         * Do not force any mode here; let audio_route apply the values
         * defined in mixer_paths.xml (voice-speaker-stereo).
         */
        ALOGI("%s: speaker active, releasing amp management to audio_route", __func__);
    } else if (ultrasound_any_output_usecase(adev)) {
        ALOGI("%s: other audio is active, leaving %s as is", __func__, US_AMP_RCV_CTL);
        ultrasound_log_usecases_locked(adev);
        ultrasound_log_amps(adev, "release-left");
    } else {
        ultrasound_elus_set_enum(adev, US_AMP_RCV_CTL, US_AMP_RCV_IDLE);
        ultrasound_log_amps(adev, "release-idle");
    }

    pthread_mutex_unlock(&adev->lock);
}

struct us_amp_snapshot {
    char rcv[24];
    char spk[24];
    unsigned int sig;
    bool valid;
};
static struct us_amp_snapshot us_snap;

static void ultrasound_amp_read(struct audio_device* adev, const char* ctl_name, char* out,
                                size_t len) {
    struct mixer_ctl* ctl = mixer_get_ctl_by_name(adev->mixer, ctl_name);
    const char* cur = ctl ? mixer_ctl_get_enum_string(ctl, mixer_ctl_get_value(ctl, 0)) : NULL;
    snprintf(out, len, "%s", cur ? cur : "");
}

static unsigned int ultrasound_usecase_signature_locked(struct audio_device* adev) {
    struct listnode* node;
    struct audio_usecase* uc;
    unsigned int h = 0x811c9dc5u;

    list_for_each(node, &adev->usecase_list) {
        uc = node_to_item(node, struct audio_usecase, list);
        if (uc->out_snd_device == SND_DEVICE_NONE) continue;
        h = ((h << 7) | (h >> 25)) ^ ((unsigned int)uc->id << 8) ^
            (unsigned int)uc->out_snd_device;
    }
    return h;
}

static void ultrasound_amp_snapshot(struct audio_device* adev) {
    us_snap.valid = false;
    pthread_mutex_lock(&adev->lock);
    if (ultrasound_any_output_usecase(adev)) {
        ultrasound_amp_read(adev, "aw87xxx_rcv_switch", us_snap.rcv, sizeof(us_snap.rcv));
        ultrasound_amp_read(adev, "aw87xxx_spk_switch", us_snap.spk, sizeof(us_snap.spk));
        us_snap.sig = ultrasound_usecase_signature_locked(adev);
        us_snap.valid = us_snap.rcv[0] && us_snap.spk[0];
    }
    pthread_mutex_unlock(&adev->lock);
    if (us_snap.valid)
        ALOGI("%s: other audio active, snapshot rcv=%s spk=%s", __func__, us_snap.rcv, us_snap.spk);
}

static void ultrasound_amp_restore_snapshot(struct audio_device* adev) {
    char cur[24];

    if (!us_snap.valid) return;
    us_snap.valid = false;

    pthread_mutex_lock(&adev->lock);

    /*
     * If speaker stereo or mono is running, NEVER restore the snapshot.
     * The snapshot captured the handset state (rcv=Receiver, spk=Off)
     * which suffocates stereo speaker playback into the earpiece.
     */
    struct listnode* node;
    struct audio_usecase* uc;
    list_for_each(node, &adev->usecase_list) {
        uc = node_to_item(node, struct audio_usecase, list);
        if (is_speaker_out_device(uc->out_snd_device)) {
            ALOGI("%s: speaker mode detected, dropping snapshot restore", __func__);
            pthread_mutex_unlock(&adev->lock);
            return;
        }
    }

    if (!ultrasound_any_output_usecase(adev) ||
        ultrasound_usecase_signature_locked(adev) != us_snap.sig) {
        ALOGI("%s: usecases changed during stop, HAL state wins, not restoring", __func__);
    } else {
        ultrasound_amp_read(adev, "aw87xxx_rcv_switch", cur, sizeof(cur));
        if (strcmp(cur, us_snap.rcv) != 0) {
            ALOGW("%s: rcv changed %s -> %s during stop, restoring", __func__, us_snap.rcv, cur);
            ultrasound_elus_set_enum(adev, "aw87xxx_rcv_switch", us_snap.rcv);
        }
        ultrasound_amp_read(adev, "aw87xxx_spk_switch", cur, sizeof(cur));
        if (strcmp(cur, us_snap.spk) != 0) {
            ALOGW("%s: spk changed %s -> %s during stop, restoring", __func__, us_snap.spk, cur);
            ultrasound_elus_set_enum(adev, "aw87xxx_spk_switch", us_snap.spk);
        }
    }
    pthread_mutex_unlock(&adev->lock);
    ultrasound_log_amps(adev, "after-restore");
}

static void ultrasound_reset_backend_for_speaker(struct audio_device* adev) {
    /* 
     * spes speaker media and speaker calls require RX_CDC_DMA_RX_1 
     * at 48kHz Two channels. Reset them before handing control to audio_route.
     */
    static const struct {
        const char* ctl;
        const char* val;
    } be_reset[] = {
        {"RX_CDC_DMA_RX_1 SampleRate", "KHZ_48"},
        {"RX_CDC_DMA_RX_1 Channels", "Two"},
    };

    for (size_t i = 0; i < sizeof(be_reset) / sizeof(be_reset[0]); i++) {
        struct mixer_ctl* ctl = mixer_get_ctl_by_name(adev->mixer, be_reset[i].ctl);
        if (ctl) {
            mixer_ctl_set_enum_by_string(ctl, be_reset[i].val);
        }
    }
}

void audio_extn_ultrasound_sync_stop(struct audio_device *adev, snd_device_t out_snd_device) {
    if (!us || atomic_load(&us->state) != ULTRASOUND_STATUS_STARTED)
        return;

    /* Only trigger if transitioning away from handset/earpiece */
    if (out_snd_device == SND_DEVICE_OUT_HANDSET ||
        out_snd_device == SND_DEVICE_OUT_VOICE_HANDSET)
        return;

    ALOGI("%s: Target device %d is not handset. Stopping ultrasound synchronously",
          __func__, out_snd_device);

    /*
     * adev->lock is ALREADY held by select_devices().
     * Try to acquire us_lock without blocking to prevent ABBA deadlock.
     */
    bool locked = (pthread_mutex_trylock(&us->us_lock) == 0);

    if (atomic_load(&us->state) == ULTRASOUND_STATUS_STARTED) {
        /* 1. Immediately close PCMs so RX_CDC_DMA_RX_1 is freed */
        if (us->tx_pcm) {
            pcm_stop(us->tx_pcm);
            pcm_close(us->tx_pcm);
            us->tx_pcm = NULL;
        }
        if (us->rx_pcm) {
            pcm_stop(us->rx_pcm);
            pcm_close(us->rx_pcm);
            us->rx_pcm = NULL;
        }

        /* 2. Fast reset of DSP controls */
        ultrasound_elus_apply_stop_end(adev);

        /* 3. Disable ultrasound audio routes */
        if (us->tx_usecase) {
            disable_audio_route(adev, us->tx_usecase);
            disable_snd_device(adev, us->tx_usecase->in_snd_device);
            free(us->tx_usecase);
            us->tx_usecase = NULL;
        }
        if (us->rx_usecase) {
            disable_audio_route(adev, us->rx_usecase);
            disable_snd_device(adev, us->rx_usecase->out_snd_device);
            free(us->rx_usecase);
            us->rx_usecase = NULL;
        }

        /* 4. Reset backend rate back down to 48kHz stereo */
        ultrasound_reset_backend_for_speaker(adev);

        us->rcv_overridden = false;
        atomic_store(&us->state, ULTRASOUND_STATUS_STOPPED);
    }

    if (locked) {
        pthread_mutex_unlock(&us->us_lock);
    }
}

static void ultrasound_teardown_locked(void) {
    struct audio_device* adev = us->adev;
    struct audio_usecase *rx_usecase, *tx_usecase;
    long long t0 = ultrasound_now_ms();

    ALOGI("%s: begin (tx_pcm=%p rx_pcm=%p tx_uc=%p rx_uc=%p)", __func__, (void*)us->tx_pcm,
          (void*)us->rx_pcm, (void*)us->tx_usecase, (void*)us->rx_usecase);
    ultrasound_amp_snapshot(adev);

    ultrasound_elus_set_enum(adev, "Ultrasound RampDown", "On");
    usleep(US_RAMPDOWN_WAIT_US);

    if (us->tx_pcm) {
        pcm_stop(us->tx_pcm);
        pcm_close(us->tx_pcm);
        us->tx_pcm = NULL;
    }
    if (us->rx_pcm) {
        pcm_stop(us->rx_pcm);
        pcm_close(us->rx_pcm);
        us->rx_pcm = NULL;
    }

    ultrasound_elus_apply_stop_end(adev);

    long long dt = ultrasound_now_ms() - t0;
    if (dt > US_SLOW_TEARDOWN_MS) {
        ALOGE("%s: DSP control phase took %lld ms, DSP wedged", __func__, dt);
        atomic_store(&us_wedged, true);
    }

    pthread_mutex_lock(&adev->lock);
    tx_usecase = us->tx_usecase;
    if (tx_usecase) {
        disable_audio_route(adev, tx_usecase);
        disable_snd_device(adev, tx_usecase->in_snd_device);
        free(tx_usecase);
        us->tx_usecase = NULL;
    }
    rx_usecase = us->rx_usecase;
    if (rx_usecase) {
        disable_audio_route(adev, rx_usecase);
        disable_snd_device(adev, rx_usecase->out_snd_device);
        free(rx_usecase);
        us->rx_usecase = NULL;
    }
    pthread_mutex_unlock(&adev->lock);

    ultrasound_amp_release(adev);
    ultrasound_amp_restore_snapshot(adev);

    atomic_store(&us->state, ULTRASOUND_STATUS_STOPPED);
    ALOGI("%s: teardown complete", __func__);
}

static void ultrasound_stop(void) {
    ALOGD("%s: enter", __func__);

    pthread_mutex_lock(&us->us_lock);
    if (atomic_load(&us->state) != ULTRASOUND_STATUS_STARTED) {
        pthread_mutex_unlock(&us->us_lock);
        ALOGD("%s: already stopped, exiting", __func__);
        return;
    }
    ultrasound_teardown_locked();
    pthread_mutex_unlock(&us->us_lock);

    ALOGD("%s: exit", __func__);
}

static int ultrasound_start(void) {
    int rx_device_id, tx_device_id;
    struct audio_usecase *rx_usecase, *tx_usecase;

    ALOGD("%s: enter", __func__);

    if (atomic_load(&us_wedged)) {
        ALOGW("%s: refusing to start, DSP was stuck on a previous stop (restart HAL/reboot)",
              __func__);
        return -ENODEV;
    }

    tx_device_id = platform_get_pcm_device_id(USECASE_AUDIO_ULTRASOUND_PROXIMITY_INPUT, PCM_CAPTURE);
    rx_device_id =
        platform_get_pcm_device_id(USECASE_AUDIO_ULTRASOUND_PROXIMITY_OUTPUT, PCM_PLAYBACK);
    if (rx_device_id < 0 || tx_device_id < 0) {
        ALOGE("%s: Invalid PCM devices (rx: %d tx: %d) for ultrasound usecase", __func__,
              rx_device_id, tx_device_id);
        return -EIO;
    }

    tx_usecase = calloc(1, sizeof(struct audio_usecase));
    rx_usecase = calloc(1, sizeof(struct audio_usecase));
    if (!tx_usecase || !rx_usecase) {
        ALOGE("%s: Out of memory!", __func__);
        free(tx_usecase);
        free(rx_usecase);
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

    if (ultrasound_earpiece_busy(us->adev)) {
        ALOGW("%s: conflicting audio is active, not starting", __func__);
        pthread_mutex_unlock(&us->adev->lock);
        pthread_mutex_unlock(&us->us_lock);
        free(tx_usecase);
        free(rx_usecase);
        return -EBUSY;
    }

    pthread_mutex_unlock(&us->adev->lock);
    ultrasound_elus_apply_start(us->adev);
    pthread_mutex_lock(&us->adev->lock);

    if (ultrasound_earpiece_busy(us->adev)) {
        ALOGW("%s: conflicting audio appeared during start, aborting", __func__);
        pthread_mutex_unlock(&us->adev->lock);
        ultrasound_elus_apply_stop_end(us->adev);
        pthread_mutex_unlock(&us->us_lock);
        free(tx_usecase);
        free(rx_usecase);
        return -EBUSY;
    }
    ultrasound_amp_take_locked(us->adev);

    tx_usecase->type = PCM_CAPTURE;
    tx_usecase->in_snd_device = SND_DEVICE_IN_ULTRASOUND;
    tx_usecase->id = USECASE_AUDIO_ULTRASOUND_PROXIMITY_INPUT;
    list_init(&tx_usecase->device_list);
    us->tx_usecase = tx_usecase;

    rx_usecase->type = PCM_PLAYBACK;
    rx_usecase->out_snd_device = SND_DEVICE_OUT_ULTRASOUND;
    rx_usecase->id = USECASE_AUDIO_ULTRASOUND_PROXIMITY_OUTPUT;
    list_init(&rx_usecase->device_list);
    us->rx_usecase = rx_usecase;

    enable_snd_device(us->adev, tx_usecase->in_snd_device);
    enable_audio_route(us->adev, tx_usecase);
    enable_snd_device(us->adev, rx_usecase->out_snd_device);
    enable_audio_route(us->adev, rx_usecase);

    ultrasound_force_backend_cfg(us->adev);

    pthread_mutex_unlock(&us->adev->lock);

    ALOGD("%s: pcm capture device id %d", __func__, tx_device_id);
    us->tx_pcm = pcm_open(us->adev->snd_card, tx_device_id, PCM_IN, &pcm_config_us_tx);
    if (!us->tx_pcm || !pcm_is_ready(us->tx_pcm)) {
        ALOGE("%s: TX pcm_open failed: %s", __func__,
              us->tx_pcm ? pcm_get_error(us->tx_pcm) : "null pcm");
        goto err;
    }

    ALOGD("%s: pcm playback device id %d", __func__, rx_device_id);
    us->rx_pcm = pcm_open(us->adev->snd_card, rx_device_id, PCM_OUT, &pcm_config_us_rx);
    if (!us->rx_pcm || !pcm_is_ready(us->rx_pcm)) {
        ALOGE("%s: RX pcm_open failed: %s", __func__,
              us->rx_pcm ? pcm_get_error(us->rx_pcm) : "null pcm");
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
    ultrasound_load_calibration(us->adev);

    pthread_mutex_unlock(&us->us_lock);
    ALOGD("%s: exit, status(0)", __func__);
    return 0;

err:
    ultrasound_teardown_locked();
    pthread_mutex_unlock(&us->us_lock);
    ALOGE("%s: exit: status(%d)", __func__, -EIO);
    return -EIO;
}

static int ultrasound_extn_stop(void) {
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
    pthread_mutex_lock(&us_init_lock);
    if (!us || atomic_load(&us->state) == ULTRASOUND_STATUS_STARTED) {
        pthread_mutex_unlock(&us_init_lock);
        return -EPERM;
    }
    pthread_mutex_unlock(&us_init_lock);
    return ultrasound_start();
}

/* ------------------------------------------------------------------------ */
/* Socket command server                                                    */
/* ------------------------------------------------------------------------ */

static int process_ultrasound_command(const char* buffer, ssize_t bytes_read) {
    int rc = 0;

    if (bytes_read < (ssize_t)sizeof(struct elliptic_payload)) {
        ALOGE("%s: Payload too small! expected %d, read %d", __func__,
              (int)sizeof(struct elliptic_payload), (int)bytes_read);
        return -1;
    }

    const struct elliptic_payload* pkt = (const struct elliptic_payload*)buffer;
    ALOGD("%s: cmd[0]=%d, cmd[1]=%d, enable=%d", __func__, pkt->cmd[0], pkt->cmd[1], pkt->enable);

    if (pkt->cmd[0] == US_CMD_HEAD && pkt->cmd[1] == US_CMD_TAIL) {
        if (pkt->enable) {
            rc = ultrasound_extn_start();
            if (rc < 0) ALOGW("%s: start returned %d (already started or busy)", __func__, rc);
        } else {
            rc = ultrasound_extn_stop();
            if (rc < 0) ALOGW("%s: stop returned %d (already stopped)", __func__, rc);
        }
    } else {
        ALOGW("%s: Unknown command: cmd[0]=%d, cmd[1]=%d", __func__, pkt->cmd[0], pkt->cmd[1]);
        rc = -1;
    }
    return rc;
}

static void handle_client_connection(int sock) {
    char buffer[MAX_CMD_LEN];

    while (atomic_load(&server_running)) {
        ssize_t bytes_read = recv(sock, buffer, sizeof(buffer), 0);
        int16_t status = 1;

        if (bytes_read <= 0) {
            if (bytes_read == 0)
                ALOGI("%s: client disconnected", __func__);
            else
                ALOGE("%s: recv error: %s", __func__, strerror(errno));
            break;
        }

        int cmd_result = process_ultrasound_command(buffer, bytes_read);
        if (cmd_result < 0 || (us && atomic_load(&us->state) != ULTRASOUND_STATUS_STARTED &&
                               atomic_load(&us->state) != ULTRASOUND_STATUS_STOPPED))
            status = 0;

        if (send(sock, &status, sizeof(status), 0) < 0) {
            ALOGW("%s: Send ACK failed", __func__);
            break;
        }
    }
}

static void* ultrasound_server_thread_func(void* arg __unused) {
    int lfd = android_get_control_socket(US_AUDIO_SOCKET_NAME);
    if (lfd < 0) {
        ALOGE("%s: android_get_control_socket(%s) failed (missing 'socket' line in init rc?)",
              __func__, US_AUDIO_SOCKET_NAME);
        return NULL;
    }
    atomic_store(&sockfd, lfd);

    if (listen(lfd, 4) < 0) {
        ALOGE("%s: listen failed: %s", __func__, strerror(errno));
        atomic_store(&sockfd, -1);
        return NULL;
    }
    ALOGI("%s: listening on %s", __func__, US_AUDIO_SOCKET_NAME);

    while (atomic_load(&server_running)) {
        int cfd = accept(lfd, NULL, NULL);
        if (cfd < 0) {
            if (!atomic_load(&server_running)) break;
            ALOGE("%s: accept failed: %s", __func__, strerror(errno));
            usleep(100000);
            continue;
        }
        atomic_store(&client_fd, cfd);
        handle_client_connection(cfd);
        atomic_store(&client_fd, -1);
        close(cfd);
    }
    return NULL;
}

/* ------------------------------------------------------------------------ */
/* setParameters entry point                                                */
/* ------------------------------------------------------------------------ */

#define US_PARAM_KEY "ultrasound-proximity"
#define US_DISABLE_PROP "persist.vendor.audio.us.disable"
#define US_START_COOLDOWN_MS 60000

static pthread_mutex_t cmd_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cmd_cond = PTHREAD_COND_INITIALIZER;
static int pending_cmd = -1;
static bool worker_running = false;
static bool worker_valid = false;
static pthread_t worker_thread;
static long long next_start_ms = 0;

static long long ultrasound_now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

#define US_POLL_MS 150

static bool us_wanted = false;
static bool us_last_conflict = false;
static bool us_wedge_logged = false;

static void ultrasound_supervise(void) {
    if (!us) return;

    bool running = atomic_load(&us->state) == ULTRASOUND_STATUS_STARTED;
    bool conflict = us_wanted ? ultrasound_conflict_now(us->adev) : false;

    if (us_wanted && conflict != us_last_conflict) {
        ALOGI("%s: conflicting audio %s", __func__, conflict ? "active" : "gone");
        if (conflict) {
            pthread_mutex_lock(&us->adev->lock);
            ultrasound_log_usecases_locked(us->adev);
            pthread_mutex_unlock(&us->adev->lock);
            ultrasound_log_amps(us->adev, "conflict-detected");
        }
        us_last_conflict = conflict;
    }

    if (running && (!us_wanted || conflict)) {
        ALOGI("%s: stopping (%s)", __func__, !us_wanted ? "sensor off" : "conflicting audio");
        int rc = ultrasound_extn_stop();
        ALOGI("%s: stop -> %d", __func__, rc);
        ultrasound_log_amps(us->adev, "after-stop");
        return;
    }

    if (running && us_wanted && !conflict) {
        ultrasound_amp_assert(us->adev);
        return;
    }

    if (!running && us_wanted && !conflict) {
        if (atomic_load(&us_wedged)) {
            if (!us_wedge_logged) {
                ALOGW("%s: not starting, DSP was stuck on a previous stop", __func__);
                us_wedge_logged = true;
            }
            return;
        }
        long long now = ultrasound_now_ms();
        if (now < next_start_ms) return;

        int rc = ultrasound_extn_start();
        ALOGI("%s: start -> %d", __func__, rc);
        if (rc == -EIO) next_start_ms = ultrasound_now_ms() + US_START_COOLDOWN_MS;
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

static void ultrasound_worker_stop(void) {
    pthread_mutex_lock(&cmd_lock);
    worker_running = false;
    pthread_cond_signal(&cmd_cond);
    bool join = worker_valid;
    worker_valid = false;
    pthread_mutex_unlock(&cmd_lock);
    if (join) pthread_join(worker_thread, NULL);
}

void audio_extn_ultrasound_set_parameters(struct audio_device* adev, struct str_parms* parms) {
    char value[32];

    if (!parms || str_parms_get_str(parms, US_PARAM_KEY, value, sizeof(value)) < 0) return;

    int enable = atoi(value);
    ALOGD("%s: %s=%s", __func__, US_PARAM_KEY, value);
    if (adev && us) ultrasound_log_amps(adev, enable ? "set_parameters=1" : "set_parameters=0");

    if (enable && property_get_bool(US_DISABLE_PROP, false)) {
        ALOGW("%s: ignoring start, disabled via %s", __func__, US_DISABLE_PROP);
        return;
    }

    pthread_mutex_lock(&cmd_lock);
    if (worker_valid) {
        pending_cmd = enable ? 1 : 0;
        pthread_cond_signal(&cmd_cond);
    } else {
        ALOGW("%s: worker not running, dropping request", __func__);
    }
    pthread_mutex_unlock(&cmd_lock);
}

/* ------------------------------------------------------------------------ */
/* Calibration                                                              */
/* ------------------------------------------------------------------------ */

#define US_CAL_MAX_BYTES 4096

static size_t ultrasound_parse_hex_text(const char* text, uint8_t* out, size_t out_max) {
    size_t n = 0;
    const char* p = text;

    while (*p && n < out_max) {
        while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t' || *p == ',') p++;
        if (!*p) break;
        char* e = NULL;
        unsigned long v = strtoul(p, &e, 16);
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

/* ------------------------------------------------------------------------ */
/* Init / deinit                                                            */
/* ------------------------------------------------------------------------ */

static void ultrasound_server_init(void) {
    if (atomic_load(&server_running)) return;
    atomic_store(&server_running, true);
    if (pthread_create(&ultrasound_server_thread, NULL, ultrasound_server_thread_func, NULL) != 0) {
        ALOGE("%s: Failed to create server thread", __func__);
        atomic_store(&server_running, false);
        return;
    }
    server_thread_valid = true;
}

void audio_extn_ultrasound_deinit(void) {
    ALOGD("%s: enter", __func__);

    ultrasound_worker_stop();
    atomic_store(&server_running, false);

    int cfd = atomic_exchange(&client_fd, -1);
    if (cfd >= 0) shutdown(cfd, SHUT_RDWR);
    int lfd = atomic_exchange(&sockfd, -1);
    if (lfd >= 0) shutdown(lfd, SHUT_RDWR);

    if (server_thread_valid) {
        pthread_join(ultrasound_server_thread, NULL);
        server_thread_valid = false;
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
    ultrasound_server_init();

    ALOGD("%s: exit, status(0)", __func__);
    return 0;
}
