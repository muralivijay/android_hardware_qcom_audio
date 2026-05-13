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
#include <unistd.h>
#include "audio_hw.h"
#include "platform.h"
#include "platform_api.h"

#define US_AUDIO_SOCKET_NAME "audio_hw_socket"
#define MAX_CMD_LEN 1024
#define US_CMD_HEAD 17
#define US_CMD_TAIL 1

#define MI_US_CAL_PATH "/mnt/vendor/persist/audio/mi_us_cal.txt"
#define MI_US_MIXER_CTL "Mi_Ultrasound Calibration Data"
#define MIUS_CALIBRATION_FLOAT_DATA_SIZE 56

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
    rx_usecase->out_snd_device = SND_DEVICE_OUT_VOICE_HANDSET;
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

static int process_ultrasound_command(struct audio_device* adev, char* buffer, int bytes_read) {
    int rc = 0;

    if (bytes_read < sizeof(struct elliptic_payload)) {
        ALOGE("%s: Payload too small! expected %zu, read %d", __func__,
              sizeof(struct elliptic_payload), bytes_read);
        return -1;
    }

    struct elliptic_payload* pkt = (struct elliptic_payload*)buffer;

    ALOGD("%s: Parsed: cmd[0]=%d, cmd[1]=%d, enable=%d", __func__, pkt->cmd[0], pkt->cmd[1],
          pkt->enable);

    if (pkt->cmd[0] == US_CMD_HEAD && pkt->cmd[1] == US_CMD_TAIL) {
        if (pkt->enable) {
            ALOGI("%s: Received ENABLE command", __func__);
            rc = ultrasound_extn_start();
            if (rc < 0) {
                ALOGW("%s: ultrasound_start returned %d (maybe already started)", __func__, rc);
            }
        } else {
            ALOGI("%s: Received DISABLE command", __func__);
            rc = ultrasound_extn_stop();
            if (rc < 0) {
                ALOGW("%s: ultrasound_stop returned %d (maybe already stopped)", __func__, rc);
            }
        }
    } else {
        ALOGW("%s: Unknown command: cmd[0]=%d, cmd[1]=%d", __func__, pkt->cmd[0], pkt->cmd[1]);
        rc = -1;
    }

    return rc;
}

static void handle_client_connection(struct audio_device* adev, int client_sock) {
    char buffer[MAX_CMD_LEN];

    while (atomic_load(&server_running)) {
        int bytes_read = recv(client_sock, buffer, sizeof(buffer), 0);
        int16_t status = 1;
        int cmd_result = 0;

        if (bytes_read <= 0) {
            if (bytes_read == 0)
                ALOGI("%s: client disconnected", __func__);
            else
                ALOGE("%s: recv error: %s", __func__, strerror(errno));

            break;
        }

        cmd_result = process_ultrasound_command(adev, buffer, bytes_read);

        if (cmd_result < 0 || (us && (atomic_load(&us->state) != ULTRASOUND_STATUS_STARTED &&
                                      atomic_load(&us->state) != ULTRASOUND_STATUS_STOPPED))) {
            status = 0;
        }

        if (send(client_sock, &status, sizeof(status), 0) < 0) {
            ALOGW("%s: Send ACK failed", __func__);
            break;
        }
    }
    close(client_sock);
}

static void* ultrasound_server_thread_func(void* arg) {
    struct audio_device* adev = (struct audio_device*)arg;

    ALOGD("%s: Thread start", __func__);

    sockfd = android_get_control_socket(US_AUDIO_SOCKET_NAME);
    if (sockfd < 0) return NULL;

    if (listen(sockfd, 4) < 0) {
        ALOGE("%s: listen failed: %s", __func__, strerror(errno));
        close(sockfd);
        sockfd = -1;
        return NULL;
    }
    ALOGI("%s: Listening on socket...", __func__);

    while (atomic_load(&server_running)) {
        struct sockaddr_un client_addr;
        socklen_t client_len = sizeof(client_addr);

        ALOGV("%s: Waiting for client...", __func__);
        int client_sock = accept(sockfd, (struct sockaddr*)&client_addr, &client_len);

        if (client_sock < 0) {
            if (!atomic_load(&server_running)) {
                ALOGI("%s: Server stopping, exiting accept loop", __func__);
                break;
            }

            ALOGE("%s: Accept failed: %s", __func__, strerror(errno));
            usleep(100000);
            continue;
        }

        ALOGV("%s: Client connected!", __func__);

        handle_client_connection(adev, client_sock);
    }

    close(sockfd);

    return NULL;
}

void audio_extn_ultrasound_deinit(void) {
    ALOGD("%s: enter", __func__);

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

void ultrasound_load_calibration(struct audio_device* adev) {
    FILE* fp;
    char buf[144];
    size_t bytes_read;
    int i;
    char* token;
    char* saveptr;
    float cal_data[MIUS_CALIBRATION_FLOAT_DATA_SIZE / sizeof(float)] = {0};
    struct mixer_ctl* ctl;

    ALOGD("%s: enter", __func__);

    fp = fopen(MI_US_CAL_PATH, "r");
    if (!fp) {
        ALOGE("%s: Cannot open calibration file: %s", __func__, MI_US_CAL_PATH);
        return;
    }

    bytes_read = fread(buf, 1, 140, fp);
    if (bytes_read == 0) {
        ALOGE("%s: Cannot read calibration file: %s", __func__, MI_US_CAL_PATH);
        fclose(fp);
        return;
    }

    buf[bytes_read] = '\0';
    i = 0;
    token = strtok_r(buf, " \n\r", &saveptr);
    while (token != NULL) {
        cal_data[i] = atof(token);
        token = strtok_r(NULL, " \n\r", &saveptr);
        i++;
        if (i >= (MIUS_CALIBRATION_FLOAT_DATA_SIZE / sizeof(float))) break;
    }
    fclose(fp);

    ctl = mixer_get_ctl_by_name(adev->mixer, MI_US_MIXER_CTL);
    if (!ctl) {
        ALOGE("%s: Could not get MI_US_CAL ctl for mixer cmd - %s", __func__, MI_US_MIXER_CTL);
        return;
    }

    if (mixer_ctl_set_array(ctl, cal_data, MIUS_CALIBRATION_FLOAT_DATA_SIZE) < 0) {
        ALOGE("%s: Could not set ctl", __func__);
    }

    ALOGD("%s: exit", __func__);

    return;
}

static void ultrasound_server_init(struct audio_device* adev) {
    if (atomic_load(&server_running)) return;

    atomic_store(&server_running, true);
    if (pthread_create(&ultrasound_server_thread, NULL, ultrasound_server_thread_func, adev) != 0) {
        ALOGE("%s: Failed to create server thread", __func__);
        atomic_store(&server_running, false);
    }
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

    ultrasound_server_init(us->adev);
    ultrasound_load_calibration(us->adev);

    ALOGD("%s: exit, status(0)", __func__);

    return 0;
}
