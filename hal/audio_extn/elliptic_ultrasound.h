#ifndef AUDIO_EXTN_ELLIPTIC_ULTRASOUND_H
#define AUDIO_EXTN_ELLIPTIC_ULTRASOUND_H

#ifdef __cplusplus
extern "C" {
#endif

struct audio_device;
struct str_parms;

/* Allocates state, starts the audio_hw_socket server thread, loads calibration. */
int audio_extn_ultrasound_init(struct audio_device* adev);

/* Handles "ultrasound-proximity=1|0" from the Elliptic sensor HAL.
 * Call from adev_set_parameters() with the parsed str_parms.
 */
void audio_extn_ultrasound_set_parameters(struct audio_device* adev, struct str_parms* parms);

/* Stops the server thread and any running ultrasound session. */
void audio_extn_ultrasound_deinit(void);

// async
void audio_extn_ultrasound_sync_stop(struct audio_device *adev, int out_snd_device);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_EXTN_ELLIPTIC_ULTRASOUND_H */
