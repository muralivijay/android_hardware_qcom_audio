/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ULTRASOUND_H
#define ULTRASOUND_H

struct audio_device;
struct str_parms;

int audio_extn_ultrasound_init(struct audio_device* adev);
void audio_extn_ultrasound_deinit(void);
void audio_extn_ultrasound_set_parameters(struct audio_device* adev, struct str_parms* parms);

#endif
