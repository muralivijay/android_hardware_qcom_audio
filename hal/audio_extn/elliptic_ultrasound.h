/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ULTRASOUND_H
#define ULTRASOUND_H

struct audio_device;

int audio_extn_ultrasound_init(struct audio_device* adev);
void audio_extn_ultrasound_deinit(void);

#endif
