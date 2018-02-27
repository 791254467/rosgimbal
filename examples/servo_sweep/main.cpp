/*
 * Copyright (c) 2017, James Jackson
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "system.h"
#include "pwm.h"
#include "led.h"

#include "revo_f4.h"

int main() {
	systemInit();

	LED info;
	info.init(LED2_GPIO, LED2_PIN);

    PWM_OUT servo_out[PWM_NUM_OUTPUTS];
	for (int i = 0; i < PWM_NUM_OUTPUTS; ++i)
	{
        servo_out[i].init(&pwm_config[i], 50, 2300, 700);
        servo_out[i].write(1.0);
	}

    // Set servos to middle position to start.
    while (millis() < 500);

	for (int i = 0; i < PWM_NUM_OUTPUTS; ++i)
	{
        servo_out[i].write(0.5);
	}

	while(1)
	{
		for (int i = 0; i < PWM_NUM_OUTPUTS; ++i)
		{
            for (float pos = 0.0; pos <= 1.0; pos+=0.006)
            {
                servo_out[i].write(pos);
                delay(15);
            }
            delay(200);
            for (float pos = 1.0; pos >= 0.0; pos-=0.006)
            {
                servo_out[i].write(pos);
                delay(15);
            }
            delay(200);
		}
        delay(500);
	}
}
