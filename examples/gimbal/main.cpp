/*
 * Copyright (c) 2018, James Jackson, Dallin Briggs
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
#include "vcp.h"

#include "revo_f4.h"

// serial communication
#define OUT_BUFFER_SIZE 14
#define OUT_START_BYTE 0xA5
#define OUT_PAYLOAD_LENGTH 12
#define OUT_MESSAGE_LENGTH 14

#define IN_BUFFER_SIZE 14
#define IN_START_BYTE 0xA5
#define IN_PAYLOAD_LENGTH 12
#define IN_MESSAGE_LENGTH 14

#define CRC_LENGTH 1
#define CRC_INITIAL_VALUE 0x00



VCP* uartPtr = NULL;

enum ParseState {
    PARSE_STATE_IDLE,
    PARSE_STATE_GOT_START_BYTE,
    PARSE_STATE_GOT_PAYLOAD
};

volatile float roll_command;
volatile float pitch_command;
volatile float yaw_command;
volatile float norm_roll;
volatile float norm_pitch;
volatile float norm_yaw;

// This value is the total range in radians the servo can travel.
static float RAD_RANGE = 3.14159;

// Offset values for aligning gimbal servos with desired coordinate frame.
// Radians. Must be positive.
static float roll_offset = 0;
static float pitch_offset = -0.45;
static float yaw_offset = 0;

// Limit the servo travel for mechanical restrictions.
// These are normalized values for the write function. Between 0 and 1.
static float roll_lower_limit = 0;
static float pitch_lower_limit = 0;
static float yaw_lower_limit = 0;

static float roll_upper_limit = 1.0;
static float pitch_upper_limit = 2.15/RAD_RANGE;
static float yaw_upper_limit = 1.0;

volatile long time_of_last_command;
volatile long time_of_last_blink;


// serial
uint8_t out_buf[OUT_BUFFER_SIZE];
uint8_t in_buf[IN_BUFFER_SIZE];

ParseState parse_state;
uint8_t in_payload_buf[IN_PAYLOAD_LENGTH];
int in_payload_index;
uint8_t in_crc_value;

LED info;


void handle_in_msg(float roll, float pitch, float yaw);
void unpack_in_payload(uint8_t buf[IN_PAYLOAD_LENGTH], float *roll, float *pitch, float *yaw);
bool parse_in_byte(uint8_t c);

uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData);

void blink_led();
void norm_commands();

//==================================================================
// handle received serial data
//==================================================================
void rx_callback(uint8_t byte)
{
    if (parse_in_byte(byte))
    {
        float roll, pitch, yaw;
        unpack_in_payload(in_payload_buf, &roll, &pitch, &yaw);
        handle_in_msg(roll, pitch, yaw);
    }
    // Use the following lines if you want to echo back what you received.
    //    uartPtr->put_byte(byte);
    //    uartPtr->flush();
}

//==================================================================
// handle received message
//==================================================================
void handle_in_msg(float roll, float pitch, float yaw)
{
    time_of_last_command = millis();
    roll_command = roll;
    pitch_command = pitch;
    yaw_command = yaw;
}

//==================================================================
// unpack incoming message payload
//==================================================================
void unpack_in_payload(uint8_t buf[], float *roll, float *pitch, float *yaw)
{
    memcpy(roll, buf,     4);
    memcpy(pitch, buf + 4, 4);
    memcpy(yaw, buf + 8, 4);
}

//==================================================================
// handle an incoming byte
//==================================================================
bool parse_in_byte(uint8_t c)
{
    bool got_message = false;
    switch (parse_state)
    {
    case PARSE_STATE_IDLE:
        if (c == IN_START_BYTE)
        {
            in_crc_value = CRC_INITIAL_VALUE;
            in_crc_value = _crc8_ccitt_update(in_crc_value, c);

            in_payload_index = 0;
            parse_state = PARSE_STATE_GOT_START_BYTE;
        }
        break;

    case PARSE_STATE_GOT_START_BYTE:
        in_crc_value = _crc8_ccitt_update(in_crc_value, c);
        in_payload_buf[in_payload_index++] = c;
        if (in_payload_index == IN_PAYLOAD_LENGTH)
        {
            parse_state = PARSE_STATE_GOT_PAYLOAD;
        }
        break;

    case PARSE_STATE_GOT_PAYLOAD:
        if (c == in_crc_value)
        {
            got_message = true;
            blink_led();
        }
        parse_state = PARSE_STATE_IDLE;
        break;
    }

    return got_message;
}


//==================================================================
// handle crc
//==================================================================
uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData)
{
    uint8_t   i;
    uint8_t   data;

    data = inCrc ^ inData;

    for ( i = 0; i < 12; i++ )
    {
        if (( data & 0x80 ) != 0 )
        {
            data <<= 1;
            data ^= 0x07;
        }
        else
        {
            data <<= 1;
        }
    }
    return data;
}


//==================================================================
// Blink LED while receiving data for hardware debugging
//==================================================================
void blink_led()
{
    if(millis() - time_of_last_blink >= 100)
    {
        time_of_last_blink = millis();
        info.toggle();
    }
}

//==================================================================
// Normalize angle commands for servo writing function
//==================================================================
void norm_commands()
{
    norm_roll = (roll_command+roll_offset)/RAD_RANGE;
    norm_pitch = -(pitch_command+pitch_offset)/RAD_RANGE;
    norm_yaw = (yaw_command+yaw_offset)/RAD_RANGE;

    if (norm_roll > roll_upper_limit)
        norm_roll = roll_upper_limit;
    else if (norm_roll < roll_lower_limit)
        norm_roll = roll_lower_limit;

    if (norm_pitch > pitch_upper_limit)
        norm_pitch = pitch_upper_limit;
    else if (norm_pitch < pitch_lower_limit)
        norm_pitch = pitch_lower_limit;

    if (norm_yaw > yaw_upper_limit)
        norm_yaw = yaw_upper_limit;
    else if (norm_yaw < yaw_lower_limit)
        norm_yaw = yaw_lower_limit;

}


int main() {
    systemInit();

    VCP vcp;
    vcp.init();
    uartPtr = &vcp;
    vcp.register_rx_callback(&rx_callback);

    parse_state = PARSE_STATE_IDLE;


    info.init(LED2_GPIO, LED2_PIN);

    PWM_OUT servo_out[3];
    for (int i = 0; i < 3; ++i)
    {
        servo_out[i].init(&pwm_config[i], 300, 2470, 530); // This works a BL815H servo.
    }
    servo_out[0].write(roll_offset/RAD_RANGE);
    servo_out[1].write(-pitch_offset/RAD_RANGE);
    servo_out[2].write(yaw_offset/RAD_RANGE);

    while(1)
    {
        while(vcp.rx_bytes_waiting())
        {
            uint8_t byte = vcp.read_byte();
            rx_callback(byte);
            norm_commands();
            if (roll_command > 500 && roll_command < 2500 || pitch_command > 500 && pitch_command < 2500 || yaw_command > 500 && yaw_command < 2500)
            {
                //            servo_out[0].writeUs(roll_command);
                servo_out[1].writeUs(pitch_command);
                servo_out[2].writeUs(yaw_command);
            }
            else
            {
                //            servo_out[0].write(norm_roll);
                servo_out[1].write(norm_pitch);
                servo_out[2].write(norm_yaw);
            }
        }
    }
}




