/*
 * Copyright (c) 2018, Dallin Briggs
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

#include "gimbal.h"

namespace gimbal {

//==================================================================
// Constructor
//==================================================================
Gimbal::Gimbal()
{
    parse_state = PARSE_STATE_IDLE;
    command_in_rate = 0.0;
    servo_command_rate = 0.0;
    info.init(LED2_GPIO, LED2_PIN);

}


//==================================================================
// handle received serial data
//==================================================================
void Gimbal::rx_callback(uint8_t byte)
{
    if (parse_in_byte(byte))
    {
        float roll, pitch, yaw;
        unpack_in_payload(in_payload_buf, &roll, &pitch, &yaw);
        handle_in_msg(roll, pitch, yaw);
    }
}

//==================================================================
// handle received message
//==================================================================
void Gimbal::handle_in_msg(float roll, float pitch, float yaw)
{
    calc_command_rate();
    time_of_last_command = millis();
    roll_command = roll;
    pitch_command = pitch;
    yaw_command = yaw;
}

//==================================================================
// unpack incoming message payload
//==================================================================
void Gimbal::unpack_in_payload(uint8_t buf[], float *roll, float *pitch, float *yaw)
{
    memcpy(roll, buf,     4);
    memcpy(pitch, buf + 4, 4);
    memcpy(yaw, buf + 8, 4);
}

//==================================================================
// handle an incoming byte
//==================================================================
bool Gimbal::parse_in_byte(uint8_t c)
{
    bool got_message = false;
    switch (parse_state)
    {
    case PARSE_STATE_IDLE:
        if (c == IN_START_BYTE)
        {
            in_crc_value = CRC_INITIAL_VALUE;
            in_crc_value = in_crc8_ccitt_update(in_crc_value, c);

            in_payload_index = 0;
            parse_state = PARSE_STATE_GOT_START_BYTE;
        }
        else
        {
            start_byte_error += 1;
        }
        break;

    case PARSE_STATE_GOT_START_BYTE:
        in_crc_value = in_crc8_ccitt_update(in_crc_value, c);
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
        else
        {
            crc_error_count += 1;
        }
        parse_state = PARSE_STATE_IDLE;
        break;
    }

    return got_message;
}


//==================================================================
// handle in crc
//==================================================================
uint8_t Gimbal::in_crc8_ccitt_update (uint8_t inCrc, uint8_t inData)
{
    uint8_t   i;
    uint8_t   data;

    data = inCrc ^ inData;

    for ( i = 0; i < IN_PAYLOAD_LENGTH; i++ )
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
// handle out crc
//==================================================================
uint8_t Gimbal::out_crc8_ccitt_update (uint8_t outCrc, uint8_t outData)
{
    uint8_t   i;
    uint8_t   data;

    data = outCrc ^ outData;

    for ( i = 0; i < OUT_PAYLOAD_LENGTH; i++ )
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
// Serialize the out message
//==================================================================
void Gimbal::tx_callback(float command_rate, float servo_rate, float roll, float pitch, float yaw)
{
    out_buf[0] = OUT_START_BYTE;
    memcpy(out_buf+1, &command_rate, sizeof(float));
    memcpy(out_buf+5, &servo_rate, sizeof(float));
    memcpy(out_buf+9, &roll, sizeof(float));
    memcpy(out_buf+13, &pitch, sizeof(float));
    memcpy(out_buf+17, &yaw, sizeof(float));

    uint8_t out_crc_value = CRC_INITIAL_VALUE;
    for (int i = 0; i < OUT_MESSAGE_LENGTH - CRC_LENGTH; i++)
    {
        out_crc_value = out_crc8_ccitt_update(out_crc_value, out_buf[i]);
    }
    out_buf[OUT_MESSAGE_LENGTH - 1] = out_crc_value;
}


//==================================================================
// Blink LED while receiving data for hardware debugging
//==================================================================
void Gimbal::blink_led()
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
void Gimbal::norm_commands()
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

void Gimbal::calc_command_rate()
{
    command_in_rate = 1000.0/float(millis() - time_of_last_command);
}

} // End gimbal namespace


int main() {
    systemInit();

    gimbal::Gimbal gimbal_obj;

    VCP vcp;
    VCP* uartPtr = nullptr;

    vcp.init();
    uartPtr = &vcp;
    vcp.register_rx_callback(std::bind(&gimbal::Gimbal::rx_callback, &gimbal_obj, std::placeholders::_1));

    PWM_OUT servo_out[3];
    int servo_frequency = 50;
    for (int i = 0; i < 3; ++i)
    {
//        servo_out[i].init(&pwm_config[i], servo_frequency, 2470, 530); // This works for a BL815H servo.
        servo_out[i].init(&pwm_config[i], servo_frequency, 2400, 600); // This works for a 9g servo.
    }
    servo_out[0].write(gimbal::Gimbal::roll_offset/gimbal::Gimbal::RAD_RANGE);
    servo_out[1].write(-gimbal::Gimbal::pitch_offset/gimbal::Gimbal::RAD_RANGE);
    servo_out[2].write(gimbal::Gimbal::yaw_offset/gimbal::Gimbal::RAD_RANGE);

    while(1)
    {
        while(vcp.rx_bytes_waiting())
        {
            uint8_t byte = vcp.read_byte();
            gimbal_obj.rx_callback(byte);
            gimbal_obj.norm_commands();
            if (gimbal_obj.roll_command > 500 && gimbal_obj.roll_command < 2500 || gimbal_obj.pitch_command > 500 && gimbal_obj.pitch_command < 2500
                    || gimbal_obj.yaw_command > 500 && gimbal_obj.yaw_command < 2500)
            {
                //            servo_out[0].writeUs(roll_command);
                servo_out[1].writeUs(gimbal_obj.pitch_command);
                servo_out[2].writeUs(gimbal_obj.yaw_command);
            }
            else
            {
                //            servo_out[0].write(norm_roll);
                gimbal_obj.servo_command_rate = 1000.0 / float(millis() - gimbal_obj.time_of_last_servo);

                servo_out[1].write(gimbal_obj.norm_pitch);
                servo_out[2].write(gimbal_obj.norm_yaw);
                gimbal_obj.tx_callback(gimbal_obj.command_in_rate, gimbal_obj.servo_command_rate,
                            gimbal_obj.roll_command, gimbal_obj.pitch_command, gimbal_obj.yaw_command);
                vcp.write(gimbal_obj.out_buf, gimbal::Gimbal::OUT_MESSAGE_LENGTH);
                vcp.flush();
            }
        }
        gimbal_obj.time_of_last_servo = millis();
    }
}
