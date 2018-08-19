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
    command_in_rate = 20.0;
    servo_command_rate = 0.0;
    time_of_last_servo = 0;
    info.init(LED2_GPIO, LED2_PIN);
    heartbeat.init(LED1_GPIO, LED1_PIN);
    param_value = PARAM_IDLE;
    spi.init(&spi_config[FLASH_SPI]);
    flash.init(&spi);

    pitch_current_pwm = pitch_start_pwm;
    yaw_current_pwm = yaw_start_pwm;
    pitch_pwm_center = (pitch_pwm_max + pitch_pwm_min)/2;
    yaw_pwm_center = (yaw_pwm_max + yaw_pwm_min)/2;
    //    time_of_last_message = millis();
}


//==================================================================
// handle received serial data
//==================================================================
void Gimbal::rx_callback(uint8_t byte)
{
    if (parse_in_byte(byte))
    {
        unpack_in_payload(in_payload_buf, &roll, &pitch, &yaw);
        handle_in_msg(roll, pitch, yaw);
        set_params(roll, pitch, yaw);

        if (roll == 4000 && has_retract)
        {
            if (first_retract)
                retract_time = millis();
            retract_gimbal();
            is_retracted = true;
            first_retract = false;
            first_extend = true;
        }
        else if (is_retracted && roll != 4000 && has_retract)
        {
            if (first_extend)
                extend_time = millis();
            extend_gimbal();

            first_retract = true;
            first_extend = false;
        }
        else if (!is_retracted || !has_retract)
        {
            calc_servo_rate();

            //            servo_out[0].writeUs(roll_pwm_command);
//            servo_out[1].writeUs(pitch_pwm_command);
//            servo_out[2].writeUs(yaw_pwm_command);
        }
        pwm_to_rad();
        tx_callback(command_in_rate, servo_command_rate,
                    roll_rad_command, pitch_current_rad, yaw_current_rad);
        vcp.write(out_buf, gimbal::Gimbal::OUT_MESSAGE_LENGTH);
//        vcp.flush();
    }
}


//==================================================================
// Get the parameters from the eeprom if they are there.
//==================================================================
void Gimbal::get_params()
{
    flash.read_config(eeprom_buffer, sizeof(EepromData));
    eeprom_data = eeprom_data;
}


//==================================================================
// Set parameters if requested. This is a super hacky way of getting params
// in without using mavlink. It just listens to the existing roll and pitch
// channels to set param values.
//==================================================================
void Gimbal::set_params(float roll, float pitch, float yaw)
{
    param_value = ParamValue(roll);
    switch (param_value) {

    case SERVO_PITCH_FREQUENCY:
        if (pitch > 10 && pitch < 355)
        {
            servo_pitch_frequency = pitch;
            eeprom_data.servo_pitch_frequency = servo_pitch_frequency;
        }
        break;
    case SERVO_PITCH_UPPER_PWM:
        pitch_pwm_max = pitch;
        eeprom_data.pitch_pwm_max = pitch_pwm_max;
        break;
    case SERVO_PITCH_LOWER_PWM:
        pitch_pwm_min = pitch;
        eeprom_data.pitch_pwm_min = pitch_pwm_min;
        break;
    case SERVO_PITCH_DIRECTION:
        if(pitch > 0)
        {
            pitch_direction = 1;
            eeprom_data.pitch_direction = pitch_direction;
        }
        else if (pitch < 0)
        {
            pitch_direction = -1;
            eeprom_data.pitch_direction = pitch_direction;
        }
        break;
    case SERVO_PITCH_RAD_RANGE:
        pitch_rad_range = pitch;
        eeprom_data.pitch_rad_range = pitch_rad_range;
        break;
    case SERVO_PITCH_RAD_OFFSET:
        pitch_rad_offset = pitch;
        eeprom_data.pitch_rad_offset = pitch_rad_offset;
        break;
    case SERVO_PITCH_START_PWM:
        pitch_start_pwm = pitch;
        eeprom_data.pitch_start_pwm = pitch_start_pwm;
        break;

    case SERVO_YAW_FREQUENCY:
        if (pitch > 10 && pitch < 355)
        {
            servo_yaw_frequency = pitch;
            eeprom_data.servo_yaw_frequency = servo_yaw_frequency;
        }
        break;
    case SERVO_YAW_UPPER_PWM:
        yaw_pwm_max = pitch;
        eeprom_data.yaw_pwm_max = yaw_pwm_max;
        break;
    case SERVO_YAW_LOWER_PWM:
        yaw_pwm_min = pitch;
        eeprom_data.yaw_pwm_min = yaw_pwm_min;
        break;
    case SERVO_YAW_DIRECTION:
        if(pitch > 0)
        {
            yaw_direction = 1;
            eeprom_data.yaw_direction = yaw_direction;
        }
        else if (pitch < 0)
        {
            yaw_direction = -1;
            eeprom_data.yaw_direction = yaw_direction;
        }
        break;
    case SERVO_YAW_RAD_RANGE:
        yaw_rad_range = pitch;
        eeprom_data.yaw_rad_range = yaw_rad_range;
        break;
    case SERVO_YAW_RAD_OFFSET:
        yaw_rad_offset = pitch;
        eeprom_data.yaw_rad_offset = yaw_rad_offset;
        break;
    case SERVO_YAW_START_PWM:
        yaw_start_pwm = pitch;
        eeprom_data.yaw_start_pwm = yaw_start_pwm;
        break;

    case SERVO_RETRACT_FREQUENCY:
        if (pitch > 10 && pitch < 355)
        {
            servo_retract_frequency = pitch;
            eeprom_data.servo_retract_frequency = servo_retract_frequency;
        }
        break;
    case SERVO_RETRACT_UPPER_PWM:
        retract_pwm_max = pitch;
        eeprom_data.retract_pwm_max = retract_pwm_max;
        break;
    case SERVO_RETRACT_LOWER_PWM:
        retract_pwm_min = pitch;
        eeprom_data.retract_pwm_min = retract_pwm_min;
        break;
    case SERVO_RETRACT_DIRECTION:
        if(pitch > 0)
        {
            retract_direction = 1;
            eeprom_data.retract_direction = retract_direction;
        }
        else if (pitch < 0)
        {
            retract_direction = -1;
            eeprom_data.retract_direction = retract_direction;
        }
        break;
    case SERVO_RETRACT_RAD_RANGE:
        retract_rad_range = pitch;
        eeprom_data.retract_rad_range = retract_rad_range;
        break;
    case SERVO_RETRACT_RAD_OFFSET:
        retract_rad_offset = pitch;
        eeprom_data.retract_rad_offset = retract_rad_offset;
        break;
    case SERVO_RETRACT_START_PWM:
        retract_start_pwm = pitch;
        eeprom_data.retract_start_pwm = retract_start_pwm;
        break;

    case WRITE_PARAMS:
        flash.write_config((uint8_t*)&eeprom_data, sizeof(EepromData));
        break;
    case READ_PARAMS:
        get_params();
        break;
    default:
        break;
    }
}



//==================================================================
// handle received message
//==================================================================
void Gimbal::handle_in_msg(float roll, float pitch, float yaw)
{
    calc_command_rate();
    time_of_last_command = millis();
    roll_rad_command = roll;
    pitch_rad_command = pitch;
    yaw_rad_command = yaw;
    if (roll < 4999)
    {
        rad_to_pwm();
    }
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

void Gimbal::blink_heartbeat()
{
    if(millis() - time_of_last_heartbeat >= 1000)
    {
        time_of_last_heartbeat = millis();
        heartbeat.toggle();
    }
}

void Gimbal::rad_to_pwm()
{
    roll_pwm_center = (roll_pwm_max + roll_pwm_min)/2;
    pitch_pwm_center = (pitch_pwm_max + pitch_pwm_min)/2;
    yaw_pwm_center = (yaw_pwm_max + yaw_pwm_min)/2;

    roll_pwm_command = roll_pwm_center + roll_direction*(roll_rad_command - roll_rad_offset)/(roll_rad_range)*(roll_pwm_max - roll_pwm_min);
    pitch_pwm_command = pitch_pwm_center + pitch_direction*(pitch_rad_command - pitch_rad_offset)/(pitch_rad_range)*(pitch_pwm_max - pitch_pwm_min);
    yaw_pwm_command = yaw_pwm_center + yaw_direction*(yaw_rad_command - yaw_rad_offset)/(yaw_rad_range)*(yaw_pwm_max - yaw_pwm_min);

    if (roll_pwm_command > roll_pwm_max)
        roll_pwm_command = roll_pwm_max;
    else if (roll_pwm_command < roll_pwm_min)
        roll_pwm_command = roll_pwm_min;

    if (pitch_pwm_command > pitch_pwm_max)
        pitch_pwm_command = pitch_pwm_max;
    else if (pitch_pwm_command < pitch_pwm_min)
        pitch_pwm_command = pitch_pwm_min;

    if (yaw_pwm_command > yaw_pwm_max)
        yaw_pwm_command = yaw_pwm_max;
    else if (yaw_pwm_command < yaw_pwm_min)
        yaw_pwm_command = yaw_pwm_min;
}

void Gimbal::pwm_to_rad()
{
    pitch_current_rad = (float(pitch_current_pwm) - pitch_pwm_center)*pitch_rad_range/(pitch_pwm_max - pitch_pwm_min)/pitch_direction + pitch_rad_offset;
    yaw_current_rad = (float(yaw_current_pwm) - yaw_pwm_center)*yaw_rad_range/(yaw_pwm_max - yaw_pwm_min)/yaw_direction + yaw_rad_offset;
}


void Gimbal::calc_command_rate()
{
    command_in_rate = 1000.0/float(millis() - time_of_last_command);
//    command_in_rate = 20;
}

void Gimbal::calc_servo_rate()
{
    servo_command_rate = 1000.0 / float(millis() - time_of_last_servo);
    time_of_last_servo = millis();
    if(isinf(servo_command_rate))
        servo_command_rate = -1;
    else if (isnan(servo_command_rate))
        servo_command_rate = -2;
    else
        servo_command_rate;
}

void Gimbal::retract_gimbal()
{
    servo_out[1].writeUs(pitch_start_pwm);
    servo_out[2].writeUs(yaw_start_pwm);
    if (millis() - retract_time > 4000)
        servo_out[3].writeUs(retract_up_pwm);
}

void Gimbal::extend_gimbal()
{
    servo_out[1].writeUs(pitch_start_pwm);
    servo_out[2].writeUs(yaw_start_pwm);
    servo_out[3].writeUs(retract_down_pwm);
    if (millis() - extend_time > 4000)
    {
        is_retracted = false;
    }
}
uint32_t sat(uint32_t min, uint32_t max, uint32_t x)
{
    if (x > max)
        return max;
    else if (x < min)
        return min;
    else
        return x;
}

void Gimbal::update_command()
{
    static uint64_t last_update_us = micros();
    if(micros() > last_update_us + 3000)
    {
        last_update_us = micros();
        pitch_current_pwm = sat(pitch_current_pwm - 1, pitch_current_pwm + 1, pitch_pwm_command);
        yaw_current_pwm = sat(yaw_current_pwm - 1, yaw_current_pwm + 1, yaw_pwm_command);
        servo_out[1].writeUs(pitch_current_pwm);
        servo_out[2].writeUs(yaw_current_pwm);
    }
}

} // End gimbal namespace


extern "C" {
/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
void HardFault_Handler( void ) __attribute__( ( naked ) );

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
void HardFault_Handler(void)
{
    __asm volatile
            (
                " tst lr, #4                                                \n"
                " ite eq                                                    \n"
                " mrseq r0, msp                                             \n"
                " mrsne r0, psp                                             \n"
                " ldr r1, [r0, #24]                                         \n"
                " ldr r2, handler2_address_const                            \n"
                " bx r2                                                     \n"
                " handler2_address_const: .word prvGetRegistersFromStack    \n"
                );
}

void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
    /* These are volatile to try and prevent the compiler/linker optimising them
  away as the variables never actually get used.  If the debugger won't show the
  values of the variables, make them global my moving their declaration outside
  of this function. */
    volatile uint32_t r0;
    volatile uint32_t r1;
    volatile uint32_t r2;
    volatile uint32_t r3;
    volatile uint32_t r12;
    volatile uint32_t lr; /* Link register. */
    volatile uint32_t pc; /* Program counter. */
    volatile uint32_t psr;/* Program status register. */

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    // avoid compiler warnings about unused variables
    (void) r0;
    (void) r1;
    (void) r2;
    (void) r3;
    (void) r12;
    (void) lr;
    (void) pc;
    (void) psr;

    /* When the following line is hit, the variables contain the register values. */
    for( ;; );

}
}


int main() {
    systemInit();

    gimbal::Gimbal gimbal_obj;

    VCP* uartPtr = nullptr;

    gimbal_obj.vcp.init();
    uartPtr = &gimbal_obj.vcp;
    gimbal_obj.vcp.register_rx_callback(std::bind(&gimbal::Gimbal::rx_callback, &gimbal_obj, std::placeholders::_1));

    gimbal_obj.servo_out[0].init(&pwm_config[0], gimbal_obj.servo_roll_frequency, gimbal_obj.roll_pwm_max, gimbal_obj.roll_pwm_min, gimbal_obj.roll_start_pwm);
    gimbal_obj.servo_out[1].init(&pwm_config[1], gimbal_obj.servo_pitch_frequency, gimbal_obj.pitch_pwm_max, gimbal_obj.pitch_pwm_min, gimbal_obj.pitch_start_pwm);
    gimbal_obj.servo_out[2].init(&pwm_config[2], gimbal_obj.servo_yaw_frequency, gimbal_obj.yaw_pwm_max, gimbal_obj.yaw_pwm_min, gimbal_obj.yaw_start_pwm);
    gimbal_obj.servo_out[3].init(&pwm_config[3], gimbal_obj.servo_retract_frequency, gimbal_obj.retract_pwm_max, gimbal_obj.retract_pwm_min, gimbal_obj.retract_start_pwm);

    if (gimbal_obj.has_retract)
    {
        gimbal_obj.servo_out[0].writeUs(gimbal_obj.roll_start_pwm);
        gimbal_obj.servo_out[1].writeUs(gimbal_obj.pitch_start_pwm);
        gimbal_obj.servo_out[2].writeUs(gimbal_obj.yaw_start_pwm);
        delay(1000);
        gimbal_obj.servo_out[3].init(&pwm_config[3], gimbal_obj.servo_retract_frequency, gimbal_obj.retract_pwm_max, gimbal_obj.retract_pwm_min, gimbal_obj.retract_up_pwm);
        gimbal_obj.retract_gimbal();
    }
    else
    {
        gimbal_obj.servo_out[0].writeUs(gimbal_obj.roll_start_pwm);
        gimbal_obj.servo_out[1].writeUs(gimbal_obj.pitch_start_pwm);
        gimbal_obj.servo_out[2].writeUs(gimbal_obj.yaw_start_pwm);
    }

    while(1)
    {
        if ((millis() - gimbal_obj.time_of_last_message) > 100000)
        {
            if (gimbal_obj.first_retract)
                gimbal_obj.retract_time = millis();
            if (gimbal_obj.first_extend)
                gimbal_obj.extend_time = millis();
            gimbal_obj.retract_gimbal();
            gimbal_obj.is_retracted = true;
            gimbal_obj.first_retract = false;
            gimbal_obj.first_extend = true;
        }

        gimbal_obj.update_command();
        while(gimbal_obj.vcp.rx_bytes_waiting())
        {
            gimbal_obj.time_of_last_message = millis();
            uint8_t byte = gimbal_obj.vcp.read_byte();
            gimbal_obj.rx_callback(byte);
        }
        gimbal_obj.blink_heartbeat();
    }
}
