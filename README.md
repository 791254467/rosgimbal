# ROSgimbal

This firmware is based on superjax's airbourne f4 library. For more examples and code, go to: https://github.com/rosflight/airbourne_f4 .

## Project

The purpose of this project is to create a gimbal that is tightly coupled with ROS allowing the user to easily push commands to a gimbal controller at a high rate and receive information back.

## Hardware

The gimbal control board is an OpenPilot CC3D Revolution flight controller. This code should work with other STM32F4 board, but there may be some changes in the makefile required for it to work correctly.

At this point in time, this gimbal controller will only control servos for the actuators. Currently, there is no feedback from the servo positions, making this open loop control. High end servos such as Align brushless servos are capable of receiving commands at high rates up to 330 Hz. Other standard servos, however, are typically limited to around 50 Hz. Do not exceed the rate capability for your servos as you will burn up your servo motors.

The STLink V2 debugger is extremely useful in debugging the code when you make changes.

## Interface

This firmware makes use of superjax's VCP library. This requires that you send and receive over the micro USB port on the board. The VCP allows for a very fast rate of communication, but the rate is typically sent at 100 hz. The baudrate is 115200. The commands are sent and received through the gimbal serializer via USB. The gimbal serializer is able to subscribe to ROS topics, serialize them and send them over the VCP.

## Parameters

Currently, the best method to set parameters is to just hard code them in. To find the maximum and minimum pwm values you can command for your servos with the hardware limitations (servo limitation and gimbal movement limitation) and then input those values into the pwm min and max values in the header file. Use the servo tester and an arduino found here: https://github.com/dallinbriggs/servo_tester . When testing, use a protractor or paper cutout of one to measure the angles that it can reach. Finally, set the direction and the offset to match your coordinate system.

A future revision will use mavlink to set the parameters. As for now, this was quick and dirty (it's horrible I know, but whatever). 
