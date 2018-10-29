## Synopsis

The `teleop` sketch interfaces with two Roboclaw controllers and a `rosserial` bridge over USB.  It subscribes to the cmd_vel topic which contains the robot's target twist and then converts these msgs to left/right wheel spin speeds.  The roboclaw's PID controllers obtain these desired speeds with the input of encoders.

`tune_pid` is used to tune (using the arduino serial moniter) the pids of the roboclaws.  To tune PID, the phidgets board will need to actually be collecting encoder data(even though you don't actually use this data) otherwise the encoders will not be powered and the roboclaws will not get an encoder signal.

## Motivation

The Roboclaw motor controller has a very powerful serial control library, but serial port options are limited when dealing with full Linux-based computers like our Jetson or NUC. Instead, we can use an Arduino as a serial bridge, parsing a message sent through `rosserial` and then passing that along to the Sabertooth motor controllers.

## Installation

Uses an arduino mega.  Need to have ros and Roboclaw arduino libarys to flash to the arduino. Serial1 is used for communication with RoboClaw controllers.  Roboclaw controllers must be wired in multi mode as per the roboclaw user manual.  roboclaw0 and roboclaw1 control the left/right sides of the robot respectively.  
