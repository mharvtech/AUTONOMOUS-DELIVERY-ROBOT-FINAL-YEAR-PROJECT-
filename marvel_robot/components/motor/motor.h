#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Initializes GPIOs and LEDC PWM channels for the L298N driver.
void motor_init(void);

// Sets wheel speeds in ROBOT coordinates, -255..255, where positive ALWAYS
// means the robot moves toward its ultrasonic sensor (its front), regardless
// of how the motors happen to be wired.
//
// Wiring quirks are absorbed by the orientation flags at the top of motor.c.
// Callers must never apply a sign-correction constant of their own - that is
// how the first version ended up driving forward in manual mode and backward
// in autonomous mode.
void motor_set(int left_speed, int right_speed);

// Cuts power; the robot coasts to a stop.
void motor_stop(void);

// Active braking - shorts each motor's terminals through the H-bridge for a
// much faster stop than motor_stop(). For e-stops and close obstacles, not
// for routine stopping.
void motor_brake(void);

#ifdef __cplusplus
}
#endif
