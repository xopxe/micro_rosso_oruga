#ifndef __mobility_skid_config_h
#define __mobility_skid_config_h

#define STOP_TIMEOUT_MS 2000

#define USE_FULL_QUADRATURE  // if undefined, will use single edge

// robot specs from https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/rover-kits/a4wd3-tracked/
static const float MOTOR_MAX_RPM = 170.0; // motor's max RPM
static const float MAX_RPM_RATIO = 0.85;  // max RPM allowed (under load reported as 145rpm)
static const float MOTOR_REDUCTION = 51;
static const float MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO;
static const unsigned int ENCODER_PPR = 12;                        // encoder resolution https://www.sameskydevices.com/blog/what-is-encoder-ppr-cpr-and-lpr
static const float WHEEL_DIAMETER = 0.202;                         // wheel's diameter in meters
#if defined(USE_FULL_QUADRATURE)
static const float COUNTS_PER_REV = ENCODER_PPR * MOTOR_REDUCTION * 4; //quadrature counts per wheel rev - 2448
static const float TICKS_TO_RAD = 2 * PI / COUNTS_PER_REV;
#elif
static const float PULSES_PER_REV = ENCODER_PPR * MOTOR_REDUCTION; // encoder ticks per wheel rev - 312
static const float TICKS_TO_RAD = 2 * PI / PULSES_PER_REV;
#endif
static const float WHEEL_RADIUS = WHEEL_DIAMETER / 2;
static const float LR_WHEELS_DISTANCE = (0.4515 + 0.2915) / 2; // distance between left and right wheels
static const float LR_WHEELS_OFFSET = 0.5*LR_WHEELS_DISTANCE; // distance between center and wheel base



// compute dynamics from motors and geometry
static const float MAX_WHEEL_ANGULAR = MAX_RPM_ALLOWED * 2 * PI / 60;        // rad/s
static const float MAX_SPEED = MAX_WHEEL_ANGULAR * WHEEL_RADIUS;             // ~1.5   (datasheet says ~1.8m/s)
static const float MAX_TURNSPEED = atan(2 * MAX_SPEED / LR_WHEELS_DISTANCE); // rad/s


// Pid for linear velocity, from speed in m/s to power level (-126..126)
static const float PID_LEFT_KF = 126 / MAX_SPEED;     // feed forward control
static const float PID_LEFT_KP = 0.0 * PID_LEFT_KF; // P control
static const float PID_LEFT_KI = 0.0000001;                // I control
static const float PID_LEFT_KD = 0.0;                 // D control

static const float PID_RIGHT_KF = 126 / MAX_SPEED; // feed forward control
static const float PID_RIGHT_KP = 0.0 * PID_RIGHT_KF;   // P control
static const float PID_RIGHT_KI = 0.0000001;                // I control
static const float PID_RIGHT_KD = 0.0;                 // D control


// Wiring pins
#define SABERTOOTH_SERIAL Serial2
#define SABERTOOTH_TX_PIN 27

#define ENCODER_fr_lft_PIN_A 37
#define ENCODER_fr_lft_PIN_B 38
#define ENCODER_fr_rgt_PIN_A 23
#define ENCODER_fr_rgt_PIN_B 19
#define ENCODER_rr_lft_PIN_A 25
#define ENCODER_rr_lft_PIN_B 26
#define ENCODER_rr_rgt_PIN_A 36
#define ENCODER_rr_rgt_PIN_B 39

// Encoder multipliers (for changing direction, for example)
static const int8_t ENCODER_lft_MULT = -1;
static const int8_t ENCODER_rgt_MULT = 1;

// Motor output multipliers (for changing direction, for example)
static const int8_t MOTOR_OUT_lft_MULT = 1; // weird i know
static const int8_t MOTOR_OUT_rgt_MULT = 1;

#endif // __mobility_skid_config_h
