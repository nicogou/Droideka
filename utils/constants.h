#define DEBUG_BOARD_BAUD_RATE 115200 // Baud rate used to talk with the servos.

#define MOTOR_NB 12          // Number of leg motors
#define MOTOR_LONG_NB 13     // Total number of motors (legs + longitudinal)
#define SERVO_DEG_RATIO 0.01 // Ratio between degrees and encoder counts. degrees = SERVO_DEG_RATIO * encoder counts.
#define LEG_NB 4             // Number of legs
#define TIBIA_LENGTH 8       // Length of the tibia : from the tip of the foot to knee axis.
#define HIP_LENGTH 8         // Length of the hip : from knee axis to hip axis.
#define BODY_LENGTH 11.0     // Distance between the front and back legs (from one vertical axis to the other)
#define BODY_WIDTH 11.0      // Distance between the left and right legs (from one vertical axis to the other)
#define TIME_SAMPLE 98       // Number of time sample used in a Droideka_Movement.
// #define MAX_LONGITUDINAL_COG_MOVE BODY_LENGTH / 10
// #define MAX_LATERAL_COG_MOVE BODY_WIDTH / 10
// #define MAX_ANGLE_COG_MOVE 1

#define Y_TOUCHING -10.0    // Z distance when the robot is standing on its legs.
#define Y_NOT_TOUCHING -6.0 // Z distance when the legs are not touching the ground anymore.
#define Y_ZERO -7.0         // Z distance at the limit between touching and not touching.
#define THETA_IDLE 45.0     // Angle value when the robot is unparked.
#define X_IDLE 6.5          // Distance between foot and vertical hip axis when unparked.

#define Y_MAINTENANCE 0.0                       // Maintenance position
#define THETA_MAINTENANCE 0.0                   // Maintenance position
#define X_MAINTENANCE TIBIA_LENGTH + HIP_LENGTH // Maintenance position

#define THETA_PARKING 90.0 // Parking position
#define X_PARKING 2.92     // Parking position
#define Y_PARKING 13.17    // Parking position

#define SERVO_BUS_WRITE_PIN 13 // Pin used to send instructions to the servos.

#define SERVOS_UNDER_VOLTAGE_LIMIT 7500    // Servos measure their voltage (in mV). When it is under this value, we don't allow moves as it could discharge the batteries.
#define VOLTAGE_CHECK_TIMER 120000         // Under normal voltage, do a voltage check every VOLTAGE_CHECK_TIMER ms.
#define VOLTAGE_CHECK_TIMER_HIGH_FREQ 1000 // Under abnormal voltage, do a voltage check every VOLTAGE_CHECK_TIMER_HIGH_FREQ ms.

#define LED_NB 3     // Number of leds
#define BLUE_LED 13  // Pin of the blue LED
#define GREEN_LED 14 // Pin of the blue LED
#define RED_LED 15   // Pin of the blue LED

// #define int_1 30
// #define int_2 31
// #define int_3 32
// #define pot_1 39
// #define pot_2 40
// #define pot_3 41
#define LONG_MOTOR_DEAD_ZONE 20 // Between [-LONG_MOTOR_DEAD_ZONE, LONG_MOTOR_DEAD_ZONE], the longitudinal motor can't make the robot move.
#define PID_SAMPLE_TIME 10      // Pretty self explanatory. Computes the PID every 10 ms.

// IMU offsets. TO DO.
#define X_GYRO_OFFSET 0
#define Y_GYRO_OFFSET 0
#define Z_GYRO_OFFSET 0
#define X_ACCEL_OFFSET 0
#define Y_ACCEL_OFFSET 0
#define Z_ACCEL_OFFSET 0