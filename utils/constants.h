#define DEBUG_BOARD_BAUD_RATE 115200

#define MOTOR_NB 12
#define SERVO_DEG_RATIO 0.01
#define LEG_NB 4
#define TIBIA_LENGTH 8
#define HIP_LENGTH 8
#define BODY_LENGTH 11.0 // TODO: verifier la valeur
#define BODY_WIDTH 11.0  // TODO: verifier la valeur
#define TIME_SAMPLE 88
#define MAX_LONGITUDINAL_COG_MOVE BODY_LENGTH / 10
#define MAX_LATERAL_COG_MOVE BODY_WIDTH / 10
#define MAX_ANGLE_COG_MOVE 1

#define Y_TOUCHING -10.0
#define Y_NOT_TOUCHING -6.0
#define Y_ZERO -7.0     // A vérifier
#define THETA_IDLE 45.0 //0.0 // A vérifier
#define X_IDLE 6.5      //5.0     // A vérifier

#define Y_MAINTENANCE 0.0                       // Maintenance
#define THETA_MAINTENANCE 0.0                   // Maintenance
#define X_MAINTENANCE TIBIA_LENGTH + HIP_LENGTH // Maintenance

#define THETA_PARKING 90.0
#define X_PARKING 2.92
#define Y_PARKING 13.17

#define SERVO_BUS_WRITE_PIN 13

#define SERVOS_UNDER_VOLTAGE_LIMIT 7500
#define VOLTAGE_CHECK_TIMER 120000
#define VOLTAGE_CHECK_TIMER_HIGH_FREQ 1000

#define LED_NB 3
#define BLUE_LED 13
#define GREEN_LED 14
#define RED_LED 15

#define int_1 30
#define int_2 31
#define int_3 32
#define pot_1 39
#define pot_2 40
#define pot_3 41
#define LONG_MOTOR_DEAD_ZONE 50

#define X_GYRO_OFFSET 0
#define Y_GYRO_OFFSET 0
#define Z_GYRO_OFFSET 0
#define X_ACCEL_OFFSET 0
#define Y_ACCEL_OFFSET 0
#define Z_ACCEL_OFFSET 0