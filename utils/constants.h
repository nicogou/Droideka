#define DEBUG_BOARD_BAUD_RATE 115200

#define MOTOR_NB 12
#define SERVO_DEG_RATIO 0.01
#define LEG_NB 4
#define TIBIA_LENGTH 7
#define HIP_LENGTH 7
#define BODY_LENGTH 26.1 // TODO: verifier la valeur
#define BODY_WIDTH 18.0  // TODO: verifier la valeur
#define TIME_SAMPLE 4
#define MAX_LONGITUDINAL_COG_MOVE BODY_LENGTH / 10
#define MAX_LATERAL_COG_MOVE BODY_WIDTH / 10
#define MAX_ANGLE_COG_MOVE 1

#define Y_TOUCHING -11.0
#define Y_NOT_TOUCHING -6.0
#define Y_ZERO -7.0    // A vérifier
#define THETA_IDLE 0.0 //45.0      // A vérifier
#define X_IDLE 6.0     // A vérifier

#define Y_MAINTENANCE 0.0     // Maintenance
#define THETA_MAINTENANCE 0.0 // Maintenance
#define X_MAINTENANCE 14.0    // Maintenance

#define THETA_PARKING 90.0
#define X_PARKING 9.8
#define Y_PARKING 9.0

#define SERVO_BUS_WRITE_PIN 13