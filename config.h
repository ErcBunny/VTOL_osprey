#define DEBUG

#define TILT_SERVO 9

#define CH440_ENA 12
#define CH440_SWITCH 13

#define COPTER 0
#define PLANE 1
#define COPTER_TO_PLANE 0
#define PLANE_TO_COPTER 1
#define NONE 2
#define TIME_CONFIRM_TRANS 1500
#define TILT_MAX 80
#define TILT_MIN 0
#define TILT_STEP 1

#define SBUS_PERIOD 14
#define SBUS_MIN 306
#define SBUS_MAX 1694
#define DATA_MIN 1000
#define DATA_MAX 2000
#define CHANNEL_MODEL 8
#define CHANNEL_APM_MODE 9
#define CHANNEL_DEBUG 10

#define THROTTLE_HOVER 0

#define SERIAL_MAVLINK Serial3
#define RATE_MSG 25
const int sysid = 255;
const int compid = 190;
const int type = MAV_TYPE_GCS;
const uint8_t system_type = MAV_TYPE_GENERIC;
const uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
const uint8_t system_mode = MAV_MODE_TEST_ARMED;
const uint32_t custom_mode = MAV_MODE_FLAG_SAFETY_ARMED;
const uint8_t system_state = MAV_STATE_STANDBY;
