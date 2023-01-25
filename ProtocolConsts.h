// #define NONE_TYPE 0x00
// #define ERR_TYPE 0xFE
// #define NOT_MY_PACKET_TYPE 0xAA
// #define POLL_MSG_TYPE 0x10
// #define RESP_MSG_TYPE 0x21
// #define FINAL_MSG_TYPE 0x22

// #define ACK_MSG_TYPE 0x23

//#define DIST_EST_MSG_TYPE 0x24
//#define SYNC_MSG_TYPE 0x25
//#define TWR_ALL_TYPE 0x01
//#define TWR_SPECIFIC_TYPE 0x02
//#define TWR_SOME_ALL_TYPE 0x08
//#define TWR_SOME_SOME_TYPE 0x09
//#define TWR_ALL_CASCADE_TYPE 0x0A
//#define NODES_AVAILABLE 0x04
//#define ALL_REPORT_TWR_TYPE 0x03
//#define TWR_REPORT_FROM_NODE 0x06
//#define NODES_COUNT_TYPE 0x07
//#define ALL_QUAL_REPORT_TYPE 0xB
//#define QUAL_REPORT_FROM_NODE 0xC
//FINAL_MSG_RESP_RX_TS_IDX
#define BROADCAST_ID 0xFF

#define SRC_IDX 0
#define SEQ_IDX 1
#define LED_IDX 11

#define POLL_MSG_RAND_BOUND_IDX 12

#define ANY_MSG_TS_LEN 5
#define SHORT_MSG_LEN 2

typedef enum send_modes{SEND_IMMEDIATE, SEND_DELAY_FIXED, SEND_DELAY_BOARDID, SEND_SHORT_DELAY_BOARDID, SEND_LONG_DELAY_BOARDID, SEND_DELAY_GIVEN, SEND_DELAY_RANDOM} SEND_MODES;


#define RESP_DURATION 2000


#define INFINITE_TIME 0xFFFFFFFFFFFFFFFF


#define MAX_POLL_LEN 12
#define MAX_RESP_LEN 8
#define MAX_FINAL_LEN 44
#define MAX_REPORT_LEN 10
// (56 if imu not included)
#define PROBE_LEN 125//125    //4 byte info + 5 byte transmission time + 6*TRXNUM + (5,8,10)*9*2 [IMU] + 2

#define TRX_NUM 6
#define ID_TS_LEN 6
#define TS_LEN 5
#define ONE_NODE_TIMESLOT 40000
#define PROBE_TX_TS_IDX 3
#define TYPICAL_TIMEOUT 60000
#define RX_TIMEOUT_AFTER_SEND 20000
#define TYPICAL_RX_TIMEOUT 60000
#define FIXED_DELAY 3000
#define DIST_LEN 2
#define IMU_LEN 2

#define DEBUG_FLAG 0



#if(DEBUG_FLAG == 1)
    // #define FIXED_DELAY 3000
    // #define TIME_SLOT_LEN 5000
    // #define MAXTIME_RESP_EXPECT 30000
    // #define MAXTIME_FINAL_EXPECT 50000
    // #define RANGING_TIMER 60000
#else
    // #define FIXED_DELAY 3000
    // #define TIME_SLOT_LEN 3000
    // #define MAXTIME_RESP_EXPECT 20000
    // #define MAXTIME_FINAL_EXPECT 28000
    // #define RANGING_TIMER 30000
#endif



#define TIME_UNIT 1/128/499.2e6

#define IMU_MODE_HIGHRATE 0  // Each packet has 10 IMUs (about 166 hZ)
#define IMU_MODE_MIDRATE 1   // Each packet has 8 IMUs (about 133.3Hz)
#define IMU_MODE_LOWRATE 2   // Each packet has 5 IMUs (about 83Hz)

#define IMU_MODE IMU_MODE_LOWRATE
#define SINGLE_IMU_BUF_LEN 12
#define RX_POWER_IDX 118

#define OUR_UWB_FEATHER 1
#define AUS_UWB_FEATHER 0

const bool use_imu  = true;
const int imu_rate = 80;
double imu_duration_ms = 1000.0 / imu_rate;

const int myid = 6;
