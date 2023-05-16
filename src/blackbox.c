#include "board.h"

#ifdef BLACKBOX
#include "mw.h"
#include "buzzer.h"

#include "blackbox_fielddefs.h"
#include "blackbox.h"

#define BLACKBOX_BAUDRATE 115200
#define BLACKBOX_INITIAL_PORT_MODE MODE_TX
#define BLACKBOX_I_INTERVAL 1 // 32

#define ARRAY_LENGTH(x) (sizeof((x)) / sizeof((x)[0]))

// Some macros to make writing FLIGHT_LOG_FIELD_* constants shorter:
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define CONCAT_HELPER(x, y) x##y
#define CONCAT(x, y) CONCAT_HELPER(x, y)

#define PREDICT(x) CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x)
#define ENCODING(x) CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x)
#define CONDITION(x) CONCAT(FLIGHT_LOG_FIELD_CONDITION_, x)
#define UNSIGNED FLIGHT_LOG_FIELD_UNSIGNED
#define SIGNED FLIGHT_LOG_FIELD_SIGNED

/*
 * Translate variable names from Cleanflight where possible to reduce the diff between editions.
 * Since it seems this will never be merged, there's no reason to maintain Baseflight style.
 */
#define motorCount 4 // numberMotor
#define masterConfig mcfg

static const char blackboxHeader[] =
    "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
    "H Blackbox version:1\n"
    "H Data version:2\n"
    "H I interval:" STR(BLACKBOX_I_INTERVAL) "\n";

static const char *const blackboxMainHeaderNames[] = {
    "I name",
    "I signed",
    "I predictor",
    "I encoding",
    "P predictor",
    "P encoding"};

#ifdef GPS
static const char *const blackboxGPSGHeaderNames[] = {
    "G name",
    "G signed",
    "G predictor",
    "G encoding"};

static const char *const blackboxGPSHHeaderNames[] = {
    "H name",
    "H signed",
    "H predictor",
    "H encoding"};
#endif

/* All field definition structs should look like this (but with longer arrs): */
typedef struct blackboxFieldDefinition_t
{
    const char *name;
    uint8_t arr[1];
} blackboxFieldDefinition_t;

typedef struct blackboxMainFieldDefinition_t
{
    const char *name;
    uint8_t isSigned;
    uint8_t Ipredict;
    uint8_t Iencode;
    uint8_t Ppredict;
    uint8_t Pencode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxMainFieldDefinition_t;

typedef struct blackboxGPSFieldDefinition_t
{
    const char *name;
    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
} blackboxGPSFieldDefinition_t;

/**
 * Description of the blackbox fields we are writing in our main intra (I) and inter (P) frames. This description is
 * written into the flight log header so the log can be properly interpreted (but these definitions don't actually cause
 * the encoding to happen, we have to encode the flight log ourselves in write{Inter|Intra}frame() in a way that matches
 * the encoding we've promised here).
 */
/*static const blackboxMainFieldDefinition_t blackboxMainFields[] = {
    {"loopIteration", UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(INC),           .Pencode = FLIGHT_LOG_FIELD_ENCODING_NULL, CONDITION(ALWAYS)},
    {"time",          UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(STRAIGHT_LINE), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP[0]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP[1]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP[2]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisI[0]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
    {"axisI[1]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
    {"axisI[2]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
    {"axisD[0]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_0)},
    {"axisD[1]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_1)},
    {"axisD[2]",      SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_2)},
    {"rcCommand[0]",  SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
    {"rcCommand[1]",  SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
    {"rcCommand[2]",  SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
    {"rcCommand[3]",  UNSIGNED, .Ipredict = PREDICT(MINTHROTTLE), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},

    {"vbatLatest",    UNSIGNED, .Ipredict = PREDICT(VBATREF), .Iencode = ENCODING(NEG_14BIT),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_VBAT},
#ifdef MAG
    {"magADC[0]",     SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
    {"magADC[1]",     SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
    {"magADC[2]",     SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
#endif
#ifdef BARO
    {"BaroAlt",       SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_BARO},
#endif

    {"gyroData[0]",   SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"gyroData[1]",   SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"gyroData[2]",   SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth[0]",  SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth[1]",  SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth[2]",  SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"motor[0]",      UNSIGNED, .Ipredict = PREDICT(MINTHROTTLE), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_1)},
    {"motor[1]",      UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_2)},
    {"motor[2]",      UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_3)},
    {"motor[3]",      UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_4)},
    {"motor[4]",      UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_5)},
    {"motor[5]",      UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_6)},
    {"motor[6]",      UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_7)},
    {"motor[7]",      UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_8)},
    {"servo[5]",      UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(TRICOPTER)}
};*/

static const blackboxMainFieldDefinition_t blackboxMainFields[] = {
    {"loopIteration", UNSIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(INC), .Pencode = FLIGHT_LOG_FIELD_ENCODING_NULL, CONDITION(ALWAYS)},
    {"time", UNSIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(STRAIGHT_LINE), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"auxState_log", UNSIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"flight_task", UNSIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"ref_roll", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"ref_pitch", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"ref_yaw", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"q_roll", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"q_pitch", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"q_yaw", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"rc_thrust", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
    {"vbat", UNSIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), CONDITION(ALWAYS)},
    {"motor[0]", UNSIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"motor[1]", UNSIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"motor[2]", UNSIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"motor[3]", UNSIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"motor[4]", UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_5)},
    {"motor[5]", UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_6)},
    {"motor[6]", UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_7)},
    {"motor[7]", UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_8)},
    {"r_x", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"r_y", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"r_z", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"wifiloopcount", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"thrust_cmd", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"roll_cmd", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"pitch_cmd", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"yaw_cmd", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"body_vx_datalog", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"cx_datalog", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"cy_datalog", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"cz_datalog", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"cqw", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"cqx", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"cqy", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"cqz", SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)}
    };

#ifdef GPS
// GPS position/vel frame
static const blackboxGPSFieldDefinition_t blackboxGpsGFields[] = {
    {"GPS_numSat", UNSIGNED, PREDICT(0), ENCODING(UNSIGNED_VB)},
    {"GPS_coord[0]", SIGNED, PREDICT(HOME_COORD), ENCODING(SIGNED_VB)},
    {"GPS_coord[1]", SIGNED, PREDICT(HOME_COORD), ENCODING(SIGNED_VB)},
    {"GPS_altitude", UNSIGNED, PREDICT(0), ENCODING(UNSIGNED_VB)},
    {"GPS_speed", UNSIGNED, PREDICT(0), ENCODING(UNSIGNED_VB)},
    {"GPS_ground_course", UNSIGNED, PREDICT(0), ENCODING(UNSIGNED_VB)}};

// GPS home frame
static const blackboxGPSFieldDefinition_t blackboxGpsHFields[] = {
    {"GPS_home[0]", SIGNED, PREDICT(0), ENCODING(SIGNED_VB)},
    {"GPS_home[1]", SIGNED, PREDICT(0), ENCODING(SIGNED_VB)}};
#endif

typedef enum BlackboxState
{
    BLACKBOX_STATE_DISABLED = 0,
    BLACKBOX_STATE_STOPPED,
    BLACKBOX_STATE_SEND_HEADER,
    BLACKBOX_STATE_SEND_FIELDINFO,
    BLACKBOX_STATE_SEND_GPS_H_HEADERS,
    BLACKBOX_STATE_SEND_GPS_G_HEADERS,
    BLACKBOX_STATE_SEND_SYSINFO,
    BLACKBOX_STATE_PRERUN,
    BLACKBOX_STATE_RUNNING
} BlackboxState;

typedef struct gpsState_t
{
    int32_t GPS_home[2], GPS_coord[2];
    uint8_t GPS_numSat;
} gpsState_t;

// From mixer.c:
// extern uint8_t motorCount;

// How many bytes should we transmit per loop iteration?
static uint8_t serialChunkSize = 16;

static BlackboxState blackboxState = BLACKBOX_STATE_DISABLED;

static struct
{
    uint32_t headerIndex;

    /* Since these fields are used during different blackbox states (never simultaneously) we can
     * overlap them to save on RAM
     */
    union
    {
        int fieldIndex;
        int serialBudget;
        uint32_t startTime;
    } u;
} xmitState;

static uint32_t blackboxConditionCache;

static uint32_t blackboxIteration;
static uint32_t blackboxPFrameIndex, blackboxIFrameIndex;

static serialPort_t *blackboxPort;

/*
 * We store voltages in I-frames relative to this, which was the voltage when the blackbox was activated.
 * This helps out since the voltage is only expected to fall from that point and we can reduce our diffs
 * to encode:
 */
static uint16_t vbatReference;
static gpsState_t gpsHistory;

// Keep a history of length 2, plus a buffer for MW to store the new values into
static blackboxValues_t blackboxHistoryRing[3];

// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
static blackboxValues_t *blackboxHistory[3];

static void blackboxWrite(uint8_t value)
{
    serialWrite(blackboxPort, value);
}

static void _putc(void *p, char c)
{
    (void)p;
    serialWrite(blackboxPort, c);
}

// printf() to the blackbox serial port with no blocking shenanigans (so it's caller's responsibility to not write too fast!)
static void blackboxPrintf(char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    tfp_format(NULL, _putc, fmt, va);
    va_end(va);
}

// Print the null-terminated string 's' to the serial port and return the number of bytes written
static int blackboxPrint(const char *s)
{
    const char *pos = s;

    while (*pos)
    {
        serialWrite(blackboxPort, *pos);
        pos++;
    }

    return pos - s;
}

/**
 * Write an unsigned integer to the blackbox serial port using variable byte encoding.
 */
static void writeUnsignedVB(uint32_t value)
{
    // While this isn't the final byte (we can only write 7 bits at a time)
    while (value > 127)
    {
        blackboxWrite((uint8_t)(value | 0x80)); // Set the high bit to mean "more bytes follow"
        value >>= 7;
    }
    blackboxWrite(value);
}

/**
 * Write a signed integer to the blackbox serial port using ZigZig and variable byte encoding.
 */
static void writeSignedVB(int32_t value)
{
    // ZigZag encode to make the value always positive
    writeUnsignedVB((uint32_t)((value << 1) ^ (value >> 31)));
}

/**
 * Write a 2 bit tag followed by 3 signed fields of 2, 4, 6 or 32 bits
 */
static void writeTag2_3S32(int32_t *values)
{
    static const int NUM_FIELDS = 3;

    // Need to be enums rather than const ints if we want to switch on them (due to being C)
    enum
    {
        BITS_2 = 0,
        BITS_4 = 1,
        BITS_6 = 2,
        BITS_32 = 3
    };

    enum
    {
        BYTES_1 = 0,
        BYTES_2 = 1,
        BYTES_3 = 2,
        BYTES_4 = 3
    };

    int x;
    int selector = BITS_2, selector2;

    /*
     * Find out how many bits the largest value requires to encode, and use it to choose one of the packing schemes
     * below:
     *
     * Selector possibilities
     *
     * 2 bits per field  ss11 2233,
     * 4 bits per field  ss00 1111 2222 3333
     * 6 bits per field  ss11 1111 0022 2222 0033 3333
     * 32 bits per field sstt tttt followed by fields of various byte counts
     */
    for (x = 0; x < NUM_FIELDS; x++)
    {
        // Require more than 6 bits?
        if (values[x] >= 32 || values[x] < -32)
        {
            selector = BITS_32;
            break;
        }

        // Require more than 4 bits?
        if (values[x] >= 8 || values[x] < -8)
        {
            if (selector < BITS_6)
                selector = BITS_6;
        }
        else if (values[x] >= 2 || values[x] < -2)
        { // Require more than 2 bits?
            if (selector < BITS_4)
                selector = BITS_4;
        }
    }

    switch (selector)
    {
    case BITS_2:
        blackboxWrite((selector << 6) | ((values[0] & 0x03) << 4) | ((values[1] & 0x03) << 2) | (values[2] & 0x03));
        break;
    case BITS_4:
        blackboxWrite((selector << 6) | (values[0] & 0x0F));
        blackboxWrite((values[1] << 4) | (values[2] & 0x0F));
        break;
    case BITS_6:
        blackboxWrite((selector << 6) | (values[0] & 0x3F));
        blackboxWrite((uint8_t)values[1]);
        blackboxWrite((uint8_t)values[2]);
        break;
    case BITS_32:
        /*
         * Do another round to compute a selector for each field, assuming that they are at least 8 bits each
         *
         * Selector2 field possibilities
         * 0 - 8 bits
         * 1 - 16 bits
         * 2 - 24 bits
         * 3 - 32 bits
         */
        selector2 = 0;

        // Encode in reverse order so the first field is in the low bits:
        for (x = NUM_FIELDS - 1; x >= 0; x--)
        {
            selector2 <<= 2;

            if (values[x] < 128 && values[x] >= -128)
                selector2 |= BYTES_1;
            else if (values[x] < 32768 && values[x] >= -32768)
                selector2 |= BYTES_2;
            else if (values[x] < 8388608 && values[x] >= -8388608)
                selector2 |= BYTES_3;
            else
                selector2 |= BYTES_4;
        }

        // Write the selectors
        blackboxWrite((selector << 6) | selector2);

        // And now the values according to the selectors we picked for them
        for (x = 0; x < NUM_FIELDS; x++, selector2 >>= 2)
        {
            switch (selector2 & 0x03)
            {
            case BYTES_1:
                blackboxWrite(values[x]);
                break;
            case BYTES_2:
                blackboxWrite(values[x]);
                blackboxWrite(values[x] >> 8);
                break;
            case BYTES_3:
                blackboxWrite(values[x]);
                blackboxWrite(values[x] >> 8);
                blackboxWrite(values[x] >> 16);
                break;
            case BYTES_4:
                blackboxWrite(values[x]);
                blackboxWrite(values[x] >> 8);
                blackboxWrite(values[x] >> 16);
                blackboxWrite(values[x] >> 24);
                break;
            }
        }
        break;
    }
}

/**
 * Write an 8-bit selector followed by four signed fields of size 0, 4, 8 or 16 bits.
 */
static void writeTag8_4S16(int32_t *values)
{

    // Need to be enums rather than const ints if we want to switch on them (due to being C)
    enum
    {
        FIELD_ZERO = 0,
        FIELD_4BIT = 1,
        FIELD_8BIT = 2,
        FIELD_16BIT = 3
    };

    uint8_t selector, buffer;
    int nibbleIndex;
    int x;

    selector = 0;
    // Encode in reverse order so the first field is in the low bits:
    for (x = 3; x >= 0; x--)
    {
        selector <<= 2;

        if (values[x] == 0)
            selector |= FIELD_ZERO;
        else if (values[x] < 8 && values[x] >= -8)
            selector |= FIELD_4BIT;
        else if (values[x] < 128 && values[x] >= -128)
            selector |= FIELD_8BIT;
        else
            selector |= FIELD_16BIT;
    }

    blackboxWrite(selector);

    nibbleIndex = 0;
    buffer = 0;
    for (x = 0; x < 4; x++, selector >>= 2)
    {
        switch (selector & 0x03)
        {
        case FIELD_ZERO:
            // No-op
            break;
        case FIELD_4BIT:
            if (nibbleIndex == 0)
            {
                // We fill high-bits first
                buffer = values[x] << 4;
                nibbleIndex = 1;
            }
            else
            {
                blackboxWrite(buffer | (values[x] & 0x0F));
                nibbleIndex = 0;
            }
            break;
        case FIELD_8BIT:
            if (nibbleIndex == 0)
                blackboxWrite(values[x]);
            else
            {
                // Write the high bits of the value first (mask to avoid sign extension)
                blackboxWrite(buffer | ((values[x] >> 4) & 0x0F));
                // Now put the leftover low bits into the top of the next buffer entry
                buffer = values[x] << 4;
            }
            break;
        case FIELD_16BIT:
            if (nibbleIndex == 0)
            {
                // Write high byte first
                blackboxWrite(values[x] >> 8);
                blackboxWrite(values[x]);
            }
            else
            {
                // First write the highest 4 bits
                blackboxWrite(buffer | ((values[x] >> 12) & 0x0F));
                // Then the middle 8
                blackboxWrite(values[x] >> 4);
                // Only the smallest 4 bits are still left to write
                buffer = values[x] << 4;
            }
            break;
        }
    }
    // Anything left over to write?
    if (nibbleIndex == 1)
        blackboxWrite(buffer);
}

/**
 * Write `valueCount` fields from `values` to the Blackbox using signed variable byte encoding. A 1-byte header is
 * written first which specifies which fields are non-zero (so this encoding is compact when most fields are zero).
 *
 * valueCount must be 8 or less.
 */
static void writeTag8_8SVB(int32_t *values, int valueCount)
{
    uint8_t header;
    int i;

    if (valueCount > 0)
    {
        // If we're only writing one field then we can skip the header
        if (valueCount == 1)
        {
            writeSignedVB(values[0]);
        }
        else
        {
            // First write a one-byte header that marks which fields are non-zero
            header = 0;

            // First field should be in low bits of header
            for (i = valueCount - 1; i >= 0; i--)
            {
                header <<= 1;

                if (values[i] != 0)
                    header |= 0x01;
            }

            blackboxWrite(header);

            for (i = 0; i < valueCount; i++)
                if (values[i] != 0)
                    writeSignedVB(values[i]);
        }
    }
}

static bool testBlackboxConditionUncached(FlightLogFieldCondition condition)
{
    switch (condition)
    {
    case FLIGHT_LOG_FIELD_CONDITION_ALWAYS:
        return true;

    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1:
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_2:
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_3:
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_4:
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_5:
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_6:
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_7:
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_8:
        return motorCount >= condition - FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1 + 1;

    case FLIGHT_LOG_FIELD_CONDITION_TRICOPTER:
        return masterConfig.mixerConfiguration == MULTITYPE_TRI;

    case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0:
    case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1:
    case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2:
        return cfg.D8[condition - FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0] != 0;

    case FLIGHT_LOG_FIELD_CONDITION_MAG:
#ifdef MAG
        return sensors(SENSOR_MAG);
#else
        return false;
#endif

    case FLIGHT_LOG_FIELD_CONDITION_BARO:
#ifdef BARO
        return sensors(SENSOR_BARO);
#else
        return false;
#endif

    case FLIGHT_LOG_FIELD_CONDITION_VBAT:
        return feature(FEATURE_VBAT);

    case FLIGHT_LOG_FIELD_CONDITION_NEVER:
        return false;
    default:
        return false;
    }
}

static void blackboxBuildConditionCache()
{
    FlightLogFieldCondition cond;

    blackboxConditionCache = 0;

    for (cond = FLIGHT_LOG_FIELD_CONDITION_FIRST; cond <= FLIGHT_LOG_FIELD_CONDITION_LAST; cond++)
    {
        if (testBlackboxConditionUncached(cond))
            blackboxConditionCache |= 1 << cond;
    }
}

static bool testBlackboxCondition(FlightLogFieldCondition condition)
{
    return (blackboxConditionCache & (1 << condition)) != 0;
}

static void blackboxSetState(BlackboxState newState)
{
    // Perform initial setup required for the new state
    switch (newState)
    {
    case BLACKBOX_STATE_SEND_HEADER:
        xmitState.headerIndex = 0;
        xmitState.u.startTime = millis();
        break;
    case BLACKBOX_STATE_SEND_FIELDINFO:
    case BLACKBOX_STATE_SEND_GPS_G_HEADERS:
    case BLACKBOX_STATE_SEND_GPS_H_HEADERS:
        xmitState.headerIndex = 0;
        xmitState.u.fieldIndex = -1;
        break;
    case BLACKBOX_STATE_SEND_SYSINFO:
        xmitState.headerIndex = 0;
        break;
    case BLACKBOX_STATE_RUNNING:
        blackboxIteration = 0;
        blackboxPFrameIndex = 0;
        blackboxIFrameIndex = 0;
        break;
    default:;
    }
    blackboxState = newState;
}

static void writeIntraframe(void)
{
    blackboxValues_t *blackboxCurrent = blackboxHistory[0];
    int x;

    blackboxWrite('I');

    writeUnsignedVB(blackboxIteration);
    writeUnsignedVB(blackboxCurrent->time);
    writeUnsignedVB(blackboxCurrent->auxState_log);
    writeUnsignedVB(blackboxCurrent->flight_task);

    writeSignedVB(blackboxCurrent->ref_roll);
    writeSignedVB(blackboxCurrent->ref_pitch);
    writeSignedVB(blackboxCurrent->ref_yaw);
    writeSignedVB(blackboxCurrent->q_roll);
    writeSignedVB(blackboxCurrent->q_pitch);
    writeSignedVB(blackboxCurrent->q_yaw);

    //for (x = 0; x < 4; x++)
    //    writeSignedVB(blackboxCurrent->rcCommand[x]);
    writeSignedVB(blackboxCurrent->rc_thrust);
    writeUnsignedVB(blackboxCurrent->vbat);

    for (x = 0; x < 4; x++)
        writeUnsignedVB(blackboxCurrent->motor[x]);

    // for (x = 0; x < XYZ_AXIS_COUNT; x++)
    //     writeSignedVB(blackboxCurrent->accSmooth[x]);

    writeSignedVB(blackboxCurrent->r_x);
    writeSignedVB(blackboxCurrent->r_y);
    writeSignedVB(blackboxCurrent->r_z);
    writeSignedVB(blackboxCurrent->wifiloopcount);
    writeSignedVB(blackboxCurrent->thrust_cmd);
    writeSignedVB(blackboxCurrent->roll_cmd);
    writeSignedVB(blackboxCurrent->pitch_cmd);
    writeSignedVB(blackboxCurrent->yaw_cmd);
    writeSignedVB(blackboxCurrent->body_vx_datalog);
    writeSignedVB(blackboxCurrent->cx_datalog);
    writeSignedVB(blackboxCurrent->cy_datalog);
    writeSignedVB(blackboxCurrent->cz_datalog);
    writeSignedVB(blackboxCurrent->cqw);
    writeSignedVB(blackboxCurrent->cqx);
    writeSignedVB(blackboxCurrent->cqy);
    writeSignedVB(blackboxCurrent->cqz);

    // Rotate our history buffers:

    // The current state becomes the new "before" state
    blackboxHistory[1] = blackboxHistory[0];
    // And since we have no other history, we also use it for the "before, before" state
    blackboxHistory[2] = blackboxHistory[0];
    // And advance the current state over to a blank space ready to be filled
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}

/*static void writeIntraframe(void)
{
    blackboxValues_t *blackboxCurrent = blackboxHistory[0];
    int x;

    blackboxWrite('I');

    writeUnsignedVB(blackboxIteration);
    writeUnsignedVB(blackboxCurrent->time);

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        writeSignedVB(blackboxCurrent->axisPID_P[x]);

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        writeSignedVB(blackboxCurrent->axisPID_I[x]);

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x))
            writeSignedVB(blackboxCurrent->axisPID_D[x]);

    for (x = 0; x < 3; x++)
        writeSignedVB(blackboxCurrent->rcCommand[x]);

    writeUnsignedVB(blackboxCurrent->rcCommand[3] - masterConfig.minthrottle); //Throttle lies in range [minthrottle..maxthrottle]

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {

        writeUnsignedVB((vbatReference - blackboxCurrent->vbatLatest) & 0x3FFF);
    }

#ifdef MAG
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG)) {
            for (x = 0; x < XYZ_AXIS_COUNT; x++)
                writeSignedVB(blackboxCurrent->magADC[x]);
        }
#endif

#ifdef BARO
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO))
            writeSignedVB(blackboxCurrent->BaroAlt);
#endif

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        writeSignedVB(blackboxCurrent->gyroData[x]);

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        writeSignedVB(blackboxCurrent->accSmooth[x]);

    //Motors can be below minthrottle when disarmed, but that doesn't happen much
    writeUnsignedVB(blackboxCurrent->motor[0] - masterConfig.minthrottle);

    //Motors tend to be similar to each other
    for (x = 1; x < motorCount; x++)
        writeSignedVB(blackboxCurrent->motor[x] - blackboxCurrent->motor[0]);

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER))
        writeSignedVB(blackboxHistory[0]->servo[5] - 1500);

    //Rotate our history buffers:

    //The current state becomes the new "before" state
    blackboxHistory[1] = blackboxHistory[0];
    //And since we have no other history, we also use it for the "before, before" state
    blackboxHistory[2] = blackboxHistory[0];
    //And advance the current state over to a blank space ready to be filled
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}*/

static void writeInterframe(void)
{
    int x;
    int32_t deltas[5];

    blackboxValues_t *blackboxCurrent = blackboxHistory[0];
    blackboxValues_t *blackboxLast = blackboxHistory[1];

    blackboxWrite('P');

    // No need to store iteration count since its delta is always 1

    /*writeSignedVB((int32_t) (blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        writeSignedVB(blackboxCurrent->axisPID_P[x] - blackboxLast->axisPID_P[x]);

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        deltas[x] = blackboxCurrent->axisPID_I[x] - blackboxLast->axisPID_I[x];

    writeTag2_3S32(deltas);

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x))
            writeSignedVB(blackboxCurrent->axisPID_D[x] - blackboxLast->axisPID_D[x]);

    for (x = 0; x < 4; x++)
        deltas[x] = blackboxCurrent->rcCommand[x] - blackboxLast->rcCommand[x];

    writeTag8_4S16(deltas);

    //Check for sensors that are updated periodically (so deltas are normally zero) VBAT, MAG, BARO
    int optionalFieldCount = 0;

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
        deltas[optionalFieldCount++] = (int32_t) blackboxCurrent->vbatLatest - blackboxLast->vbatLatest;
    }

#ifdef MAG
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG)) {
        for (x = 0; x < XYZ_AXIS_COUNT; x++)
            deltas[optionalFieldCount++] = blackboxCurrent->magADC[x] - blackboxLast->magADC[x];
    }
#endif

#ifdef BARO
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO))
        deltas[optionalFieldCount++] = blackboxCurrent->BaroAlt - blackboxLast->BaroAlt;
#endif
    writeTag8_8SVB(deltas, optionalFieldCount);

    //Since gyros, accs and motors are noisy, base the prediction on the average of the history:
    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        writeSignedVB(blackboxHistory[0]->gyroData[x] - (blackboxHistory[1]->gyroData[x] + blackboxHistory[2]->gyroData[x]) / 2);

    for (x = 0; x < XYZ_AXIS_COUNT; x++)
        writeSignedVB(blackboxHistory[0]->accSmooth[x] - (blackboxHistory[1]->accSmooth[x] + blackboxHistory[2]->accSmooth[x]) / 2);

    for (x = 0; x < motorCount; x++)
        writeSignedVB(blackboxHistory[0]->motor[x] - (blackboxHistory[1]->motor[x] + blackboxHistory[2]->motor[x]) / 2);

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER))
        writeSignedVB(blackboxCurrent->servo[5] - blackboxLast->servo[5]);*/

    // Rotate our history buffers
    blackboxHistory[2] = blackboxHistory[1];
    blackboxHistory[1] = blackboxHistory[0];
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}

static int gcd(int num, int denom)
{
    if (denom == 0)
        return num;
    return gcd(denom, num % denom);
}

static void validateBlackboxConfig()
{
    int div;

    if (masterConfig.blackbox_rate_num == 0 || masterConfig.blackbox_rate_denom == 0 || masterConfig.blackbox_rate_num >= masterConfig.blackbox_rate_denom)
    {
        masterConfig.blackbox_rate_num = 1;
        masterConfig.blackbox_rate_denom = 1;
    }
    else
    {
        div = gcd(masterConfig.blackbox_rate_num, masterConfig.blackbox_rate_denom);

        masterConfig.blackbox_rate_num /= div;
        masterConfig.blackbox_rate_denom /= div;
    }
}

static void configureBlackboxPort(void)
{
    serialInit(115200);

    blackboxPort = core.mainport;

    /*
     * We want to write at about 7200 bytes per second to give the OpenLog a good chance to save to disk. If
     * about looptime microseconds elapse between our writes, this is the budget of how many bytes we should
     * transmit with each write.
     *
     * 9 / 1250 = 7200 / 1000000
     */
    serialChunkSize = max((masterConfig.looptime * 9) / 1250, 4);
}

static void releaseBlackboxPort(void)
{
    // Give the serial port back to the CLI
    serialInit(masterConfig.serial_baudrate);
}

void startBlackbox(void)
{
    if (blackboxState == BLACKBOX_STATE_STOPPED)
    {
        validateBlackboxConfig();

        configureBlackboxPort();

        if (!blackboxPort)
        {
            blackboxSetState(BLACKBOX_STATE_DISABLED);
            return;
        }

        memset(&gpsHistory, 0, sizeof(gpsHistory));

        blackboxHistory[0] = &blackboxHistoryRing[0];
        blackboxHistory[1] = &blackboxHistoryRing[1];
        blackboxHistory[2] = &blackboxHistoryRing[2];

        vbatReference = vbatLatest;

        // No need to clear the content of blackboxHistoryRing since our first frame will be an intra which overwrites it

        /*
         * We use conditional tests to decide whether or not certain fields should be logged. Since our headers
         * must always agree with the logged data, the results of these tests must not change during logging. So
         * cache those now.
         */
        blackboxBuildConditionCache();

        blackboxSetState(BLACKBOX_STATE_SEND_HEADER);
    }
}

void finishBlackbox(void)
{
    if (blackboxState != BLACKBOX_STATE_DISABLED && blackboxState != BLACKBOX_STATE_STOPPED)
    {
        blackboxSetState(BLACKBOX_STATE_STOPPED);

        releaseBlackboxPort();
    }
}

#ifdef GPS
static void writeGPSHomeFrame()
{
    blackboxWrite('H');

    writeSignedVB(GPS_home[0]);
    writeSignedVB(GPS_home[1]);
    // TODO it'd be great if we could grab the GPS current time and write that too

    gpsHistory.GPS_home[0] = GPS_home[0];
    gpsHistory.GPS_home[1] = GPS_home[1];
}

static void writeGPSFrame()
{
    blackboxWrite('G');

    writeUnsignedVB(GPS_numSat);
    writeSignedVB(GPS_coord[0] - gpsHistory.GPS_home[0]);
    writeSignedVB(GPS_coord[1] - gpsHistory.GPS_home[1]);
    writeUnsignedVB(GPS_altitude);
    writeUnsignedVB(GPS_speed);
    writeUnsignedVB(GPS_ground_course);

    gpsHistory.GPS_numSat = GPS_numSat;
    gpsHistory.GPS_coord[0] = GPS_coord[0];
    gpsHistory.GPS_coord[1] = GPS_coord[1];
}
#endif

static void loadBlackboxState(void)
{
    blackboxValues_t *blackboxCurrent = blackboxHistory[0];
    int i;

    blackboxCurrent->time = currentTime;
    blackboxCurrent->auxState_log = auxState_log;
    blackboxCurrent->flight_task = flight_task;

    blackboxCurrent->ref_roll = ref_roll_angle;
    blackboxCurrent->ref_pitch = ref_pitch_angle;
    blackboxCurrent->ref_yaw = ref_yaw_angle;
    blackboxCurrent->q_roll = quaternionroll;
    blackboxCurrent->q_pitch = quaternionpitch;
    blackboxCurrent->q_yaw = quaternionyaw;

    //for (i = 0; i < 4; i++)
    //    blackboxCurrent->rcCommand[i] = rcCommand[i];
    blackboxCurrent->rc_thrust = rcCommand[3];
    blackboxCurrent->vbat = vbat;

    blackboxCurrent->motor[0] = store_pwm[0];
    blackboxCurrent->motor[1] = store_pwm[1];
    blackboxCurrent->motor[2] = store_pwm[2];
    blackboxCurrent->motor[3] = store_pwm[3];

    blackboxCurrent->r_x = lrintf(100.0f*rigidbody_x);
    blackboxCurrent->r_y = lrintf(100.0f*rigidbody_y);
    blackboxCurrent->r_z = lrintf(100.0f*rigidbody_z);
    blackboxCurrent->wifiloopcount = wifiloopcount;
    blackboxCurrent->thrust_cmd = thrust_cmd;
    blackboxCurrent->roll_cmd = roll_cmd;
    blackboxCurrent->pitch_cmd = pitch_cmd;
    blackboxCurrent->yaw_cmd = yaw_cmd;
    blackboxCurrent->body_vx_datalog = body_vx_datalog;
    blackboxCurrent->cx_datalog = cx_datalog;
    blackboxCurrent->cy_datalog = cy_datalog;
    blackboxCurrent->cz_datalog = cz_datalog;
    blackboxCurrent->cqw = lrintf(1000.0f*q.w);
    blackboxCurrent->cqx = lrintf(1000.0f*q.x);
    blackboxCurrent->cqy = lrintf(1000.0f*q.y);
    blackboxCurrent->cqz = lrintf(1000.0f*q.z);
}

/**
 * Fill the current state of the blackbox using values read from the flight controller
 */
/*static void loadBlackboxState(void)
{
    blackboxValues_t *blackboxCurrent = blackboxHistory[0];
    int i;

    blackboxCurrent->time = currentTime;

    for (i = 0; i < XYZ_AXIS_COUNT; i++)
        blackboxCurrent->axisPID_P[i] = 0;
    for (i = 0; i < XYZ_AXIS_COUNT; i++)
        blackboxCurrent->axisPID_I[i] = 0;
    for (i = 0; i < XYZ_AXIS_COUNT; i++)
        blackboxCurrent->axisPID_D[i] = 0;

    for (i = 0; i < 4; i++)
        blackboxCurrent->rcCommand[i] = rcCommand[i];

    for (i = 0; i < XYZ_AXIS_COUNT; i++)
        blackboxCurrent->gyroData[i] = gyroData[i];

    for (i = 0; i < XYZ_AXIS_COUNT; i++)
        blackboxCurrent->accSmooth[i] = accSmooth[i];

    for (i = 0; i < motorCount; i++)
        blackboxCurrent->motor[i] = motor[i];

    blackboxCurrent->vbatLatest = vbatLatest;

#ifdef MAG
    for (i = 0; i < XYZ_AXIS_COUNT; i++)
        blackboxCurrent->magADC[i] = magADC[i];
#endif

#ifdef BARO
    blackboxCurrent->BaroAlt = BaroAlt;
#endif

    //Tail servo for tricopters
    blackboxCurrent->servo[5] = servo[5];
}*/

/**
 * Transmit the header information for the given field definitions. Transmitted header lines look like:
 *
 * H Field I name:a,b,c
 * H Field I predictor:0,1,2
 *
 * Provide an array 'conditions' of FlightLogFieldCondition enums if you want these conditions to decide whether a field
 * should be included or not. Otherwise provide NULL for this parameter and NULL for secondCondition.
 *
 * Set xmitState.headerIndex to 0 and xmitState.u.fieldIndex to -1 before calling for the first time.
 *
 * secondFieldDefinition and secondCondition element pointers need to be provided in order to compute the stride of the
 * fieldDefinition and secondCondition arrays.
 *
 * Returns true if there is still header left to transmit (so call again to continue transmission).
 */
static bool sendFieldDefinition(const char *const *headerNames, unsigned int headerCount, const void *fieldDefinitions,
                                const void *secondFieldDefinition, int fieldCount, const uint8_t *conditions, const uint8_t *secondCondition)
{
    const blackboxFieldDefinition_t *def;
    int charsWritten;
    static bool needComma = false;
    size_t definitionStride = (char *)secondFieldDefinition - (char *)fieldDefinitions;
    size_t conditionsStride = (char *)secondCondition - (char *)conditions;

    /*
     * We're chunking up the header data so we don't exceed our datarate. So we'll be called multiple times to transmit
     * the whole header.
     */
    if (xmitState.u.fieldIndex == -1)
    {
        if (xmitState.headerIndex >= headerCount)
            return false; // Someone probably called us again after we had already completed transmission

        charsWritten = blackboxPrint("H Field ");
        charsWritten += blackboxPrint(headerNames[xmitState.headerIndex]);
        charsWritten += blackboxPrint(":");

        xmitState.u.fieldIndex++;
        needComma = false;
    }
    else
        charsWritten = 0;

    for (; xmitState.u.fieldIndex < fieldCount && charsWritten < serialChunkSize; xmitState.u.fieldIndex++)
    {
        def = (const blackboxFieldDefinition_t *)((const char *)fieldDefinitions + definitionStride * xmitState.u.fieldIndex);

        if (!conditions || testBlackboxCondition(conditions[conditionsStride * xmitState.u.fieldIndex]))
        {
            if (needComma)
            {
                blackboxWrite(',');
                charsWritten++;
            }
            else
                needComma = true;

            // The first header is a field name
            if (xmitState.headerIndex == 0)
            {
                charsWritten += blackboxPrint(def->name);
            }
            else
            {
                // The other headers are integers
                if (def->arr[xmitState.headerIndex - 1] >= 10)
                {
                    blackboxWrite(def->arr[xmitState.headerIndex - 1] / 10 + '0');
                    blackboxWrite(def->arr[xmitState.headerIndex - 1] % 10 + '0');
                    charsWritten += 2;
                }
                else
                {
                    blackboxWrite(def->arr[xmitState.headerIndex - 1] + '0');
                    charsWritten++;
                }
            }
        }
    }

    // Did we complete this line?
    if (xmitState.u.fieldIndex == fieldCount)
    {
        blackboxWrite('\n');
        xmitState.headerIndex++;
        xmitState.u.fieldIndex = -1;
    }

    return xmitState.headerIndex < headerCount;
}

/**
 * Transmit a portion of the system information headers. Call the first time with xmitState.headerIndex == 0. Returns
 * true iff transmission is complete, otherwise call again later to continue transmission.
 */
static bool blackboxWriteSysinfo()
{
    union floatConvert_t
    {
        float f;
        uint32_t u;
    } floatConvert;

    if (xmitState.headerIndex == 0)
    {
        xmitState.u.serialBudget = 0;
        xmitState.headerIndex = 1;
    }

    // How many bytes can we afford to transmit this loop?
    xmitState.u.serialBudget = min(xmitState.u.serialBudget + serialChunkSize, 64);

    // Most headers will consume at least 20 bytes so wait until we've built up that much link budget
    if (xmitState.u.serialBudget < 20)
    {
        return false;
    }

    switch (xmitState.headerIndex)
    {
    case 0:
        // Shouldn't ever get here
        break;
    case 1:
        blackboxPrintf("H Firmware type:Baseflight\n");

        xmitState.u.serialBudget -= strlen("H Firmware type:Baseflight\n");
        break;
    case 2:
        // No firmware revision info to write
        break;
    case 3:
        blackboxPrintf("H Firmware date:%s %s\n", __DATE__, __TIME__);

        /* Don't need to be super exact about the budget so don't mind the fact that we're including the length of
         * the placeholder "%s"
         */
        xmitState.u.serialBudget -= strlen("H Firmware date:%s %s\n") + strlen(__DATE__) + strlen(__TIME__);
        break;
    case 4:
        blackboxPrintf("H P interval:%d/%d\n", masterConfig.blackbox_rate_num, masterConfig.blackbox_rate_denom);

        xmitState.u.serialBudget -= strlen("H P interval:%d/%d\n");
        break;
    case 5:
        blackboxPrintf("H rcRate:%d\n", cfg.rcRate8);

        xmitState.u.serialBudget -= strlen("H rcRate:%d\n");
        break;
    case 6:
        blackboxPrintf("H minthrottle:%d\n", masterConfig.minthrottle);

        xmitState.u.serialBudget -= strlen("H minthrottle:%d\n");
        break;
    case 7:
        blackboxPrintf("H maxthrottle:%d\n", masterConfig.maxthrottle);

        xmitState.u.serialBudget -= strlen("H maxthrottle:%d\n");
        break;
    case 8:
        floatConvert.f = gyro.scale;
        blackboxPrintf("H gyro.scale:0x%x\n", floatConvert.u);

        xmitState.u.serialBudget -= strlen("H gyro.scale:0x%x\n") + 6;
        break;
    case 9:
        blackboxPrintf("H acc_1G:%u\n", acc_1G);

        xmitState.u.serialBudget -= strlen("H acc_1G:%u\n");
        break;
    case 10:
        blackboxPrintf("H vbatscale:%u\n", masterConfig.vbatscale);

        xmitState.u.serialBudget -= strlen("H vbatscale:%u\n");
        break;
    case 11:
        blackboxPrintf("H vbatcellvoltage:%u,0,%u\n", masterConfig.vbatmincellvoltage, masterConfig.vbatmaxcellvoltage);

        xmitState.u.serialBudget -= strlen("H vbatcellvoltage:%u,0,%u\n");
        break;
    case 12:
        blackboxPrintf("H vbatref:%u\n", vbatReference);

        xmitState.u.serialBudget -= strlen("H vbatref:%u\n");
        break;
    default:
        return true;
    }

    xmitState.headerIndex++;
    return false;
}

// Beep the buzzer and write the current time to the log as a synchronization point
static void blackboxPlaySyncBeep()
{
    uint32_t now = micros();

    /*
     * The regular beep routines aren't going to work for us, because they queue up the beep to be executed later.
     * Our beep is timing sensitive, so start beeping now without setting the beeperIsOn flag.
     */
    BEEP_ON;

    // Have the regular beeper code turn off the beep for us eventually, since that's not timing-sensitive
    buzzer(BUZZER_ARMING);

    blackboxWrite('E');
    blackboxWrite(FLIGHT_LOG_EVENT_SYNC_BEEP);

    writeUnsignedVB(now);
}

void handleBlackbox(void)
{
    int i;

    switch (blackboxState)
    {
    case BLACKBOX_STATE_SEND_HEADER:
        // On entry of this state, xmitState.headerIndex is 0 and startTime is intialised

        /*
         * Once the UART has had time to init, transmit the header in chunks so we don't overflow our transmit
         * buffer.
         */
        if (millis() > xmitState.u.startTime + 100)
        {
            for (i = 0; i < serialChunkSize && blackboxHeader[xmitState.headerIndex] != '\0'; i++, xmitState.headerIndex++)
                blackboxWrite(blackboxHeader[xmitState.headerIndex]);

            if (blackboxHeader[xmitState.headerIndex] == '\0')
                blackboxSetState(BLACKBOX_STATE_SEND_FIELDINFO);
        }
        break;
    case BLACKBOX_STATE_SEND_FIELDINFO:
        // On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition(blackboxMainHeaderNames, ARRAY_LENGTH(blackboxMainHeaderNames), blackboxMainFields, blackboxMainFields + 1,
                                 ARRAY_LENGTH(blackboxMainFields), &blackboxMainFields[0].condition, &blackboxMainFields[1].condition))
        {
#ifdef GPS
            if (feature(FEATURE_GPS))
                blackboxSetState(BLACKBOX_STATE_SEND_GPS_H_HEADERS);
            else
#endif
                blackboxSetState(BLACKBOX_STATE_SEND_SYSINFO);
        }
        break;
#ifdef GPS
    case BLACKBOX_STATE_SEND_GPS_H_HEADERS:
        // On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition(blackboxGPSHHeaderNames, ARRAY_LENGTH(blackboxGPSHHeaderNames), blackboxGpsHFields, blackboxGpsHFields + 1,
                                 ARRAY_LENGTH(blackboxGpsHFields), NULL, NULL))
        {
            blackboxSetState(BLACKBOX_STATE_SEND_GPS_G_HEADERS);
        }
        break;
    case BLACKBOX_STATE_SEND_GPS_G_HEADERS:
        // On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition(blackboxGPSGHeaderNames, ARRAY_LENGTH(blackboxGPSGHeaderNames), blackboxGpsGFields, blackboxGpsGFields + 1,
                                 ARRAY_LENGTH(blackboxGpsGFields), NULL, NULL))
        {
            blackboxSetState(BLACKBOX_STATE_SEND_SYSINFO);
        }
        break;
#endif
    case BLACKBOX_STATE_SEND_SYSINFO:
        // On entry of this state, xmitState.headerIndex is 0

        // Keep writing chunks of the system info headers until it returns true to signal completion
        if (blackboxWriteSysinfo())
            blackboxSetState(BLACKBOX_STATE_PRERUN);
        break;
    case BLACKBOX_STATE_PRERUN:
        blackboxPlaySyncBeep();

        blackboxSetState(BLACKBOX_STATE_RUNNING);
        break;
    case BLACKBOX_STATE_RUNNING:
        // On entry to this state, blackboxIteration, blackboxPFrameIndex and blackboxIFrameIndex are reset to 0

        // Write a keyframe every BLACKBOX_I_INTERVAL frames so we can resynchronise upon missing frames
        if (blackboxPFrameIndex == 0)
        {
            // Copy current system values into the blackbox
            loadBlackboxState();
            writeIntraframe();
        }
        else
        {
            // if ((blackboxPFrameIndex + masterConfig.blackbox_rate_num - 1) % masterConfig.blackbox_rate_denom < masterConfig.blackbox_rate_num) {
            //     loadBlackboxState();
            //     writeInterframe();
            // }
#ifdef GPS
            if (feature(FEATURE_GPS))
            {
                /*
                 * If the GPS home point has been updated, or every 128 intraframes (~10 seconds), write the
                 * GPS home position.
                 *
                 * We write it periodically so that if one Home Frame goes missing, the GPS coordinates can
                 * still be interpreted correctly.
                 */
                if (GPS_home[0] != gpsHistory.GPS_home[0] || GPS_home[1] != gpsHistory.GPS_home[1] || (blackboxPFrameIndex == BLACKBOX_I_INTERVAL / 2 && blackboxIFrameIndex % 128 == 0))
                {

                    writeGPSHomeFrame();
                    writeGPSFrame();
                }
                else if (GPS_numSat != gpsHistory.GPS_numSat || GPS_coord[0] != gpsHistory.GPS_coord[0] || GPS_coord[1] != gpsHistory.GPS_coord[1])
                {
                    // We could check for velocity changes as well but I doubt it changes independent of position
                    writeGPSFrame();
                }
            }
#endif
        }

        blackboxIteration++;
        blackboxPFrameIndex++;

        if (blackboxPFrameIndex == BLACKBOX_I_INTERVAL)
        {
            blackboxPFrameIndex = 0;
            blackboxIFrameIndex++;
        }
        break;
    default:
        break;
    }
}

static bool canUseBlackboxWithCurrentConfiguration(void)
{
    // if (!feature(FEATURE_BLACKBOX))
    //     return false;

    return true;
}

void initBlackbox(void)
{
    if (canUseBlackboxWithCurrentConfiguration())
        blackboxSetState(BLACKBOX_STATE_STOPPED);
    else
        blackboxSetState(BLACKBOX_STATE_DISABLED);
}

#endif
