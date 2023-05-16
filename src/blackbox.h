/*
 * blackbox.h
 *
 *      Author: Nicholas Sherlock
 */
#include "board.h"
#ifdef BLACKBOX

#ifndef BLACKBOX_H_
#define BLACKBOX_H_

#include <stdint.h>

#ifndef XYZ_AXIS_COUNT
#define XYZ_AXIS_COUNT 3
#endif

typedef struct blackboxValues_t {
    uint32_t time;
    int16_t auxState_log;
    int16_t flight_task;
    int16_t ref_roll;
    int16_t ref_pitch;
    int16_t ref_yaw;
    int16_t q_roll;
    int16_t q_pitch;
    int16_t q_yaw;
    int16_t rc_thrust;
	uint16_t vbat;
    int16_t motor[MAX_MOTORS];
    int16_t r_x;
    int16_t r_y;
    int16_t r_z;
    int16_t wifiloopcount;
    int16_t thrust_cmd;
    int16_t roll_cmd;
    int16_t pitch_cmd;
    int16_t yaw_cmd;
    int16_t body_vx_datalog;
    int16_t cx_datalog;
    int16_t cy_datalog;
    int16_t cz_datalog;
    int16_t cqw;
    int16_t cqx;
    int16_t cqy;
    int16_t cqz;

} blackboxValues_t;

/*typedef struct blackboxValues_t {
    uint32_t time;

    int32_t axisPID_P[XYZ_AXIS_COUNT], axisPID_I[XYZ_AXIS_COUNT], axisPID_D[XYZ_AXIS_COUNT];

    int16_t rcCommand[4];
    int16_t gyroData[XYZ_AXIS_COUNT];
    int16_t accSmooth[XYZ_AXIS_COUNT];
    int16_t motor[MAX_MOTORS];
    int16_t servo[MAX_SERVOS];
    
    uint16_t vbatLatest;

#ifdef BARO
    int32_t BaroAlt;
#endif
#ifdef MAG
    int16_t magADC[XYZ_AXIS_COUNT];
#endif
} blackboxValues_t;*/

void initBlackbox(void);
void handleBlackbox(void);
void startBlackbox(void);
void finishBlackbox(void);

#endif /* BLACKBOX_H_ */

#endif /* BLACKBOX */
