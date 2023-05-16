/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "board.h"
#include "mw.h"
#include "cli.h"
#include "telemetry_common.h"
#include "blackbox.h"
#include "buzzer.h"

#define BARO
#define MAG
#define ATTITUDE_TUNING           //use this if you "ONLY" want to tuning attitude controller
//#define POSITION_TUNING           //use this if you want to tuning postion controller before actual fly, NEED TO CHOOSE YOUR OWN "magHold" !
//#define FLY_BY_ATTITUDE_CONTROL   //fly by attitude control,no x,y position control

#ifdef BLACKBOX
    #include "blackbox.h"
#endif

uint8_t gpsstate;
uint8_t gpstypecheck;
uint8_t gpsInitUbloxcheck;
uint8_t ubx_init_statecheck;

flags_t f;
int16_t debug[4];
uint32_t currentTime = 0;
uint32_t previousTime = 0;
uint16_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
int16_t headFreeModeHold;

uint16_t vbat;                  // battery voltage in 0.1V steps
uint16_t vbatLatest = 0;

int16_t taskOrdercheck;

int32_t amperage;               // amperage read by current sensor in centiampere (1/100th A)
int32_t mAhdrawn;               // milliampere hours drawn from the battery since start
int16_t telemTemperature1;      // gyro sensor temperature

int16_t failsafeCnt = 0;
int16_t failsafeEvents = 0;
int16_t rcData[RC_CHANS];       // interval [1000;2000]
int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];     // lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE
uint16_t rssi;                  // range: [0;1023]
rcReadRawDataPtr rcReadRawFunc = NULL;  // receive data from default (pwm/ppm) or additional (spek/sbus/?? receiver drivers)
int16_t store_alt = 0;
int16_t store_pwm[5];

//static void pidMultiWii(void);
//static void pidRewrite(void);
//pidControllerFuncPtr pid_controller = pidMultiWii; // which pid controller are we using, defaultMultiWii

//uint8_t dynP8[3], dynI8[3], dynD8[3];
uint8_t rcOptions[CHECKBOXITEMS];

//int16_t axisPID[3];

//int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];

static int16_t initialThrottle = 0;

// GPS
// **********************
int32_t GPS_coord[2];
int32_t GPS_home[3];
int32_t GPS_hold[3];
uint8_t GPS_numSat;
uint16_t GPS_distanceToHome;        // distance to home point in meters
int16_t GPS_directionToHome;        // direction to home or hol point in degrees
uint16_t GPS_altitude, GPS_speed;   // altitude in 0.1m and speed in 0.1m/s
uint8_t GPS_update = 0;             // it's a binary toogle to distinct a GPS position update
int16_t GPS_angle[3] = { 0, 0, 0 }; // it's the angles that must be applied for GPS correction
uint16_t GPS_ground_course = 0;     // degrees * 10
int16_t nav[2];
int16_t nav_rated[2];               // Adding a rate controller to the navigation to make it smoother
int8_t nav_mode = NAV_MODE_NONE;    // Navigation mode
uint8_t GPS_numCh;                  // Number of channels
uint8_t GPS_svinfo_chn[32];         // Channel number
uint8_t GPS_svinfo_svid[32];        // Satellite ID
uint8_t GPS_svinfo_quality[32];     // Bitfield Qualtity
uint8_t GPS_svinfo_cno[32];         // Carrier to Noise Ratio (Signal Strength)
uint32_t GPS_update_rate[2];        // GPS coordinates updating rate (column 0 = last update time, 1 = current update ms)
uint32_t GPS_svinfo_rate[2];        // GPS svinfo updating rate (column 0 = last update time, 1 = current update ms)
uint32_t GPS_HorizontalAcc;         // Horizontal accuracy estimate (mm)
uint32_t GPS_VerticalAcc;           // Vertical accuracy estimate (mm)

// Automatic ACC Offset Calibration
bool AccInflightCalibrationArmed = false;
bool AccInflightCalibrationMeasurementDone = false;
bool AccInflightCalibrationSavetoEEProm = false;
bool AccInflightCalibrationActive = false;
bool calibrating = false;
uint16_t InflightcalibratingA = 0;

// Battery monitoring stuff
uint8_t batteryCellCount = 2;       // cell count
uint16_t batteryWarningVoltage;     // slow buzzer after this one, recommended 80% of battery used. Time to land.
uint16_t batteryCriticalVoltage;    // annoying buzzer after this one, battery is going to be dead.

// Time of automatic disarm when "Don't spin the motors when armed" is enabled.
//static uint32_t disarmTime = 0;

// stop flag
bool stop = false;
bool landing = false;
bool mode1 = false;
bool mode2 = false;

//wifidata add at 202221102
float rigidbody_x,rigidbody_y,rigidbody_z;
float original_x = 0.0f;
float original_y = 0.0f;
float original_z = 0.0f;    
int16_t lost_gps_check_count = 0;

// for controller
bool org_init = false;
bool mode1_trigger = false;
bool mode2_trigger = true;
bool mode3_trigger = true;
bool landing_trigger = false;
int16_t OX;
int16_t OY;
int16_t OZ;
quaternion quat_hold;
int16_t rcCommand_store[4];
float align_maghold_sine;
float align_maghold_cosine;
float last_x;
float last_y;
int16_t auxState_log;
int16_t ref_roll_angle;
int16_t ref_pitch_angle;
int16_t ref_yaw_angle;
float px_ref;    //set desired x (MOCAP coordinate)
float py_ref;    //set desired y (MOCAP coordinate)
float pz_ref;    //set desired height (MOCAP coordinate)
float bvx_ref;   //body_x velocity ref
int16_t flight_task = 0;
bool bankturn_task = false;

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;

    for (r = 0; r < repeat; r++) {
        for (i = 0; i < num; i++) {
            LED0_TOGGLE;            // switch LEDPIN state
            BEEP_ON;
            delay(wait);
            BEEP_OFF;
        }
        delay(60);
    }
}

void annexCode(void)
{
    static uint32_t calibratedAccTime;
//    int32_t tmp, tmp2;
//    int32_t axis, prop1, prop2;

    // vbat shit
    static uint8_t vbatTimer = 0;
    static int32_t vbatRaw = 0;
    static int32_t amperageRaw = 0;
    static int64_t mAhdrawnRaw = 0;
    static int32_t vbatCycleTime = 0;

	if (count > 1500 && rcData[0] >= 1300){    // Mode 1 , enable initialThrottle
		initialThrottle = 1;
    }

	if (initialThrottle == 1){
        if(mode1){      // Mode 1 manual input 
            rcCommand[THROTTLE] = (rcData[0] - rcCommand_store[0]) / 2;
            rcCommand[ROLL] = (rcData[2] - rcCommand_store[2]) / 9;
            rcCommand[PITCH] = (rcData[3] - rcCommand_store[3]) / 130;

            if(count % 30 == 0){
                rcCommand[YAW] = rcCommand[YAW] + (rcData[1] - rcCommand_store[1])/130;
            }

            if (rcCommand[YAW] > 360)
                rcCommand[YAW] -= 360;
            if (rcCommand[YAW] < -360)
                rcCommand[YAW] += 360;  
        }
        if(mode2){      // Mode 2 manual input    
            rcCommand[THROTTLE] = (rcData[0] - rcCommand_store[0]) / 2;
            rcCommand[ROLL] = (rcData[2] - rcCommand_store[2]) / 9;
            rcCommand[PITCH] = (rcData[3] - rcCommand_store[3]) / 130;

            if(count % 30 == 0){
                rcCommand[YAW] = rcCommand[YAW] + (rcData[1] - rcCommand_store[1])/130;
            }		
            if (rcCommand[YAW] > 360)
                rcCommand[YAW] -= 360;
            if (rcCommand[YAW] < -360)
                rcCommand[YAW] += 360; 
        }
    }		

    if (feature(FEATURE_VBAT)) {
        vbatCycleTime += cycleTime;
        if (!(++vbatTimer % VBATFREQ)) {
            vbatRaw -= vbatRaw / 8;
            vbatLatest = adcGetChannel(ADC_BATTERY);
            vbatRaw += vbatLatest;
            vbat = batteryAdcToVoltage(vbatRaw / 8);
            
            if (mcfg.power_adc_channel > 0) {
                amperageRaw -= amperageRaw / 8;
                amperageRaw += adcGetChannel(ADC_EXTERNAL_CURRENT);
                amperage = currentSensorToCentiamps(amperageRaw / 8);
                mAhdrawnRaw += (amperage * vbatCycleTime) / 1000;
                mAhdrawn = mAhdrawnRaw / (3600 * 100);
                vbatCycleTime = 0;
            }
            
        }
        // Buzzers for low and critical battery levels
        if (vbat <= batteryCriticalVoltage)
            buzzer(BUZZER_BAT_CRIT_LOW);     // Critically low battery
        else if (vbat <= batteryWarningVoltage)
            buzzer(BUZZER_BAT_LOW);     // low battery
    }
		
    if ((calibratingA > 0 && sensors(SENSOR_ACC)) || (calibratingG > 0)) {      // Calibration phasis
        LED0_TOGGLE;
    } else {
        if (f.ACC_CALIBRATED)
            LED0_OFF;
        if (f.ARMED)
            LED0_ON;
    }

    if ((int32_t)(currentTime - calibratedAccTime) >= 0) {
        if (!f.SMALL_ANGLE) {
            f.ACC_CALIBRATED = 0; // the multi uses ACC and is not calibrated or is too much inclinated
            LED0_TOGGLE;
            calibratedAccTime = currentTime + 500000;
        } else {
            f.ACC_CALIBRATED = 1;
        }
    }

    serialCom();

    if (sensors(SENSOR_GPS)) {
        static uint32_t GPSLEDTime;
        if ((int32_t)(currentTime - GPSLEDTime) >= 0 && (GPS_numSat >= 5)) {
            GPSLEDTime = currentTime + 150000;
            LED1_TOGGLE;
        }
    }
    // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
    if (gyro.temperature)
        gyro.temperature(&telemTemperature1);

}

uint16_t pwmReadRawRC(uint8_t chan)
{
    return pwmRead(mcfg.rcmap[chan]);
}

void computeRC(void)
{
    uint16_t capture;
    int i, chan;

    if (feature(FEATURE_SERIALRX)) {
        for (chan = 0; chan < 8; chan++)
            rcData[chan] = rcReadRawFunc(chan);
    } else {
        static int16_t rcDataAverage[8][4];
        static int rcAverageIndex = 0;

        for (chan = 0; chan < 8; chan++) {
            capture = rcReadRawFunc(chan);

            // validate input
            if (capture < PULSE_MIN || capture > PULSE_MAX)
                capture = mcfg.midrc;
            rcDataAverage[chan][rcAverageIndex % 4] = capture;
            // clear this since we're not accessing it elsewhere. saves a temp var
            rcData[chan] = 0;
            for (i = 0; i < 4; i++)
                rcData[chan] += rcDataAverage[chan][i];
            rcData[chan] /= 4;
        }
        rcAverageIndex++;
    }
}

static void mwArm(void)
{
    if (calibratingG == 0 && f.ACC_CALIBRATED) {
        // TODO: feature(FEATURE_FAILSAFE) && failsafeCnt < 2
        // TODO: && ( !feature || ( feature && ( failsafecnt > 2) )
        if (!f.ARMED) {         // arm now!
            f.ARMED = 1;
			stop = false;
            rcCommand_store[0] = rcData[0];
            rcCommand_store[1] = rcData[1];
            rcCommand_store[2] = rcData[2];
            rcCommand_store[3] = rcData[3];
            magHold = (int16_t)(quaternionyaw/10);
            org_init = true;
            quat_hold.w = q.w;
            quat_hold.x = q.x;
            quat_hold.y = q.y;
            quat_hold.z = q.z;

            float dir;  //for align mocap coordinate to earth coordinate
            dir = DEGREES_TO_RADIANS( (float)(-magHold) );
            align_maghold_sine = sin_approx( dir );
            align_maghold_cosine = cos_approx( dir );

#ifdef ATTITUDE_TUNING
            desired_quaternion.w = q.w;
            desired_quaternion.x = q.x;
            desired_quaternion.y = q.y;
            desired_quaternion.z = q.z;
#endif

#ifdef BLACKBOX
            	startBlackbox();
#endif
        }
    }
}

static void mwDisarm(void)
{
    if (f.ARMED) {
        f.ARMED = 0;
        mode1 = false;
        mode2 = false;
        landing = false;
		stop = true;
		initialThrottle = 0;
			
#ifdef BLACKBOX
        	finishBlackbox();
#endif
				
    }
}

void align_MOCAP_coordinat_to_earth_coordinate(float *mocap_x, float *mocap_y){
//this function is rotate 'optitrack xy direction' align 'maghold' before we start position_controller, 
//since we might have different maghold everytime or every experiment setup.
    float tmp;
    tmp = *mocap_x;
    *mocap_x = *mocap_x * align_maghold_cosine + *mocap_y * align_maghold_sine * (-1.0f);
    *mocap_y = tmp * align_maghold_sine + *mocap_y * align_maghold_cosine;
}

#ifdef GPS
void lost_gps_check(void){
    if(gpsstate == 5){          //we lost gps 
        lost_gps_check_count++;
    }
    else{
        lost_gps_check_count = 0;
    }
}
#endif

int16_t roll_cmd = 0, pitch_cmd = 0, yaw_cmd = 0, thrust_cmd = 0;
void loop(void)
{
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    static uint8_t rcSticks;            // this hold sticks position for command combos
	static int flag = 0;
	static int m1_count_store;
    static int m2_count_store;
    static int m3_count_store;
    uint8_t stTmp = 0;
    int i;
    static uint32_t rcTime = 0;
#ifdef BARO
    static int16_t initialThrottleHold;
#endif
    static uint32_t loopTime;
    uint16_t auxState = 0;

#ifdef GPS
    static uint8_t GPSNavReset = 1;
#endif	

//    bool isThrottleLow = false;
    bool rcReady = false;

    // calculate rc stuff from serial-based receivers (spek/sbus)
    if (feature(FEATURE_SERIALRX)) {
        switch (mcfg.serialrx_type) {
            case SERIALRX_SPEKTRUM1024:
            case SERIALRX_SPEKTRUM2048:
                rcReady = spektrumFrameComplete();
                break;
            case SERIALRX_SBUS:
                rcReady = sbusFrameComplete();
                break;
            case SERIALRX_SUMD:
                rcReady = sumdFrameComplete();
                break;
            case SERIALRX_MSP:
                rcReady = mspFrameComplete();
                break;
        }
    }

    if (((int32_t)(currentTime - rcTime) >= 0) || rcReady) { // 50Hz or data driven
        rcReady = false;
        rcTime = currentTime + 20000;
        computeRC();

        // Read rssi value
        rssi = RSSI_getValue();

        // ------------------ STICKS COMMAND HANDLER --------------------
        // checking sticks positions
        for (i = 0; i < 4; i++) {
            stTmp >>= 2;
            if (rcData[i] > mcfg.mincheck)
                stTmp |= 0x80;  // check for MIN
            if (rcData[i] < mcfg.maxcheck)
                stTmp |= 0x40;  // check for MAX
        }
        if (stTmp == rcSticks) {
            if (rcDelayCommand < 250)
                rcDelayCommand++;
        } else
            rcDelayCommand = 0;
        rcSticks = stTmp;

        // Check AUX switches

        if (rcDelayCommand == 20) {
            if (f.ARMED) {      // actions during armed

            } 
			else {            // actions during not armed
                i = 0;
                // GYRO BARO calibration
                
                if (rcData[2] < 1200){  //use stick to activate acc,gyro and baro calibration
                     calibratingA = CALIBRATING_ACC_CYCLES;
                     calibratingG = CALIBRATING_GYRO_CYCLES;
                     calibratingB = CALIBRATING_BARO_CYCLES;
#ifdef GPS
                if (feature(FEATURE_GPS))
                    GPS_reset_home_position();
#endif
                     LED0_TOGGLE;
                }
                if (rcData[3] < 1200){  //use stick to activate mag calibration
                    f.CALIBRATE_MAG = 1;
                    LED0_TOGGLE;
                }            
            }
        }

        for (i = 0; i < 2; i++)
            auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);
						
        if(auxState == 20)
				{
					  mwArm();
                      mode1 = true;
					  mode2 = false;
					  landing = false;
				}
				else if (auxState == 12)
					  {
                        mode1 = false;
					    mode2 = false;
                        landing = true;
                      }
				else if (auxState == 36)
					 {
                        mode1 = false;
					    mode2 = true;
                        landing = false;
                     } 
				else if (auxState == 17 || auxState == 33 || auxState == 9)
					  mwDisarm();
    
#ifdef GPS
        if (sensors(SENSOR_GPS)) {
            if (f.GPS_FIX && GPS_numSat >= 5) {
                // if both GPS_HOME & GPS_HOLD are checked => GPS_HOME is the priority
                if (rcOptions[BOXGPSHOME] || f.FW_FAILSAFE_RTH_ENABLE ) {
                    if (!f.GPS_HOME_MODE) {
                        f.GPS_HOME_MODE = 1;
                        f.GPS_HOLD_MODE = 0;
                        GPSNavReset = 0;
                        GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);
                        nav_mode = NAV_MODE_WP;
                        GPS_hold[ALT] = GPS_altitude;
                        f.CLIMBOUT_FW = 1;
                    }
                } else {
                    f.GPS_HOME_MODE = 0;
                    if (rcOptions[BOXGPSHOLD] && abs(rcCommand[ROLL]) < cfg.ap_mode && abs(rcCommand[PITCH]) < cfg.ap_mode) {
                        if (!f.GPS_HOLD_MODE) {
                            f.GPS_HOLD_MODE = 1;
                            GPSNavReset = 0;
                            GPS_hold[LAT] = GPS_coord[LAT];
                            GPS_hold[LON] = GPS_coord[LON];
                            GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
                            nav_mode = NAV_MODE_POSHOLD;
                            GPS_hold[ALT] = GPS_altitude;
                            f.CLIMBOUT_FW = 0;
                        }
                    } else {
                        f.GPS_HOLD_MODE = 0;
                        // both boxes are unselected here, nav is reset if not already done
                        if (GPSNavReset == 0) {
                            GPSNavReset = 1;
                            GPS_reset_nav();
                            f.CLIMBOUT_FW = 0;
                        }
                    }
                }
                // Beep for indication that GPS has found satellites and naze32 is ready to fly
                buzzer(BUZZER_READY_BEEP);
            } else {
                f.GPS_HOME_MODE = 0;
                f.GPS_HOLD_MODE = 0;
                nav_mode = NAV_MODE_NONE;
            }
        }
#endif

    }
    else{                        // not in rc loop
        static int taskOrder = 0;   // never call all function in the same loop, to avoid high delay spikes
        taskOrdercheck = taskOrder;

        switch (taskOrder) {
        case 0:
            taskOrder++;
            if (sensors(SENSOR_MAG) && Mag_getADC())
                break;
        case 1:
            taskOrder++;
            if (sensors(SENSOR_BARO) && Baro_update())
                break;
        case 2:
            taskOrder++;
            if (sensors(SENSOR_BARO) && getEstimatedAltitude())
                break;
        
        case 3:
            taskOrder++;

//!!!!!!!!!!!!!!         DO NOT UNCOMMENT IF YOU ARE USING WIFI        !!!!!!!!!!!!!//
// if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
// hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
// change this based on available hardware
/*#ifdef GPS
            if (feature(FEATURE_GPS)) {
                gpsThread();
                break;
                }
#endif*/

        case 4:
            taskOrder = 0;
        }
    }
    currentTime = micros();
    if(mcfg.looptime == 0 || (int32_t)(currentTime - loopTime) >= 0){
        loopTime = currentTime + mcfg.looptime;

        computeIMU();
        imuUpdateAttitude(currentTime);   // 20220913 add quaternion calculate   
        use_quaternion_angle();   // 20220913 add quaternion calculate

        // Measure loop rate just afer reading the sensors
        currentTime = micros();
        cycleTime = (int32_t)(currentTime - previousTime);
        previousTime = currentTime;
        // non IMU critical, temeperatur, serialcom
        annexCode();

#ifdef GPS
        if (sensors(SENSOR_GPS)) {
            if ((f.GPS_HOME_MODE || f.GPS_HOLD_MODE) && f.GPS_FIX_HOME) {
                float sin_yaw_y = sinf(heading * 0.0174532925f);
                float cos_yaw_x = cosf(heading * 0.0174532925f);
                if (!f.FIXED_WING) {
                    if (cfg.nav_slew_rate) {
                        nav_rated[LON] += constrain(wrap_18000(nav[LON] - nav_rated[LON]), -cfg.nav_slew_rate, cfg.nav_slew_rate); // TODO check this on uint8
                        nav_rated[LAT] += constrain(wrap_18000(nav[LAT] - nav_rated[LAT]), -cfg.nav_slew_rate, cfg.nav_slew_rate);
                        GPS_angle[ROLL] = (nav_rated[LON] * cos_yaw_x - nav_rated[LAT] * sin_yaw_y) / 10;
                        GPS_angle[PITCH] = (nav_rated[LON] * sin_yaw_y + nav_rated[LAT] * cos_yaw_x) / 10;
                    } else {
                        GPS_angle[ROLL] = (nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) / 10;
                        GPS_angle[PITCH] = (nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) / 10;
                    }
                } else fw_nav();
            } else {
                GPS_angle[ROLL] = 0;
                GPS_angle[PITCH] = 0;
                GPS_angle[YAW] = 0;
            }
        }
#endif

			if( count<= 1500){  //when armed and initialthrottle == 0, wait for 'annexCode' Function to enable initialThrottle
                motor[0] = 1100;
                motor[1] = 1100;
                servo[3] = 1500;
                servo[4] = 1500;
                LED0_TOGGLE;
            }
            
            // align optitrack coordinate to earth   
            rigidbody_x = (float)tmp_px / 100.0f;   //cm to m	
            rigidbody_y = (float)tmp_py / 100.0f;
            rigidbody_z = (float)tmp_pz / 100.0f;	
            align_MOCAP_coordinat_to_earth_coordinate(&rigidbody_x,&rigidbody_y);    

			if( f.ARMED && initialThrottle == 1 ) {	
                //store position xyz before fly
                if(org_init){
                //original_x = rigidbody_x; //may be useful in the future
                //original_y = rigidbody_y; //may be useful in the future
                original_z = rigidbody_z;
                OX = tmp_px;
                OY = tmp_py;
                OZ = tmp_pz;
                org_init = false;
                } 

				if(flag == 0){	
                    flag = 1;
                    m1_count_store = count;
                    store_alt = EstAlt;     //store initial ALT from acc and baro
                    rcCommand[ROLL] = 0;
                    rcCommand[PITCH] = 0;
                    rcCommand[YAW] = 0;   
                    rcCommand[THROTTLE] = 0;                             
                }

                if(mode1){
                    flight_task = 1; //hover 
                    ref_roll_angle = 0;
                    ref_pitch_angle = -90;  //hover
                    ref_yaw_angle = -magHold;
                    px_ref = (float)OX/100.0f + 0.0f;    //set desired x (MOCAP coordinate)
                    py_ref = (float)OY/100.0f + 0.0f;     //set desired y (MOCAP coordinate)
                    pz_ref = 0.6f;    //set desired height (MOCAP coordinate)
                    bvx_ref = 0.0f;   //body_x velocity ref

                    static uint32_t pT;
                    uint32_t cT = micros();
                    uint32_t dT;
                    dT = cT - pT;

                    if(mode1_trigger == 1){ //when switch back to mode1
                        m1_count_store = count;
                        magHold = lrintf( (float)quaternionyaw/10.0f ); //reset maghold when we change back to mode1 from mode2
                        mode1_trigger = false;
                        mode2_trigger = true;
                        mode3_trigger = true;
                    }

//*********** trajectory and position_controller at 20hz, attitude_controller at 200hz **************//     
//dont use these lines of code if you want to tune attitude only(quaternion_desired = quaternion_current when you armed!)
#ifndef ATTITUDE_TUNING
                    if (dT < 50000){    //50000 microsecends means 20HZ
                        //pass
                    }
                    else{                        
                        pT = cT;
                        float x_error,y_error,z_error;

//========= trajectory generate: =========//
                        voltage_manergor(&flight_task, vbat);
                        trajectory_generator(flight_task);    // 1:hover, , it will update px_ref....ref_roll_angle...etc for controllers


//========= position controller: =========//
                        align_MOCAP_coordinat_to_earth_coordinate(&px_ref,&py_ref);
                        x_error = x_PID_controller(px_ref);
                        y_error = y_PID_controller(py_ref);
                        z_error = z_PID_controller(pz_ref);  

                        imuComputeQuaternionFromRPY(ref_roll_angle, ref_pitch_angle, ref_yaw_angle, &quaternion_ref);   //update quaternion_ref, input(degree)
    #ifdef POSITION_TUNING                       
                        quaternion_ref.w = quat_hold.w;
                        quaternion_ref.x = quat_hold.x;
                        quaternion_ref.y = quat_hold.y;
                        quaternion_ref.z = quat_hold.z;
                        px_ref = 0.0f;
                        py_ref = 0.0f;
                        pz_ref = 0.0f;
                        magHold = 270;  // NEED TO CHOOSE YOUR OWN magHold(flapping wing facing) ,since every setup was different!
                        align_MOCAP_coordinat_to_earth_coordinate(&px_ref,&py_ref);
                        x_error = x_PID_controller(px_ref);
                        y_error = y_PID_controller(py_ref);
                        z_error = z_PID_controller(pz_ref);
    #endif
                        quaternion_ref_to_RotationMatrix(&quaternion_ref);      //update rotation matrix of quaternion_ref(qMat)
                        position_controller(x_error, y_error, z_error);      //input euler_angle_ref and position_ref, output quaternion_desired
                        cal_body_velocity(cT);
                        thrust_cmd = thrust_controller(bvx_ref, pz_ref);
                    }
#endif

//========= attitude controller: =========//
#ifdef FLY_BY_ATTITUDE_CONTROL
                    imuComputeQuaternionFromRPY(ref_roll_angle, ref_pitch_angle, ref_yaw_angle, &desired_quaternion);
#endif

                    calculate_quaternion_error();   //input quaternion_desired , output quaternion_error      
                    roll_cmd = roll_quaternion_controller();
                    pitch_cmd = pitch_quaternion_controller();
                    yaw_cmd = yaw_quaternion_controller();
                    thrust_cmd = z_baro_PID_controller(60); //input cm
                    bounding(&thrust_cmd ,1900,1350);                		               
                    motor[0] = thrust_cmd - yaw_cmd;
                    motor[1] = thrust_cmd + yaw_cmd;
                    servo[3] = 1500 - roll_cmd - pitch_cmd;
                    servo[4] = 1500 - roll_cmd + pitch_cmd;
                    auxState_log = 1;   //for blackbox 

#ifdef ATTITUDE_TUNING
                    //motor[0] = rcData[0];
                    //motor[1] = rcData[0];
                    //servo[3] = 1500;
                    //servo[4] = 1500;

                    motor[0] = 1650;
                    motor[1] = 1650;
                    servo[3] = 1500 - roll_cmd;
                    servo[4] = 1500 - roll_cmd;

                    //motor[0] = 1650;
                    //motor[1] = 1650;
                    //servo[3] = 1500 - pitch_cmd;
                    //servo[4] = 1500 + pitch_cmd;

                    //motor[0] = 1650 - yaw_cmd;
                    //motor[1] = 1650 + yaw_cmd;
                    //servo[3] = 1500;
                    //servo[4] = 1500;  
#endif
                    //thrsut_curve_n_point(5, 600, 1950, 1150, m1_count_store);      // use this function if you want to get thrust curve, remember to comment "voltage manager"!!!!!!                   
                }
            }
			else
			{
					flag = 0;
			}               

                if(mode2){  //horiztal mode
                    static uint32_t pT_2;
                    uint32_t cT = micros();
                    uint32_t dT;
                    dT = cT - pT_2;

                    if(mode2_trigger == 1){ //when switch to mode2
                        m2_count_store = count;
                        //magHold = lrintf(quaternionyaw/10); //reset maghold when we change back to mode2 from mode1
                        mode1_trigger = true;
                        mode2_trigger = false;
                        mode3_trigger = true;
                        flight_task = 2;    //start fly foreward
                    }

//*********** trajectory and position_controller at 20hz, attitude_controller at 200hz **************//      
//dont use these lines of code if you want to tune attitude only(quaternion_desired = quaternion_current when you armed!)
#ifndef ATTITUDE_TUNING
                    if (dT < 50000){    //50000 microsecends means 20HZ
                        //pass
                    }
                    else{         
                        pT_2 = cT;
                        float x_error,y_error,z_error;

//========= trajectory generate: =========// 
//it will update px_ref....ref_roll_angle...etc for controllers

                        if( ( quaternionpitch > (-500) ) && ( (count-m2_count_store)>200 ) ){    //if we near level fly aoa, start to bankturn
                            bankturn_task = true;
                        }
                        if(bankturn_task){
                            flight_task = 3;    // 3:left bankturn after level flight                     
                        }

                        voltage_manergor(&flight_task, vbat);
                        trajectory_generator(flight_task);

//========= position controller: =========//
                        align_MOCAP_coordinat_to_earth_coordinate(&px_ref,&py_ref); // transfer px_ref and py_ref to earth coordinate
                        x_error = x_PID_controller(px_ref);
                        y_error = y_PID_controller(py_ref);
                        z_error = z_PID_controller(pz_ref);  

                        imuComputeQuaternionFromRPY(ref_roll_angle, ref_pitch_angle, ref_yaw_angle, &quaternion_ref);       //update quaternion_ref, input(degree)
    #ifdef POSITION_TUNING                       
                        quaternion_ref.w = quat_hold.w;
                        quaternion_ref.x = quat_hold.x;
                        quaternion_ref.y = quat_hold.y;
                        quaternion_ref.z = quat_hold.z;
                        px_ref = 0.0f;
                        py_ref = 0.0f;
                        pz_ref = 0.0f;
                        magHold = 270;  // NEED TO CHOOSE YOUR OWN magHold(flapping wing facing) ,since every setup was different!
                        align_MOCAP_coordinat_to_earth_coordinate(&px_ref,&py_ref);
                        x_error = x_PID_controller(px_ref);
                        y_error = y_PID_controller(py_ref);
                        z_error = z_PID_controller(pz_ref);
    #endif
                        quaternion_ref_to_RotationMatrix(&quaternion_ref);      //update rotation matrix of quaternion_ref(qMat)
                        position_controller(x_error, y_error, z_error);      //input euler_angle_ref and position_ref, output quaternion_desired
                        cal_body_velocity(cT);

//========= thrust controller: =========//
                        thrust_cmd = thrust_controller(bvx_ref, pz_ref);
                    }
#endif


//========= attitude controller: =========//
#ifdef FLY_BY_ATTITUDE_CONTROL
                    imuComputeQuaternionFromRPY(ref_roll_angle, ref_pitch_angle, ref_yaw_angle, &desired_quaternion);
#endif

                    calculate_quaternion_error();   //input quaternion_desired , output quaternion_error      
                    roll_cmd = roll_quaternion_controller();
                    pitch_cmd = pitch_quaternion_controller();
                    yaw_cmd = yaw_quaternion_controller();
                    thrust_cmd = z_baro_PID_controller(60); //input cm
                    bounding(&thrust_cmd ,1900,1350);     

                    motor[0] = thrust_cmd - yaw_cmd;
                    motor[1] = thrust_cmd + yaw_cmd;
                    servo[3] = 1500 - roll_cmd - pitch_cmd;
                    servo[4] = 1500 - roll_cmd + pitch_cmd;

                    auxState_log = 2;   //for blackbox
                }

				if ( landing ){     //manual landing
                    static uint32_t pT_3;
                    uint32_t cT = micros();
                    uint32_t dT;
                    dT = cT - pT_3;

                    if(mode3_trigger == 1){ //when switch to mode3(landing)
                        m3_count_store = count;
                        mode1_trigger = true;
                        mode2_trigger = true;
                        mode3_trigger = false;
                        ref_roll_angle = 0;
                        ref_pitch_angle = -90;  //hover
                        ref_yaw_angle = -magHold;   //reset maghold when we change back to landing from mode1
                        px_ref = last_x;    //set desired x
                        py_ref = last_y;    //set desired y
                        pz_ref = 0.6f;    //set desired height
                        bvx_ref = 0.0f;   //body_x velocity ref
                    }

                    pz_ref = pz_ref - 0.0005f;   //decrease desired altitude(200hz)
                    if(pz_ref<0.1){
                        pz_ref = 0.1f;
                    }

//*********** trajectory and position_controller at 20hz, attitude_controller at 200hz **************//                
//dont use these lines of code if you want to tune attitude only(quaternion_desired = quaternion_current when you armed!)
//dont need voltage manegor and trajectory generator since we already start landing
#ifndef ATTITUDE_TUNING
                    if (dT < 50000){    //50000 microsecends means 20HZ
                        //pass
                    }
                    else{                        
                        pT_3 = cT;
                        float x_error,y_error,z_error;

//========= position controller: =========//   
                        align_MOCAP_coordinat_to_earth_coordinate(&px_ref,&py_ref);
                        x_error = x_PID_controller(px_ref);
                        y_error = y_PID_controller(py_ref);
                        z_error = z_PID_controller(pz_ref);  

                        imuComputeQuaternionFromRPY(ref_roll_angle, ref_pitch_angle, ref_yaw_angle, &quaternion_ref);       //update quaternion_ref, input(degree)
                        quaternion_ref_to_RotationMatrix(&quaternion_ref);      //update rotation matrix of quaternion_ref(qMat)
                        position_controller(x_error, y_error, z_error);      //input euler_angle_ref and position_ref, output quaternion_desired
                        cal_body_velocity(cT);

//========= thrust controller: =========//
                        thrust_cmd = thrust_controller(bvx_ref, pz_ref);
                    }
#endif

//========= attitude controller: =========//
#ifdef FLY_BY_ATTITUDE_CONTROL
                    imuComputeQuaternionFromRPY(ref_roll_angle, ref_pitch_angle, ref_yaw_angle, &desired_quaternion);
#endif

                    calculate_quaternion_error();   //input quaternion_desired , output quaternion_error      
                    roll_cmd = roll_quaternion_controller();
                    pitch_cmd = pitch_quaternion_controller();
                    yaw_cmd = yaw_quaternion_controller();
                    thrust_cmd = z_baro_PID_controller( lrintf(pz_ref*100.0f) ); //input cm
                    bounding(&thrust_cmd ,1900,1350);                		               
                    motor[0] = thrust_cmd - yaw_cmd;
                    motor[1] = thrust_cmd + yaw_cmd;
                    servo[3] = 1500 - roll_cmd - pitch_cmd;
                    servo[4] = 1500 - roll_cmd + pitch_cmd;
                    auxState_log = 3;   //for blackbox
                    
                    if( (tmp_pz-OZ) < 20){   //close enought to the ground, then stop 
                        stop = true;
                    }
				}

				if ( stop ) {
					  motor[0] = 1100;
                      motor[1] = 1100;
					  servo[3] = 1500;
		  			  servo[4] = 1500;
					  //servo[5] = 1500;
				}

        last_x = (float)tmp_px/100.0f;    //store last position for swiching different flight modes
        last_y = (float)tmp_py/100.0f;

        bounding(&motor[0], 1850, 1100);        //saturate pwm output, max:1950 but it will damage flapping wing
        bounding(&motor[1], 1850, 1100);
        bounding(&servo[3], 2300, 700);
        bounding(&servo[4], 2300, 700);

		store_pwm[0] = motor[0];
        store_pwm[1] = motor[1];
		store_pwm[2] = servo[3];
		store_pwm[3] = servo[4];
		count++;

        writeServos();
        writeMotors();

#ifdef BLACKBOX
        handleBlackbox();   //200hz
#endif

    //printf("qr = %d ",quaternionroll);
    //printf("qp = %d ",quaternionpitch);
    //printf("qy = %d ",quaternionyaw);
    //printf("   \r\n"); 
    //printf("qw = %d ",lrintf(1000*q.w));
    //printf("qx = %d ",lrintf(1000*q.x));
    //printf("qy = %d ",lrintf(1000*q.y));
    //printf("qz = %d ",lrintf(1000*q.z));
    //printf("   \r\n");    
    //printf("rr = %d ",ref_roll_angle);
    //printf("rp = %d ",ref_pitch_angle);
    //printf("rp = %d ",ref_yaw_angle);
    //printf("   \r\n");

	} 
     
#ifdef WIFI 
    //get optitrack data(x,y,z) by wifi
    getwifidata();
    update_wifidata();  //40hz update rate
    wifi_ledblink();
#endif

    //printf("x = %d ",wifidata_x);
    //printf("y = %d ",wifidata_y);
    //printf("z = %d ",wifidata_z);
    //printf("px = %d ",tmp_px);
    //printf("py = %d ",tmp_py);
    //printf("pz = %d ",tmp_pz);
    //printf("loop = %d ",wifiloop);
    //printf("   \r\n");         
}
