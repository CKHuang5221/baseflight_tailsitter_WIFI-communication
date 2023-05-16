#include "board.h"
#include "mw.h"

#define BARO
#define MAG
#define sq(x) ((x)*(x)) //add at 20221007

int16_t gyroADC[3], accADC[3], accSmooth[3], magADC[3];
int32_t accSum[3];
uint32_t accTimeSum = 0;        // keep track for integration of acc
int accSumCount = 0;
int16_t smallAngle = 0;
int32_t baroPressure = 0;
int32_t baroTemperature = 0;
uint32_t baroPressureSum = 0;
int32_t BaroAlt = 0;
float sonarTransition = 0;
int32_t baroAlt_offset = 0;
int32_t sonarAlt = -1;         // in cm , -1 indicate sonar is not in range 
int32_t EstAlt;                // in cm
int32_t BaroPID = 0;
int32_t AltHold;
int32_t setVelocity = 0;
uint8_t velocityControl = 0;
int32_t errorVelocityI = 0;
int32_t vario = 0;                      // variometer in cm/s
int16_t throttleAngleCorrection = 0;    // correction of throttle in lateral wind,
float magneticDeclination = 0.0f;       // calculated at startup from config
float accVelScale;
float throttleAngleScale;
float fc_acc;

//--------------- FOR QUATERNION start----------------//
int32_t asum[3];
float aavg[3];
int32_t gysum[3];
float gyavg[3];
int32_t accgyrocount = 1;
int16_t quaternionyaw;
int16_t quaternionpitch;
int16_t quaternionroll;
//int32_t getaccgyrosumcheck = 0;
bool acc_check;
bool mag_check;
int8_t use_mag_check = 0;
int8_t use_acc_check = 0;

//--------------- FOR QUATERNION end------------------//

float redefineangle = 0;            //add at 20221007
int16_t accADCtmp[3] = { 0, 0, 0 };
int16_t gyroADCtmp[3] = { 0, 0, 0 };
int16_t magADCtmp[3] = { 0, 0, 0 };
int16_t acctmpADC;
int16_t gyrotmpADC;
int16_t magtmpADC;

// **************
// gyro+acc IMU
// **************
int16_t gyroData[3] = { 0, 0, 0 };
int16_t gyroZero[3] = { 0, 0, 0 };
int16_t angle[2] = { 0, 0 };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
float anglerad[2] = { 0.0f, 0.0f };    // absolute angle inclination in radians

static void getEstimatedAttitude(void);

void imuInit(void)
{
    smallAngle = lrintf(acc_1G * cosf(RAD * cfg.small_angle));
    accVelScale = 9.80665f / acc_1G / 10000.0f;
    throttleAngleScale = (1800.0f / M_PI) * (900.0f / cfg.throttle_correction_angle);
    
    fc_acc = 0.5f / (M_PI * cfg.accz_lpf_cutoff); // calculate RC time constant used in the accZ lpf

#ifdef MAG
    // if mag sensor is enabled, use it
    if (sensors(SENSOR_MAG))
        Mag_init();
#endif
}

void computeIMU(void)
{
    static int16_t gyroYawSmooth = 0;

    Gyro_getADC();
    if (sensors(SENSOR_ACC)) {
        ACC_getADC();
        getEstimatedAttitude();
    } else {
        accADC[X] = 0;
        accADC[Y] = 0;
        accADC[Z] = 0;
    }

    //if (mcfg.mixerConfiguration == MULTITYPE_TRI) {
        gyroData[YAW] = (gyroYawSmooth * 2 + gyroADC[YAW]) / 3;
        gyroYawSmooth = gyroData[YAW];
    //} else {
    //    gyroData[YAW] = gyroADC[YAW];
    //}
    gyroData[ROLL] = gyroADC[ROLL];
    gyroData[PITCH] = gyroADC[PITCH];
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

#define INV_GYR_CMPF_FACTOR   (1.0f / ((float)mcfg.gyro_cmpf_factor + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / ((float)mcfg.gyro_cmpfm_factor + 1.0f))

/*typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;*/

t_fp_vector EstG;

// Normalize a vector
void normalizeV(struct fp_vector *src, struct fp_vector *dest)
{
    float length;

    length = sqrtf(src->X * src->X + src->Y * src->Y + src->Z * src->Z);
    if (length != 0) {
        dest->X = src->X / length;
        dest->Y = src->Y / length;
        dest->Z = src->Z / length;
    }
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float *delta)
{
    struct fp_vector v_tmp = *v;

    // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(delta[ROLL]);
    sinx = sinf(delta[ROLL]);
    cosy = cosf(delta[PITCH]);
    siny = sinf(delta[PITCH]);
    cosz = cosf(delta[YAW]);
    sinz = sinf(delta[YAW]);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = cosz * cosy;
    mat[0][1] = -cosy * sinz;
    mat[0][2] = siny;
    mat[1][0] = sinzcosx + (coszsinx * siny);
    mat[1][1] = coszcosx - (sinzsinx * siny);
    mat[1][2] = -sinx * cosy;
    mat[2][0] = (sinzsinx) - (coszcosx * siny);
    mat[2][1] = (coszsinx) + (sinzcosx * siny);
    mat[2][2] = cosy * cosx;

    v->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
}

int32_t applyDeadband(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

// rotate acc into Earth frame and calculate acceleration in it
void acc_calc(uint32_t deltaT)
{
    static int32_t accZoffset = 0;
    static float accz_smooth = 0;
    float dT = 0;
    float rpy[3];

    t_fp_vector accel_ned;
    

    // deltaT is measured in us ticks
    dT = (float)deltaT * 1e-6f;

    // the accel values have to be rotated into the earth frame
    rpy[0] = -(float)anglerad[ROLL];
    rpy[1] = -(float)anglerad[PITCH];
    rpy[2] = -(float)heading * RAD;

    accel_ned.V.X = accSmooth[0];
    accel_ned.V.Y = accSmooth[1];
    accel_ned.V.Z = accSmooth[2];

    rotateV(&accel_ned.V, rpy);

    if (cfg.acc_unarmedcal == 1) {
        if (!f.ARMED) {
            accZoffset -= accZoffset / 64;
            accZoffset += accel_ned.V.Z;
        }
        accel_ned.V.Z -= accZoffset / 64;  // compensate for gravitation on z-axis
    } else
        accel_ned.V.Z -= acc_1G;

    accz_smooth = accz_smooth + (dT / (fc_acc + dT)) * (accel_ned.V.Z - accz_smooth); // low pass filter

    // apply Deadband to reduce integration drift and vibration influence and
    // sum up Values for later integration to get velocity and distance
    accSum[X] += applyDeadband(lrintf(accel_ned.V.X), cfg.accxy_deadband);
    accSum[Y] += applyDeadband(lrintf(accel_ned.V.Y), cfg.accxy_deadband);
    accSum[Z] += applyDeadband(lrintf(accz_smooth), cfg.accz_deadband);
    
    accTimeSum += deltaT;
    accSumCount++;
}

void accSum_reset(void)
{
    accSum[0] = 0;
    accSum[1] = 0;
    accSum[2] = 0;
    accSumCount = 0;
    accTimeSum = 0;
}

// baseflight calculation by Luggi09 originates from arducopter
static int16_t calculateHeading(t_fp_vector *vec)
{
    int16_t head;

    float cosineRoll = cosf(anglerad[ROLL]);
    float sineRoll = sinf(anglerad[ROLL]);
    float cosinePitch = cosf(anglerad[PITCH]);
    float sinePitch = sinf(anglerad[PITCH]);
    float Xh = vec->A[X] * cosinePitch + vec->A[Y] * sineRoll * sinePitch + vec->A[Z] * sinePitch * cosineRoll;
    float Yh = vec->A[Y] * cosineRoll - vec->A[Z] * sineRoll;
    float hd = (atan2f(Yh, Xh) * 1800.0f / M_PI + magneticDeclination) / 10.0f;
    head = lrintf(hd);
    if (head < 0)
        head += 360;

    return head;
}

static void getEstimatedAttitude(void)
{
    int32_t axis;
    int32_t accMag = 0;
    static t_fp_vector EstM;
    static t_fp_vector EstN = { .A = { 1.0f, 0.0f, 0.0f } };
    static float accLPF[3];
    static uint32_t previousT;
    uint32_t currentT = micros();
    uint32_t deltaT;
    float scale, deltaGyroAngle[3];
    deltaT = currentT - previousT;
    scale = deltaT * gyro.scale;
    previousT = currentT;
    static float pitchsmooth, rollsmoooth;
		//static int16_t headsmooth;

    // Initialization
    for (axis = 0; axis < 3; axis++) {
        deltaGyroAngle[axis] = gyroADC[axis] * scale;
        if (cfg.acc_lpf_factor > 0) {
            accLPF[axis] = accLPF[axis] * (1.0f - (1.0f / cfg.acc_lpf_factor)) + accADC[axis] * (1.0f / cfg.acc_lpf_factor);
            accSmooth[axis] = accLPF[axis];
        } else {
            accSmooth[axis] = accADC[axis];
        }
        accMag += (int32_t)accSmooth[axis] * accSmooth[axis];
    }
    accMag = accMag * 100 / ((int32_t)acc_1G * acc_1G);

    rotateV(&EstG.V, deltaGyroAngle);

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    if (72 < (uint16_t)accMag && (uint16_t)accMag < 133) {
        for (axis = 0; axis < 3; axis++)
            EstG.A[axis] = (EstG.A[axis] * (float)mcfg.gyro_cmpf_factor + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }

    //f.SMALL_ANGLE = (EstG.A[Z] > smallAngle);
    f.SMALL_ANGLE = 1;

    // Attitude of the estimated vector
    anglerad[ROLL] = atan2f(EstG.V.Y, EstG.V.Z);
    anglerad[PITCH] = atan2f(-EstG.V.X, sqrtf(EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z));
		anglerad[ROLL] = (rollsmoooth*4+anglerad[ROLL])/5;
		anglerad[PITCH] = (pitchsmooth*9+anglerad[PITCH])/10;
    angle[ROLL] = lrintf(anglerad[ROLL] * (1800.0f / M_PI));
    angle[PITCH] = lrintf(anglerad[PITCH] * (1800.0f / M_PI));
		rollsmoooth = anglerad[ROLL];
		pitchsmooth = anglerad[PITCH];

    if (sensors(SENSOR_MAG)) {
        rotateV(&EstM.V, deltaGyroAngle);
        for (axis = 0; axis < 3; axis++)
            EstM.A[axis] = (EstM.A[axis] * (float)mcfg.gyro_cmpfm_factor + magADC[axis]) * INV_GYR_CMPFM_FACTOR;
        heading = calculateHeading(&EstM);
			  //heading = (headsmooth*4 + heading)/5;
			  //headsmooth = heading;
    } else {
        rotateV(&EstN.V, deltaGyroAngle);
        normalizeV(&EstN.V, &EstN.V);
        heading = calculateHeading(&EstN);
    }

    acc_calc(deltaT); // rotate acc vector into earth frame

    if (cfg.throttle_correction_value) {

        float cosZ = EstG.V.Z / sqrtf(EstG.V.X * EstG.V.X + EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z);

        if (cosZ <= 0.015f) { // we are inverted, vertical or with a small angle < 0.86 deg
            throttleAngleCorrection = 0;
        } else {
            int deg = lrintf(acosf(cosZ) * throttleAngleScale);
            if (deg > 900)
                deg = 900;
            throttleAngleCorrection = lrintf(cfg.throttle_correction_value * sinf(deg / (900.0f * M_PI / 2.0f)));
        }

    }
    
}

#ifdef BARO
#define UPDATE_INTERVAL 25000   // 40hz update rate (20hz LPF on acc)

int getEstimatedAltitude(void)
{
    static uint32_t previousT;
    uint32_t currentT = micros();
    uint32_t dTime;
    int32_t error;
    int32_t baroVel;
    int32_t vel_tmp;
    int32_t BaroAlt_tmp;
    int32_t setVel;
    float dt;
    float vel_acc;
    float accZ_tmp;
    static float accZ_old = 0.0f;
    static float vel = 0.0f;
    static float accAlt = 0.0f;
    
    static int32_t lastBaroAlt;
    static int32_t baroGroundAltitude = 0;
    static int32_t baroGroundPressure = 0;
    int16_t tiltAngle = max(abs(angle[ROLL]), abs(angle[PITCH]));

    dTime = currentT - previousT;
    if (dTime < UPDATE_INTERVAL)
        return 0;
    previousT = currentT;

    if (calibratingB > 0) {
        baroGroundPressure -= baroGroundPressure / 8;
        baroGroundPressure += baroPressureSum / (cfg.baro_tab_size - 1);
        baroGroundAltitude = (1.0f - powf((baroGroundPressure / 8) / 101325.0f, 0.190295f)) * 4433000.0f;
        vel = 0;
        accAlt = 0;
        calibratingB--;
    }

    // calculates height from ground via baro readings
    // see: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
    BaroAlt_tmp = lrintf((1.0f - powf((float)(baroPressureSum / (cfg.baro_tab_size - 1)) / 101325.0f, 0.190295f)) * 4433000.0f); // in cm
    BaroAlt_tmp -= baroGroundAltitude;
    BaroAlt = lrintf((float)BaroAlt * cfg.baro_noise_lpf + (float)BaroAlt_tmp * (1.0f - cfg.baro_noise_lpf)); // additional LPF to reduce baro noise

    // calculate sonar altitude only if the sonar is facing downwards(<25deg)
    if (tiltAngle > 250)
        sonarAlt = -1;
    else
        sonarAlt = sonarAlt * (900.0f - tiltAngle) / 900.0f;

    // do sonarAlt and baroAlt fusion
    if (sonarAlt > 0 && sonarAlt < 200) {
        baroAlt_offset = BaroAlt - sonarAlt;
        BaroAlt = sonarAlt;
    } else {
        BaroAlt -= baroAlt_offset;
        if (sonarAlt > 0) {
            sonarTransition = (300 - sonarAlt) / 100.0f;
            BaroAlt = sonarAlt * sonarTransition + BaroAlt * (1.0f - sonarTransition);
        }
    }

    dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

    // Integrator - velocity, cm/sec
    accZ_tmp = (float)accSum[2] / (float)accSumCount;
    vel_acc = accZ_tmp * accVelScale * (float)accTimeSum;

    // Integrator - Altitude in cm
    accAlt += (vel_acc * 0.5f) * dt + vel * dt;                                         // integrate velocity to get distance (x= a/2 * t^2)
    accAlt = accAlt * cfg.baro_cf_alt + (float)BaroAlt * (1.0f - cfg.baro_cf_alt);      // complementary filter for altitude estimation (baro & acc)

    // when the sonar is in his best range
    if (sonarAlt > 0 && sonarAlt < 200)
        EstAlt = BaroAlt;
    else
        EstAlt = accAlt;

    vel += vel_acc; 

#if 0
    debug[0] = accSum[2] / accSumCount; // acceleration
    debug[1] = vel;                     // velocity
    debug[2] = accAlt;                  // height
#endif

    accSum_reset();

    baroVel = (BaroAlt - lastBaroAlt) * 1000000.0f / dTime;
    lastBaroAlt = BaroAlt;

    baroVel = constrain(baroVel, -1500, 1500);    // constrain baro velocity +/- 1500cm/s
    baroVel = applyDeadband(baroVel, 10);         // to reduce noise near zero

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * cfg.baro_cf_vel + baroVel * (1 - cfg.baro_cf_vel);
    vel_tmp = lrintf(vel);


    // set vario
    vario = applyDeadband(vel_tmp, 5);

    if (tiltAngle < 800) { // only calculate pid if the copters thrust is facing downwards(<80deg)
        // Altitude P-Controller
        if (!velocityControl) {
            error = constrain(AltHold - EstAlt, -500, 500);
            error = applyDeadband(error, 10);       // remove small P parametr to reduce noise near zero position
            setVel = constrain((cfg.P8[PIDALT] * error / 128), -300, +300); // limit velocity to +/- 3 m/s
        } else {
            setVel = setVelocity;
        }
        
        // Velocity PID-Controller
        // P
        error = setVel - vel_tmp;
        BaroPID = constrain((cfg.P8[PIDVEL] * error / 32), -300, +300);

        // I
        errorVelocityI += (cfg.I8[PIDVEL] * error);
        errorVelocityI = constrain(errorVelocityI, -(8196 * 200), (8196 * 200));
        BaroPID += errorVelocityI / 8196;     // I in the range of +/-200

        // D
        BaroPID -= constrain(cfg.D8[PIDVEL] * (accZ_tmp + accZ_old) / 512, -150, 150);

    } else {
        BaroPID = 0;
    }

    accZ_old = accZ_tmp;
    return 1;
}
#endif /* BARO */

//----------------------------------------------------------------------20220913 add quaternion-----------------------------------------------------------------------//
// from cleanflight imu.c and CSDN (https://blog.csdn.net/lqj11/article/details/107423334)

void getaccgyro_sum_and_avg (void){         //  200hz update rate

    if(accgyrocount /5 == 1){
        asum[X] = accSmooth[X];
        asum[Y] = accSmooth[Y];
        asum[Z] = accSmooth[Z];
        gysum[X] = applyDeadband(gyroADC[X],7);
        gysum[Y] = applyDeadband(gyroADC[Y],1);
        gysum[Z] = applyDeadband(gyroADC[Z],1);
        accgyrocount = 1;

    }else{
        asum[X] += accSmooth[X];
        asum[Y] += accSmooth[Y];
        asum[Z] += accSmooth[Z];
        gysum[X] += applyDeadband(gyroADC[X],7);
        gysum[Y] += applyDeadband(gyroADC[Y],1);
        gysum[Z] += applyDeadband(gyroADC[Z],1);

    }

    aavg[X] = (float)(asum[X]/accgyrocount)/4096.0f * 9.8f;        //convert to (m/s^2)
    aavg[Y] = (float)(asum[Y]/accgyrocount)/4096.0f * 9.8f;
    aavg[Z] = (float)(asum[Z]/accgyrocount)/4096.0f * 9.8f;

    gyavg[X] = (float)(gysum[X]/accgyrocount)*(4.0f / 16.4f);  //remember convert to (rad/s) when AHRSupdate's input (16.4 dps/lsb, baseflight在獲取繳速度adc時除以4，這裡要*4回來，看baseflight sensor.c)
    gyavg[Y] = (float)(gysum[Y]/accgyrocount)*(4.0f / 16.4f);
    gyavg[Z] = (float)(gysum[Z]/accgyrocount)*(4.0f / 16.4f);
    
    accgyrocount ++;
    //getaccgyrosumcheck ++;
}

//------------------------------------------define math stuff start --------------------------------------------------//

#define M_PIf       3.14159265358979323846f
#define sinPolyCoef3 -1.666568107e-1f
#define sinPolyCoef5  8.312366210e-3f
#define sinPolyCoef7 -1.849218155e-4f
#define sinPolyCoef9  0

float sin_approx(float x)
{
    int32_t xint = x;
    if (xint < -32 || xint > 32) return 0.0f;                               // Stop here on error input (5 * 360 Deg)
    while (x >  M_PIf) x -= (2.0f * M_PIf);                                 // always wrap input angle to -PI..PI
    while (x < -M_PIf) x += (2.0f * M_PIf);
    if (x >  (0.5f * M_PIf)) x =  (0.5f * M_PIf) - (x - (0.5f * M_PIf));   // We just pick -90..+90 Degree
    else if (x < -(0.5f * M_PIf)) x = -(0.5f * M_PIf) - ((0.5f * M_PIf) + x);
    float x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float cos_approx(float x)
{
    return sin_approx(x + (0.5f * M_PIf));
}

// Initial implementation by Crashpilot1000 (https://github.com/Crashpilot1000/HarakiriWebstore1/blob/396715f73c6fcf859e0db0f34e12fe44bace6483/src/mw.c#L1292)
// Polynomial coefficients by Andor (http://www.dsprelated.com/showthread/comp.dsp/21872-1.php) optimized by Ledvinap to save one multiplication
// Max absolute error 0,000027 degree
// atan2_approx maximum absolute error = 7.152557e-07 rads (4.098114e-05 degree)
float atan2_approx(float y, float x)
{
    #define atanPolyCoef1  3.14551665884836e-07f
    #define atanPolyCoef2  0.99997356613987f
    #define atanPolyCoef3  0.14744007058297684f
    #define atanPolyCoef4  0.3099814292351353f
    #define atanPolyCoef5  0.05030176425872175f
    #define atanPolyCoef6  0.1471039133652469f
    #define atanPolyCoef7  0.6444640676891548f

    float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = max(absX, absY);
    if (res) res = min(absX, absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (M_PIf / 2.0f) - res;
    if (x < 0) res = M_PIf - res;
    if (y < 0) res = -res;
    return res;
}

// http://http.developer.nvidia.com/Cg/acos.html
// Handbook of Mathematical Functions
// M. Abramowitz and I.A. Stegun, Ed.
// acos_approx maximum absolute error = 6.760856e-05 rads (3.873685e-03 degree)
float acos_approx(float x)
{
    float xa = fabsf(x);
    float result = sqrtf(1.0f - xa) * (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
    if (x < 0.0f)
        return M_PIf - result;
    else
        return result;
}

//invsqrt from CSDN
static float invSqrt(float x) 		//快速计算 1/Sqrt(x)
{

	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//static float invSqrt(float x)
//{
//    return 1.0f / sqrtf(x);
//}

//版权声明：本文为CSDN博主「大写的小写字母」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
//原文链接：https://blog.csdn.net/lqj11/article/details/107423334

//----------------------------------------------define math stuff end------------------------------------------------------------//

#define SPIN_RATE_LIMIT 20
#define ATTITUDE_RESET_QUIET_TIME 250000   // 250ms - gyro quiet period after disarm before attitude reset
#define ATTITUDE_RESET_GYRO_LIMIT 15       // 15 deg/sec - gyro limit for quiet period
#define ATTITUDE_RESET_KP_GAIN    25.0     // dcmKpGain value to use during attitude reset
#define ATTITUDE_RESET_ACTIVE_TIME 500000  // 500ms - Time to wait for attitude to converge at high gain
#define GPS_COG_MIN_GROUNDSPEED 500        // 500cm/s minimum groundspeed for a gps heading to be considered valid

#define sq(x) ((x)*(x))

static imuRuntimeConfig_t imuRuntimeConfig;
mag_t mag_quat;                   // mag access functions

float accAverage[XYZ_AXIS_COUNT];
float rMat[3][3];

// quaternion of sensor frame relative to earth frame
quaternion q = QUATERNION_INITIALIZE;
quaternionProducts qP = QUATERNION_PRODUCTS_INITIALIZE;
// headfree quaternions
quaternion headfree = QUATERNION_INITIALIZE;
quaternion offset = QUATERNION_INITIALIZE;

attitudeEulerAngles_t attitude = EULER_INITIALIZE;

void imuComputeRotationMatrix(void){    //body to earth
    imuQuaternionComputeProducts(&q, &qP);

    rMat[0][0] = 1.0f - 2.0f * qP.yy - 2.0f * qP.zz;
    rMat[0][1] = 2.0f * (qP.xy + -qP.wz);
    rMat[0][2] = 2.0f * (qP.xz - -qP.wy);

    rMat[1][0] = 2.0f * (qP.xy - -qP.wz);
    rMat[1][1] = 1.0f - 2.0f * qP.xx - 2.0f * qP.zz;
    rMat[1][2] = 2.0f * (qP.yz + -qP.wx);

    rMat[2][0] = 2.0f * (qP.xz + -qP.wy);
    rMat[2][1] = 2.0f * (qP.yz - -qP.wx);
    rMat[2][2] = 1.0f - 2.0f * qP.xx - 2.0f * qP.yy;
}

static void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az,
                                bool useMag, float mx, float my, float mz,
                                bool useCOG, float courseOverGround)
{
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki
    float dcmKpGain = 2.5f;
    float dcmKiGain  = 0.3f;
    float dcm_ki = 0.3; //等於0則不使用ki
    acc_check = useAcc;
    mag_check = useMag;

    // Calculate general spin rate (rad/s)
    const float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));

    // Use raw heading error (from GPS or whatever else)
    float ex = 0, ey = 0, ez = 0;
    if (useCOG) {
        while (courseOverGround >  M_PIf) {
            courseOverGround -= (2.0f * M_PIf);
        }

        while (courseOverGround < -M_PIf) {
            courseOverGround += (2.0f * M_PIf);
        }

        const float ez_ef = (- sin_approx(courseOverGround) * rMat[0][0] - cos_approx(courseOverGround) * rMat[1][0]);

        ex = rMat[2][0] * ez_ef;
        ey = rMat[2][1] * ez_ef;
        ez = rMat[2][2] * ez_ef;
    }

#ifdef MAG
    // Use measured magnetic field vector
    float recipMagNorm = sq(mx) + sq(my) + sq(mz);
    if (useMag && recipMagNorm > 0.01f) {
        // Normalise magnetometer measurement
        recipMagNorm = invSqrt(recipMagNorm);
        mx *= recipMagNorm;
        my *= recipMagNorm;
        mz *= recipMagNorm;

        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles

        // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
        // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
        const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
        const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
        const float bx = sqrtf(hx * hx + hy * hy);

        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        const float ez_ef = -(hy * bx);

        // Rotate mag error vector back to BF and accumulate
        ex += rMat[2][0] * ez_ef;
        ey += rMat[2][1] * ez_ef;
        ez += rMat[2][2] * ez_ef;

    }
#endif

    // Use measured acceleration vector
    float recipAccNorm = sq(ax) + sq(ay) + sq(az);
    if (useAcc && recipAccNorm > 0.01f) {
        // Normalise accelerometer measurement
        recipAccNorm = invSqrt(recipAccNorm);
        ax *= recipAccNorm;
        ay *= recipAccNorm;
        az *= recipAccNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * rMat[2][2] - az * rMat[2][1]);
        ey += (az * rMat[2][0] - ax * rMat[2][2]);
        ez += (ax * rMat[2][1] - ay * rMat[2][0]);
    }

    // Compute and apply integral feedback if enabled
    if (dcm_ki > 0.0f) {
        // Stop integrating if spinning beyond the certain limit
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
            const float dcmKiGain = imuRuntimeConfig.dcm_ki;
            integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    } else {
        integralFBx = 0.0f;    // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Apply proportional and integral feedback
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    quaternion buffer;
    buffer.w = q.w;
    buffer.x = q.x;
    buffer.y = q.y;
    buffer.z = q.z;

    q.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
    q.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
    q.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
    q.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);

    // Normalise quaternion
    float recipNorm = invSqrt(sq(q.w) + sq(q.x) + sq(q.y) + sq(q.z));
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();
}

void imuUpdateEulerAngles(void) //output euler angle sequence is 'ZYX'
{
       attitude.values.roll = lrintf(atan2_approx(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf));
       attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat[2][0])) * (1800.0f / M_PIf));
       attitude.values.yaw = lrintf((-atan2_approx(rMat[1][0], rMat[0][0]) * (1800.0f / M_PIf)));

    if (attitude.values.yaw < 0)
        attitude.values.yaw += 3600;
}

static void imuCalculateEstimatedAttitude(timeUs_t currentTimeUs)
{
    //timeUs_t currentTimeUs = micros();
    static timeUs_t previousIMUUpdateTime;
    bool useAcc = true;
    bool useMag = true;
    bool useCOG = false; // Whether or not correct yaw via imuMahonyAHRSupdate from our ground course
    float courseOverGround = 0; // To be used when useCOG is true.  Stored in Radians

    const timeDelta_t deltaT = currentTimeUs - previousIMUUpdateTime;
    previousIMUUpdateTime = currentTimeUs;

#ifdef USE_MAG
    if (sensors(SENSOR_MAG) && compassIsHealthy()) {
        useMag = true;
    }
#endif
#if defined(USE_GPS)
    if (!useMag && sensors(SENSOR_GPS) && STATE(GPS_FIX) && gpsSol.numSat >= 5 && gpsSol.groundSpeed >= GPS_COG_MIN_GROUNDSPEED) {
        // Use GPS course over ground to correct attitude.values.yaw
        if (STATE(FIXED_WING)) {
            courseOverGround = DECIDEGREES_TO_RADIANS(gpsSol.groundCourse);
            useCOG = true;
        } else {
            // If GPS rescue mode is active and we can use it, go for it.  When we're close to home we will
            // probably stop re calculating GPS heading data.  Other future modes can also use this extern

            if (canUseGPSHeading) {
                courseOverGround = DECIDEGREES_TO_RADIANS(gpsSol.groundCourse);

                useCOG = true;
            }
        }

        if (useCOG && shouldInitializeGPSHeading()) {
            // Reset our reference and reinitialize quaternion.  This will likely ideally happen more than once per flight, but for now,
            // shouldInitializeGPSHeading() returns true only once.
            imuComputeQuaternionFromRPY(&qP, attitude.values.roll, attitude.values.pitch, gpsSol.groundCourse);

            useCOG = false; // Don't use the COG when we first reinitialize.  Next time around though, yes.
        }
    }
#endif

    getaccgyro_sum_and_avg();

    imuMahonyAHRSupdate(deltaT * 1e-6f,
                        DEGREES_TO_RADIANS(gyavg[X]), DEGREES_TO_RADIANS(gyavg[Y]), DEGREES_TO_RADIANS(gyavg[Z]),
                        useAcc, aavg[X], aavg[Y], aavg[Z],
                        useMag, magADC[X], magADC[Y], magADC[Z],
                        useCOG, courseOverGround);
    imuUpdateEulerAngles();
}


void imuUpdateAttitude(timeUs_t currentTimeUs)
{
    if (sensors(SENSOR_ACC)) {

        imuCalculateEstimatedAttitude(currentTimeUs);

    }          
}

void imuQuaternionComputeProducts(quaternion *quat, quaternionProducts *quatProd)
{
    quatProd->ww = quat->w * quat->w;
    quatProd->wx = quat->w * quat->x;
    quatProd->wy = quat->w * quat->y;
    quatProd->wz = quat->w * quat->z;
    quatProd->xx = quat->x * quat->x;
    quatProd->xy = quat->x * quat->y;
    quatProd->xz = quat->x * quat->z;
    quatProd->yy = quat->y * quat->y;
    quatProd->yz = quat->y * quat->z;
    quatProd->zz = quat->z * quat->z;
}

void use_quaternion_angle(void){
    quaternionpitch = attitude.values.pitch;
    quaternionroll = attitude.values.roll;
    quaternionyaw = attitude.values.yaw;
}

