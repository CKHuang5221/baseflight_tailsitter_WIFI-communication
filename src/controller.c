
#include "board.h"
#include "mw.h"

#define M_PI_2		1.57079632679489661923  //add at 20221007
#define M_PI_4		0.78539816339744830962

float qMat[3][3];
quaternion quaternion_ref =  QUATERNION_INITIALIZE;  //quaternion desired from desired euler
quaternion desired_quaternion =  QUATERNION_INITIALIZE;  //quaternion desired from desired euler
quaternionProducts q_ref_P = QUATERNION_PRODUCTS_INITIALIZE;

float correct_angle_x;
float correct_angle_y;
float correct_angle_z;
float max_correct_angle_y = DEGREES_TO_RADIANS(15);
float max_correct_angle_z = DEGREES_TO_RADIANS(15);
int16_t body_vx_datalog;
int16_t cx_datalog;
int16_t cy_datalog;
int16_t cz_datalog;

void bounding(int16_t *input, int upper_bound, int lower_bound){
    if(*input >= upper_bound){
        *input = upper_bound;
    }
    else if(*input < lower_bound){
        *input = lower_bound;
    }
    else{
        *input = *input;
    }
}

//================================== POSITION_controller (20hz)======================================//

float x_PID_controller(float cmd)
{
    float kp = 0.12f;
    float ki = 0.0f;
    float kd = 0.001f;
	static float error_i_x, last_error_x;
	float error_d_x, error;
    float output;

    error = cmd - rigidbody_x;
	error_i_x = error_i_x + error * 0.005;
    error_d_x = (error - last_error_x) / 0.005;
    output = error * kp + error_i_x * ki + error_d_x * kd;
	last_error_x = error;
    return output;	
}

float y_PID_controller(float cmd)
{
    float kp = 0.12f;
    float ki = 0.0f;
    float kd = 0.001f;
	static float error_i_y, last_error_y;
	float error_d_y, error;
    float output;

	error = cmd - rigidbody_y;
	error_i_y = error_i_y + error * 0.005;
    error_d_y = (error - last_error_y) / 0.005;
    output = error * kp + error_i_y * ki + error_d_y * kd;
	last_error_y = error;
    return output;	
}

float z_PID_controller(float cmd)
{
    float kp = 0.12f;
    float ki = 0.0f;
    float kd = 0.001f;
	static float error_i_z, last_error_z;
	float error_d_z, error;
    float output;

    error = cmd - (rigidbody_z-original_z);    //without imu, just optitrack
	error_i_z = error_i_z + error * 0.005;
    error_d_z = (error - last_error_z) / 0.005;
    output = error * kp + error_i_z * ki + error_d_z * kd;
	last_error_z = error;
    return output;	
}

void imuQuaternionMultiplication(quaternion *q1, quaternion *q2, quaternion *result)    //q1*q2
{   
    const float A = (q1->w + q1->x) * (q2->w + q2->x);
    const float B = (q1->z - q1->y) * (q2->y - q2->z);
    const float C = (q1->w - q1->x) * (q2->y + q2->z);
    const float D = (q1->y + q1->z) * (q2->w - q2->x);
    const float E = (q1->x + q1->z) * (q2->x + q2->y);
    const float F = (q1->x - q1->z) * (q2->x - q2->y);
    const float G = (q1->w + q1->y) * (q2->w - q2->z);
    const float H = (q1->w - q1->y) * (q2->w + q2->z);

    result->w = B + (- E - F + G + H) / 2.0f;
    result->x = A - (+ E + F + G + H) / 2.0f;
    result->y = C + (+ E - F + G - H) / 2.0f;
    result->z = D + (+ E - F - G + H) / 2.0f;
}

void imuComputeQuaternionFromRPY(int16_t initialRoll, int16_t initialPitch, int16_t initialYaw, quaternion *q){ //input euler angle sequence must be 'ZYX'!!!
    if (initialRoll > 180) {
        initialRoll -= 360;
    }
    if (initialPitch > 180) {
        initialPitch -= 360;
    }
    if (initialYaw > 180) {
        initialYaw -= 360;
    }
    const float cosRoll = cos_approx(DEGREES_TO_RADIANS(initialRoll) * 0.5f);
    const float sinRoll = sin_approx(DEGREES_TO_RADIANS(initialRoll) * 0.5f);
    const float cosPitch = cos_approx(DEGREES_TO_RADIANS(initialPitch) * 0.5f);
    const float sinPitch = sin_approx(DEGREES_TO_RADIANS(initialPitch) * 0.5f);
    const float cosYaw = cos_approx(DEGREES_TO_RADIANS(initialYaw) * 0.5f);
    const float sinYaw = sin_approx(DEGREES_TO_RADIANS(initialYaw) * 0.5f);

    const float q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    const float q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    const float q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    const float q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

    q->w = q0;
    q->x = q1;
    q->y = q2;
    q->z = q3;
}

void quaternion_ref_to_RotationMatrix(quaternion *q){
    imuQuaternionComputeProducts(q, &q_ref_P);

    qMat[0][0] = 1.0f - 2.0f * q_ref_P.yy - 2.0f * q_ref_P.zz;
    qMat[0][1] = 2.0f * (q_ref_P.xy + -q_ref_P.wz);
    qMat[0][2] = 2.0f * (q_ref_P.xz - -q_ref_P.wy);

    qMat[1][0] = 2.0f * (q_ref_P.xy - -q_ref_P.wz);
    qMat[1][1] = 1.0f - 2.0f * q_ref_P.xx - 2.0f * q_ref_P.zz;
    qMat[1][2] = 2.0f * (q_ref_P.yz + -q_ref_P.wx);

    qMat[2][0] = 2.0f * (q_ref_P.xz + -q_ref_P.wy);
    qMat[2][1] = 2.0f * (q_ref_P.yz - -q_ref_P.wx);
    qMat[2][2] = 1.0f - 2.0f * q_ref_P.xx - 2.0f * q_ref_P.yy;
}

void position_controller(float x_error, float y_error, float z_error){      
    quaternion quat_from_correct_angle;       

    correct_angle_y = (-1) * (x_error*qMat[0][2] + y_error*qMat[1][2] +  z_error*qMat[2][2]);       //remember to transfer qMat matrix
    correct_angle_z = x_error*qMat[0][1] + y_error*qMat[1][1] +  z_error*qMat[2][1];       //rad

    if(correct_angle_y >= max_correct_angle_y){
        correct_angle_y = max_correct_angle_y;
    }
    if(correct_angle_y < -max_correct_angle_y){
        correct_angle_y = -max_correct_angle_y;
    }
    if(correct_angle_z >= max_correct_angle_z){
        correct_angle_z = max_correct_angle_z;
    }
    if(correct_angle_z < -max_correct_angle_z){
        correct_angle_z = -max_correct_angle_z;
    }

    if (abs(angle[PITCH]) < 600){
        correct_angle_x = (-1)*correct_angle_z*cosf( abs(DEGREES_TO_RADIANS(quaternionpitch/10)) )*cosf( abs(DEGREES_TO_RADIANS(quaternionroll/10)) ) ;
    }
    else{
        correct_angle_x = 0;
    }    
    
    cx_datalog = lrintf(RADIANS_TO_DEGREES(correct_angle_x));
    cy_datalog = lrintf(RADIANS_TO_DEGREES(correct_angle_y));
    cz_datalog = lrintf(RADIANS_TO_DEGREES(correct_angle_z));

    imuComputeQuaternionFromRPY((int16_t)RADIANS_TO_DEGREES(correct_angle_x), (int16_t)RADIANS_TO_DEGREES(correct_angle_y), (int16_t)RADIANS_TO_DEGREES(correct_angle_z), &quat_from_correct_angle);
   
    imuQuaternionMultiplication(&quaternion_ref, &quat_from_correct_angle, &desired_quaternion);

}


//================================== quaternion_controller (200hz)======================================//
quaternion quaternion_error;

void quatconj(quaternion *q){
    q->w = q->w;
    q->x = q->x * (-1);
    q->y = q->y * (-1);
    q->z = q->z * (-1);
}

float quaternion_norm_sqare(quaternion *q){
    float q_norm_sqare;
    q_norm_sqare = (q->w)*(q->w) + (q->x)*(q->x) + (q->y)*(q->y) + (q->z)*(q->z);
    return q_norm_sqare;
}

void calculate_quaternion_error(void){

    quaternion quaternion_conjugate;
    quaternion quaternion_diff;
    quaternion quaternion_sum;

    quaternion_conjugate = q;
    quatconj(&quaternion_conjugate);

    quaternion_diff.w = q.w - desired_quaternion.w;
    quaternion_diff.x = q.x - desired_quaternion.x;
    quaternion_diff.y = q.y - desired_quaternion.y;
    quaternion_diff.z = q.z - desired_quaternion.z;
    quaternion_sum.w = q.w + desired_quaternion.w;
    quaternion_sum.x = q.x + desired_quaternion.x;
    quaternion_sum.y = q.y + desired_quaternion.y;
    quaternion_sum.z = q.z + desired_quaternion.z;
    
    if ( quaternion_norm_sqare(&quaternion_diff) <= quaternion_norm_sqare(&quaternion_sum) ){   //eq3.31
        imuQuaternionMultiplication(&quaternion_conjugate, &desired_quaternion , &quaternion_error);
    }
    else{    
        desired_quaternion.w = desired_quaternion.w * (-1);
        desired_quaternion.x = desired_quaternion.x * (-1);
        desired_quaternion.y = desired_quaternion.y * (-1);
        desired_quaternion.z = desired_quaternion.z * (-1);
        imuQuaternionMultiplication(&quaternion_conjugate, &desired_quaternion , &quaternion_error);
    }
}

int16_t roll_quaternion_controller(void){      
    //flapping wing:
    float kp = 2400.0f;
    float ki = 150.0f;
    float kd = 120.0f;
    
	static float last_error_roll;
	static float error_d_roll, error_i_roll; 
    float error;
	int32_t output;
	
	error = quaternion_error.x ;
	error_i_roll = error_i_roll + error * 0.005;
    error_d_roll = (error - last_error_roll) / 0.005;
    output = error * kp + error_i_roll * ki + error_d_roll * kd;
    last_error_roll = error;
	return output;
    
}

int16_t pitch_quaternion_controller(void){      
    //flapping wing:
    float kp = 1600.0f;
    float ki = 120.0f;
    float kd = 290.0f;
    //float kp = 1550.0f;
    //float ki = 110.0f;
    //float kd = 290.0f;

	static float last_error_pitch;
	static float error_d_pitch, error_i_pitch; 
    float error;
	int32_t output;
	
	error = quaternion_error.y ;
	error_i_pitch = error_i_pitch + error * 0.005;
    error_d_pitch = (error - last_error_pitch) / 0.005;
    output = error * kp + error_i_pitch * ki + error_d_pitch * kd;
    last_error_pitch = error;
	return output;
    
}

int16_t yaw_quaternion_controller(void){      
    //flapping wing:
    //float kp = 1950.0f;
    //float ki = 65.0f;
    //float kd = 280.0f;
    float kp = 1200.0f;
    float ki = 30.0f;
    float kd = 200.0f;

	static float last_error_yaw;
	static float error_d_yaw, error_i_yaw; 
    float error;
	int32_t output;
	
	error = quaternion_error.z ;
	error_i_yaw = error_i_yaw + error * 0.005;
    error_d_yaw = (error - last_error_yaw) / 0.005;
    output = error * kp + error_i_yaw * ki + error_d_yaw * kd;
    last_error_yaw = error;
	return output;
    
}


//====================================  THRUST CONTROLLER (20hz)=======================================//
float body_vx;      //ux

void cal_body_velocity(uint32_t cur_time){   
    //need to align optitrack xy coordinate to earth xy coordinate by using magHold !!  
    float global_vx;
    float global_vy;
    float global_vz;
    static float last_rigidbody_x, last_rigidbody_y, last_rigidbody_z;
    static uint32_t prev_time;
    float delta_time = (cur_time - prev_time)* 1e-6f;      //(sec)

    global_vx = (rigidbody_x - last_rigidbody_x) / delta_time;     //(m/s)
    global_vy = (rigidbody_y - last_rigidbody_y) / delta_time;
    global_vz = (rigidbody_z - last_rigidbody_z) / delta_time;

    body_vx = rMat[0][0]*global_vx + rMat[1][0]*global_vy + rMat[2][0]*global_vz;       //calculate body X velocity. Remeber transfer rotation matrix,since rMat is body to global

    last_rigidbody_x = rigidbody_x;
    last_rigidbody_y = rigidbody_y;
    last_rigidbody_z = rigidbody_z;
    prev_time = cur_time;

    body_vx_datalog = lrintf(100*body_vx);  // (cm/s) for blackbox
}

void force_to_pwm(int16_t *des_fx_pwm, float fx_des){
    *des_fx_pwm = lrintf( 337.75f * fx_des + 1163.8f );//+ (87 - vbat)*10;  //from experience test, vbat compensate pwm
}

int16_t thrust_controller(float body_vx_ref, float h_ref){    //velocity(m/s), h_ref(m)
    float kup = 2.0f;
    float khp = 10.0f;
    float mass = 0.15f;
    float weight = mass*9.81f;   
    //float max_thrust = 2.31;   //(N),propellor:4025, motor:mc1108
    int16_t max_thrust_pwm = 1900;  //we leave 50 pwm for yaw(two throttle differ)
    float sinpitch = abs(sin_approx( DEGREES_TO_RADIANS( (float)quaternionpitch/10.0f )) );
    float fx_des;
    int16_t fx_desired_pwm; 
    
    fx_des = weight*sinpitch + mass*kup*(body_vx_ref - body_vx) + mass*khp*( h_ref - (rigidbody_z-original_z) )*sinpitch;

    force_to_pwm(&fx_desired_pwm, fx_des);  //force(N) transfer to PWM

    bounding(&fx_desired_pwm, max_thrust_pwm, 1150);

    return fx_desired_pwm;
}


//====================================  Trajectory start (20hz)============================================//

bool hit_waypoint(float waypoint_x, float waypoint_y, float hit_dis_set){ //point to point flight
    float dis;
    float cur_x = (float)tmp_px/100.0f; //(m), global
    float cur_y = (float)tmp_py/100.0f;
    float x_sq, y_sq;
    
    x_sq = (cur_x - waypoint_x);
    y_sq = (cur_y - waypoint_y);
    dis = sqrt( x_sq*x_sq + y_sq*y_sq );

    if(dis < hit_dis_set){
        return 1;
    }  
    else{
        return 0;
    }
}

int16_t accumulation_yaw = 0;   //positive: counter clock wise
void correct_des_global_position(float *body_x, float *body_y, float set_dis_x, float set_dis_y){
    float tmp;
    float ref_heading_rad;
    float b2g_cosine;
    float b2g_sine;

    ref_heading_rad = accumulation_yaw;    // positive: CCW
    ref_heading_rad = DEGREES_TO_RADIANS(ref_heading_rad);
    b2g_cosine = cos_approx(ref_heading_rad);
    b2g_sine = sin_approx(ref_heading_rad);

    tmp = set_dis_x;
    set_dis_x = set_dis_x * b2g_cosine + set_dis_y * b2g_sine * (-1.0f);
    set_dis_y = tmp * b2g_sine + set_dis_y * b2g_cosine;

    *body_x = *body_x + set_dis_x;
    *body_y = *body_y + set_dis_y;
}

float cal_level_flight_aoa(float current_level_flight_speed){
    float level_flight_aoa;
    level_flight_aoa = 2.426f * expf(-0.4619f * current_level_flight_speed);

    return level_flight_aoa;
}

bool CoC_init = false;
bool straight_flight_init = true;
bool hit_flag = false;
bool turn_init = true;
bool FT_init = true;    //init when each flight task started 
float final_x, final_y;

void trajectory_generator(uint8_t flight_task_order){   
//  xyz input will be center of circle while bankturn
//  output position_ref, quaternion_ref and vx_ref for controller 
    switch (flight_task_order)
    {      
        case take_off:
        {
            ref_roll_angle = 0;
            ref_pitch_angle = -90;  //hover
            ref_yaw_angle = -magHold;
            px_ref = (float)OX/100.0f + 0.0f;    //set desired x (MOCAP coordinate)
            py_ref = (float)OY/100.0f + 0.0f;     //set desired y (MOCAP coordinate)
            pz_ref = 0.6f;    //set desired height (MOCAP coordinate)
            bvx_ref = 1.0f;   //body_x velocity ref
            break;
        }

        case switch_to_hover:
        {
            ref_roll_angle = 0;
            ref_pitch_angle = -90;  //hover
            ref_yaw_angle = -magHold;
            px_ref = (float)OX/100.0f + 0.0f;    //set desired x (MOCAP coordinate)
            py_ref = (float)OY/100.0f + 0.0f;     //set desired y (MOCAP coordinate)
            pz_ref = 0.6f;    //set desired height (MOCAP coordinate)
            bvx_ref = 0.0f;   //body_x velocity ref
            break;
        }

        case level_flight:
        {
            int16_t level_flight_aoa;   //degree
            float level_flight_V = 2.5f;    //set desired body_x velocity  (m/s)
            float tmp_aoa;  //aoa (rad)

            tmp_aoa = 2.426f * expf(-0.4619*level_flight_V); //aoa (rad)
            level_flight_aoa = lrintf( RADIANS_TO_DEGREES(tmp_aoa) );   //aoa (degree)

            px_ref = (float)tmp_px/100.0f;    //set desired x (MOCAP coordinate), it will transfer to earth coordinate later
            py_ref = (float)tmp_py/100.0f;    //set desired y (MOCAP coordinate), it will transfer to earth coordinate later
            pz_ref = 0.6f;    //set desired height (MOCAP coordinate)

            ref_roll_angle = 0;
            ref_yaw_angle = -magHold;
            ref_pitch_angle = ref_pitch_angle + 3;
            if( ref_pitch_angle > level_flight_aoa*(-1) ){
                ref_pitch_angle = level_flight_aoa*(-1);
            } 

            bvx_ref = level_flight_V*cos_approx( tmp_aoa );   //body_x velocity ref

            CoC_init = true;
            break;
        }

        case left_bankturn:
        {
            int16_t left_bankturn_angle;    //degree
            int16_t level_flight_aoa;   //degree
            int16_t heading_ref;        //degree
            int16_t center_of_circle_x; //cm
            int16_t center_of_circle_y; //cm
            int16_t roll_ref_max = 60;  //degree max roll angle (degree)
            float bankturn_radius = 0.8f;    //set desired radius for circling (m)
            float bankturn_radius_x_tmp;
            float bankturn_radius_y_tmp;
            float rad_quaternionyaw = DEGREES_TO_RADIANS( (float)quaternionyaw/10.0f ); //rad
            float gamma;    //rad
            //float aero_k = 0.07287f;   // (2*mass)/(rho*S*cl_alpha), mass(kg), rho(air density), for calculate roll ref
            float aero_k = 0.36f;
            //float aero_k = 0.47f;
            float tmp_euler_roll;
            float tmp_euler_pitch;
            float tmp_aoa;  //aoa (rad)
            float level_flight_V = 3.0f;    //  (m/s)

            //set center of circling on the left hand side of flapping wing , in MOCAP coordinate( use magHold ):
            if(CoC_init){
                float z_up_yaw_tmp = -( rad_quaternionyaw - DEGREES_TO_RADIANS(magHold) ) + M_PI_2;   // left: +M_PI_2, right: -M_PI_2
                float r_ca = bankturn_radius*100.0f *cos_approx(z_up_yaw_tmp);
                float r_sa = bankturn_radius*100.0f *sin_approx(z_up_yaw_tmp);
                //center_of_circle_x = lrintf( (float)tmp_px + r_ca );  //cm , MOCAP coordinate
                //center_of_circle_y = lrintf( (float)tmp_py + r_sa );  //cm , MOCAP coordinate
                center_of_circle_x = 0;  //cm , MOCAP coordinate
                center_of_circle_y = 0;  //cm , MOCAP coordinate
                CoC_init = false;
            }
        
            //calculate desired heading, in MOCAP coordinate:
            if( (tmp_py - center_of_circle_y) >= 0){    // quadrant I and II
                gamma = atan2_approx( (float)(tmp_py-center_of_circle_y), (float)(tmp_px-center_of_circle_x) );   //rad
            }
            else{                                           // quadrant III and IV
                gamma = atan2_approx( (float)(tmp_py-center_of_circle_y), (float)(tmp_px-center_of_circle_x) ) + (2.0f * M_PI) ;    //rad
            }

            heading_ref = lrintf( RADIANS_TO_DEGREES( gamma + M_PI_2 ) );   // (degree) left turn: gamma + M_PI_2, right turn: gamma - M_PI_2
            if (heading_ref > 360){
                heading_ref = heading_ref - 360;
            }

            //calculate level flight AOA:
            tmp_aoa =  2.426f * expf(-0.4619*level_flight_V); //aoa (rad)
            level_flight_aoa = lrintf( RADIANS_TO_DEGREES(tmp_aoa) );   //aoa (degree)

            //calculate desired roll:
            tmp_euler_pitch = DEGREES_TO_RADIANS( abs( (float)quaternionpitch/10.0f) ); //rad
            tmp_euler_roll = aero_k /( bankturn_radius * tmp_euler_pitch );

            if(tmp_euler_roll > 1.0f){  //prevent we got image number from arcsine function
                tmp_euler_roll = 1.0f;
            }
            if(tmp_euler_roll < -1.0f){
                tmp_euler_roll = -1.0f;
            }

            tmp_euler_roll = asinf( tmp_euler_roll );    //tmp_euler_roll (rad)
            left_bankturn_angle = RADIANS_TO_DEGREES(tmp_euler_roll);    //roll_ref (degree)

            if ( abs(left_bankturn_angle) > roll_ref_max){
                left_bankturn_angle = roll_ref_max;
            }
            
            //update position refference:
            bankturn_radius_x_tmp = bankturn_radius*cos_approx(gamma);  //(m), in MOCAP coordinate
            bankturn_radius_y_tmp = bankturn_radius*sin_approx(gamma);  //(m), in MOCAP coordinate
            px_ref = ((float)center_of_circle_x/100.0f) + bankturn_radius_x_tmp;    //(m), in MOCAP coordinate, it will transfer to earth coordinate later
            py_ref = ((float)center_of_circle_y/100.0f) + bankturn_radius_y_tmp;    //(m), in MOCAP coordinate, it will transfer to earth coordinate later
            pz_ref = 0.6;   //(m)
            
            //update rpy refference:
            ref_roll_angle = left_bankturn_angle * (-1);   // left -1, right 1
            ref_pitch_angle = level_flight_aoa * (-1);      //body_coordinate z_up -1, body_coordinate z_down 1
            ref_yaw_angle = -magHold + heading_ref; // transfer to earth coordinate for controllers

            //update body_x_velocity refference:
            bvx_ref = level_flight_V*cos_approx( tmp_aoa );
            
            break;
        }

        case right_bankturn:    //remember direction *(-1)
        {    
            break;
        }

        case switch_back_hover:
        {   
            if(FT_init){
                ref_roll_angle = 0;
                ref_pitch_angle = -90;  //hover
                ref_yaw_angle = -(int16_t)(quaternionyaw/10.0f);
                px_ref = last_x;     //set desired x (MOCAP coordinate)
                py_ref = last_y;      //set desired y (MOCAP coordinate)
                FT_init = false;
            }

            pz_ref = 0.6f;  //set desired z (MOCAP coordinate)
            bvx_ref = 0.0f;   //body_x velocity ref
            break;
        }

        case slow_landing:
        {   
            //px_ref and py_ref are the same when "switch_back_hover"
            ref_roll_angle = 0;
            ref_pitch_angle = -90;  //hover
            ref_yaw_angle = -magHold;
            bvx_ref = -0.01f;   //body_x velocity ref

            pz_ref = pz_ref - 0.005;    //(20hz)
            if(pz_ref < 0.1){
                pz_ref = 0.1f;    
            }

            if( (rigidbody_z-original_z) < 0.2){  //if current altitude is lower 20 cm, then stop.
                stop = true;
            }
            break;
        }
           
        case straight_flight:   //straight_flight during hover, always set desired waypoint ahead to make it fly foreward
        {   
            float max_straight_flight = 1.0f;  //1(m)
            float pos_ahead = 0.3;
            float body_x, body_y;
            body_x = (float)tmp_px/100.0f;  //current position(m)
            body_y = (float)tmp_py/100.0f;

            if(straight_flight_init){
                final_x = body_x;  //original at body coordinate
                final_y = body_y;
                if(accumulation_yaw > 360){
                    accumulation_yaw = accumulation_yaw - 360;
                }
                correct_des_global_position(&final_x, &final_y, max_straight_flight, 0);    //calculate final waypoint (global coordinate)
                straight_flight_init = false;
            }

                correct_des_global_position(&body_x, &body_y, pos_ahead, 0);  // correct desired position with heading

                if( hit_waypoint(final_x, final_y, 0.3) ){
                    hit_flag = true;
                }
                if(hit_flag){
                    px_ref = final_x;
                    py_ref = final_y;
                    pz_ref = 0.6f;
                }
                else{
                    px_ref = body_x;
                    py_ref = body_y;
                    pz_ref = 0.6f;
                }

                ref_roll_angle = 0;
                ref_pitch_angle = -90;
                ref_yaw_angle = (-magHold) + accumulation_yaw;        
                bvx_ref = 0.0f;   //body_x velocity ref

            break;
        }

        case left_turn: //left turn during hover
        {
            if(turn_init){
                int16_t left_turn_angle = 90;   //CCW
                hit_flag = false;   //let it ready for next straight flight
                px_ref = (float)tmp_px/100.0f;  //current position(m)
                py_ref = (float)tmp_py/100.0f;
                accumulation_yaw = accumulation_yaw + left_turn_angle;   //left turn 90 degree
                turn_init = false;
            }
            pz_ref = 0.6;
            ref_roll_angle = 0;
            ref_pitch_angle = -90;
            ref_yaw_angle = (-magHold) + accumulation_yaw;
            bvx_ref = 0.0f;   //body_x velocity ref

            break;
        }
           
        default:
            break;
    }

}


//====================================  voltage manergor (20hz)============================================//

void voltage_manergor(int16_t *flight_task, uint16_t voltage){
    static int16_t low_voltage_count;
    static int16_t stable_for_landing;
    if(voltage <= 65){   //define 6.5V for LiHV battries(2s) low voltage
        low_voltage_count = low_voltage_count + 1;
    }
    else{
        low_voltage_count = 0;
    }

    if(low_voltage_count >= 60){    //if we got low voltage for 3 sec
        landing_trigger = true;
    }

    if(landing_trigger){
        stable_for_landing = stable_for_landing + 1;
        *flight_task = 5;   //switch back to hover

        if(stable_for_landing > 100)    //5 sec for stable the flapping wing, then start landing process
        {
            *flight_task = 6;   //force to landing 
        }
    }
}


//===================================== Thrust test  ==============================================//
void thrsut_curve_n_point(int8_t n_interval, int16_t time_interval, int16_t max_throttle, int16_t min_throttle, int c_store){
    int16_t interval_pwm;
    int start_time = 33;
    static int cc;
    interval_pwm = (max_throttle - min_throttle) / n_interval;

    if( (count - c_store) > start_time + time_interval*cc ){
        cc = cc+1;
    }

    motor[0] = min_throttle + interval_pwm * cc;
    motor[1] = min_throttle + interval_pwm * cc;
    servo[3] = 1500;
    servo[4] = 1500;

    if(cc > n_interval)  //stop
    { 
        motor[0] = 1100;
        motor[1] = 1100;
        servo[3] = 1500;
        servo[4] = 1500;        
    }       
}


//===================================== others =========================================//
int16_t balance ;
int16_t z_baro_PID_controller(int16_t cmd)
{
	static int16_t error_i_z, last_error_z;
	int16_t error_d_z, output, error;
    //float voltage = vbat;
    //float cos_pitch_angle = cosf( angle[PITCH]/10 );
    //balance = lrintf((-5.528f * voltage + 2092.4f));
    balance = 1690; //flapping wing

	//error = cmd  - (EstAlt - store_alt); // use acc and baro for ALT
    error = cmd  - (tmp_pz - OZ);   //use optitrack z
	error_i_z = error_i_z + error * 0.005;
    error_d_z = (error - last_error_z) / 0.005;
    output = error * 10 + error_i_z * 1 + error_d_z * 3;
	last_error_z = error;
	output = balance + output;    
    bounding(&output, 1900, balance-150);
    return output;	
}

