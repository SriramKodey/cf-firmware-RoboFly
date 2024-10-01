#include "arm_math.h"

#include "flyController_PID.h"

/* Fliter coefficients & constants */
#define NUM_SECTIONS 2 // 4th order ButterWorth so 2 biquads cascaded

/* Define the state buffers for each filter (4 * NUM_SECTIONS) */
static float32_t xpos_filter_state[4 * NUM_SECTIONS];
static float32_t ypos_filter_state[4 * NUM_SECTIONS];
static float32_t zpos_filter_state[4 * NUM_SECTIONS];
static float32_t quatw_filter_state[4 * NUM_SECTIONS];
static float32_t quatx_filter_state[4 * NUM_SECTIONS];
static float32_t quaty_filter_state[4 * NUM_SECTIONS];
static float32_t quatz_filter_state[4 * NUM_SECTIONS];

static float32_t attitude_error0_filter_state[4 * NUM_SECTIONS];
static float32_t attitude_error1_filter_state[4 * NUM_SECTIONS];

static float32_t filter_coeffs[NUM_SECTIONS*5] = {
    0.00482434, 0.00964869, 0.00482434, 1.04859958, -0.29614036, // Section 1
    1, 2, 1, 1.32091343, -0.63273879 // Section 2
};

void flyController_PID_Init(flyController_PID_t* flyController)
{
    /* Needs to be static when declared */
    /* All Controller structs and variables need to live here */

    /* Current State */

    /* Params */
    flyController->p1.altitude_ON = true;
    flyController->p1.lateral_ON = true;
    flyController->p1.attitude_ON = true;

    /* body Model */
    flyController->p1.weight = 0.0014f;
    flyController->p1.Jx = 2.5e-9;
    flyController->p1.Jy = 6e-9;
    flyController->p1.Jz = 5e-9;
    flyController->p1.r = 1.5e-2;
    flyController->p1.liftoff_V = 145;

    /* PID Gains */
    flyController->p1.roll_Kp = 324.0f;
    flyController->p1.roll_Ki = 81.0f;
    flyController->p1.roll_Kd = 0.0f;

    flyController->p1.pitch_Kp = 324.0f;
    flyController->p1.pitch_Kp = 81.0f;
    flyController->p1.pitch_Kd = 0.0f;

    flyController->p1.attitude_damping = -13.0f;

    flyController->p1.altitude_Kp = 0.7f;
    flyController->p1.altitude_Ki = 0.0f;
    flyController->p1.altitude_Kd = 3.0f;

    flyController->p1.lateralX_Kp = 5.0f;
    flyController->p1.lateralX_Ki = 0.05f;
    flyController->p1.lateralX_Kd = 2.0f;

    flyController->p1.lateralY_Kp = 5.0f;
    flyController->p1.lateralY_Kp = 0.05f;
    flyController->p1.lateralY_Kp = 2.0f;

    // Initialise the controlConfigs using a function - write later!!
    flyController->p1.dt = 0.005;

    /* PID Structs */
    PID_Init(&(flyController->altitude_PID), flyController->p1.altitude_Kp, flyController->p1.altitude_Ki, flyController->p1.altitude_Kd, flyController->p1.dt);
    PID_Init(&(flyController->lateralX_PID), flyController->p1.lateralX_Kp, flyController->p1.lateralX_Ki, flyController->p1.altitude_Kd, flyController->p1.dt);
    PID_Init(&(flyController->lateralY_PID), flyController->p1.lateralY_Kp, flyController->p1.lateralY_Ki, flyController->p1.altitude_Kd, flyController->p1.dt);
    PID_Init(&(flyController->pitch_PID), flyController->p1.pitch_Kp, flyController->p1.pitch_Ki, flyController->p1.pitch_Kd, flyController->p1.dt);
    PID_Init(&(flyController->roll_PID), flyController->p1.roll_Kp, flyController->p1.roll_Ki, flyController->p1.roll_Kd, flyController->p1.dt);

    /* Initialize Filters */
    arm_biquad_cascade_df1_init_f32(&(flyController->xpos_Bf), NUM_SECTIONS, filter_coeffs, xpos_filter_state);
    arm_biquad_cascade_df1_init_f32(&(flyController->ypos_Bf), NUM_SECTIONS, filter_coeffs, ypos_filter_state);
    arm_biquad_cascade_df1_init_f32(&(flyController->zpos_Bf), NUM_SECTIONS, filter_coeffs, zpos_filter_state);
    arm_biquad_cascade_df1_init_f32(&(flyController->quatw_Bf), NUM_SECTIONS, filter_coeffs, quatw_filter_state);
    arm_biquad_cascade_df1_init_f32(&(flyController->quatx_Bf), NUM_SECTIONS, filter_coeffs, quatx_filter_state);
    arm_biquad_cascade_df1_init_f32(&(flyController->quaty_Bf), NUM_SECTIONS, filter_coeffs, quaty_filter_state);
    arm_biquad_cascade_df1_init_f32(&(flyController->quatz_Bf), NUM_SECTIONS, filter_coeffs, quatz_filter_state);
    
    arm_biquad_cascade_df1_init_f32(&(flyController->attitude_Error0), NUM_SECTIONS, filter_coeffs, attitude_error0_filter_state);
    arm_biquad_cascade_df1_init_f32(&(flyController->attitude_Error1), NUM_SECTIONS, filter_coeffs, attitude_error1_filter_state);

    /* Flags */
    flyController->first_ite = true;
    flyController->second_ite = false;

    /* Accelerations */
    flyController->roll_acc = 0;
    flyController->pitch_acc = 0;
    flyController->z_acc = 0;

    /* Initial Output */
    flyController->output.amplitude = 0;
}

void saturate_output(float* output, float ulimit, float llimit)
{
    *output = fmin(fmax(*output, llimit), ulimit);
}

static float bwFilter_Process(arm_biquad_casd_df1_inst_f32 *fliter, float input) {
    float output;
    arm_biquad_cascade_df1_f32(fliter, &input, &output, 1);
    return output;
}

void altitude_controller(flyController_PID_t* flyController, flyState_t actual, desriedPosition_t set_point)
{
    /* Operates in terms of accelerations */
    float output = calc_PID_Output(&(flyController->altitude_PID), actual.altitudeZ, set_point.Z);
    saturate_output(&output, 0.5, -1.0);
    flyController->z_acc = output;
}

void lateral_controller(flyController_PID_t* flyController, flyState_t actual, desriedPosition_t set_point)
{
    /* Operates in terms of accelerations 
     * generates the desired attitude 
     * Desired attitude acheived through attitude controller
     */
    float zdw_0 = calc_PID_Output(&(flyController->lateralX_PID), actual.positionX, set_point.X);
    float zdw_1 = calc_PID_Output(&(flyController->lateralY_PID), actual.positionY, set_point.Y);

    saturate_output(&zdw_0, 1, -1);
    saturate_output(&zdw_1, 1, -1);

    flyController->setAttitude.zdw_0 = zdw_0;
    flyController->setAttitude.zdw_1 = zdw_1;
    flyController->setAttitude.zdw_2 = 1;
}

void euler_calc(flyController_PID_t* flyController, flyState_t actual)
{
    /* Roll */
    float sinr_cosp = 2*(actual.quat_w * actual.quat_i + actual.quat_j * actual.quat_k);
    float cosr_cosp = 1 - 2*(actual.quat_i * actual.quat_i + actual.quat_j * actual.quat_j);
    flyController->e_angles.roll = atan2(sinr_cosp, cosr_cosp);

    /* Pitch */
    float sinp = 2*(actual.quat_w * actual.quat_j - actual.quat_k * actual.quat_i);
    if (fabs(sinp) >= 1) {
        flyController->e_angles.pitch = copysign(M_PI_2, sinp);
    }
    else {
        flyController->e_angles.pitch = asin(sinp);
    }

    /* Yaw */
    float siny_cosp = 2*(actual.quat_w * actual.quat_k + actual.quat_i * actual.quat_j);
    float cosy_cosp = 1 - 2*(actual.quat_j * actual.quat_j + actual.quat_k * actual.quat_k);
    flyController->e_angles.yaw = atan2(siny_cosp, cosy_cosp);
}

void compute_omega(flyController_PID_t* flyController, flyState_t actual, flyState_t prev_s, flyState_t prev_prev_s) 
{
    /* Calculate the conjugate quaternion */
    flyState_t state_conjugate;
    state_conjugate.quat_w = actual.quat_w;
    state_conjugate.quat_i = -actual.quat_i;
    state_conjugate.quat_j = -actual.quat_j;
    state_conjugate.quat_k = -actual.quat_k;

    float dq[4] = {0};

    if(flyController->first_ite) {
        flyController->first_ite = false;
        prev_s = actual;
        prev_prev_s = actual;
        flyController->second_ite = true;
    }
    else if (flyController->second_ite) {
        flyController->second_ite = false;
    }
    else {
        // 2 * flyController->p1.dt since its two time steps difference
        dq[0] = (actual.quat_w - prev_prev_s.quat_w)/(2 * flyController->p1.dt); 
        dq[1] = (actual.quat_i - prev_prev_s.quat_i)/(2 * flyController->p1.dt); 
        dq[2] = (actual.quat_j - prev_prev_s.quat_j)/(2 * flyController->p1.dt); 
        dq[3] = (actual.quat_k - prev_prev_s.quat_k)/(2 * flyController->p1.dt); 
    }

    flyController->omega_body.roll = 2*(state_conjugate.quat_i*dq[0] + state_conjugate.quat_w*dq[1] - state_conjugate.quat_k*dq[2] + state_conjugate.quat_j*dq[3]);
    flyController->omega_body.pitch = 2*(state_conjugate.quat_j*dq[0] + state_conjugate.quat_k*dq[1] + state_conjugate.quat_w*dq[2] - state_conjugate.quat_i*dq[3]);
    flyController->omega_body.yaw = 2*(state_conjugate.quat_k*dq[0] - state_conjugate.quat_j*dq[1] +state_conjugate.quat_i*dq[2] + state_conjugate.quat_w*dq[3]);

    flyController->prev_prev_state = prev_s;
    flyController->prev_state = actual;
}

void rot_mat_calc(flyController_PID_t* flyController, float (*R)[3])
{
    float cz = cos(flyController->e_angles.yaw);
    float cy = cos(flyController->e_angles.pitch);
    float cx = cos(flyController->e_angles.roll);
    float sz = sin(flyController->e_angles.yaw);
    float sy = sin(flyController->e_angles.pitch);
    float sx = sin(flyController->e_angles.roll);

    R[0][0] = cz*cy;
    R[0][1] = cz*sy*sx - cx*sz;
    R[0][2] = sz*sx + cz*cx*sy;
    R[1][0] = cy*sz;
    R[1][1] = cz*cx + sz*sy*sx;
    R[1][2] = cx*sz*sy - cz*sx;
    R[2][0] = -sy;
    R[2][1] = cy*sx;
    R[2][2] = cy*cx;
}

void attitude_controller(flyController_PID_t* flyController, flyState_t actual, desiredAttitude_t set_point)
{
    float rot_mat[3][3];
    euler_calc(flyController, actual);
    compute_omega(flyController, actual, flyController->prev_state, flyController->prev_prev_state);
    rot_mat_calc(flyController, rot_mat);

    float z_vec[3] = {rot_mat[0][2], rot_mat[1][2], rot_mat[2][2]};

    float body_zvec_error[3] = {0};
    body_zvec_error[0] = (rot_mat[0][0] * (set_point.zdw_0 - z_vec[0])) + (rot_mat[1][0] * (set_point.zdw_1 - z_vec[1])) + rot_mat[2][0] * (set_point.zdw_2 - z_vec[2]);
    body_zvec_error[1] = (rot_mat[0][1] * (set_point.zdw_0 - z_vec[0])) + (rot_mat[1][1] * (set_point.zdw_1 - z_vec[1])) + rot_mat[2][1] * (set_point.zdw_2 - z_vec[2]);

    // Filter    
    // Use CMSIS-DSP arm_math.h for Filters
    // Assume you did Filtering
    body_zvec_error[0] = bwFilter_Process(&(flyController->attitude_Error0), body_zvec_error[0]);
    body_zvec_error[1] = bwFilter_Process(&(flyController->attitude_Error1), body_zvec_error[1]);

    float pitch_acc = calc_PID_Output(&(flyController->pitch_PID), 0, body_zvec_error[0]);
    float roll_acc = calc_PID_Output(&(flyController->roll_PID), 0, -1*body_zvec_error[1]);

    // Set attitude_damping
    pitch_acc += flyController->omega_body.pitch * flyController->p1.attitude_damping;
    roll_acc += flyController->omega_body.roll * flyController->p1.attitude_damping;

    saturate_output(&pitch_acc, 200, -200);
    saturate_output(&roll_acc, 200, -200);

    flyController->pitch_acc = pitch_acc;
    flyController->roll_acc = roll_acc;
}

void compute_control_voltages(flyController_PID_t* flyController) {
    /* Use this to convert accelerations to Voltages */
    // Equivalent to body model
    float dc_wing_angle_rad_per_pitch_deltaV = 0.5f/100.0f;
    float liftoff_thrust = flyController->p1.weight/2.0f;
    float thrust_per_Vp2p = liftoff_thrust/flyController->p1.liftoff_V;
    float mass_per_thruster = liftoff_thrust/9.81f;
    float thrust_per_thruster = (flyController->z_acc + 9.81f) * mass_per_thruster;
    float amplitude_Vp2P = thrust_per_thruster/thrust_per_Vp2p;
    float roll_deltaV = (flyController->roll_acc * flyController->p1.Jx) / (flyController->p1.r * thrust_per_Vp2p);
    float pitch_deltaV = (flyController->pitch_acc * flyController->p1.Jy) / (2.0f * flyController->p1.r * dc_wing_angle_rad_per_pitch_deltaV * thrust_per_thruster);

    saturate_output(&roll_deltaV, 45, -45);
    saturate_output(&pitch_deltaV, 45, -45);

    flyController->output.amplitude = amplitude_Vp2P;
    flyController->output.delta_amplitude = roll_deltaV;
    flyController->output.offset = pitch_deltaV;
}

void control(flyController_PID_t* flyController, flyState_t state_vector, desriedPosition_t setPoint)
{
    // Save actual state in the struct somewhere

    /* Needs to call all the PID layers and convert to Voltages */
    if (flyController->p1.altitude_ON)
    {
        altitude_controller(flyController, state_vector, setPoint);
    }

    if (flyController->p1.lateral_ON)
    {
        /* Sets the zdw used for the desired attitude */
        lateral_controller(flyController, state_vector, setPoint);
        // add ramp up logic 
    }

    if(flyController->p1.attitude_ON)
    {
        attitude_controller(flyController, state_vector, flyController->setAttitude);
    }

    /* Converts accelerations to Voltages */
    compute_control_voltages(flyController);
}
