#ifndef __FLY_CONTROLLER_PID_H__
#define __FLY_CONTROLLER_PID_H__

#include <stdbool.h>
#include "math.h"

#include "arm_math.h"

#include "PID.h"

typedef struct {
    bool altitude_ON;
    bool lateral_ON;
    bool attitude_ON;

    /* Limits */
    float pitch_limit, roll_limit, amp_limit;

    /* baseline voltages and liftoff constants */
    float liftoff_V;

    /* PID Gains */
    float roll_Kp, roll_Ki, roll_Kd;
    float pitch_Kp, pitch_Ki, pitch_Kd;

    /* Mass & Rotational Inertias */
    float weight;
    float Jx, Jy, Jz;

    /* dt - Important stuff, think about implementation */
    float dt;

    /* Gains */
    float altitude_Kp, altitude_Ki, altitude_Kd;
    float lateralX_Kp, lateralX_Ki, lateralX_Kd;
    float lateralY_Kp, lateralY_Ki, lateralY_Kd;

    /* Miscellaneous */
    float r;
    float attitude_damping;
} controlConfigs_t;

typedef struct {
    float X;
    float Y;
    float Z;
} desriedPosition_t;

typedef struct {
    float zdw_0;
    float zdw_1;
    float zdw_2;
} desiredAttitude_t;

typedef struct {
    float roll; // euler_angles(0)
    float pitch; // euler_angles(1)
    float yaw; // euler_angles(2)
} eulerAngles_t;

typedef struct {
    float amplitude;
    float delta_amplitude;
    float offset;
} flyControl_t;

typedef struct {
    float positionX;
    float positionY;
    float altitudeZ;

    float quat_w; // quat(0)
    float quat_i; // quat(1)
    float quat_j; // quat(2)
    float quat_k; // quat(3)
} flyState_t;

typedef struct {
    /* Configs */
    controlConfigs_t p1; 

    /* PID structs */ 
    PID_t altitude_PID;
    PID_t lateralX_PID;
    PID_t lateralY_PID;
    PID_t pitch_PID; 
    PID_t roll_PID;

    /* Butterworth Filter Instances */
    arm_biquad_casd_df1_inst_f32 xpos_Bf;
    arm_biquad_casd_df1_inst_f32 ypos_Bf;
    arm_biquad_casd_df1_inst_f32 zpos_Bf;
    arm_biquad_casd_df1_inst_f32 quatw_Bf;
    arm_biquad_casd_df1_inst_f32 quatx_Bf;
    arm_biquad_casd_df1_inst_f32 quaty_Bf;
    arm_biquad_casd_df1_inst_f32 quatz_Bf;

    arm_biquad_casd_df1_inst_f32 attitude_Error0;
    arm_biquad_casd_df1_inst_f32 attitude_Error1;

    /* accelerations */
    float roll_acc;
    float pitch_acc;
    float z_acc;

    /* misc structs */
    /* State related structs*/
    flyState_t state;
    eulerAngles_t e_angles;
    flyState_t prev_state;
    flyState_t prev_prev_state;
    eulerAngles_t omega_body;

    /* desired state structs */
    desriedPosition_t setPoint;
    desiredAttitude_t setAttitude;

    /* Output in Voltages */
    flyControl_t output;

    /* Flags */
    bool first_ite;
    bool second_ite;
} flyController_PID_t;

void flyController_PID_Init(flyController_PID_t* flyController);

/* Misc functions used within the individual controllers */
void eulaer_calc(flyController_PID_t* flyController, flyState_t actual);

/* Individual Controllers */
void altitude_controller(flyController_PID_t* flyController, flyState_t actual, desriedPosition_t set_point);

void lateral_controller(flyController_PID_t* flyConntroller, flyState_t actual, desriedPosition_t set_point);

/* Gets input from lateral_controller */
void attitude_controller(flyController_PID_t* flyController, flyState_t actual, desiredAttitude_t set_point);

/* Calls all layers of PID functions sequentially */
void control(flyController_PID_t* flyController, flyState_t state_vector, desriedPosition_t setPoint);

/* Use to convert control to Voltages */
void compute_control(flyController_PID_t* flyController);

#endif
