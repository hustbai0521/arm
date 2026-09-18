/**
 * @file    ArmTrajectory.h
 * @brief   TP_JOINT trajectory adapter for the MYACTUATOR + DM-J4340 arm
 *
 * The kinematics and joint-path generator are provided by Arm2R, copied from
 * ArmPlatform_H7.  This adapter only performs feedback calibration, TP_JOINT
 * execution and conversion back to the two motor protocols.
 */

#ifndef ARM_TRAJECTORY_H
#define ARM_TRAJECTORY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_TRAJECTORY_MAX_POINTS 45U

/* Mechanism geometry and the logical joint convention used by Arm2R. */
#define ARM_TRAJECTORY_L1_MM 360.0f
#define ARM_TRAJECTORY_L2_MM 365.0f
#define ARM_TRAJECTORY_BASE_DEG_PER_UNIT 0.9f        //90/100
#define ARM_TRAJECTORY_JOINT_DEG_PER_UNIT 2.9f      //145*2/100
#define ARM_TRAJECTORY_JOINT_OFFSET_DEG 145.0f

/* Measured motor calibration anchors supplied for this mechanism. */
#define ARM_TRAJECTORY_BIG_ZERO_RAD (-1.06f)                   //大臂theta1 = 0°时电机反馈位置
#define ARM_TRAJECTORY_BIG_AT_BASE_100_RAD 2.118f             //大臂逻辑关节100%（本项目中对应实际角度theta1 = 90°）电机反馈位置
#define ARM_TRAJECTORY_SMALL_ZERO_RAD 0.034f                 //小臂逻辑关节零点，本项目中对应实际角度theta2 = 145°电机反馈位置
#define ARM_TRAJECTORY_SMALL_AT_THETA2_90_RAD 1.40516f         //小臂在theta2 = 90°时电机反馈位置
#define ARM_TRAJECTORY_SMALL_AT_THETA2_0_RAD 3.6764f             //待标定：小臂在theta2 = 0°时电机反馈位置；0 表示尚未启用该标定段
#define ARM_TRAJECTORY_SMALL_AT_THETA2_NEG_90_RAD 6.079f        //待标定：小臂在theta2 = -90°时电机反馈位置；0 表示尚未启用该标定段

typedef enum {
    ARM_TP_IDLE = 0,
    ARM_TP_READY,
    ARM_TP_RUNNING,
} ArmTrajectoryState;

/* Values deliberately match ArmPlatform_H7/Arm2R builder return codes. */
typedef enum {
    ARM_TP_ERROR_NONE = 0,
    ARM_TP_ERROR_IK_FAILED,
    ARM_TP_ERROR_BUILD_SPEED_PLAN,
    ARM_TP_ERROR_BUILD_OUT_OF_POINTS,
    ARM_TP_ERROR_BUILD_IK_BOUNDS,
    ARM_TP_ERROR_EMPTY_TRAJ,
    ARM_TP_ERROR_STATE_MACHINE,
} ArmTrajectoryError;

typedef struct {
    float big_position_rad;
    float big_max_speed_rad_s;
    float small_position_rad;
    float small_speed_rad_s;
} ArmTrajectoryMotorCommand;

typedef struct {
    float TargetX;
    float TargetY;
    float MeasureX;
    float MeasureY;
    float BaseMeasure;
    float JointMeasure;
    float CurBasePos;
    float CurJointPos;
    float CurBaseVel;
    float CurJointVel;
    float TargetBase;
    float TargetJoint;
    float PlanMaxSpeedJoint;
    float PlanMaxAccelJoint;
    float time_estimate;
    uint16_t PointCount;
    uint16_t CurrentPoint;
    uint16_t LastTarget;
    uint32_t AntiBlockTick;
    float TrajBase[ARM_TRAJECTORY_MAX_POINTS];
    float TrajJoint[ARM_TRAJECTORY_MAX_POINTS];
    float TrajX[ARM_TRAJECTORY_MAX_POINTS];
    float TrajY[ARM_TRAJECTORY_MAX_POINTS];
    float TrajBaseVel[ARM_TRAJECTORY_MAX_POINTS];
    float TrajJointVel[ARM_TRAJECTORY_MAX_POINTS];
    float TrajVelocityBuffer[ARM_TRAJECTORY_MAX_POINTS];
    ArmTrajectoryState State;
    ArmTrajectoryError Error;
} ArmTrajectory;

void ArmTrajectory_Init(ArmTrajectory* trajectory);

/** Build and start an ArmPlatform_H7-compatible TP_JOINT move to an XY pose. */
bool ArmTrajectory_StartJointXY(ArmTrajectory* trajectory,
                                float target_x,
                                float target_y,
                                float big_feedback_rad,
                                float small_feedback_rad,
                                uint32_t now_tick);

/**
 * Build and start a TP_JOINT move to the requested physical joint angles.
 * theta1_deg is the absolute base-link angle; theta2_deg is the relative
 * elbow angle used by the standard planar 2R forward-kinematics formula.
 * The generated trajectory applies the configured 2/3 theta1 compensation to
 * theta2 according to both joints' motion directions and clamps theta2 to
 * [-145, 145] deg. TargetX/TargetY describe the compensated final pose.
 */
bool ArmTrajectory_StartJointAngles(ArmTrajectory* trajectory,
                                    float theta1_deg,
                                    float theta2_deg,
                                    float big_feedback_rad,
                                    float small_feedback_rad,
                                    uint32_t now_tick);

/** Advance nearest-point tracking and return this cycle's motor setpoints. */
bool ArmTrajectory_Update(ArmTrajectory* trajectory,
                          float big_feedback_rad,
                          float small_feedback_rad,
                          uint32_t now_tick,
                          ArmTrajectoryMotorCommand* command);

void ArmTrajectory_Abort(ArmTrajectory* trajectory);

void ArmTrajectory_MotorToJoint(float big_position_rad,
                                float small_position_rad,
                                float* base,
                                float* joint);

void ArmTrajectory_JointToMotor(float base,
                                float joint,
                                float* big_position_rad,
                                float* small_position_rad);

void ArmTrajectory_GetFinalMotorPosition(const ArmTrajectory* trajectory,
                                         float* big_position_rad,
                                         float* small_position_rad);

#ifdef __cplusplus
}
#endif

#endif /* ARM_TRAJECTORY_H */
