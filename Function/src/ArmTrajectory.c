/**
 * @file    ArmTrajectory.c
 * @brief   ArmPlatform_H7 TP_JOINT execution adapted to the current motors
 */

#include "ArmTrajectory.h"
#include "Arm2R.h"
#include <math.h>
#include <stddef.h>

#define ARM_TP_DEG_TO_RAD (3.14159265f / 180.0f)
#define ARM_TP_RAD_TO_DEG (180.0f / 3.14159265f)
#define ARM_TP_TRAJ_DT_MS 2U
#define ARM_TP_TRACK_AHEAD_POINTS 1.0f
#define ARM_TP_FINISH_UNIT_RANGE 1.0f
#define ARM_TP_EXIT_EARLY_POINTS 1U
#define ARM_TP_ANTI_BLOCK_TIMEOUT_TICKS 1000U

/* A 25 unit/s path caps the calibrated big-arm motor just below 0.8 rad/s. */
#define ARM_TP_DEFAULT_MAX_SPEED_JOINT 40.0f
#define ARM_TP_DEFAULT_MAX_ACCEL_JOINT 100.0f
#define ARM_TP_BIG_MIN_SPEED_RAD_S 0.017454f
#define ARM_TP_BIG_FINAL_SPEED_RAD_S 1.0f

#define ARM_TP_BIG_RAD_PER_UNIT \
    ((ARM_TRAJECTORY_BIG_AT_BASE_100_RAD - ARM_TRAJECTORY_BIG_ZERO_RAD) / \
     100.0f)
#define ARM_TP_SMALL_CALIBRATION_EPSILON_RAD 1e-6f
#define ARM_TP_SMALL_THETA2_145_DEG 145.0f
#define ARM_TP_SMALL_THETA2_90_DEG 90.0f
#define ARM_TP_SMALL_THETA2_0_DEG 0.0f
#define ARM_TP_SMALL_THETA2_NEG_90_DEG (-90.0f)
#define ARM_TP_SMALL_THETA2_MIN_DEG (-145.0f)
#define ARM_TP_SMALL_THETA2_MAX_DEG 145.0f
#define ARM_TP_THETA2_COMPENSATION_RATIO (2.0f / 3.0f)
#define ARM_TP_DIRECTION_EPSILON_DEG 1e-4f

typedef struct {
    float theta_a_deg;
    float motor_a_rad;
    float theta_b_deg;
    float motor_b_rad;
} ArmTrajectorySmallCalibrationSegment;

static float ArmTrajectory_Clamp(float value, float minimum, float maximum) {
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

static float ArmTrajectory_LinearMap(float value,
                                     float input_a,
                                     float output_a,
                                     float input_b,
                                     float output_b) {
    float input_span = input_b - input_a;

    if (fabsf(input_span) <= ARM_TP_SMALL_CALIBRATION_EPSILON_RAD)
        return output_a;
    return output_a + (value - input_a) *
                          (output_b - output_a) / input_span;
}

static bool ArmTrajectory_SmallTheta0CalibrationValid(void) {
    float first_delta = ARM_TRAJECTORY_SMALL_AT_THETA2_90_RAD -
                        ARM_TRAJECTORY_SMALL_ZERO_RAD;
    float second_delta = ARM_TRAJECTORY_SMALL_AT_THETA2_0_RAD -
                         ARM_TRAJECTORY_SMALL_AT_THETA2_90_RAD;

    return fabsf(ARM_TRAJECTORY_SMALL_AT_THETA2_0_RAD) >
               ARM_TP_SMALL_CALIBRATION_EPSILON_RAD &&
           fabsf(second_delta) > ARM_TP_SMALL_CALIBRATION_EPSILON_RAD &&
           first_delta * second_delta > 0.0f;
}

static bool ArmTrajectory_SmallNeg90CalibrationValid(void) {
    float second_delta = ARM_TRAJECTORY_SMALL_AT_THETA2_0_RAD -
                         ARM_TRAJECTORY_SMALL_AT_THETA2_90_RAD;
    float third_delta = ARM_TRAJECTORY_SMALL_AT_THETA2_NEG_90_RAD -
                        ARM_TRAJECTORY_SMALL_AT_THETA2_0_RAD;

    return ArmTrajectory_SmallTheta0CalibrationValid() &&
           fabsf(ARM_TRAJECTORY_SMALL_AT_THETA2_NEG_90_RAD) >
               ARM_TP_SMALL_CALIBRATION_EPSILON_RAD &&
           fabsf(third_delta) > ARM_TP_SMALL_CALIBRATION_EPSILON_RAD &&
           second_delta * third_delta > 0.0f;
}

static ArmTrajectorySmallCalibrationSegment
ArmTrajectory_SelectSmallSegmentByTheta(float theta2_deg) {
    ArmTrajectorySmallCalibrationSegment segment;

    if (theta2_deg >= ARM_TP_SMALL_THETA2_90_DEG ||
        !ArmTrajectory_SmallTheta0CalibrationValid()) {
        segment.theta_a_deg = ARM_TP_SMALL_THETA2_145_DEG;
        segment.motor_a_rad = ARM_TRAJECTORY_SMALL_ZERO_RAD;
        segment.theta_b_deg = ARM_TP_SMALL_THETA2_90_DEG;
        segment.motor_b_rad = ARM_TRAJECTORY_SMALL_AT_THETA2_90_RAD;
    } else if (theta2_deg >= ARM_TP_SMALL_THETA2_0_DEG ||
               !ArmTrajectory_SmallNeg90CalibrationValid()) {
        segment.theta_a_deg = ARM_TP_SMALL_THETA2_90_DEG;
        segment.motor_a_rad = ARM_TRAJECTORY_SMALL_AT_THETA2_90_RAD;
        segment.theta_b_deg = ARM_TP_SMALL_THETA2_0_DEG;
        segment.motor_b_rad = ARM_TRAJECTORY_SMALL_AT_THETA2_0_RAD;
    } else {
        segment.theta_a_deg = ARM_TP_SMALL_THETA2_0_DEG;
        segment.motor_a_rad = ARM_TRAJECTORY_SMALL_AT_THETA2_0_RAD;
        segment.theta_b_deg = ARM_TP_SMALL_THETA2_NEG_90_DEG;
        segment.motor_b_rad = ARM_TRAJECTORY_SMALL_AT_THETA2_NEG_90_RAD;
    }
    return segment;
}

static ArmTrajectorySmallCalibrationSegment
ArmTrajectory_SelectSmallSegmentByMotor(float motor_position_rad) {
    ArmTrajectorySmallCalibrationSegment segment =
        ArmTrajectory_SelectSmallSegmentByTheta(
            ARM_TP_SMALL_THETA2_145_DEG);
    float direction = (ARM_TRAJECTORY_SMALL_AT_THETA2_90_RAD >=
                       ARM_TRAJECTORY_SMALL_ZERO_RAD)
                          ? 1.0f
                          : -1.0f;

    if (!ArmTrajectory_SmallTheta0CalibrationValid() ||
        (motor_position_rad - ARM_TRAJECTORY_SMALL_AT_THETA2_90_RAD) *
                direction <=
            0.0f) {
        return segment;
    }

    segment = ArmTrajectory_SelectSmallSegmentByTheta(
        ARM_TP_SMALL_THETA2_0_DEG);
    if (!ArmTrajectory_SmallNeg90CalibrationValid() ||
        (motor_position_rad - ARM_TRAJECTORY_SMALL_AT_THETA2_0_RAD) *
                direction <=
            0.0f) {
        return segment;
    }

    return ArmTrajectory_SelectSmallSegmentByTheta(
        ARM_TP_SMALL_THETA2_NEG_90_DEG);
}

static float ArmTrajectory_SmallMotorToJoint(float motor_position_rad) {
    ArmTrajectorySmallCalibrationSegment segment =
        ArmTrajectory_SelectSmallSegmentByMotor(motor_position_rad);
    float theta2_deg = ArmTrajectory_LinearMap(motor_position_rad,
                                               segment.motor_a_rad,
                                               segment.theta_a_deg,
                                               segment.motor_b_rad,
                                               segment.theta_b_deg);

    return (ARM_TRAJECTORY_JOINT_OFFSET_DEG - theta2_deg) /
           ARM_TRAJECTORY_JOINT_DEG_PER_UNIT;
}

static float ArmTrajectory_SmallJointToMotor(float joint) {
    float theta2_deg = ARM_TRAJECTORY_JOINT_OFFSET_DEG -
                       joint * ARM_TRAJECTORY_JOINT_DEG_PER_UNIT;
    ArmTrajectorySmallCalibrationSegment segment =
        ArmTrajectory_SelectSmallSegmentByTheta(theta2_deg);

    return ArmTrajectory_LinearMap(theta2_deg,
                                   segment.theta_a_deg,
                                   segment.motor_a_rad,
                                   segment.theta_b_deg,
                                   segment.motor_b_rad);
}

static float ArmTrajectory_SmallMotorRadPerJointUnit(float joint) {
    float theta2_deg = ARM_TRAJECTORY_JOINT_OFFSET_DEG -
                       joint * ARM_TRAJECTORY_JOINT_DEG_PER_UNIT;
    ArmTrajectorySmallCalibrationSegment segment =
        ArmTrajectory_SelectSmallSegmentByTheta(theta2_deg);

    return (segment.motor_b_rad - segment.motor_a_rad) /
           (segment.theta_b_deg - segment.theta_a_deg) *
           (-ARM_TRAJECTORY_JOINT_DEG_PER_UNIT);
}

static float ArmTrajectory_Theta1FromBase(float base) {
    return base * ARM_TRAJECTORY_BASE_DEG_PER_UNIT;
}

static float ArmTrajectory_Theta2FromJoint(float joint) {
    return ARM_TRAJECTORY_JOINT_OFFSET_DEG -
           joint * ARM_TRAJECTORY_JOINT_DEG_PER_UNIT;
}

static float ArmTrajectory_JointFromTheta2(float theta2_deg) {
    return (ARM_TRAJECTORY_JOINT_OFFSET_DEG - theta2_deg) /
           ARM_TRAJECTORY_JOINT_DEG_PER_UNIT;
}

/**
 * Opposite joint directions use positive compensation; matching directions
 * use negative compensation.  A stationary joint disables compensation.
 */
static float ArmTrajectory_SelectTheta2CompensationSign(
    float current_theta1_deg,
    float current_theta2_deg,
    float target_theta1_deg,
    float target_theta2_deg) {
    float delta_theta1 = target_theta1_deg - current_theta1_deg;
    float delta_theta2 = target_theta2_deg - current_theta2_deg;

    if (fabsf(delta_theta1) <= ARM_TP_DIRECTION_EPSILON_DEG ||
        fabsf(delta_theta2) <= ARM_TP_DIRECTION_EPSILON_DEG) {
        return 0.0f;
    }
    return delta_theta1 * delta_theta2 < 0.0f ? 1.0f : -1.0f;
}

static void ArmTrajectory_ApplyTheta2Compensation(
    ArmTrajectory* trajectory,
    float compensation_sign) {
    uint16_t i;

    if (trajectory == NULL || compensation_sign == 0.0f)
        return;

    for (i = 0U; i < trajectory->PointCount; i++) {
        float theta1_deg =
            ArmTrajectory_Theta1FromBase(trajectory->TrajBase[i]);
        float theta2_deg =
            ArmTrajectory_Theta2FromJoint(trajectory->TrajJoint[i]);
        float compensated_theta2_deg =
            theta2_deg + compensation_sign *
                             ARM_TP_THETA2_COMPENSATION_RATIO *
                             theta1_deg;
        bool was_clamped =
            compensated_theta2_deg < ARM_TP_SMALL_THETA2_MIN_DEG ||
            compensated_theta2_deg > ARM_TP_SMALL_THETA2_MAX_DEG;

        compensated_theta2_deg = ArmTrajectory_Clamp(
            compensated_theta2_deg,
            ARM_TP_SMALL_THETA2_MIN_DEG,
            ARM_TP_SMALL_THETA2_MAX_DEG);
        trajectory->TrajJoint[i] =
            ArmTrajectory_JointFromTheta2(compensated_theta2_deg);

        if (was_clamped) {
            trajectory->TrajJointVel[i] = 0.0f;
        } else {
            float theta1_velocity_deg_s =
                trajectory->TrajBaseVel[i] *
                ARM_TRAJECTORY_BASE_DEG_PER_UNIT;
            float theta2_velocity_deg_s =
                -trajectory->TrajJointVel[i] *
                ARM_TRAJECTORY_JOINT_DEG_PER_UNIT;
            float compensated_theta2_velocity_deg_s =
                theta2_velocity_deg_s + compensation_sign *
                                                ARM_TP_THETA2_COMPENSATION_RATIO *
                                                theta1_velocity_deg_s;

            trajectory->TrajJointVel[i] =
                -compensated_theta2_velocity_deg_s /
                ARM_TRAJECTORY_JOINT_DEG_PER_UNIT;
        }
    }
}

void ArmTrajectory_MotorToJoint(float big_position_rad,
                                float small_position_rad,
                                float* base,
                                float* joint) {
    if (base != NULL) {
        *base = (big_position_rad - ARM_TRAJECTORY_BIG_ZERO_RAD) /
                ARM_TP_BIG_RAD_PER_UNIT;
    }
    if (joint != NULL) {
        *joint = ArmTrajectory_SmallMotorToJoint(small_position_rad);
    }
}

void ArmTrajectory_JointToMotor(float base,
                                float joint,
                                float* big_position_rad,
                                float* small_position_rad) {
    if (big_position_rad != NULL) {
        *big_position_rad = ARM_TRAJECTORY_BIG_ZERO_RAD +
                            base * ARM_TP_BIG_RAD_PER_UNIT;
    }
    if (small_position_rad != NULL) {
        *small_position_rad = ArmTrajectory_SmallJointToMotor(joint);
    }
}

static void ArmTrajectory_UpdateMeasure(ArmTrajectory* trajectory,
                                        float big_feedback_rad,
                                        float small_feedback_rad) {
    ArmTrajectory_MotorToJoint(big_feedback_rad,
                               small_feedback_rad,
                               &trajectory->BaseMeasure,
                               &trajectory->JointMeasure);
    Arm2R_FK(trajectory->BaseMeasure,
             trajectory->JointMeasure,
             ARM_TRAJECTORY_L1_MM,
             ARM_TRAJECTORY_L2_MM,
             ARM_TRAJECTORY_BASE_DEG_PER_UNIT,
             ARM_TRAJECTORY_JOINT_DEG_PER_UNIT,
             ARM_TRAJECTORY_JOINT_OFFSET_DEG,
             ARM_TP_DEG_TO_RAD,
             &trajectory->MeasureX,
             &trajectory->MeasureY);
}

void ArmTrajectory_Init(ArmTrajectory* trajectory) {
    uint16_t i;

    if (trajectory == NULL)
        return;

    trajectory->TargetX = 0.0f;
    trajectory->TargetY = 0.0f;
    trajectory->MeasureX = 0.0f;
    trajectory->MeasureY = 0.0f;
    trajectory->BaseMeasure = 0.0f;
    trajectory->JointMeasure = 0.0f;
    trajectory->CurBasePos = 0.0f;
    trajectory->CurJointPos = 0.0f;
    trajectory->CurBaseVel = 0.0f;
    trajectory->CurJointVel = 0.0f;
    trajectory->TargetBase = 0.0f;
    trajectory->TargetJoint = 0.0f;
    trajectory->PlanMaxSpeedJoint = ARM_TP_DEFAULT_MAX_SPEED_JOINT;
    trajectory->PlanMaxAccelJoint = ARM_TP_DEFAULT_MAX_ACCEL_JOINT;
    trajectory->time_estimate = 0.0f;
    trajectory->PointCount = 0U;
    trajectory->CurrentPoint = 0U;
    trajectory->LastTarget = 0U;
    trajectory->AntiBlockTick = 0U;
    trajectory->State = ARM_TP_IDLE;
    trajectory->Error = ARM_TP_ERROR_NONE;

    for (i = 0U; i < ARM_TRAJECTORY_MAX_POINTS; i++) {
        trajectory->TrajBase[i] = 0.0f;
        trajectory->TrajJoint[i] = 0.0f;
        trajectory->TrajX[i] = 0.0f;
        trajectory->TrajY[i] = 0.0f;
        trajectory->TrajBaseVel[i] = 0.0f;
        trajectory->TrajJointVel[i] = 0.0f;
        trajectory->TrajVelocityBuffer[i] = 0.0f;
    }
}

static void ArmTrajectory_SelectSolution(float b1,
                                         float j1,
                                         float b2,
                                         float j2,
                                         bool v1,
                                         bool v2,
                                         float* b_out,
                                         float* j_out) {
    if (v1 && v2) {
        if (fabsf(j1) < fabsf(j2)) {
            *b_out = b1;
            *j_out = j1;
        } else {
            *b_out = b2;
            *j_out = j2;
        }
    } else {
        *b_out = v1 ? b1 : b2;
        *j_out = v1 ? j1 : j2;
    }
}

static void ArmTrajectory_BuildTrajXY(ArmTrajectory* trajectory) {
    uint16_t count = trajectory->PointCount;
    uint16_t i;

    if (count > ARM_TRAJECTORY_MAX_POINTS)
        count = ARM_TRAJECTORY_MAX_POINTS;

    for (i = 0U; i < count; i++) {
        Arm2R_FK(trajectory->TrajBase[i],
                 trajectory->TrajJoint[i],
                 ARM_TRAJECTORY_L1_MM,
                 ARM_TRAJECTORY_L2_MM,
                 ARM_TRAJECTORY_BASE_DEG_PER_UNIT,
                 ARM_TRAJECTORY_JOINT_DEG_PER_UNIT,
                 ARM_TRAJECTORY_JOINT_OFFSET_DEG,
                 ARM_TP_DEG_TO_RAD,
                 &trajectory->TrajX[i],
                 &trajectory->TrajY[i]);
    }
}

static bool ArmTrajectory_StartJointTarget(ArmTrajectory* trajectory,
                                           float target_base,
                                           float target_joint,
                                           float target_x,
                                           float target_y,
                                           float big_feedback_rad,
                                           float small_feedback_rad,
                                           float theta2_compensation_sign,
                                           uint32_t now_tick) {
    float plan_start_joint;
    int point_count = 0;
    uint8_t ret_code;

    if (trajectory == NULL || !isfinite(target_base) ||
        !isfinite(target_joint) || !isfinite(target_x) ||
        !isfinite(target_y) || !isfinite(big_feedback_rad) ||
        !isfinite(small_feedback_rad)) {
        return false;
    }
    if (trajectory->State != ARM_TP_IDLE) {
        trajectory->Error = ARM_TP_ERROR_STATE_MACHINE;
        return false;
    }

    ArmTrajectory_UpdateMeasure(trajectory,
                                big_feedback_rad,
                                small_feedback_rad);
    plan_start_joint = trajectory->JointMeasure;
    if (theta2_compensation_sign != 0.0f) {
        float current_theta1_deg =
            ArmTrajectory_Theta1FromBase(trajectory->BaseMeasure);
        float current_theta2_deg =
            ArmTrajectory_Theta2FromJoint(trajectory->JointMeasure);
        float nominal_start_theta2_deg = current_theta2_deg -
            theta2_compensation_sign * ARM_TP_THETA2_COMPENSATION_RATIO *
                current_theta1_deg;

        nominal_start_theta2_deg = ArmTrajectory_Clamp(
            nominal_start_theta2_deg,
            ARM_TP_SMALL_THETA2_MIN_DEG,
            ARM_TP_SMALL_THETA2_MAX_DEG);
        plan_start_joint =
            ArmTrajectory_JointFromTheta2(nominal_start_theta2_deg);
    }
    trajectory->TargetX = target_x;
    trajectory->TargetY = target_y;
    trajectory->TargetBase = target_base;
    trajectory->TargetJoint = target_joint;
    trajectory->Error = ARM_TP_ERROR_NONE;
    trajectory->PointCount = 0U;
    trajectory->CurrentPoint = 0U;
    trajectory->State = ARM_TP_READY;

    ret_code = Arm2R_BuildJoint(trajectory->BaseMeasure,
                                plan_start_joint,
                                trajectory->TargetBase,
                                trajectory->TargetJoint,
                                trajectory->PlanMaxSpeedJoint,
                                trajectory->PlanMaxAccelJoint,
                                (int)ARM_TP_TRAJ_DT_MS,
                                (int)ARM_TRAJECTORY_MAX_POINTS,
                                trajectory->TrajVelocityBuffer,
                                trajectory->TrajBase,
                                trajectory->TrajJoint,
                                trajectory->TrajBaseVel,
                                trajectory->TrajJointVel,
                                &point_count,
                                &trajectory->time_estimate);
    if (ret_code != 0U) {
        trajectory->Error = (ArmTrajectoryError)ret_code;
        trajectory->State = ARM_TP_IDLE;
        return false;
    }
    if (point_count <= 0 || point_count > (int)ARM_TRAJECTORY_MAX_POINTS) {
        trajectory->Error = ARM_TP_ERROR_EMPTY_TRAJ;
        trajectory->State = ARM_TP_IDLE;
        return false;
    }

    trajectory->PointCount = (uint16_t)point_count;
    ArmTrajectory_ApplyTheta2Compensation(trajectory,
                                          theta2_compensation_sign);
    trajectory->TargetBase = trajectory->TrajBase[point_count - 1];
    trajectory->TargetJoint = trajectory->TrajJoint[point_count - 1];
    Arm2R_FK(trajectory->TargetBase,
             trajectory->TargetJoint,
             ARM_TRAJECTORY_L1_MM,
             ARM_TRAJECTORY_L2_MM,
             ARM_TRAJECTORY_BASE_DEG_PER_UNIT,
             ARM_TRAJECTORY_JOINT_DEG_PER_UNIT,
             ARM_TRAJECTORY_JOINT_OFFSET_DEG,
             ARM_TP_DEG_TO_RAD,
             &trajectory->TargetX,
             &trajectory->TargetY);
    ArmTrajectory_BuildTrajXY(trajectory);
    trajectory->CurrentPoint = 0U;
    trajectory->LastTarget = 0U;
    trajectory->AntiBlockTick = now_tick;
    trajectory->State = ARM_TP_RUNNING;
    return true;
}

bool ArmTrajectory_StartJointXY(ArmTrajectory* trajectory,
                                float target_x,
                                float target_y,
                                float big_feedback_rad,
                                float small_feedback_rad,
                                uint32_t now_tick) {
    float b1 = 0.0f;
    float j1 = 0.0f;
    float b2 = 0.0f;
    float j2 = 0.0f;
    bool v1 = false;
    bool v2 = false;

    if (trajectory == NULL || !isfinite(target_x) || !isfinite(target_y) ||
        !isfinite(big_feedback_rad) || !isfinite(small_feedback_rad)) {
        return false;
    }
    if (trajectory->State != ARM_TP_IDLE) {
        trajectory->Error = ARM_TP_ERROR_STATE_MACHINE;
        return false;
    }

    if (!Arm2R_IK(target_x,
                  target_y,
                  ARM_TRAJECTORY_L1_MM,
                  ARM_TRAJECTORY_L2_MM,
                  ARM_TP_RAD_TO_DEG /
                      ARM_TRAJECTORY_BASE_DEG_PER_UNIT,
                  ARM_TP_RAD_TO_DEG /
                      ARM_TRAJECTORY_JOINT_DEG_PER_UNIT,
                  ARM_TRAJECTORY_JOINT_OFFSET_DEG /
                      ARM_TRAJECTORY_JOINT_DEG_PER_UNIT,
                  true,
                  &b1,
                  &j1,
                  &b2,
                  &j2,
                  &v1,
                  &v2)) {
        trajectory->Error = ARM_TP_ERROR_IK_FAILED;
        return false;
    }

    ArmTrajectory_SelectSolution(b1,
                                 j1,
                                 b2,
                                 j2,
                                 v1,
                                 v2,
                                 &b1,
                                 &j1);

    return ArmTrajectory_StartJointTarget(trajectory,
                                          b1,
                                          j1,
                                          target_x,
                                          target_y,
                                          big_feedback_rad,
                                          small_feedback_rad,
                                          0.0f,
                                          now_tick);
}

bool ArmTrajectory_StartJointAngles(ArmTrajectory* trajectory,
                                    float theta1_deg,
                                    float theta2_deg,
                                    float big_feedback_rad,
                                    float small_feedback_rad,
                                    uint32_t now_tick) {
    float target_base;
    float target_joint;
    float target_x;
    float target_y;
    float current_base;
    float current_joint;
    float current_theta1_deg;
    float current_theta2_deg;
    float theta2_compensation_sign;

    if (trajectory == NULL || !isfinite(theta1_deg) ||
        !isfinite(theta2_deg) || !isfinite(big_feedback_rad) ||
        !isfinite(small_feedback_rad)) {
        return false;
    }
    if (trajectory->State != ARM_TP_IDLE) {
        trajectory->Error = ARM_TP_ERROR_STATE_MACHINE;
        return false;
    }

    theta2_deg = ArmTrajectory_Clamp(theta2_deg,
                                     ARM_TP_SMALL_THETA2_MIN_DEG,
                                     ARM_TP_SMALL_THETA2_MAX_DEG);
    target_base = theta1_deg / ARM_TRAJECTORY_BASE_DEG_PER_UNIT;
    target_joint = ArmTrajectory_JointFromTheta2(theta2_deg);
    if (target_base < -ARM2R_IK_TOLERANCE ||
        target_base > 100.0f + ARM2R_IK_TOLERANCE ||
        target_joint < -ARM2R_IK_TOLERANCE ||
        target_joint > 100.0f + ARM2R_IK_TOLERANCE) {
        trajectory->Error = ARM_TP_ERROR_IK_FAILED;
        return false;
    }

    ArmTrajectory_MotorToJoint(big_feedback_rad,
                               small_feedback_rad,
                               &current_base,
                               &current_joint);
    current_theta1_deg = ArmTrajectory_Theta1FromBase(current_base);
    current_theta2_deg = ArmTrajectory_Theta2FromJoint(current_joint);
    theta2_compensation_sign =
        ArmTrajectory_SelectTheta2CompensationSign(current_theta1_deg,
                                                    current_theta2_deg,
                                                    theta1_deg,
                                                    theta2_deg);

    Arm2R_FK(target_base,
             target_joint,
             ARM_TRAJECTORY_L1_MM,
             ARM_TRAJECTORY_L2_MM,
             ARM_TRAJECTORY_BASE_DEG_PER_UNIT,
             ARM_TRAJECTORY_JOINT_DEG_PER_UNIT,
             ARM_TRAJECTORY_JOINT_OFFSET_DEG,
             ARM_TP_DEG_TO_RAD,
             &target_x,
             &target_y);

    return ArmTrajectory_StartJointTarget(trajectory,
                                          target_base,
                                          target_joint,
                                          target_x,
                                          target_y,
                                          big_feedback_rad,
                                          small_feedback_rad,
                                          theta2_compensation_sign,
                                          now_tick);
}

static uint16_t ArmTrajectory_FindNearestJoint(const ArmTrajectory* trajectory) {
    uint16_t best = 0U;
    uint16_t i;
    float best_d2 = 1e30f;

    for (i = 0U; i < trajectory->PointCount; i++) {
        float db = trajectory->TrajBase[i] - trajectory->BaseMeasure;
        float dj = trajectory->TrajJoint[i] - trajectory->JointMeasure;
        float d2 = db * db + dj * dj;

        if (d2 < best_d2) {
            best_d2 = d2;
            best = i;
        }
    }
    return best;
}

static uint16_t ArmTrajectory_GetTrackPoint(ArmTrajectory* trajectory,
                                            uint32_t now_tick) {
    uint16_t nearest = ArmTrajectory_FindNearestJoint(trajectory);
    uint16_t last_idx = (uint16_t)(trajectory->PointCount - 1U);
    uint16_t target = nearest;

    if (target < trajectory->LastTarget)
        target = trajectory->LastTarget;

    if (target == trajectory->LastTarget) {
        uint32_t elapsed = now_tick - trajectory->AntiBlockTick;
        if (elapsed >= ARM_TP_ANTI_BLOCK_TIMEOUT_TICKS && target < last_idx) {
            uint32_t extra_points =
                elapsed / ARM_TP_ANTI_BLOCK_TIMEOUT_TICKS;
            uint32_t room = (uint32_t)last_idx - (uint32_t)target;
            if (extra_points > room)
                extra_points = room;
            target = (uint16_t)((uint32_t)target + extra_points);
            trajectory->AntiBlockTick +=
                extra_points * ARM_TP_ANTI_BLOCK_TIMEOUT_TICKS;
        }
    } else {
        trajectory->AntiBlockTick = now_tick;
        trajectory->LastTarget = target;
    }

    /* Preserve ArmPlatform_H7's integer-clamped pre-lookahead behavior. */
    {
        uint32_t target_u32 =
            (uint32_t)((float)target + ARM_TP_TRACK_AHEAD_POINTS);
        if (target_u32 > (uint32_t)last_idx)
            target_u32 = (uint32_t)last_idx;
        target = (uint16_t)target_u32;
    }
    return target;
}

static void ArmTrajectory_ApplyPoint(ArmTrajectory* trajectory,
                                     uint16_t point_idx) {
    uint16_t count = trajectory->PointCount;
    uint16_t target_idx = point_idx;
    float t = 0.0f;
    float target_pos_f;
    uint16_t idx;
    float frac;

    if (target_idx < count - 1U) {
        uint16_t next_idx = target_idx + 1U;
        float ab_base = trajectory->TrajBase[next_idx] -
                        trajectory->TrajBase[target_idx];
        float ab_joint = trajectory->TrajJoint[next_idx] -
                         trajectory->TrajJoint[target_idx];
        float am_base = trajectory->BaseMeasure -
                        trajectory->TrajBase[target_idx];
        float am_joint = trajectory->JointMeasure -
                         trajectory->TrajJoint[target_idx];
        float ab_sq = ab_base * ab_base + ab_joint * ab_joint;

        if (ab_sq > 1e-6f) {
            t = (am_base * ab_base + am_joint * ab_joint) / ab_sq;
            t = ArmTrajectory_Clamp(t, 0.0f, 1.0f);
        }
    }

    target_pos_f = (float)target_idx + t + ARM_TP_TRACK_AHEAD_POINTS;
    target_pos_f = ArmTrajectory_Clamp(target_pos_f,
                                       0.0f,
                                       (float)(count - 1U));
    idx = (uint16_t)target_pos_f;
    frac = target_pos_f - (float)idx;

    if (idx >= count - 1U) {
        trajectory->CurBasePos = trajectory->TrajBase[count - 1U];
        trajectory->CurJointPos = trajectory->TrajJoint[count - 1U];
        trajectory->CurBaseVel = trajectory->TrajBaseVel[count - 1U];
        trajectory->CurJointVel = trajectory->TrajJointVel[count - 1U];
    } else {
        trajectory->CurBasePos =
            trajectory->TrajBase[idx] +
            frac * (trajectory->TrajBase[idx + 1U] -
                    trajectory->TrajBase[idx]);
        trajectory->CurJointPos =
            trajectory->TrajJoint[idx] +
            frac * (trajectory->TrajJoint[idx + 1U] -
                    trajectory->TrajJoint[idx]);
        trajectory->CurBaseVel =
            trajectory->TrajBaseVel[idx] +
            frac * (trajectory->TrajBaseVel[idx + 1U] -
                    trajectory->TrajBaseVel[idx]);
        trajectory->CurJointVel =
            trajectory->TrajJointVel[idx] +
            frac * (trajectory->TrajJointVel[idx + 1U] -
                    trajectory->TrajJointVel[idx]);
    }
}

static void ArmTrajectory_AtTarget(ArmTrajectory* trajectory) {
    uint16_t last_idx = (uint16_t)(trajectory->PointCount - 1U);

    trajectory->CurBasePos = trajectory->TrajBase[last_idx];
    trajectory->CurJointPos = trajectory->TrajJoint[last_idx];
    trajectory->CurBaseVel = 0.0f;
    trajectory->CurJointVel = 0.0f;
    trajectory->State = ARM_TP_IDLE;
}

static void ArmTrajectory_FillMotorCommand(
    const ArmTrajectory* trajectory,
    ArmTrajectoryMotorCommand* command) {
    float big_speed;

    ArmTrajectory_JointToMotor(trajectory->CurBasePos,
                               trajectory->CurJointPos,
                               &command->big_position_rad,
                               &command->small_position_rad);
    big_speed = fabsf(trajectory->CurBaseVel * ARM_TP_BIG_RAD_PER_UNIT);
    if (trajectory->State == ARM_TP_IDLE) {
        big_speed = ARM_TP_BIG_FINAL_SPEED_RAD_S;
    } else {
        big_speed = ArmTrajectory_Clamp(big_speed,
                                        ARM_TP_BIG_MIN_SPEED_RAD_S,
                                        ARM_TP_BIG_FINAL_SPEED_RAD_S);
    }
    command->big_max_speed_rad_s = big_speed;
    command->small_speed_rad_s =
        trajectory->CurJointVel *
        ArmTrajectory_SmallMotorRadPerJointUnit(trajectory->CurJointPos);
}

bool ArmTrajectory_Update(ArmTrajectory* trajectory,
                          float big_feedback_rad,
                          float small_feedback_rad,
                          uint32_t now_tick,
                          ArmTrajectoryMotorCommand* command) {
    uint16_t last_idx;
    uint16_t target_idx;

    if (trajectory == NULL || command == NULL ||
        !isfinite(big_feedback_rad) || !isfinite(small_feedback_rad) ||
        trajectory->PointCount == 0U) {
        return false;
    }

    ArmTrajectory_UpdateMeasure(trajectory,
                                big_feedback_rad,
                                small_feedback_rad);

    if (trajectory->State == ARM_TP_RUNNING) {
        if (trajectory->PointCount == 1U) {
            ArmTrajectory_AtTarget(trajectory);
        } else {
            last_idx = (uint16_t)(trajectory->PointCount - 1U);
            target_idx = ArmTrajectory_GetTrackPoint(trajectory, now_tick);
            ArmTrajectory_ApplyPoint(trajectory, target_idx);
            trajectory->CurrentPoint = target_idx;

            if (ARM_TP_EXIT_EARLY_POINTS > 0U &&
                (uint32_t)target_idx + ARM_TP_EXIT_EARLY_POINTS >=
                    (uint32_t)last_idx) {
                ArmTrajectory_AtTarget(trajectory);
            } else if (fabsf(trajectory->TrajBase[last_idx] -
                             trajectory->BaseMeasure) <=
                           ARM_TP_FINISH_UNIT_RANGE &&
                       fabsf(trajectory->TrajJoint[last_idx] -
                             trajectory->JointMeasure) <=
                           ARM_TP_FINISH_UNIT_RANGE) {
                ArmTrajectory_AtTarget(trajectory);
            }
        }
    }

    ArmTrajectory_FillMotorCommand(trajectory, command);
    return true;
}

void ArmTrajectory_Abort(ArmTrajectory* trajectory) {
    if (trajectory == NULL)
        return;

    trajectory->State = ARM_TP_IDLE;
    trajectory->PointCount = 0U;
    trajectory->CurrentPoint = 0U;
    trajectory->LastTarget = 0U;
    trajectory->AntiBlockTick = 0U;
    trajectory->CurBaseVel = 0.0f;
    trajectory->CurJointVel = 0.0f;
}

void ArmTrajectory_GetFinalMotorPosition(const ArmTrajectory* trajectory,
                                         float* big_position_rad,
                                         float* small_position_rad) {
    if (trajectory == NULL)
        return;

    ArmTrajectory_JointToMotor(trajectory->TargetBase,
                               trajectory->TargetJoint,
                               big_position_rad,
                               small_position_rad);
}
