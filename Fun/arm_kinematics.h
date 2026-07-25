#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 机械臂安装方向。
 * 不同方向只影响底座电机（Motor1）的零点偏移，不改变连杆逆解公式。
 */
typedef enum
{
    ARM_DIRECTION_1 = 1,
    ARM_DIRECTION_2 = 2,
    ARM_DIRECTION_3 = 3,
    ARM_DIRECTION_4 = 4
} ArmDirection;

typedef enum
{
    ARM_IK_OK = 0,                       /* 解算成功 */
    ARM_IK_NULL_POINTER,                 /* 传入了空指针 */
    ARM_IK_INVALID_CONFIG,               /* 机械尺寸或传动参数非法 */
    ARM_IK_INVALID_DIRECTION,            /* direction 不在 1~4 范围内 */
    ARM_IK_DIRECTION_NOT_CONFIGURED,     /* 当前方向的底座偏移尚未标定 */
    ARM_IK_TARGET_TOO_CLOSE,             /* 目标点在机械臂最小可达半径以内 */
    ARM_IK_TARGET_TOO_FAR,               /* 目标点超过机械臂最大可达半径 */
    ARM_IK_NUMERIC_ERROR,                /* 输入为 NaN/Inf 或发生数值异常 */
    ARM_IK_MOTOR2_NEGATIVE               /* Motor2 结果低于原机构允许的零点 */
} ArmIkStatus;

typedef struct
{
    /* 机械尺寸，单位 mm；名称与原 robocal.py 中的连杆名称一致。 */
    float oa_mm;
    float ab_mm;
    float bc_mm;
    float cd1_mm;

    /* Gamma 关节的机械最小角度，单位为度。 */
    float gamma_min_deg;

    /* 三个关节的传动比，以及电机旋转一圈对应的脉冲数。 */
    float motor1_ratio;
    float motor2_ratio;
    float motor3_gamma_ratio;
    uint32_t pulses_per_revolution;

    /*
     * 方向 1~4 对应的底座零点偏移，单位为有符号脉冲。
     * valid 为 0 时禁止使用该方向，避免未标定时误动作。
     */
    int32_t direction_offset_pulse[4];
    uint8_t direction_offset_valid[4];
} ArmIkConfig;

typedef struct
{
    int32_t motor1_pulse;
    int32_t motor2_pulse;
    int32_t motor3_pulse;

    /* 中间关节角，单位为度，便于串口调试和机械零点标定。 */
    float alpha_deg;
    float beta_deg;
    float gamma_deg;
} ArmIkResult;

/* 装载与 robocal.py 一致的默认参数；方向 4 默认未标定。 */
void ArmIk_LoadDefaultConfig(ArmIkConfig *config);

/* 设置某个安装方向的底座零点偏移，同时将该方向标记为可用。 */
ArmIkStatus ArmIk_SetDirectionOffset(ArmIkConfig *config,
                                     ArmDirection direction,
                                     int32_t offset_pulse);

/* 将某个安装方向标记为不可用，防止未标定时误动作。 */
ArmIkStatus ArmIk_ClearDirectionOffset(ArmIkConfig *config,
                                       ArmDirection direction);

/*
 * 机械臂逆运动学解算：末端笛卡尔坐标 -> 三个电机目标脉冲。
 *
 * 输入：x/y/z，单位 mm，坐标系与 robocal.py 相同：
 *       Z 为竖直方向；X、Y 位于底座水平面；Alpha = atan2(X, Y)。
 * 输出：相对于各电机已标定零点的有符号绝对目标脉冲。
 *
 * 注意：本函数只负责数学解算，不会发送 CAN 报文，也不会驱动电机；
 *       无 HAL 依赖、无动态内存、无阻塞延时，可在普通任务中直接调用。
 */
ArmIkStatus ArmIk_Solve(const ArmIkConfig *config,
                        ArmDirection direction,
                        float x_mm,
                        float y_mm,
                        float z_mm,
                        ArmIkResult *result);

const char *ArmIk_StatusString(ArmIkStatus status);

#ifdef __cplusplus
}
#endif

#endif /* ARM_KINEMATICS_H */
