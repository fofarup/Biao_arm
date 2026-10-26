/*
  程序说明: 底盘直线纠偏、相对/绝对动作、静止保持及定位异常恢复。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-26
*/

#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

/*
 * 文件作用：底盘控制器。
 * 速度模式按车体速度指令生成轮速；位置模式使用 OPS9/HW101 做外部闭环，
 * 再通过麦克纳姆轮解算生成四个电机的 F6 速度指令。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "X_V2.h"
#include "chassis_mecanum.h"

/* 底盘各位置、航向、横向纠偏及保持环共用这一组 PID 参数。
 * P 单位 1/s，I 单位 1/s²，D 无量纲；KI 为 0 时关闭积分。
 * 各环分别保存积分状态，位置和航向之间不共用累计误差。
 * 加减速及速度上限均为电机侧单位，车体侧限制由运动学换算。 */
#define CHASSIS_KP                  (1.6f)
#define CHASSIS_KI                  (0.01f)
#define CHASSIS_KD                  (0.055f)
#define CHASSIS_ACC_MAX_RPM_S       (800.0f)
#define CHASSIS_DEC_MAX_RPM_S       (1200.0f)
#define CHASSIS_SPEED_MAX_RPM       (3000.0f)

/* OPS 固定坐标转车体坐标：-1 为 R(-yaw)；车体位移转 OPS 坐标用其逆变换。 */
#define CHASSIS_COORD_ROTATION_SIGN (-1.0f)
/*
 * 控制器数据流：
 * OPS9 X/Y + HW101 航向角
 *          ↓
 * 位置/速度控制与加减速限制
 *          ↓
 * 麦克纳姆轮解算
 *          ↓
 * 四个带符号逻辑 RPM，再转换为 X 固件方向位。
 */
#define CHASSIS_PI                 (3.14159265358979323846f) /* 圆周率，用于角度换算。 */
#define CHASSIS_MIN_DT_S           (0.001f)  /* 控制周期下限，单位秒。 */
#define CHASSIS_MAX_DT_S           (0.100f)  /* 控制周期上限，单位秒。 */
#define CHASSIS_DEFAULT_SENSOR_TIMEOUT_MS CHASSIS_SENSOR_FAULT_TIMEOUT_MS
#define CHASSIS_SENSOR_HOLD_MS              (200U) /* 暂停门限，不是锁存故障门限。 */
#define CHASSIS_SENSOR_RECOVERY_STABLE_MS    (100U)
#define CHASSIS_SENSOR_RECOVERY_MIN_FRAMES   (5U)   /* 按新帧计数，重复读取缓存不计数。 */
#define CHASSIS_SENSOR_POSITION_MARGIN_MM   (20.0f)
#define CHASSIS_SENSOR_YAW_MARGIN_DEG        (5.0f)
/* 顺序动作的阶段切换与最终到位分别判定；XY 误差为到终点的平面距离。 */
#define CHASSIS_SEQUENCE_TRANSLATE_TOLERANCE_MM (200.0f) /* 开始边平移边旋转的 XY 距离，不是停车距离，单位毫米。 */
#define CHASSIS_SEQUENCE_POSITION_TOLERANCE_MM (5.0f) /* 整个动作最终 XY 到位误差，单位毫米。 */
#define CHASSIS_SEQUENCE_YAW_TOLERANCE_DEG     (0.2f) /* 整个动作最终航向到位误差，单位度。 */
#define CHASSIS_SEQUENCE_TRANSLATE_SETTLE_CYCLES (3U) /* 最终到位所需的连续新采样次数，过渡不等待停稳。 */
#define CHASSIS_SEQUENCE_TRANSLATE_SPEED_TOLERANCE_MM_S (20.0f) /* 最终到位的实测平移速度上限，不用于进入过渡阶段。 */
#define CHASSIS_SEQUENCE_WAIT_PERIOD_MS        (20U) /* 等待动作完成的检查间隔，单位毫秒。 */
#define CHASSIS_SEQUENCE_TIMEOUT_MS         (30000U) /* 单次动作超时，单位毫秒。 */
#define CHASSIS_STRAIGHT_TURN_EPSILON        (0.0001f) /* 防止零位移或零转角参与除法。 */
#define CHASSIS_STRAIGHT_TURN_PROGRESS_LEAD  (0.10f) /* 参考允许领先较慢实测轴的进度。 */
#define CHASSIS_STRAIGHT_TURN_SETTLE_CYCLES (3U) /* XY、yaw 连续三个控制周期达标。 */
#define CHASSIS_STRAIGHT_TURN_VELOCITY_ALPHA (0.35f) /* 新传感器帧的速度低通系数。 */
#define CHASSIS_STRAIGHT_TURN_BRAKE_MARGIN   (0.65f) /* 实际驱动响应不等于软件斜率，提前制动。 */
#define CHASSIS_STRAIGHT_TURN_CURVE_RESERVE  (0.50f) /* 轮速变化额度一半留给车身转向。 */
#define CHASSIS_STRAIGHT_TURN_LEAD_FILTER_S  (0.15f) /* 实测轴进度噪声不直接改变进度限速。 */
#define CHASSIS_STRAIGHT_TURN_BRAKE_LAG_S    (0.04f) /* 传感器、任务与驱动响应余量。 */
#define CHASSIS_STRAIGHT_TURN_BRAKE_FLOOR    (0.20f) /* 一轴领先时仍允许另一轴追上。 */
#define CHASSIS_STRAIGHT_TURN_SPEED_TOLERANCE_MM_S (10.0f)
#define CHASSIS_STRAIGHT_TURN_YAW_RATE_TOLERANCE_DEG_S (2.0f)
#define CHASSIS_STRAIGHT_TURN_WHEEL_TOLERANCE_RPM (2.0f)


#define CHASSIS_SENSOR_FAULT_TIMEOUT_MS (3000U)
#define CHASSIS_SENSOR_FRESH_MS          (100U)

typedef enum
{
    CHASSIS_CONTROL_STOP = 0U, /* 受控停止。 */
    CHASSIS_CONTROL_VELOCITY,  /* 速度指令模式。 */
    CHASSIS_CONTROL_POSE,      /* OPS9/HW101 位置闭环模式。 */
    CHASSIS_CONTROL_FAULT,     /* 故障停车，数值仍为 0x03。 */
    CHASSIS_CONTROL_SENSOR_HOLD /* 传感器暂时掉帧，减速并保留原动作。 */
} ChassisControlMode_t;

/* 按位记录故障来源；只有 OPS9/HW101 超时允许自动恢复原位置动作。 */
typedef enum
{
    CHASSIS_FAULT_NONE = 0U,
    CHASSIS_FAULT_OPS9_TIMEOUT = 1U,
    CHASSIS_FAULT_HW101_TIMEOUT = 2U,
    CHASSIS_FAULT_POSITION_JUMP = 4U,
    CHASSIS_FAULT_YAW_UNCERTAIN = 8U,
    CHASSIS_FAULT_INVALID_SAMPLE = 16U,
    CHASSIS_FAULT_CAN_TX = 32U,
    CHASSIS_FAULT_CONTROL = 64U,
    CHASSIS_FAULT_INITIALIZATION = 128U
} ChassisFaultReason_t;

/* 顺序动作先平移，进入过渡距离后同时平移与旋转；同步直线转向另有独立阶段。 */
typedef enum
{
    CHASSIS_SEQUENCE_IDLE = 0U,
    CHASSIS_SEQUENCE_TRANSLATE,
    CHASSIS_SEQUENCE_ROTATE, /* 转向并继续修正到原 XY 终点。 */
    CHASSIS_SEQUENCE_DONE,
    CHASSIS_SEQUENCE_STRAIGHT_TURN,
    CHASSIS_SEQUENCE_STRAIGHT_TURN_DONE
} ChassisSequencePhase_t;

typedef struct
{
    float x_mm;     /* 相对上电原点的 X 坐标，mm。 */
    float y_mm;     /* 相对上电原点的 Y 坐标，mm。 */
    float yaw_deg;  /* 相对上电航向的角度，deg。 */
} ChassisPose_t;

typedef struct
{
    ChassisMecanumConfig_t mecanum;
    uint8_t motor_addr[CHASSIS_MOTOR_COUNT];
    X_V2_Direction_t motor_positive_dir[CHASSIS_MOTOR_COUNT];

    /* 电机侧限制：默认取公共参数，自定义值可进一步降低限制。 */
    float motor_acc_rpm_per_s[CHASSIS_MOTOR_COUNT];
    float motor_dec_rpm_per_s[CHASSIS_MOTOR_COUNT];

    float motor_max_rpm[CHASSIS_MOTOR_COUNT];

    float max_x_right_mm_s;    /* 车体 +X 最大速度，mm/s。 */
    float max_y_forward_mm_s;  /* 车体 +Y 最大速度，mm/s。 */
    float max_yaw_deg_s;       /* 车体最大角速度，deg/s。 */
    /* 车体加减速由公共电机侧限制换算，初始化时统一生成。 */
    float max_x_acc_mm_s2;     /* 车体 X 加速度上限，mm/s²。 */
    float max_y_acc_mm_s2;     /* 车体 Y 加速度上限，mm/s²。 */
    float max_yaw_acc_deg_s2;  /* 车体角加速度上限，deg/s²。 */
    float max_x_dec_mm_s2;     /* 车体 X 减速度上限，mm/s²。 */
    float max_y_dec_mm_s2;     /* 车体 Y 减速度上限，mm/s²。 */
    float max_yaw_dec_deg_s2;  /* 车体角减速度上限，deg/s²。 */

    uint32_t sensor_timeout_ms; /* 掉线故障门限，默认 3000 ms；不用于判定到位。 */
} ChassisControlConfig_t;

typedef struct
{
    float signed_rpm[CHASSIS_MOTOR_COUNT];
} ChassisWheelSetpoint_t;

typedef struct
{
    ChassisControlMode_t mode; /* 当前控制模式。 */
    ChassisPose_t target_pose; /* 当前闭环目标。 */
    ChassisPose_t current_pose; /* 传感器测得的当前位置。 */
    ChassisPose_t pose_error; /* 目标减当前位置。 */
    ChassisBodyVelocity_t body_velocity_command; /* 当前输出车体速度。 */
    uint8_t origin_valid; /* 是否已经锁定传感器原点。 */
    uint8_t sensor_fault; /* OPS9/HW101 超时。 */
    ChassisSequencePhase_t sequence_phase; /* 相对动作的当前阶段。 */
    uint8_t sequence_done; /* 相对动作是否完成。 */
    ChassisFaultReason_t fault_reason; /* 当前故障；正常恢复后清零。 */
    ChassisFaultReason_t last_fault_reason; /* 最近故障，恢复后保留便于定位原因。 */
    uint32_t last_fault_ms; /* 最近一次记录故障的 HAL 毫秒时间。 */
    uint32_t ops9_age_ms; /* 最近可信 OPS9 数据的年龄；未收到时为 UINT32_MAX。 */
    uint32_t hw101_age_ms;
    uint32_t last_sensor_gap_ms; /* 本次/最近一次暂停中观测到的最长断帧时间。 */
    uint32_t recovery_count; /* 自动恢复成功次数。 */
    uint32_t paused_ms; /* 累计暂停时钟，供阻塞接口扣除等待恢复的时间。 */
    uint32_t motion_id; /* 动作代号；主动停止/新动作使旧等待函数失效。 */
    uint8_t recovery_active; /* 1：当前函数正在等待传感器恢复，不应重发原指令。 */
    ChassisPose_t remaining_pose; /* 原终点减当前位置；XY 始终属于原固定坐标系。 */
} ChassisControlState_t;

/*
  函数说明: 装载当前底盘的几何尺寸、电机地址/方向、速度限制和 PD 默认参数。
  使用方法：系统启动时先声明 ChassisControlConfig_t，再调用本函数，随后传给 ChassisControl_Init()。
  返回：无；config 为 NULL 时不执行任何操作。
  调用示例: ChassisControl_LoadDefaultConfig(config);
  参数意义: config 为对应模块的配置结构。
*/
void ChassisControl_LoadDefaultConfig(ChassisControlConfig_t *config);

/*
  函数说明: 初始化底盘控制器及其运行状态。
  使用方法：加载默认或自定义配置后调用一次；初始化完成后保持 STOP，需再调用速度/位置接口。
  返回：无；config 为 NULL 时进入未初始化故障状态。
  调用示例: ChassisControl_Init(config);
  参数意义: config 为对应模块的配置结构。
*/
void ChassisControl_Init(const ChassisControlConfig_t *config);

/*
  函数说明: main_task 与位移接口共用此检查；3000 ms 旧数据不能用于开始新动作。
  调用示例: ChassisControl_SensorsReady();
  参数意义: 无。
*/
uint8_t ChassisControl_SensorsReady(void);

/*
  函数说明: 设置车体速度目标，由 Update() 执行加减速限制和四轮解算。
  本模式未使用实测车速构成整车速度误差闭环。
  使用方法：周期任务或上层控制器调用；x/y 单位为 mm/s，yaw 单位为 deg/s，调用不阻塞。
  返回：1 表示目标已接受，0 表示未初始化、参数无效或当前处于故障状态。
  调用示例: ChassisControl_SetVelocity(x_right_mm_s, y_forward_mm_s, yaw_ccw_deg_s);
  参数意义: x_right_mm_s 为车体向右速度，mm/s。
            y_forward_mm_s 为车体向前速度，mm/s。
            yaw_ccw_deg_s 为逆时针角速度，deg/s。
*/
uint8_t ChassisControl_SetVelocity(float x_right_mm_s,
                                   float y_forward_mm_s,
                                   float yaw_ccw_deg_s);

/*
  函数说明: 设置相对上电原点的绝对车体目标位置和航向。
  使用方法：传入 x/y(mm)、yaw(deg) 后由 OPS9/HW101 位置环和航向环闭环逼近，调用不阻塞。
  返回：1 表示目标已接受，0 表示未初始化、参数无效或当前处于故障状态。
  调用示例: ChassisControl_SetPoseTarget(x_mm, y_mm, yaw_deg);
  参数意义: x_mm 为X 坐标或相对位移，mm，坐标系见函数说明。
            y_mm 为Y 坐标或相对位移，mm，坐标系见函数说明。
            yaw_deg 为航向或转角，度，参考系见函数说明。
*/
uint8_t ChassisControl_SetPoseTarget(float x_mm,
                                     float y_mm,
                                     float yaw_deg);

/*
  函数说明: 非阻塞启动相对动作，先平移，进入过渡距离后边平移边旋转；成功返回 1。
  调用示例: ChassisControl_StartRelativeSequential(100, 0, 0, 1000, 1500, 3000);
  参数意义: dx_mm 为发令瞬间车体向右的相对位移，mm。
            dy_mm 为发令瞬间车体向前的相对位移，mm。
            dyaw_deg 为相对转角，度，正值逆时针。
            acc_rpm_per_s 为电机加速度，RPM/s。
            dec_rpm_per_s 为电机减速度，RPM/s。
            max_rpm 为本次动作最大轮速，RPM。
*/
uint8_t ChassisControl_StartRelativeSequential(
    float dx_mm, float dy_mm, float dyaw_deg,
    float acc_rpm_per_s, float dec_rpm_per_s, float max_rpm);

/*
  函数说明: 执行相对动作，先平移，进入过渡距离后边平移边旋转；阻塞等待实际到位，失败返回 0。
  调用示例: ChassisControl_MoveRelativeSequential(100, 0, 90, 1000, 1500, 3000);
  参数意义: dx_mm 为发令瞬间车体向右的相对位移，mm。
            dy_mm 为发令瞬间车体向前的相对位移，mm。
            dyaw_deg 为相对转角，度，正值逆时针。
            acc_rpm_per_s 为电机加速度，RPM/s。
            dec_rpm_per_s 为电机减速度，RPM/s。
            max_rpm 为本次动作最大轮速，RPM。
*/
uint8_t ChassisControl_MoveRelativeSequential(
    float dx_mm,
    float dy_mm,
    float dyaw_deg,
    float acc_rpm_per_s,
    float dec_rpm_per_s,
    float max_rpm);

/*
  函数说明: 使用相对发令时车身向右偏 45 度的坐标系移动，阻塞等待实际到位。
  使用方法：从 RTOS 任务调用；先将斜向 X/Y 换算为车体位移，再复用原顺序动作。
            每次发令重新采用当前车身方向；本次终点确定后，不随运动中的转向重算。
            新 +Y 指向车右前方，新 +X 指向车右后方；本函数不额外转动车身。
  返回：1 表示实际到位，0 表示参数无效、未就绪、忙、故障、取消或超时。
  调用示例: ChassisControl_MoveDiagonalRelative(0, 100, 0, 100, 150, 800);
  参数意义: dx_mm 为沿新 X 轴的相对位移，mm，正值向车右后方。
            dy_mm 为沿新 Y 轴的相对位移，mm，正值向车右前方。
            dyaw_deg 为相对当前航向的转角，度，正值逆时针；0 保持当前航向。
            acc_rpm_per_s 为电机加速度，RPM/s。
            dec_rpm_per_s 为电机减速度，RPM/s。
            max_rpm 为本次动作最大轮速，RPM。
*/
uint8_t ChassisControl_MoveDiagonalRelative(
    float dx_mm, float dy_mm, float dyaw_deg,
    float acc_rpm_per_s, float dec_rpm_per_s, float max_rpm);

/*
  函数说明: 在固定地面直线上平移，同时按相同进度旋转 yaw，并阻塞等待双轴到位。
  使用方法：从 main_task 等 RTOS 任务调用；x/y 是调用瞬间的车体相对位移，
         +X 为车右、+Y 为车前，单位 mm；yaw 为相对角度，正值逆时针。
         acc/dcc 单位 RPM/s，max_speed 单位 RPM，四轮共用这些限制。
         本函数不发送完成帧，也不读取串口；底盘任务仍每 20 ms 做闭环。
  返回：1 为 XY 和 yaw 连续到位；0 为参数、传感器、故障、中断或超时。
  相对当前位置沿固定直线平移，同时旋转车身，并等待两者都到位。
  x/y 是发令瞬间车体系中的相对位移：+X 车右、+Y 车前，单位 mm；
  yaw 是相对转角，正值按底盘定义为逆时针，单位 deg。
  acc/dcc 为四轮统一的加/减速度限制，单位 RPM/s；max_speed 为
  四轮统一的最大转速限制，单位 RPM。主机协议不传入本地加减速参数。
  必须从运行中的 RTOS 任务调用；函数阻塞等待，底盘任务每 20 ms 闭环。
  main_task 调用示例：
  ChassisControl_MoveRelativeStraightTurn(1000.0f, 0.0f, 180.0f,
                                          1000.0f, 1500.0f, 3000.0f);
  返回 1 表示 XY 和 yaw 均稳定到位；掉帧时内部暂停，位置可信后续跑原路径。
  参数无效、不可恢复故障、主动取消或有效运动累计 30 秒超时返回 0。
  HW101 缺帧过久而无法判断多圈角度时保持故障，不猜测累计圈数。
  调用示例: ChassisControl_MoveRelativeStraightTurn(100, 0, 90, 1000, 1500, 3000);
  参数意义: x_mm 为X 坐标或相对位移，mm，坐标系见函数说明。
            y_mm 为Y 坐标或相对位移，mm，坐标系见函数说明。
            yaw_deg 为航向或转角，度，参考系见函数说明。
            acc_rpm_per_s 为电机加速度，RPM/s。
            dcc_rpm_per_s 为电机减速度，RPM/s。
            max_speed_rpm 为本次动作最大轮速，RPM。
*/
uint8_t ChassisControl_MoveRelativeStraightTurn(
    float x_mm,
    float y_mm,
    float yaw_deg,
    float acc_rpm_per_s,
    float dcc_rpm_per_s,
    float max_speed_rpm);

/*
  函数说明: 查询最近一次相对动作是否首次到位，支持顺序和同步直线转向两种模式。
  使用方法：供其他任务或调试代码查询；返回 1 后仍持续保持目标位姿，直到停止、改目标或故障。
  调用示例: ChassisControl_IsMoveComplete();
  参数意义: 无。
*/
uint8_t ChassisControl_IsMoveComplete(void);

/*
  函数说明: 取消当前速度/位置动作，并将目标速度设为零。
  使用方法：上层需要正常停车时调用；后续控制周期仍会按减速度把实际轮速降到零。
  调用示例: ChassisControl_Stop();
  参数意义: 无。
*/
void ChassisControl_Stop(void);
/*
  函数说明: 清除控制器记录的传感器故障标志并回到 STOP 状态。
  使用方法：确认外部故障已排除后调用；它会重新等待有效传感器建立原点。
  调用示例: ChassisControl_ClearFault();
  参数意义: 无。
*/
void ChassisControl_ClearFault(void);
/*
  函数说明: 锁存底盘故障并立即把控制目标清零。
  使用方法：检测到传感器、反馈或解算异常时调用；底盘任务随后应发送零速/停止命令。
  调用示例: ChassisControl_FaultStop();
  参数意义: 无。
*/
void ChassisControl_FaultStop(void);
/*
  函数说明: 锁存具体故障原因、清空运动目标并取消自动续跑。
  调用示例: ChassisControl_FaultStopWithReason(reason);
  参数意义: reason 为底盘故障原因位图。
*/
void ChassisControl_FaultStopWithReason(ChassisFaultReason_t reason);

/*
  函数说明: 执行一次底盘控制周期，完成传感器采样、位置/航向 PD、速度加减速、麦克纳姆解算和轮速限幅。
  使用方法：由 chassisTask 以固定周期调用（当前为 20 ms），把输出 setpoint 交给电机命令构造函数。
  返回：1 表示本周期输出有效，0 表示参数/传感器/反馈/解算异常；setpoint 不能为 NULL。
  调用示例: ChassisControl_Update(dt_s, setpoint);
  参数意义: dt_s 为控制周期，秒。
            setpoint 为四轮带符号目标转速，RPM。
*/
uint8_t ChassisControl_Update(float dt_s,
                              ChassisWheelSetpoint_t *setpoint);

/*
  函数说明: 把四轮带符号逻辑 RPM 转换为每个 X 电机的地址、CW/CCW 方向、速度和 F6 加速度字段。
  使用方法：先调用 ChassisControl_Update() 得到 setpoint，再调用本函数；本函数只组包，不发送 CAN。
  调用示例: ChassisControl_BuildMotorCommands(setpoint, commands);
  参数意义: setpoint 为四轮带符号目标转速，RPM。
            commands 为多电机命令数组。
*/
void ChassisControl_BuildMotorCommands(
    const ChassisWheelSetpoint_t *setpoint,
    X_V2_Vel_Command_t commands[CHASSIS_MOTOR_COUNT]);

/*
  函数说明: 复制当前控制器状态、目标/实测位姿、误差、故障和动作阶段。
  使用方法：调试 Watch 或上层任务传入有效 state 指针读取；函数在临界区内复制，返回后可安全查看。
  调用示例: ChassisControl_GetState(&state);
  参数意义: state 为状态值或状态快照。
*/
void ChassisControl_GetState(ChassisControlState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_CONTROL_H */
