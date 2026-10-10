#include "main_task.h"

#include "cmsis_os.h"
#include "main.h"

#include "X_V2.h"
#include "arm_motion.h"
#include "chassis_control.h"
#include "end_level_task.h"
#include "hw101.h"
#include "ops9.h"
#include "claw.h"
#include "zdt_can.h"

/* main_task 负责一次性初始化和动作流程；实时调平仍由 endLevelTask 执行。 */
#define MAIN_TASK_ARM_DRIVER_STARTUP_MS  (2000U)
#define MAIN_TASK_ARM_STOP_GAP_MS        (20U)
#define MAIN_TASK_ARM_MOTOR_COUNT        (3U)
#define MAIN_TASK_SENSOR_TIMEOUT_MS      (200U)
#define MAIN_TASK_IDLE_PERIOD_MS         (100U)

/* 配置必须长期有效：ArmMotion_SetActiveConfig() 保存的是它的指针。 */
static ArmMotionConfig main_task_arm_config;
volatile ArmMotionStatus main_task_arm_status = ARM_MOTION_NOT_INITIALIZED;
volatile ArmMotionStatus main_task_arm_recovery_status =
    ARM_MOTION_NOT_INITIALIZED;
volatile uint8_t main_task_arm_ready = 0U;

/* 严重故障时停止并关闭 ID1～4，不在位置未知时继续下一动作。 */
static void MainTask_ArmFaultStop(void)
{
    main_task_arm_ready = 0U;
    EndLevelTask_SetControlEnabled(false);

    (void)X_V2_Stop_Now(END_LEVEL_MOTOR_ID, false);
    osDelay(MAIN_TASK_ARM_STOP_GAP_MS);
    (void)X_V2_En_Control(END_LEVEL_MOTOR_ID, false, false);
    (void)ArmMotion_StopAll(&main_task_arm_config);
    osDelay(MAIN_TASK_ARM_STOP_GAP_MS);
    (void)ArmMotion_EnableAll(&main_task_arm_config, false);

    for (;;)
    {
        osDelay(1000U);
    }
}

/* 只在 main_task 上电时调用一次；ID4 的持续调平仍归 endLevelTask。 */
static uint8_t MainTask_InitializeArm(void)
{
    uint8_t motor_index;

    EndLevelTask_SetControlEnabled(false);
    ArmMotion_LoadDefaultConfig(&main_task_arm_config);

    if (ZDT_CAN_Init() != HAL_OK)
    {
        main_task_arm_status = ARM_MOTION_CAN_ERROR;
        return 0U;
    }

    main_task_arm_status = ArmMotion_SetActiveConfig(&main_task_arm_config);
    if (main_task_arm_status != ARM_MOTION_OK)
    {
        return 0U;
    }
    osDelay(MAIN_TASK_ARM_DRIVER_STARTUP_MS);

    /* 这只是软件确认，不会驱动归零：上电前必须人工确认物理零位。 */
    ArmMotion_ConfirmZero(&main_task_arm_config, true);

    for (motor_index = 0U; motor_index < MAIN_TASK_ARM_MOTOR_COUNT;
         ++motor_index)
    {
        if (X_V2_Reset_Clog_Pro(
                main_task_arm_config.motor_addr[motor_index]) != HAL_OK)
        {
            main_task_arm_status = ARM_MOTION_CAN_ERROR;
            return 0U;
        }
        osDelay(10U);
    }

    main_task_arm_status = ArmMotion_EnableAll(&main_task_arm_config, true);
    if (main_task_arm_status != ARM_MOTION_OK)
    {
        return 0U;
    }

    /* ID4 不在 XYZ 三轴配置内，需要单独清保护、使能。 */
    if (X_V2_Reset_Clog_Pro(END_LEVEL_MOTOR_ID) != HAL_OK)
    {
        main_task_arm_status = ARM_MOTION_CAN_ERROR;
        return 0U;
    }
    osDelay(10U);
    if (X_V2_En_Control(END_LEVEL_MOTOR_ID, true, false) != HAL_OK)
    {
        main_task_arm_status = ARM_MOTION_CAN_ERROR;
        return 0U;
    }
    osDelay(120U);

    main_task_arm_status = ArmMotion_SetIkBranch(
        &main_task_arm_config, ARM_IK_RADIAL_POSITIVE);
    if (main_task_arm_status != ARM_MOTION_OK)
    {
        return 0U;
    }

    /* 陀螺仪完成初始化和静止校准后，才允许打开 ID4 自动调平。 */
//    if (!EndLevelTask_WaitReady(END_LEVEL_TASK_READY_TIMEOUT_MS))
//    {
//        main_task_arm_status = ARM_MOTION_FEEDBACK_TIMEOUT;
//        return 0U;
//    }

    ///EndLevelTask_SetControlEnabled(END_LEVEL_CONTROL_DEFAULT != 0U);
    main_task_arm_recovery_status = ARM_MOTION_OK;
    main_task_arm_ready = 1U;
    return 1U;
}

/**
 * 功能：在 main_task 中执行一个末端 XYZ 动作，参数最后一项为秒。
 * 逆解超限或请求时间不可实现时未发运动指令，可记录错误并继续下一动作；
 * 到位超时后核实三轴停稳并从实际位置恢复下一动作起点；恢复失败时
 * 不再发机械臂运动，但主流程继续。其他 CAN/驱动器故障仍停机。
 */
void ArmTask_MoveToXYZOrFault(float x, float y, float z, float time_s)
{
    if (main_task_arm_ready == 0U)
    {
        /* 保留上一条动作/恢复失败状态，便于在 Watch 中定位原因。 */
        return;
    }

    main_task_arm_status = ArmMotion_MoveToXYZ(x, y, z, time_s);
    if (main_task_arm_status == ARM_MOTION_REACHED_TIMEOUT)
    {
        main_task_arm_recovery_status =
            ArmMotion_RecoverAfterTimeout(&main_task_arm_config);
        if (main_task_arm_recovery_status != ARM_MOTION_OK)
        {
            /* 没有可信起点时，跳过后续 XYZ，不陷入永久等待。 */
            main_task_arm_ready = 0U;
        }
        return;
    }
    if ((main_task_arm_status == ARM_MOTION_OK) ||
        (main_task_arm_status == ARM_MOTION_IK_ERROR) ||
        (main_task_arm_status == ARM_MOTION_TIME_UNACHIEVABLE))
    {
        return;
    }

    MainTask_ArmFaultStop();
}

/**
 * 功能：按需检查 OPS9 和 HW101 是否持续输出有效数据。
 * 使用方法：仅在准备执行底盘动作时调用；机械臂 ID1～3 不依赖这两路传感器。
 */
uint8_t MainTask_ChassisSensorsReady(void)
{
    OPS9_Data_t ops9;
    HW101_Data_t hw101;
    uint32_t now_ms;

    if ((OPS9_GetLatest(&ops9) == 0U) ||
        (HW101_GetData(&hw101) == 0U))
    {
        return 0U;
    }

    now_ms = HAL_GetTick();
    if ((now_ms - ops9.last_update_ms) > MAIN_TASK_SENSOR_TIMEOUT_MS)
    {
        return 0U;
    }
    if ((now_ms - hw101.last_update_ms) > MAIN_TASK_SENSOR_TIMEOUT_MS)
    {
        return 0U;
    }

    return 1U;
}

/**
 * 功能：一次性初始化机械臂，然后调度夹爪、底盘和机械臂动作。
 * 使用方法：由 CubeMX 生成的 StartMainTask() 调用；实时控制仍由各自任务执行。
 */
void MainTask_Run(void *argument)
{   
    (void)argument;
//osDelay(10000);
    if (MainTask_InitializeArm() == 0U)
    {
        MainTask_ArmFaultStop();
    }
		Claw_SetAngle(0.0f,1000u);//夹爪初始化
    while (MainTask_ChassisSensorsReady() == 0U)
    {
        osDelay(MAIN_TASK_IDLE_PERIOD_MS);
    }
      
	
//    ChassisControl_MoveRelativeSequential(-1000.0f, 150.0f, 0.0f,50.0f, 300.0f, 400.0f);
//		
//    ChassisControl_MoveRelativeSequential(0.0f, 850.0f, 0.0f,50.0f, 300.0f, 400.0f);  
//		
//    ChassisControl_MoveRelativeSequential(900.0f, 0.0f, 90.0f,50.0f, 300.0f, 400.0f);
//		
//		ChassisControl_MoveRelativeSequential(0.0f, 1800.0f, 0.0f,50.0f, 300.0f, 400.0f);
//		
//		ChassisControl_MoveRelativeSequential(800.0f, 0.0f, -90.0f,50.0f, 300.0f, 400.0f);
//		
//		ChassisControl_MoveRelativeSequential(800.0f, 0.0f, 0.0f,50.0f, 300.0f, 400.0f);
//		
//		ChassisControl_MoveRelativeSequential(950.0f, 0.0f, 0.0f,50.0f, 300.0f, 400.0f);
//		
//		ChassisControl_MoveRelativeSequential(0.0f, -1500.0f, 0.0f,50.0f, 300.0f, 400.0f);
//		


    /* 上电后先等 OPS9 和 HW101 都有 200 ms 内的有效数据，再启动底盘动作。 */


while(1){
//			ArmTask_MoveToXYZOrFault(50,150,300,2);
//    ArmTask_MoveToXYZOrFault(230,230,80,2);
//		Claw_SetAngle(80,500);
//		osDelay(500);
//		ArmTask_MoveToXYZOrFault(50,150,300,2);
//		ArmTask_MoveToXYZOrFault(-50,150,300,1);	
//		ArmTask_MoveToXYZOrFault(-230,230,80,2);
//    Claw_SetAngle(50,500);
//		ArmTask_MoveToXYZOrFault(-50,150,300,2);
//    ArmTask_MoveToXYZOrFault(-230,230,80,2);
//		Claw_SetAngle(80,500);
//		osDelay(500);
//    ArmTask_MoveToXYZOrFault(-50,150,300,2);
//    ArmTask_MoveToXYZOrFault(50,150,300,1);
//		ArmTask_MoveToXYZOrFault(230,230,80,2);
//	  Claw_SetAngle(50,500);
//	  osDelay(500);

	 ChassisControl_MoveRelativeSequential(0.0f, 1000.0f, 180.0f,50.0f, 300.0f, 300.0f);
	 //osDelay(200);
	 
	
    }
}
