#include "arm_task.h"

#include "cmsis_os.h"
#include "main.h"

#include "X_V2.h"
#include "arm_motion.h"
#include "zdt_can.h"

#define ARM_DRIVER_STARTUP_MS    (2000U)

static ArmMotionConfig arm_motion_config;
static volatile ArmMotionStatus arm_motion_status = ARM_MOTION_OK;

static void ArmTask_FaultStop(void)
{
    /* 运动失败时先立即停止三轴，然后关闭使能。 */
    (void)ArmMotion_StopAll(&arm_motion_config);
    osDelay(20U);
    (void)ArmMotion_EnableAll(&arm_motion_config, false);

    for (;;)
    {
        osDelay(1000U);
    }
}

void ArmTask_Run(void)
{
    uint8_t motor_index;

    if (ZDT_CAN_Init() != HAL_OK)
    {
        Error_Handler();
    }

    ArmMotion_LoadDefaultConfig(&arm_motion_config);
    arm_motion_status = ArmMotion_SetActiveConfig(&arm_motion_config);
    if (arm_motion_status != ARM_MOTION_OK)
    {
        ArmTask_FaultStop();
    }
    osDelay(ARM_DRIVER_STARTUP_MS);

    /* 调用前必须确保三个关节已在物理零位，且驱动器已设零。 */
    ArmMotion_ConfirmZero(&arm_motion_config, true);

    for (motor_index = 0U; motor_index < 3U; ++motor_index)
    {
        if (X_V2_Reset_Clog_Pro(
                arm_motion_config.motor_addr[motor_index]) != HAL_OK)
        {
            ArmTask_FaultStop();
        }
        osDelay(10U);
    }

    arm_motion_status = ArmMotion_EnableAll(&arm_motion_config, true);
    if (arm_motion_status != ARM_MOTION_OK)
    {
        ArmTask_FaultStop();
    }
    osDelay(100U);

    arm_motion_status = ArmMotion_SetIkBranch(
        &arm_motion_config, ARM_IK_RADIAL_POSITIVE);
    if (arm_motion_status != ARM_MOTION_OK)
    {
        ArmTask_FaultStop();
    }

    /* 设置末端坐标和三轴共同到达时间；最后一个参数的单位为秒。 */
  //  arm_motion_status = ArmMotion_MoveToXYZ( 0.0f ,   180.0f , 210.0f , 3.0f);
		while(1)
		{
				 arm_motion_status = ArmMotion_MoveToXYZ( 200.0f , 200.0f , 100.0f , 2);
				if (arm_motion_status != ARM_MOTION_OK)
				{
						ArmTask_FaultStop();
				}
				arm_motion_status = ArmMotion_MoveToXYZ( -200.0f , 250.0f , 200.0f , 2);
				if (arm_motion_status != ARM_MOTION_OK)
				{
						ArmTask_FaultStop();
				}
				arm_motion_status = ArmMotion_MoveToXYZ( 0.0f , 170.0f , 200.0f , 2);
				if (arm_motion_status != ARM_MOTION_OK)
				{
						ArmTask_FaultStop();
				}
				arm_motion_status = ArmMotion_MoveToXYZ( -200.0f , -200.0f , 300.0f , 2.5f);
				if (arm_motion_status != ARM_MOTION_OK)
				{
						ArmTask_FaultStop();
				}
				arm_motion_status = ArmMotion_MoveToXYZ( 0.0f , 170.0f , 200.0f , 2.5f);
				if (arm_motion_status != ARM_MOTION_OK)
				{
						ArmTask_FaultStop();
				}

	}

//    for (;;)
//    {
//        /* 接收电机返回帧，可在 rx_frame 处打断点观察。 */
//        if (ZDT_CAN_GetRx(&rx_frame, osWaitForever) == osOK)
//        {
//            /* 返回数据的状态解析待后续接入。 */
//        }
//    }
}
