/*
  程序说明: 九节点地图最短路、超声波扫描、路段遇障回退及跑图流程。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

#include "Route.h"
#include <stddef.h>

typedef struct { RouteNode_t a; RouteNode_t b; } RouteEdge_t;

/* 9 个节点、12 条相邻路段；双向通行，路段长度暂按 800 mm。 */
static const RouteEdge_t edges[12] = {
    {ROUTE_A, ROUTE_B}, {ROUTE_B, ROUTE_C},
    {ROUTE_D, ROUTE_E}, {ROUTE_E, ROUTE_F},
    {ROUTE_G, ROUTE_H}, {ROUTE_H, ROUTE_I},
    {ROUTE_A, ROUTE_D}, {ROUTE_D, ROUTE_G},
    {ROUTE_B, ROUTE_E}, {ROUTE_E, ROUTE_H},
    {ROUTE_C, ROUTE_F}, {ROUTE_F, ROUTE_I}
};

/*
  函数说明: 查找两个相邻地图节点之间的无向边下标。
  调用示例: RouteGraph_EdgeIndex(a, b);
  参数意义: a 为道路第一个端点。
            b 为道路第二个端点。
*/
static int8_t RouteGraph_EdgeIndex(RouteNode_t a, RouteNode_t b)
{
    uint8_t i;
    for (i = 0U; i < 12U; ++i)
    {
        if (((edges[i].a == a) && (edges[i].b == b)) ||
            ((edges[i].a == b) && (edges[i].b == a)))
        {
            return (int8_t)i;
        }
    }
    return -1;
}

/*
  函数说明: 初始化九节点地图的节点状态和十二条边的通行状态。
  调用示例: RouteGraph_Init(graph);
  参数意义: graph 为九节点地图及占用状态。
*/
void RouteGraph_Init(RouteGraph_t *graph)
{
    /* 开始一轮任务时清空占用图：未知不等于已确认可走。 */
    uint8_t i;
    if (graph == NULL) return;
    for (i = 0U; i < ROUTE_NODE_COUNT; ++i) graph->node[i] = ROUTE_UNKNOWN;
    for (i = 0U; i < 12U; ++i) graph->edge[i] = ROUTE_UNKNOWN;
}

/*
  函数说明: 取得指定节点在某一朝向上的相邻节点；越界返回 0。
  调用示例: RouteGraph_Neighbor(from, direction, neighbor);
  参数意义: from 为路径起点节点。
            direction 为方向枚举。
            neighbor 为输出相邻地图节点。
*/
uint8_t RouteGraph_Neighbor(RouteNode_t from, RouteHeading_t direction,
                            RouteNode_t *neighbor)
{
    /* 九宫格边缘朝场外没有相邻节点，外围栏不应登记成地图节点。 */
    uint8_t row;
    uint8_t col;
    if ((from >= ROUTE_NODE_COUNT) || (neighbor == NULL)) return 0U;
    row = (uint8_t)from / 3U;
    col = (uint8_t)from % 3U;
    switch (direction)
    {
    case ROUTE_NORTH: if (row == 0U) return 0U; *neighbor = (RouteNode_t)(from - 3U); break;
    case ROUTE_EAST:  if (col == 2U) return 0U; *neighbor = (RouteNode_t)(from + 1U); break;
    case ROUTE_SOUTH: if (row == 2U) return 0U; *neighbor = (RouteNode_t)(from + 3U); break;
    case ROUTE_WEST:  if (col == 0U) return 0U; *neighbor = (RouteNode_t)(from - 1U); break;
    default: return 0U;
    }
    return 1U;
}

/*
  函数说明: 更新地图中一条无向边的占用状态。
  调用示例: RouteGraph_SetEdge(graph, a, b, state);
  参数意义: graph 为九节点地图及占用状态。
            a 为道路第一个端点。
            b 为道路第二个端点。
            state 为状态值或状态快照。
*/
void RouteGraph_SetEdge(RouteGraph_t *graph, RouteNode_t a,
                        RouteNode_t b, RouteOccupancy_t state)
{
    int8_t index = RouteGraph_EdgeIndex(a, b);
    if ((graph != NULL) && (index >= 0)) graph->edge[(uint8_t)index] = state;
}

/*
  函数说明: 查询两个节点之间的通行状态。
  调用示例: RouteGraph_GetEdge(graph, a, b);
  参数意义: graph 为九节点地图及占用状态。
            a 为道路第一个端点。
            b 为道路第二个端点。
*/
RouteOccupancy_t RouteGraph_GetEdge(const RouteGraph_t *graph,
                                    RouteNode_t a, RouteNode_t b)
{
    int8_t index = RouteGraph_EdgeIndex(a, b);
    if ((graph == NULL) || (index < 0)) return ROUTE_BLOCKED;
    return graph->edge[(uint8_t)index];
}

/*
  函数说明: 更新一个地图节点的占用状态。
  调用示例: RouteGraph_SetNode(graph, node, state);
  参数意义: graph 为九节点地图及占用状态。
            node 为地图节点。
            state 为状态值或状态快照。
*/
void RouteGraph_SetNode(RouteGraph_t *graph, RouteNode_t node,
                        RouteOccupancy_t state)
{
    if ((graph != NULL) && (node < ROUTE_NODE_COUNT)) graph->node[node] = state;
}

/*
  函数说明: 求相邻两节点之间的地图行进朝向。
  调用示例: RouteGraph_Direction(from, to, direction);
  参数意义: from 为路径起点节点。
            to 为路径终点节点。
            direction 为方向枚举。
*/
uint8_t RouteGraph_Direction(RouteNode_t from, RouteNode_t to,
                             RouteHeading_t *direction)
{
    uint8_t d;
    RouteNode_t neighbor;
    if (direction == NULL) return 0U;
    for (d = 0U; d < 4U; ++d)
    {
        if ((RouteGraph_Neighbor(from, (RouteHeading_t)d, &neighbor) != 0U) &&
            (neighbor == to))
        {
            *direction = (RouteHeading_t)d;
            return 1U;
        }
    }
    return 0U;
}

/*
  函数说明: 执行最短路搜索，输出当前位置走向目标的下一节点。
  调用示例: RouteGraph_Next(graph, from, goal, next);
  参数意义: graph 为九节点地图及占用状态。
            from 为路径起点节点。
            goal 为路径目标节点。
            next 为下一地图节点。
*/
uint8_t RouteGraph_Next(const RouteGraph_t *graph, RouteNode_t from,
                        RouteNode_t goal, RouteNode_t *next)
{
    /* 朴素 Dijkstra：过滤已封闭的节点和路段，返回最短路的第一跳。
     * 每个路口重新调用，扫描发现新障碍后路线就会更新。 */
    uint16_t distance[ROUTE_NODE_COUNT];
    uint8_t used[ROUTE_NODE_COUNT] = {0};
    RouteNode_t first[ROUTE_NODE_COUNT];
    uint8_t i;
    uint8_t j;
    uint8_t k;
    uint8_t best;
    uint16_t candidate;
    RouteNode_t other;

    if ((graph == NULL) || (next == NULL) || (from >= ROUTE_NODE_COUNT) ||
        (goal >= ROUTE_NODE_COUNT) || (graph->node[goal] == ROUTE_BLOCKED) ||
        (from == goal)) return 0U;
    for (i = 0U; i < ROUTE_NODE_COUNT; ++i)
    {
        distance[i] = 0xFFFFU;
        first[i] = ROUTE_NODE_COUNT;
    }
    distance[from] = 0U;
    for (i = 0U; i < ROUTE_NODE_COUNT; ++i)
    {
        best = ROUTE_NODE_COUNT;
        for (j = 0U; j < ROUTE_NODE_COUNT; ++j)
        {
            if ((used[j] == 0U) && (distance[j] != 0xFFFFU) &&
                ((best == ROUTE_NODE_COUNT) || (distance[j] < distance[best])))
            {
                best = j;
            }
        }
        if (best == ROUTE_NODE_COUNT) break;
        if (best == goal) break;
        used[best] = 1U;
        for (k = 0U; k < 12U; ++k)
        {
            if ((graph->edge[k] == ROUTE_BLOCKED) ||
                ((edges[k].a != best) && (edges[k].b != best))) continue;
            other = (edges[k].a == best) ? edges[k].b : edges[k].a;
            if ((used[other] != 0U) || (graph->node[other] == ROUTE_BLOCKED)) continue;
            /* 未知路暂允许规划，到它的起点后仍须扫描才走。
             * 外圈路段加 1 的平局代价：真实距离仍按 800 mm，
             * 只在路段数相同的候选路线之间优先选中心十字。 */
            candidate = (uint16_t)(distance[best] + 800U +
                        ((k == 2U || k == 3U || k == 8U || k == 9U) ? 0U : 1U));
            if (candidate < distance[other])
            {
                distance[other] = candidate;
                first[other] = (best == from) ? other : first[best];
            }
        }
    }
    if (first[goal] == ROUTE_NODE_COUNT) return 0U;
    *next = first[goal];
    return 1U;
}


#include "cmsis_os.h"
#include "main.h"
#include "chassis_control.h"
#include "ultrasonic_task.h"
#include <math.h>

/* 底盘跑图主流程：I 启停位出发，按 F→B→H→D→B→H→D→I 的任务顺序行驶。
 * 只检查准备进入的未知路段，遇障退回节点并更新地图，然后重算路径；
 * 本文件暂不控制机械臂和二维码识别。 */

/* 九宫格相邻节点暂按 800 mm；12 条路尚未逐条标定。下面三个速度参数
 * 沿用原跑图代码的加速度、减速度、最大转速。 */
#define ROUTE_EDGE_MM (800.0f)
#define ROUTE_ACC_RPM_S (100.0f)
#define ROUTE_DEC_RPM_S (150.0f)
#define ROUTE_MAX_RPM (800.0f)
/* 等待探头任务给出三次新读数的最长时间。 */
#define ROUTE_SCAN_TIMEOUT_MS (1800U)
/* 急停后需等底盘真正停稳，不能在减速滑行中计算退回距离。 */
#define ROUTE_STOP_WAIT_MS (12000U)
#define ROUTE_STOP_POLL_MS (100U)
#define ROUTE_STOP_STABLE_SAMPLES (4U)
#define ROUTE_STOP_POSITION_DELTA_MM (3.0f)
#define ROUTE_STOP_COMMAND_MM_S (10.0f)
#define ROUTE_STOP_COMMAND_DEG_S (2.0f)
/* 28 cm 车体在 40 cm 车道内只剩约 60 mm 单侧余量。 */
#define ROUTE_RECOVERY_MAX_LATERAL_MM (50.0f)
#define ROUTE_RECOVERY_MAX_YAW_DEG (10.0f)
#define ROUTE_RECOVERY_BEHIND_MARGIN_MM (20.0f)
#define ROUTE_RECOVERY_OVERRUN_MARGIN_MM (60.0f)
#define ROUTE_RECOVERY_SKIP_MOVE_MM (3.0f)
#define ROUTE_MAX_DECISIONS (48U)
#define ROUTE_PI (3.14159265358979323846f)

/* 路口扫描结果：不确定、可走、路段有障碍、相邻节点有障碍。 */
typedef enum
{
    SCAN_UNCERTAIN = 0U,
    SCAN_CLEAR,
    SCAN_EDGE_BLOCKED,
    SCAN_NODE_BLOCKED
} ScanClass_t;

volatile Route_Status_t route_nav_status;
static RouteGraph_t graph;
static RouteNode_t current_node;
static RouteHeading_t heading;

static uint8_t route_front_armed;
static uint8_t route_emergency;
static HCSR04_Reading_t route_emergency_reading;

/*
  函数说明: 开启或关闭路段前方防撞检测，并同步切换前探头优先采样。
  调用示例: Route_ArmFrontStop(enabled);
  参数意义: enabled 为非零表示启用，零表示关闭。
*/
static void Route_ArmFrontStop(uint8_t enabled)
{
    route_front_armed = enabled != 0U;
    UltrasonicTask_SetFrontPriority(enabled);
}

/*
  函数说明: 清除本次路段遇障标志与连续近障计数。
  调用示例: Route_ClearEmergency();
  参数意义: 无。
*/
static void Route_ClearEmergency(void)
{
    route_emergency = 0U;
    route_emergency_reading.status = HCSR04_NOT_READY;
}

/*
  函数说明: 复制最近导致路段遇障停车的测距结果。
  调用示例: Route_GetEmergencyReading(reading);
  参数意义: reading 为测距结果、状态、采样时刻与序号。
*/
static void Route_GetEmergencyReading(HCSR04_Reading_t *reading)
{
    if (reading != NULL) *reading = route_emergency_reading;
}


/*
  函数说明: 所有平移和转向都走同一个底盘接口；动作失败或急停时立即停止流程。
  调用示例: Route_Move(x, y, yaw);
  参数意义: x 为车体向右的相对位移，mm。
            y 为车体向前的相对位移，mm。
            yaw 为相对转角，度。
*/
static uint8_t Route_Move(float x, float y, float yaw)
{
    ChassisControlState_t state;
    HCSR04_Reading_t sample;
    uint32_t start_ms = HAL_GetTick();
    uint32_t last_sample_ms = start_ms;
    uint32_t motion_id;
    uint32_t pause_base;
    uint32_t sequence;
    uint32_t elapsed;
    uint8_t near_count = 0U;
    UltrasonicTask_GetReading(HCSR04_FRONT, &sample);
    sequence = sample.sequence;
    if (!ChassisControl_StartRelativeSequential(x, y, yaw,
            ROUTE_ACC_RPM_S, ROUTE_DEC_RPM_S, ROUTE_MAX_RPM)) goto failed;
    ChassisControl_GetState(&state);
    motion_id = state.motion_id;
    pause_base = state.paused_ms;
    for (;;)
    {
        ChassisControl_GetState(&state);
        if (state.motion_id != motion_id) goto failed;
        if (route_front_armed && state.sequence_phase == CHASSIS_SEQUENCE_TRANSLATE)
        {
            UltrasonicTask_GetReading(HCSR04_FRONT, &sample);
            if (sample.sequence != sequence)
            {
                sequence = sample.sequence;
                if ((uint32_t)(HAL_GetTick() - sample.timestamp_ms) > 350U ||
                    sample.status == HCSR04_STUCK_HIGH || sample.status == HCSR04_NOT_READY) goto failed;
                last_sample_ms = HAL_GetTick();
                if (sample.status == HCSR04_VALID && sample.distance_mm < 300U)
                {
                    if (++near_count >= 2U)
                    {
                        route_emergency_reading = sample;
                        route_emergency = 1U;
                        goto failed;
                    }
                }
                else near_count = 0U;
            }
            if ((uint32_t)(HAL_GetTick() - last_sample_ms) > 350U) goto failed;
        }
        if (!state.recovery_active)
        {
            if (state.mode != CHASSIS_CONTROL_POSE) goto failed;
            if (state.sequence_done && state.sequence_phase == CHASSIS_SEQUENCE_DONE) return 1U;
            elapsed = HAL_GetTick() - start_ms;
            if (elapsed >= state.paused_ms - pause_base &&
                elapsed - (state.paused_ms - pause_base) >= CHASSIS_SEQUENCE_TIMEOUT_MS) goto failed;
        }
        osDelay(20U);
    }
failed:
    route_nav_status.result = route_emergency ? ROUTE_NAV_EMERGENCY_STOP : ROUTE_NAV_MOVE_FAILED;
    Route_ArmFrontStop(0U);
    ChassisControl_Stop();
    return 0U;
}


/*
  函数说明: 通过底盘相对转向接口将车头转到所需地图朝向。
  调用示例: Route_Face(wanted);
  参数意义: wanted 为期望地图朝向。
*/
static uint8_t Route_Face(RouteHeading_t wanted)
{
    /* 方向编号按北、东、南、西排列；底盘顺时针 90° 对应 yaw=-90°。 */
    uint8_t clockwise = (uint8_t)(((uint8_t)wanted + 4U - (uint8_t)heading) % 4U);
    float yaw = 0.0f;
    if (clockwise == 1U) yaw = -90.0f;
    else if (clockwise == 2U) yaw = 180.0f;
    else if (clockwise == 3U) yaw = 90.0f;
    if ((yaw != 0.0f) && (Route_Move(0.0f, 0.0f, yaw) == 0U)) return 0U;
    heading = wanted;
    route_nav_status.heading = heading;
    return 1U;
}

/*
  函数说明: 根据测距状态和距离门限将扫描结果分类为可通行、受阻或无效。
  调用示例: Route_Classify(reading);
  参数意义: reading 为测距结果、状态、采样时刻与序号。
*/
static ScanClass_t Route_Classify(const HCSR04_Reading_t *reading)
{
    /* 距离单位为 mm：20～50 cm 归入路段；55～75 cm 归入相邻节点。
     * 50～55 cm 留作模糊区。近于 20 cm 也按路段封闭，避免漏掉近障碍。
     * 无回波暂认为可走，但实车必须先验证探头确实能读到已知障碍。 */
    if (reading->status == HCSR04_NO_ECHO) return SCAN_CLEAR;
    if (reading->status != HCSR04_VALID) return SCAN_UNCERTAIN;
    if (reading->distance_mm <= 500U) return SCAN_EDGE_BLOCKED;
    if (reading->distance_mm < 550U) return SCAN_UNCERTAIN;
    if (reading->distance_mm <= 750U) return SCAN_NODE_BLOCKED;
    return SCAN_CLEAR;
}

/*
  函数说明: 等待指定探头的多次新测量，按既定扫描规则判断道路状态。
  调用示例: Route_SampleDirection(channel);
  参数意义: channel 为前方或右侧探头枚举。
*/
static ScanClass_t Route_SampleDirection(HCSR04_Channel_t channel)
{
    /* 只取本次调用后产生的三次新测量，至少两次同类才接受，
     * 避免单次串扰或偶发回波直接改动地图。 */
    uint8_t count[4] = {0U, 0U, 0U, 0U};
    uint8_t samples = 0U;
    uint32_t last_sequence;
    uint32_t start_ms = HAL_GetTick();
    HCSR04_Reading_t reading;
    ScanClass_t category;
    uint8_t i;

    UltrasonicTask_GetReading(channel, &reading);
    last_sequence = reading.sequence;
    while ((samples < 3U) &&
           ((uint32_t)(HAL_GetTick() - start_ms) < ROUTE_SCAN_TIMEOUT_MS))
    {
        UltrasonicTask_GetReading(channel, &reading);
        if (reading.sequence != last_sequence)
        {
            last_sequence = reading.sequence;
            category = Route_Classify(&reading);
            ++count[category];
            ++samples;
        }
        else
        {
            osDelay(5U);
        }
    }
    if (samples < 3U) return SCAN_UNCERTAIN;
    for (i = 1U; i < 4U; ++i)
    {
        if (count[i] >= 2U) return (ScanClass_t)i;
    }
    return SCAN_UNCERTAIN;
}

/*
  函数说明: 把指定节点朝向的探头扫描结果写入地图。
  调用示例: Route_ApplyScan(at, direction, channel);
  参数意义: at 为执行扫描的地图节点。
            direction 为方向枚举。
            channel 为前方或右侧探头枚举。
*/
static void Route_ApplyScan(RouteNode_t at, RouteHeading_t direction,
                                HCSR04_Channel_t channel)
{
    RouteNode_t neighbor;
    ScanClass_t observed;
    /* 朝地图外的方向没有邻居，不把外围挡板登记成障碍节点。 */
    if (RouteGraph_Neighbor(at, direction, &neighbor) == 0U) return;
    observed = Route_SampleDirection(channel);
    if (observed == SCAN_NODE_BLOCKED)
    {
        RouteGraph_SetNode(&graph, neighbor, ROUTE_BLOCKED);
        RouteGraph_SetEdge(&graph, at, neighbor, ROUTE_BLOCKED);
    }
    else if (observed == SCAN_EDGE_BLOCKED)
    {
        RouteGraph_SetEdge(&graph, at, neighbor, ROUTE_BLOCKED);
    }
    else if (observed == SCAN_CLEAR)
    {
        /* 比赛障碍物视为本轮静止：之前判为封闭的节点或边不因一次空读重开。 */
        if (graph.node[neighbor] == ROUTE_UNKNOWN)
            RouteGraph_SetNode(&graph, neighbor, ROUTE_CLEAR);
        if ((graph.node[neighbor] != ROUTE_BLOCKED) &&
            (RouteGraph_GetEdge(&graph, at, neighbor) != ROUTE_BLOCKED))
        {
            RouteGraph_SetEdge(&graph, at, neighbor, ROUTE_CLEAR);
        }
    }
    else
    {
        /* 模糊读数采取保守处理：本次不走这条边。 */
        ++route_nav_status.uncertain_scan_count;
        RouteGraph_SetEdge(&graph, at, neighbor, ROUTE_BLOCKED);
    }
}

/*
  函数说明: 只检查最短路准备进入的那条路；已确认畅通的路不重复扫描。
  车头或右探头已对准时直接测，否则转到目标方向后用车头测。
  调用示例: Route_ScanSelectedEdge(direction);
  参数意义: direction 为方向枚举。
*/
static uint8_t Route_ScanSelectedEdge(RouteHeading_t direction)
{
    HCSR04_Channel_t channel;
    RouteHeading_t right =
        (RouteHeading_t)(((uint8_t)heading + 1U) % 4U);

    if (direction == heading) channel = HCSR04_FRONT;
    else if (direction == right) channel = HCSR04_RIGHT;
    else
    {
        if (Route_Face(direction) == 0U) return 0U;
        channel = HCSR04_FRONT;
    }
    ++route_nav_status.selected_scan_count;
    Route_ApplyScan(current_node, direction, channel);
    return 1U;
}

/*
  函数说明: 将相对转向角归一化，供停车回退和地图方向校验。
  调用示例: Route_WrapYaw(yaw);
  参数意义: yaw 为相对转角，度。
*/
static float Route_WrapYaw(float yaw)
{
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    return yaw;
}

/*
  函数说明: ChassisControl_Stop() 会取消动作，但电机还需减速。
  同时检查控制器指令和连续 OPS9 位移，确认车确实已经停稳。
  调用示例: Route_WaitForStop(pose);
  参数意义: pose 为输出停稳后的底盘位姿。
*/
static uint8_t Route_WaitForStop(ChassisPose_t *pose)
{
    ChassisControlState_t state;
    ChassisPose_t previous = {0.0f, 0.0f, 0.0f};
    uint32_t start_ms = HAL_GetTick();
    uint8_t have_previous = 0U;
    uint8_t stable = 0U;

    while ((uint32_t)(HAL_GetTick() - start_ms) < ROUTE_STOP_WAIT_MS)
    {
        ChassisControl_GetState(&state);
        if (state.mode == CHASSIS_CONTROL_FAULT) return 0U;
        if ((state.mode == CHASSIS_CONTROL_STOP) &&
            (state.origin_valid != 0U) &&
            (state.ops9_age_ms <= CHASSIS_SENSOR_FRESH_MS) &&
            (state.hw101_age_ms <= CHASSIS_SENSOR_FRESH_MS) &&
            (fabsf(state.body_velocity_command.x_right_mm_s) <=
                ROUTE_STOP_COMMAND_MM_S) &&
            (fabsf(state.body_velocity_command.y_forward_mm_s) <=
                ROUTE_STOP_COMMAND_MM_S) &&
            (fabsf(state.body_velocity_command.yaw_ccw_deg_s) <=
                ROUTE_STOP_COMMAND_DEG_S))
        {
            if ((have_previous != 0U) &&
                (fabsf(state.current_pose.x_mm - previous.x_mm) <=
                    ROUTE_STOP_POSITION_DELTA_MM) &&
                (fabsf(state.current_pose.y_mm - previous.y_mm) <=
                    ROUTE_STOP_POSITION_DELTA_MM) &&
                (fabsf(Route_WrapYaw(state.current_pose.yaw_deg -
                                         previous.yaw_deg)) <= 1.0f))
            {
                if (++stable >= ROUTE_STOP_STABLE_SAMPLES)
                {
                    *pose = state.current_pose;
                    return 1U;
                }
            }
            else stable = 0U;
            previous = state.current_pose;
            have_previous = 1U;
        }
        else
        {
            stable = 0U;
            have_previous = 0U;
        }
        osDelay(ROUTE_STOP_POLL_MS);
    }
    return 0U;
}

/*
  函数说明: 路段中途不直接拐弯：用 OPS9 计算停车位置，沿原路退回上一节点，
  将遇障路段封闭。调用者随后从这个节点重新运行最短路搜索。
  调用示例: Route_RecoverBlockedEdge(next, start_pose);
  参数意义: next 为下一地图节点。
            start_pose 为进入当前路段前的底盘位姿。
*/
static uint8_t Route_RecoverBlockedEdge(RouteNode_t next,
                                            const ChassisPose_t *start_pose)
{
    ChassisPose_t stopped_pose;
    HCSR04_Reading_t event_reading;
    float start_rad;
    float stop_rad;
    float cosine;
    float sine;
    float traveled_x;
    float traveled_y;
    float progress;
    float lateral;
    float back_world_x;
    float back_world_y;
    float back_body_x;
    float back_body_y;
    float back_yaw;

    route_nav_status.recovery_active = 1U;
    route_nav_status.last_blocked_from = current_node;
    route_nav_status.last_blocked_to = next;
    RouteGraph_SetEdge(&graph, current_node, next, ROUTE_BLOCKED);
    Route_GetEmergencyReading(&event_reading);
    route_nav_status.last_obstacle_distance_mm =
        (event_reading.status == HCSR04_VALID) ? event_reading.distance_mm : 0U;

    if ((start_pose == NULL) || (Route_WaitForStop(&stopped_pose) == 0U))
        goto recovery_failed;

    /* 与底盘控制器使用同一 OPS9/车体坐标变换，沿原行驶方向投影。 */
    start_rad = -CHASSIS_COORD_ROTATION_SIGN *
                start_pose->yaw_deg * ROUTE_PI / 180.0f;
    cosine = cosf(start_rad);
    sine = sinf(start_rad);
    traveled_x = stopped_pose.x_mm - start_pose->x_mm;
    traveled_y = stopped_pose.y_mm - start_pose->y_mm;
    progress = -sine * traveled_x + cosine * traveled_y;
    lateral = cosine * traveled_x + sine * traveled_y;
    route_nav_status.last_stop_progress_mm = progress;
    if ((progress < -ROUTE_RECOVERY_BEHIND_MARGIN_MM) ||
        (progress > ROUTE_EDGE_MM + ROUTE_RECOVERY_OVERRUN_MARGIN_MM) ||
        (fabsf(lateral) > ROUTE_RECOVERY_MAX_LATERAL_MM) ||
        (fabsf(Route_WrapYaw(stopped_pose.yaw_deg -
                                 start_pose->yaw_deg)) > ROUTE_RECOVERY_MAX_YAW_DEG))
    {
        goto recovery_failed;
    }

    back_world_x = start_pose->x_mm - stopped_pose.x_mm;
    back_world_y = start_pose->y_mm - stopped_pose.y_mm;
    stop_rad = -CHASSIS_COORD_ROTATION_SIGN *
               stopped_pose.yaw_deg * ROUTE_PI / 180.0f;
    cosine = cosf(stop_rad);
    sine = sinf(stop_rad);
    back_body_x = cosine * back_world_x + sine * back_world_y;
    back_body_y = -sine * back_world_x + cosine * back_world_y;
    back_yaw = Route_WrapYaw(start_pose->yaw_deg - stopped_pose.yaw_deg);

    /* 后退时车头仍朝障碍物；关闭前向急停，走刚刚经过的已知路段。 */
    Route_ClearEmergency();
    if (((fabsf(back_body_x) > ROUTE_RECOVERY_SKIP_MOVE_MM) ||
         (fabsf(back_body_y) > ROUTE_RECOVERY_SKIP_MOVE_MM) ||
         (fabsf(back_yaw) > 0.5f)) &&
        (Route_Move(back_body_x, back_body_y, back_yaw) == 0U))
    {
        goto recovery_failed;
    }
    ++route_nav_status.recovery_count;
    ++route_nav_status.replan_count;
    route_nav_status.recovery_active = 0U;
    route_nav_status.result = ROUTE_NAV_RUNNING;
    return 1U;

recovery_failed:
    route_nav_status.recovery_active = 0U;
    route_nav_status.result = ROUTE_NAV_RECOVERY_FAILED;
    ChassisControl_Stop();
    return 0U;
}

/*
  函数说明: 循环选路、扫描、行走及遇障回退，直到到达指定地图节点。
  调用示例: Route_GoTo(target);
  参数意义: target 为目标地图节点。
*/
static uint8_t Route_GoTo(RouteNode_t target)
{
    /* 先按已知地图选最短路，只测即将进入的未知路段。
     * 扫描封路就在当前节点重选；途中遇障则退回当前节点再重选。 */
    RouteNode_t next;
    RouteHeading_t direction;
    RouteOccupancy_t edge_state;
    ChassisControlState_t chassis_state;
    ChassisPose_t departure_pose;
    uint8_t hops = 0U;
    route_nav_status.target = target;

    while (current_node != target)
    {
        if (++hops > ROUTE_MAX_DECISIONS)
        {
            route_nav_status.result = ROUTE_NAV_NO_PATH;
            return 0U;
        }
        RouteGraph_SetNode(&graph, current_node, ROUTE_CLEAR);
        if (RouteGraph_Next(&graph, current_node, target, &next) == 0U)
        {
            route_nav_status.result = ROUTE_NAV_NO_PATH;
            return 0U;
        }
        if (RouteGraph_Direction(current_node, next, &direction) == 0U)
        {
            route_nav_status.result = ROUTE_NAV_NO_PATH;
            return 0U;
        }
        edge_state = RouteGraph_GetEdge(&graph, current_node, next);
        if (edge_state == ROUTE_UNKNOWN)
        {
            if (Route_ScanSelectedEdge(direction) == 0U) return 0U;
            if ((RouteGraph_GetEdge(&graph, current_node, next) != ROUTE_CLEAR) ||
                (graph.node[next] == ROUTE_BLOCKED))
            {
                route_nav_status.last_blocked_from = current_node;
                route_nav_status.last_blocked_to = next;
                ++route_nav_status.replan_count;
                continue;
            }
        }
        if (Route_Face(direction) == 0U) return 0U;

        /* 记下已确认节点的实际 OPS9 位姿，途中停车后才能原路退回。 */
        ChassisControl_GetState(&chassis_state);
        if ((chassis_state.origin_valid == 0U) ||
            (ChassisControl_SensorsReady() == 0U))
        {
            route_nav_status.result = ROUTE_NAV_MOVE_FAILED;
            ChassisControl_Stop();
            return 0U;
        }
        departure_pose = chassis_state.current_pose;

        Route_ClearEmergency();
        Route_ArmFrontStop(1U);
        if (Route_Move(0.0f, ROUTE_EDGE_MM, 0.0f) == 0U)
        {
            Route_ArmFrontStop(0U);
            if (route_nav_status.result != ROUTE_NAV_EMERGENCY_STOP) return 0U;
            if (Route_RecoverBlockedEdge(next, &departure_pose) == 0U)
                return 0U;
            continue;
        }
        Route_ArmFrontStop(0U);
        current_node = next;
        route_nav_status.current = current_node;
    }
    return 1U;
}

/*
  函数说明: 在服务节点执行原来的靠站与退出位移动作。
  调用示例: Route_ServicePose(node);
  参数意义: node 为地图节点。
*/
static uint8_t Route_ServicePose(RouteNode_t node)
{
    /* 到任务节点后把车头对向工位：B 朝北、H 朝南、D 朝西。
     * 目前机械臂不动作，只演示 B 点额外靠近 200 mm 并退回节点。 */
    RouteHeading_t face;
    if (node == ROUTE_B) face = ROUTE_NORTH;
    else if (node == ROUTE_H) face = ROUTE_SOUTH;
    else face = ROUTE_WEST; /* D 点精加工区 */
    if (Route_Face(face) == 0U) return 0U;
    if (node == ROUTE_B)
    {
        if (Route_Move(0.0f, 200.0f, 0.0f) == 0U) return 0U;
        /* 机械臂动作暂未启用；退回 B 路口中心后再继续路径规划。 */
        if (Route_Move(0.0f, -200.0f, 0.0f) == 0U) return 0U;
    }
    return 1U;
}

/*
  函数说明: 执行原九节点地图的跑图流程；由 main_task 显式调用，失败会停止底盘。
  调用示例: result = Route_Run(); 在 main_task 中显式调用。
  参数意义: 无。
*/
uint8_t Route_Run(void)
{
    /* 任务顺序固定，路径则由路口扫描结果实时决定。 */
    static const RouteNode_t mission[] = {
        ROUTE_F, ROUTE_B, ROUTE_H, ROUTE_D,
        ROUTE_B, ROUTE_H, ROUTE_D, ROUTE_I
    };
    uint8_t step;

    Route_ClearEmergency();
    Route_ArmFrontStop(0U);
    RouteGraph_Init(&graph);
    current_node = ROUTE_I;
    heading = ROUTE_WEST; /* 起始车头朝 H，车右侧朝 F。 */
    route_nav_status.result = ROUTE_NAV_RUNNING;
    route_nav_status.current = ROUTE_NODE_COUNT; /* 尚在启停位，未到 I 路口中心。 */
    route_nav_status.heading = heading;
    route_nav_status.uncertain_scan_count = 0U;
    route_nav_status.emergency_distance_mm = 300U;

    route_nav_status.selected_scan_count = 0U;
    route_nav_status.replan_count = 0U;
    route_nav_status.recovery_count = 0U;
    route_nav_status.last_obstacle_distance_mm = 0U;
    route_nav_status.last_stop_progress_mm = 0.0f;
    route_nav_status.last_blocked_from = ROUTE_NODE_COUNT;
    route_nav_status.last_blocked_to = ROUTE_NODE_COUNT;
    route_nav_status.recovery_active = 0U;

    /* 启停位→I：车体右移 100 mm、前进 150 mm，合成一条斜线动作。 */
    if (Route_Move(100.0f, 150.0f, 0.0f) == 0U) return 0U;
    route_nav_status.current = ROUTE_I;

    for (step = 0U; step < (uint8_t)(sizeof(mission) / sizeof(mission[0])); ++step)
    {
        route_nav_status.mission_step = step;
        if (Route_GoTo(mission[step]) == 0U)
        {
            ChassisControl_Stop();
            return 0U;
        }
        if ((mission[step] == ROUTE_B) || (mission[step] == ROUTE_H) ||
            (mission[step] == ROUTE_D))
        {
            if (Route_ServicePose(mission[step]) == 0U) return 0U;
        }
        else if (mission[step] == ROUTE_F)
        {
            /* 不论如何绕行，到 F 扫码位后都转回原定的朝北姿态。 */
            if (Route_Face(ROUTE_NORTH) == 0U) return 0U;
        }
        /* 二维码识别和机械臂抓放还未接入本次跑图。 */
    }
    if (Route_Face(ROUTE_WEST) == 0U) return 0U;
    /* I→启停位：车头恢复原朝向后，原路反向斜行。 */
    if (Route_Move(-100.0f, -150.0f, 0.0f) == 0U) return 0U;
    route_nav_status.result = ROUTE_NAV_COMPLETE;
    return 1U;
}

/*
  函数说明: 执行重构前 paotu_alone 的 14 段固定底盘动作，保留坐标、转角与速度参数。
            此入口沿固定顺序行走；使用超声波避障时调用 Route_Run。
  调用示例: if (!Route_RunFixedSequence()) { ChassisControl_Stop(); }
  参数意义: 无；全部动作成功返回 1，任一步失败则停止并返回 0。
*/
uint8_t Route_RunFixedSequence(void)
{
    static const float actions[][3] = {
        { 190.0f,   190.0f, 0.0f }, /* 启停区到扫码区。 */
				{ 850.0f,   0.0f, -90.0f },
        { -850.0f,     0.0f,   0.0f },
        {    0.0f,  950.0f,   0.0f }, /* 第一次物料盘。 */
				
        {    0.0f, -1800.0f, 180.0f }, /* 粗加工区。 */
        {    0.0f,  -850.0f,   0.0f },
        {  850.0f,     0.0f, -90.0f }, /* 精加工区。 */
        {    0.0f,  -850.0f,   0.0f },
        { 950.0f,     0.0f, -90.0f }, /* 第二次物料盘。 */
        {    0.0f, -1800.0f, 180.0f },
        {    0.0f,  -850.0f,   0.0f },
        {  850.0f,     0.0f, -90.0f },
        {    0.0f, -1700.0f,   0.0f },
        { -850.0f,     0.0f,   0.0f },
        { -150.0f,  -150.0f,   0.0f }  /* 返回启停区。 */
    };
    uint8_t index;
    for (index = 0U; index < sizeof(actions) / sizeof(actions[0]); ++index)
    {
        if (!ChassisControl_MoveRelativeSequential(actions[index][0], actions[index][1],
                actions[index][2], 100.0f, 150.0f, 800.0f))
        {
            ChassisControl_Stop();
            return 0U;
        }
    }
    return 1U;
}
