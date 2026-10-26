/*
  程序说明: 九节点地图最短路、超声波扫描、路段遇障回退及跑图流程。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

#ifndef FUN_ROUTE_MODULE_H
#define FUN_ROUTE_MODULE_H
#include <stdint.h>

typedef enum
{
    /* 地图按从上到下、每行从左到右编号。 */
    ROUTE_A = 0U, ROUTE_B, ROUTE_C,
    ROUTE_D, ROUTE_E, ROUTE_F,
    ROUTE_G, ROUTE_H, ROUTE_I,
    ROUTE_NODE_COUNT
} RouteNode_t;

typedef enum
{
    ROUTE_NORTH = 0U, ROUTE_EAST, ROUTE_SOUTH, ROUTE_WEST
} RouteHeading_t;

typedef enum
{
    ROUTE_UNKNOWN = 0U, ROUTE_CLEAR, ROUTE_BLOCKED
} RouteOccupancy_t;

typedef struct
{
    RouteOccupancy_t node[ROUTE_NODE_COUNT];
    RouteOccupancy_t edge[12];
} RouteGraph_t;

/*
  函数说明: 初始化九节点地图的节点状态和十二条边的通行状态。
  调用示例: RouteGraph_Init(graph);
  参数意义: graph 为九节点地图及占用状态。
*/
void RouteGraph_Init(RouteGraph_t *graph);
/*
  函数说明: 取得指定节点在某一朝向上的相邻节点；越界返回 0。
  调用示例: RouteGraph_Neighbor(from, direction, neighbor);
  参数意义: from 为路径起点节点。
            direction 为方向枚举。
            neighbor 为输出相邻地图节点。
*/
uint8_t RouteGraph_Neighbor(RouteNode_t from, RouteHeading_t direction,
                            RouteNode_t *neighbor);
/*
  函数说明: 更新地图中一条无向边的占用状态。
  调用示例: RouteGraph_SetEdge(graph, a, b, state);
  参数意义: graph 为九节点地图及占用状态。
            a 为道路第一个端点。
            b 为道路第二个端点。
            state 为状态值或状态快照。
*/
void RouteGraph_SetEdge(RouteGraph_t *graph, RouteNode_t a,
                        RouteNode_t b, RouteOccupancy_t state);
/*
  函数说明: 查询两个节点之间的通行状态。
  调用示例: RouteGraph_GetEdge(graph, a, b);
  参数意义: graph 为九节点地图及占用状态。
            a 为道路第一个端点。
            b 为道路第二个端点。
*/
RouteOccupancy_t RouteGraph_GetEdge(const RouteGraph_t *graph,
                                    RouteNode_t a, RouteNode_t b);
/*
  函数说明: 更新一个地图节点的占用状态。
  调用示例: RouteGraph_SetNode(graph, node, state);
  参数意义: graph 为九节点地图及占用状态。
            node 为地图节点。
            state 为状态值或状态快照。
*/
void RouteGraph_SetNode(RouteGraph_t *graph, RouteNode_t node,
                        RouteOccupancy_t state);
/*
  函数说明: 执行最短路搜索，输出当前位置走向目标的下一节点。
  调用示例: RouteGraph_Next(graph, from, goal, next);
  参数意义: graph 为九节点地图及占用状态。
            from 为路径起点节点。
            goal 为路径目标节点。
            next 为下一地图节点。
*/
uint8_t RouteGraph_Next(const RouteGraph_t *graph, RouteNode_t from,
                        RouteNode_t goal, RouteNode_t *next);
/*
  函数说明: 求相邻两节点之间的地图行进朝向。
  调用示例: RouteGraph_Direction(from, to, direction);
  参数意义: from 为路径起点节点。
            to 为路径终点节点。
            direction 为方向枚举。
*/
uint8_t RouteGraph_Direction(RouteNode_t from, RouteNode_t to,
                             RouteHeading_t *direction);

typedef enum
{
    ROUTE_NAV_IDLE = 0U,
    ROUTE_NAV_RUNNING,
    ROUTE_NAV_COMPLETE,
    ROUTE_NAV_NO_PATH,
    ROUTE_NAV_SCAN_FAULT,
    ROUTE_NAV_MOVE_FAILED,
    ROUTE_NAV_EMERGENCY_STOP,
    ROUTE_NAV_RECOVERY_FAILED
} Route_Result_t;

/* Keil Watch 可直接观察当前任务、确认到达的节点和运行结果。 */
typedef struct
{
    Route_Result_t result;
    RouteNode_t current;
    RouteNode_t target;
    RouteHeading_t heading;
    uint8_t mission_step;
    uint16_t uncertain_scan_count;
    uint16_t emergency_distance_mm;
    uint16_t selected_scan_count;       /* 实际检查过的候选路段次数。 */
    uint16_t replan_count;              /* 扫描封路或途中遇障后的改路次数。 */
    uint16_t recovery_count;            /* 途中遇障后成功退回节点的次数。 */
    uint16_t last_obstacle_distance_mm; /* 上次急停时车头探头的读数。 */
    float last_stop_progress_mm;        /* 上次急停位置距出发节点的沿路距离。 */
    RouteNode_t last_blocked_from;
    RouteNode_t last_blocked_to;
    uint8_t recovery_active;
} Route_Status_t;

/*
  函数说明: 执行原九节点地图的跑图流程；由 main_task 显式调用，失败会停止底盘。
  调用示例: result = Route_Run(); 在 main_task 中显式调用。
  参数意义: 无。
*/
uint8_t Route_Run(void);
extern volatile Route_Status_t route_nav_status;
/*
  函数说明: 执行重构前 paotu_alone 的 14 段固定底盘动作，保留坐标、转角与速度参数。
            此入口沿固定顺序行走；使用超声波避障时调用 Route_Run。
  调用示例: if (!Route_RunFixedSequence()) { ChassisControl_Stop(); }
  参数意义: 无；全部动作成功返回 1，任一步失败则停止并返回 0。
*/
uint8_t Route_RunFixedSequence(void);
#endif
