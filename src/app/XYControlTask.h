#ifndef XY_CONTROL_TASK_H
#define XY_CONTROL_TASK_H

#include "librm.hpp"
#include "struct_typedef.h"

#ifdef __cplusplus
extern "C"
{
#endif

  using namespace rm;
  using namespace rm::hal;
  using namespace rm::device;              // 引入电机类
  using namespace rm::modules::algorithm;  // 引入PID模板

  extern void XYControlTask(void const *argument);

  // 二维平面控制类
  class XYControl
  {
   public:
    XYControl();             // 构造函数，用来初始化电机(CAN, CANID, PID参数)
    ~XYControl() = default;  // 默认析构显示表达

    // x、y电机对象
    M2006 x_motor;
    M2006 y_motor;
    // 单速度环PID参数
    PID<PIDType::kPosition> x_pid_speed;
    PID<PIDType::kPosition> y_pid_speed;

    // 位置计数 (单位：圈，1圈=2mm)
    fp64 x_pos = 0;
    fp64 y_pos = 0;
    // 兑换槽初始坐标
    fp64 x_pos_new = 0;
    fp64 y_pos_new = 0;
    // 运动方向
    int8_t x_direction = 1;
    int8_t y_direction = 1;

    // 计时
    uint32_t exchange_start_time = 0;
    // 胜利点
    u16 manul_victory_point = 0;
    u16 auto_victory_point = 0;
  };

  // 兑换状态枚举
  enum ExchangeState
  {
    EXCHANGE_IDLE,   // 空闲状态
    EXCHANGE_READY,  // 可兑换(绿灯亮)
  };

  // 兑换等级枚举
  enum ExchangeLevel
  {
    LEVEL_0,
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
    LEVEL_4
  };

  extern bool green_light;
  extern bool over_time;
  extern bool exchange_success;
  extern ExchangeLevel exchange_level;

#ifdef __cplusplus
}
#endif

#endif /* XY_CONTROL_TASK_H */
