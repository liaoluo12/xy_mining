#include "XYControlTask.h"

#include <cstdlib>
#include "cmsis_os.h"
#include "can.h"
#include "usart.h"

#include "TimingThread.h"
#include "oled.h"

using rm::hal::Can;                  // 引入CAN总线
Can can1(hcan1);                     // 创建CAN对象
XYControl *XYcontrol;                // 创建XY二维控制对象
rm::f32 motor_speed_static = 20000;  // 正常移动速度变量
rm::f32 motor_speed_move = 8000;     // 匀速移动速度变量

// 遥控器对象以及电机遥控数据变量创建
static rm::hal::Serial *remote_uart;
static DR16 *remote;

// 全局变量声明
ExchangeState exchange_state = EXCHANGE_IDLE;
ExchangeLevel exchange_level = LEVEL_0;
ExchangeLevel last_exchange_level = LEVEL_0;

// 运动相关全局变量
float rc_x_data = 0;
float rc_y_data = 0;
fp64 x_error = 0;
fp64 y_error = 0;
fp64 step = static_cast<fp64>(2) / 36;
const fp32 position_tolerance = 1.0f;  // 位置容差(mm)

// 计时相关全局变量
char time_str[12];         // 时间字符串
char manul_point_str[16];  // 胜利点字符串
char auto_point_str[16];   // 胜利点字符串
fp32 proportion = 1.0f;
uint32_t move_time = 5000;

// 各种标志位
bool single_random = false;
bool reset_flag = false;
bool level_selected = false;
bool green_light = false;
bool exchange_success = false;
bool over_time = false;

/**
 * @brief 创建一个电机控制类(初始化列表)
 *
 */
XYControl::XYControl() :
    x_motor(can1, 1),
    x_pid_speed(12, 0, 0, 10000, 0),

    y_motor(can1, 2),
    y_pid_speed(12, 0, 0, 10000, 0)
{
}

/**
 * @brief extern "C" 声明函数在C++中可见，个人习惯，不强制要求
 *
 */
extern "C"
{
  // OLED显示指定字符初始化
  void OLED_ShowInit()
  {
    // 字符内容初始化
    OLED_NewFrame();
    // OLED_PrintString(46, 0, "IRBOT", &font16x16, OLED_COLOR_NORMAL);
    OLED_PrintString(0, 6, "TIME:", &font16x16, OLED_COLOR_NORMAL);
    OLED_PrintString(0, 24, "MANPOINT:", &font16x16, OLED_COLOR_NORMAL);
    OLED_PrintString(0, 42, "AUTOPOINT:", &font16x16, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
  }

  // OLED显示手动和自动兑矿胜利点
  void OLED_ShowPoint()
  {
    sprintf(manul_point_str, "%lu ", XYcontrol->manul_victory_point);
    OLED_PrintASCIIString(80, 24, manul_point_str, &afont16x8, OLED_COLOR_NORMAL);

    sprintf(auto_point_str, "%lu ", XYcontrol->auto_victory_point);
    OLED_PrintASCIIString(90, 42, auto_point_str, &afont16x8, OLED_COLOR_NORMAL);
  }

  // OLED显示单次兑矿成功后的用时
  void OLED_ShowSingleTime()
  {
    if (over_time)
    {
      OLED_PrintString(60, 6, "OVERTIME", &font16x16, OLED_COLOR_NORMAL);
    }
    else if (!exchange_success)
    {
      OLED_PrintASCIIString(60, 6, "0          ", &afont16x8, OLED_COLOR_NORMAL);  // 打印转换后的字符串
    }
  }

  // 实时显示兑矿时间
  void OLED_LiveShowSingleTime()
  {
    if (!reset_flag)
    {
      sprintf(time_str, "%lu", (HAL_GetTick() - XYcontrol->exchange_start_time - move_time) / 1000);  // 将sys_tick的值转换为字符串
      OLED_PrintASCIIString(60, 6, time_str, &afont16x8, OLED_COLOR_NORMAL);                          // 打印转换后的字符串
    }
  }

  /*************************************/

  // 更新位置计数(x轴导程14mm, y轴导程8mm)，内径x760(-330~0~330)->(-300~0~300),y400(0~200~400)->(-100~0~100)
  void UpdatePosition()
  {
    static int32_t last_x_encoder = 0;
    static int32_t last_y_encoder = 0;

    int32_t current_x_encoder = XYcontrol->x_motor.encoder();
    int32_t current_y_encoder = XYcontrol->y_motor.encoder();

    if (abs(current_x_encoder - last_x_encoder) > 4096)
    {
      XYcontrol->x_pos += (current_x_encoder < last_x_encoder) ? step * 7 : -step * 7;
    }
    if (abs(current_y_encoder - last_y_encoder) > 4096)
    {
      XYcontrol->y_pos += (current_y_encoder < last_y_encoder) ? step * 4 : -step * 4;
    }

    last_x_encoder = current_x_encoder;
    last_y_encoder = current_y_encoder;
  }

  // 复位档摇杆检测
  void CheckResetSwitch()
  {
    if (remote->switch_l() == RcSwitchState::kDown || remote->switch_r() == RcSwitchState::kDown)
    {
      reset_flag = true;
    }
  }

  // 运动逻辑状态机(选择运动模式)
  void SelectExchangeLevel()
  {
    if (remote->switch_l() == RcSwitchState::kMid && remote->switch_r() != RcSwitchState::kDown)
    {
      if (remote->switch_r() == RcSwitchState::kMid)
      {
        if (exchange_level != LEVEL_1)
        {
          single_random = false;
          exchange_level = LEVEL_1;
        }
        if (!single_random)
        {
          XYcontrol->x_pos_new = (rand() % 601) - 300;  // -300~300
          XYcontrol->y_pos_new = -100;
          single_random = true;
        }
      }
      else if (remote->switch_r() == RcSwitchState::kUp)
      {
        if (exchange_level != LEVEL_2)
        {
          single_random = false;
          exchange_level = LEVEL_2;
        }
        if (!single_random)
        {
          XYcontrol->x_pos_new = (rand() % 601) - 300;  // -300~300
          XYcontrol->y_pos_new = (rand() % 201) - 100;  // -100~100
          single_random = true;
        }
      }
    }
    else if (remote->switch_l() == RcSwitchState::kUp && remote->switch_r() != RcSwitchState::kDown)
    {
      if (remote->switch_r() == RcSwitchState::kMid)
      {
        exchange_level = LEVEL_3;
        XYcontrol->y_pos_new = -100;
      }
      else if (remote->switch_r() == RcSwitchState::kUp)
      {
        exchange_level = LEVEL_4;
      }
    }
    else
    {
      if (exchange_level != LEVEL_0)
      {
        exchange_level = LEVEL_0;
        single_random = false;
        level_selected = false;
        green_light = false;
        over_time = false;
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);  // 灭红灯
      }

      // 复位或手控
      if (remote->dial() > 500)
      {
        /*遥控器设置中点，摇杆控制电机*/
        rc_x_data = utils::Map(remote->left_x(), -660, 660, -10000, 10000);
        XYcontrol->x_pid_speed.Update(rc_x_data, XYcontrol->x_motor.rpm());
        XYcontrol->x_motor.SetCurrent(XYcontrol->x_pid_speed.value());

        rc_y_data = utils::Map(remote->left_y(), -660, 660, -10000, 10000);
        XYcontrol->y_pid_speed.Update(rc_y_data, XYcontrol->y_motor.rpm());
        XYcontrol->y_motor.SetCurrent(XYcontrol->y_pid_speed.value());

        XYcontrol->x_pos = 0;
        XYcontrol->y_pos = 0;
      }
      else
      {
        // 复位档位
        XYcontrol->x_pos_new = 0;
        XYcontrol->y_pos_new = 0;
      }
    }
  }

  // 兑矿等级正确切换
  void UpdateExchangeState()
  {
    if ((exchange_level != last_exchange_level) && (exchange_level != LEVEL_0))
    {
      // 切换标志置1
      level_selected = true;
      green_light = false;
      exchange_success = false;

      reset_flag = false;

      // 超时标志置0
      over_time = false;
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);

      // 记录开始时间
      XYcontrol->exchange_start_time = HAL_GetTick();
    }
    last_exchange_level = exchange_level;  // 记录当前等级
  }

  /*************************************/

  // 计算手动和自动兑矿胜利点
  void CalculateVictoryPoint(uint32_t used_time = 0)
  {
    if (used_time >= 0 && used_time < 15)
    {
      proportion = 1.0f;
    }
    else if (used_time >= 15 && used_time < 30)
    {
      proportion = 0.8f;
    }
    else if (used_time >= 30 && used_time < 45)
    {
      proportion = 0.5f;
    }
    else if (used_time >= 45 && used_time < 60)
    {
      proportion = 0.2f;
    }
    else
    {
      over_time = true;
    }

    switch (exchange_level)
    {
      case LEVEL_1:
        XYcontrol->manul_victory_point = 10 * proportion;
        XYcontrol->auto_victory_point = 40 * proportion;
        break;
      case LEVEL_2:
        XYcontrol->manul_victory_point = 20 * proportion;
        XYcontrol->auto_victory_point = 50 * proportion;
        break;
      case LEVEL_3:
        XYcontrol->manul_victory_point = 50 * proportion;
        XYcontrol->auto_victory_point = 150 * proportion;
        break;
      case LEVEL_4:
        XYcontrol->manul_victory_point = 80 * proportion;
        XYcontrol->auto_victory_point = 240 * proportion;
        break;

      default:
        XYcontrol->manul_victory_point = 0;
        XYcontrol->auto_victory_point = 0;
        break;
    }
  }

  // 微动开关触发复位
  void ButtonTrigger()
  {
    if (button_changed)
    {
      button_changed = false;
      level_selected = false;
      green_light = false;
      exchange_success = true;
      CalculateVictoryPoint((HAL_GetTick() - XYcontrol->exchange_start_time - move_time) / 1000);  // 计算胜利点
      exchange_state = EXCHANGE_IDLE;
    }
  }

  /*************************************/

  // 停止状态
  void power_off()
  {
    XYcontrol->x_pid_speed.Update(0, XYcontrol->x_motor.rpm());
    XYcontrol->x_motor.SetCurrent(XYcontrol->x_pid_speed.value());
    x_error = 0;
    XYcontrol->y_pid_speed.Update(0, XYcontrol->y_motor.rpm());
    XYcontrol->y_motor.SetCurrent(XYcontrol->y_pid_speed.value());
    y_error = 0;
  }

  // 兑换槽移动控制
  void MoveExchangeSlot()
  {
    fp64 kp_speed = 0.6;  // 比例增益

    // 计算位置误差
    x_error = XYcontrol->x_pos_new - XYcontrol->x_pos;
    y_error = XYcontrol->y_pos_new - XYcontrol->y_pos;

    switch (exchange_level)
    {
      // 静止状态位置控制
      case LEVEL_0:
      case LEVEL_1:
      case LEVEL_2:
      {
        if (fabs(x_error) < position_tolerance || fabs(y_error) < position_tolerance)
        {
          // X轴控制（比例速度）
          if (fabs(x_error) < position_tolerance)
          {
            if (fabs(x_error) > 0.1)  // 增加死区防止震荡
            {
              fp64 x_speed_target = fabs(x_error) * kp_speed;
              if (x_speed_target > motor_speed_static) x_speed_target = motor_speed_static;

              XYcontrol->x_pid_speed.Update(x_error > 0 ? 2 * x_speed_target : -2 * x_speed_target, XYcontrol->x_motor.rpm());
              XYcontrol->x_motor.SetCurrent(XYcontrol->x_pid_speed.value());
            }
            else
            {
              XYcontrol->x_pid_speed.Update(0, XYcontrol->x_motor.rpm());
              XYcontrol->x_motor.SetCurrent(XYcontrol->x_pid_speed.value());
              x_error = 0;
            }
          }
          else
          {
            XYcontrol->x_pid_speed.Update(x_error > 0 ? motor_speed_static : -motor_speed_static, XYcontrol->x_motor.rpm());
            XYcontrol->x_motor.SetCurrent(XYcontrol->x_pid_speed.value());
          }

          // Y轴控制（比例速度）
          if (fabs(y_error) < position_tolerance)
          {
            if (fabs(y_error) > 0.1)  // 增加死区防止震荡
            {
              fp64 y_speed_target = fabs(y_error) * kp_speed;
              if (y_speed_target > motor_speed_static) y_speed_target = motor_speed_static;

              XYcontrol->y_pid_speed.Update(y_error > 0 ? 2 * y_speed_target : -2 * y_speed_target, XYcontrol->y_motor.rpm());
              XYcontrol->y_motor.SetCurrent(XYcontrol->y_pid_speed.value());
            }
            else
            {
              XYcontrol->y_pid_speed.Update(0, XYcontrol->y_motor.rpm());
              XYcontrol->y_motor.SetCurrent(XYcontrol->y_pid_speed.value());
              y_error = 0;
            }
          }
          else
          {
            XYcontrol->y_pid_speed.Update(y_error > 0 ? motor_speed_static : -motor_speed_static, XYcontrol->y_motor.rpm());
            XYcontrol->y_motor.SetCurrent(XYcontrol->y_pid_speed.value());
          }
        }
        else
        {
          XYcontrol->x_pid_speed.Update(x_error > 0 ? motor_speed_static : -motor_speed_static, XYcontrol->x_motor.rpm());
          XYcontrol->x_motor.SetCurrent(XYcontrol->x_pid_speed.value());

          XYcontrol->y_pid_speed.Update(y_error > 0 ? motor_speed_static : -motor_speed_static, XYcontrol->y_motor.rpm());
          XYcontrol->y_motor.SetCurrent(XYcontrol->y_pid_speed.value());
        }
      }
      break;

      // 三级：X匀速，Y固定
      case LEVEL_3:
      {
        // X轴匀速运动
        XYcontrol->x_pid_speed.Update(XYcontrol->x_direction == 1 ? motor_speed_move : -motor_speed_move, XYcontrol->x_motor.rpm());
        XYcontrol->x_motor.SetCurrent(XYcontrol->x_pid_speed.value());

        // Y轴保持位置
        if (fabs(y_error) < position_tolerance)
        {
          // Y轴控制（比例速度）
          if (fabs(y_error) > 0.1)  // 增加死区防止震荡
          {
            fp64 y_speed_target = fabs(y_error) * kp_speed;
            if (y_speed_target > motor_speed_static) y_speed_target = motor_speed_static;

            XYcontrol->y_pid_speed.Update(y_speed_target, XYcontrol->y_motor.rpm());
            XYcontrol->y_motor.SetCurrent(y_error > 0 ? XYcontrol->y_pid_speed.value() : -XYcontrol->y_pid_speed.value());
          }
          else
          {
            XYcontrol->y_pid_speed.Update(0, XYcontrol->y_motor.rpm());
            XYcontrol->y_motor.SetCurrent(XYcontrol->y_pid_speed.value());
            y_error = 0;
          }
        }
        else
        {
          XYcontrol->y_pid_speed.Update(motor_speed_static, XYcontrol->y_motor.rpm());
          XYcontrol->y_motor.SetCurrent(y_error > 0 ? XYcontrol->y_pid_speed.value() : -XYcontrol->y_pid_speed.value());
        }
      }
      break;

      // 四级：XY匀速
      case LEVEL_4:
      {
        // X轴匀速
        XYcontrol->x_pid_speed.Update(XYcontrol->x_direction == 1 ? motor_speed_move : -motor_speed_move, XYcontrol->x_motor.rpm());
        XYcontrol->x_motor.SetCurrent(XYcontrol->x_pid_speed.value());

        // Y轴匀速
        XYcontrol->y_pid_speed.Update(XYcontrol->y_direction == 1 ? motor_speed_move : -motor_speed_move, XYcontrol->y_motor.rpm());
        XYcontrol->y_motor.SetCurrent(XYcontrol->y_pid_speed.value());
        XYcontrol->y_pos_new += 0.1f;
      }
      break;

      default:
        power_off();
        break;
    }
  }

  // 移动时间检查
  void MoveTimeCheck()
  {
    if ((HAL_GetTick() - XYcontrol->exchange_start_time > move_time && level_selected) || (reset_flag))
    {
      if (!reset_flag)
      {
        green_light = true;
      }
      exchange_state = EXCHANGE_READY;
    }
  }

  // 60秒超时检查
  void OverTimeCheck()
  {
    if (((HAL_GetTick() - XYcontrol->exchange_start_time - move_time) > 60000 && !exchange_success) || reset_flag)
    {
      if (!reset_flag)
      {
        over_time = true;
        green_light = false;
        level_selected = false;
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);  // 亮红灯
      }
      exchange_state = EXCHANGE_IDLE;
    }
    reset_flag = false;
  }

  // 边界检查函数
  void CheckBoundaries()
  {
    switch (exchange_level)
    {
      case LEVEL_0:
      case LEVEL_1:
      case LEVEL_2:
        // 到达边界时清零误差，防止持续输出电流
        if (XYcontrol->x_pos >= 300 || XYcontrol->x_pos <= -300) x_error = 0;
        if (XYcontrol->y_pos >= 100 || XYcontrol->y_pos <= -100) y_error = 0;
        break;

      case LEVEL_3:
        // X轴到达边界时反转方向
        if (XYcontrol->x_pos >= 300)
        {
          XYcontrol->x_direction = -1;
        }
        else if (XYcontrol->x_pos <= -300)
        {
          XYcontrol->x_direction = 1;
        }
        break;

      case LEVEL_4:
        // XY轴到达边界时都反转方向
        if (XYcontrol->x_pos >= 300)
        {
          XYcontrol->x_direction = -1;
        }
        else if (XYcontrol->x_pos <= -300)
        {
          XYcontrol->x_direction = 1;
        }

        if (XYcontrol->y_pos >= 100)
        {
          XYcontrol->y_direction = -1;
        }
        else if (XYcontrol->y_pos <= -100)
        {
          XYcontrol->y_direction = 1;
        }
        break;

      default:
        power_off();
        break;
    }
  }

  // 兑矿槽运动一
  void MoveExchangeSlot_idel()
  {
    if (!(remote->dial() > 500))  // 避免与手控复位冲突
    {
      if (!(exchange_success || over_time) || reset_flag)
      {
        MoveExchangeSlot();
      }
      else
      {
        power_off();
      }
    }
    CheckBoundaries();  // 边界检查
  }

  // 兑矿槽运动二
  void MoveExchangeSlot_ready()
  {
    if (exchange_level == LEVEL_3 || exchange_level == LEVEL_4)
    {
      if (!(exchange_success || over_time) || reset_flag)
      {
        MoveExchangeSlot();
      }
      else
      {
        power_off();
      }
    }
    CheckBoundaries();  // 边界检查
  }

  /*************************************/

  /*初始化XY控制系统的函数*/
  void XYControlInit()
  {
    // CAN1初始化
    can1.SetFilter(0, 0);
    can1.Begin();

    // 遥控器初始化
    remote_uart = new hal::Serial(huart1, 18, hal::stm32::UartMode::kNormal, hal::stm32::UartMode::kDma);
    remote = new DR16(*remote_uart);
    remote->Begin();

    // XY二维控制对象赋值
    XYcontrol = new XYControl();

    // 随机数种子初始化
    srand(HAL_GetTick());
  }
}

/**
 * @brief XY二维电机控制线程
 *
 * @param argument
 */
void XYControlTask(void const *argument)
{
  UNUSED(argument);

  XYControlInit();

  OLED_ShowInit();

  while (1)
  {
    OLED_ShowPoint();

    UpdatePosition();

    CheckResetSwitch();

    switch (exchange_state)
    {
      case EXCHANGE_IDLE:
        SelectExchangeLevel();

        UpdateExchangeState();

        OLED_ShowSingleTime();

        MoveExchangeSlot_idel();

        MoveTimeCheck();
        break;

      case EXCHANGE_READY:

        OLED_LiveShowSingleTime();

        MoveExchangeSlot_ready();

        ButtonTrigger();

        OverTimeCheck();
        break;
    }

    M2006::SendCommand();

    osDelay(1);
  }
}
