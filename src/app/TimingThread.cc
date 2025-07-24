#include "TimingThread.h"
#include "cmsis_os.h"
#include "main.h"

#include "XYControlTask.h"
#include "oled.h"

typedef struct
{
  uint32_t press_start_time;   // 按钮按下时刻
  uint8_t prev_button_state;   // 记录上一次的按钮状态
  uint8_t button_debounced;    // 消抖后按钮状态
  uint8_t long_press_handled;  // 长按已处理标志
} ButtonState;
ButtonState button_state = {0};

uint32_t sys_tick = 0;        // 系统时间，用于计算微动开关触发有效时间
bool button_changed = false;  // 按钮按下标志(计时相关标志位)

// 个人习惯，不强制要求
extern "C"
{
  /**
   * @brief 更新按钮状态（带消抖）
   */
  void update_button_states(void)
  {
    static uint32_t last_debounce_time = 0;
    const uint16_t debounce_delay = 20;  // 20ms消抖时间

    uint8_t current_state = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_9);  // 获取当前按钮状态

    // 若当前状态与上一次状态不同，则进入消抖逻辑
    if (current_state != button_state.prev_button_state)
    {
      last_debounce_time = sys_tick;
    }

    // 若消抖时间超过阈值，则更新按钮状态
    if ((sys_tick - last_debounce_time) > debounce_delay)
    {
      button_state.button_debounced = current_state;
    }

    // 记录当前状态
    button_state.prev_button_state = current_state;
  }

  /**
   * @brief 处理按钮逻辑
   */
  void process_buttons(void)
  {
    // 按钮按下（消抖后）
    if (button_state.button_debounced == GPIO_PIN_RESET)
    {
      // 记录首次按下时间
      if (button_state.press_start_time == 0)
      {
        button_state.press_start_time = sys_tick;
        button_state.long_press_handled = 0;  // 重置长按处理标志
      }

      if (exchange_level == LEVEL_0 || exchange_level == LEVEL_1 || exchange_level == LEVEL_2)
      {
        // 按下超过1秒且未处理过
        if (!button_state.long_press_handled && (sys_tick - button_state.press_start_time) >= 1000 && !over_time && !exchange_success)
        {
          green_light = !green_light;  // 切换LED显示模式
          button_changed = true;       // 按钮按下标志置位
          button_state.long_press_handled = 1;
        }
      }
      else if (exchange_level == LEVEL_3 || exchange_level == LEVEL_4)
      {
        // 按下超过40毫秒且未处理过
        if (!button_state.long_press_handled && (sys_tick - button_state.press_start_time) >= 40 && !over_time && !exchange_success)
        {
          green_light = !green_light;  // 切换LED显示模式
          button_changed = true;       // 按钮按下标志置位
          button_state.long_press_handled = 1;
        }
      }
    }
    // 按钮松开
    else
    {
      // 复位按钮状态
      button_state.press_start_time = 0;
      button_state.long_press_handled = 0;
    }
  }

  /**
   * @brief 更新LED显示
   */
  void update_leds(void)
  {
    if (green_light)
    {
      // 常亮模式
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    }
    else
    {
      // 常灭模式
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    }
  }
}

/**
 * @brief 计时线程
 *
 * @param argument
 */
void TimingThread(void const *argument)
{
  (void)argument;

  button_state.prev_button_state = 1;  // 按钮初始状态为松开

  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);  // 常灭模式
  green_light = false;                                    // 初始状态为常灭

  while (1)
  {
    // 更新系统时间
    sys_tick = HAL_GetTick();

    // OLED屏幕刷新
    OLED_ShowFrame();

    // 更新按钮状态
    update_button_states();

    // 处理按钮逻辑
    process_buttons();

    // 更新LED显示
    update_leds();

    // 冷却cd
    osDelay(10);
  }
}
