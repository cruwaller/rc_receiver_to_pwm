/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "led.h"
#include "uart.h"
#include "helpers.h"
#if PROTO_CRSF
#include "crsf.h"
#elif PROTO_SBUS
#include "sbus.h"
#elif PROTO_GHST
#include "ghst.h"
#endif
#include "defines.h"
#include <string.h>


/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);


// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
  //return (uint32_t)(__LL_RCC_CALC_PCLK1_FREQ(HAL_RCC_GetHCLKFreq(), LL_RCC_GetAPB1Prescaler()));
  return HAL_RCC_GetPCLK1Freq();
}


/* Private user code ---------------------------------------------------------*/

#if STM32F0
  #define APB1PERIPH_BASE (APBPERIPH_BASE + 0x00000000)
  #define APB2PERIPH_BASE (APBPERIPH_BASE + 0x00010000)
#endif

// Check if a peripheral clock has been enabled
uint32_t is_enabled_pclock(uint32_t periph_base)
{
  if (periph_base < APB2PERIPH_BASE) {
    uint32_t pos = (periph_base - APB1PERIPH_BASE) / 0x400;
    return RCC->APB1ENR & (1 << pos);
  } else if (periph_base < AHBPERIPH_BASE) {
    uint32_t pos = (periph_base - APB2PERIPH_BASE) / 0x400;
    return RCC->APB2ENR & (1 << pos);
  } else {
    uint32_t pos = (periph_base - AHBPERIPH_BASE) / 0x400;
    return RCC->AHBENR & (1 << pos);
  }
}


// Enable a peripheral clock
void enable_pclock(uint32_t periph_base)
{
  if (is_enabled_pclock(periph_base))
    return;

  if (periph_base < APB2PERIPH_BASE) {
    uint32_t pos = (periph_base - APB1PERIPH_BASE) / 0x400;
    RCC->APB1ENR |= (1 << pos);
    RCC->APB1ENR;
  } else if (periph_base < AHBPERIPH_BASE) {
    uint32_t pos = (periph_base - APB2PERIPH_BASE) / 0x400;
    RCC->APB2ENR |= (1 << pos);
    RCC->APB2ENR;
  } else {
    uint32_t pos = (periph_base - AHBPERIPH_BASE) / 0x400;
    RCC->AHBENR |= (1 << pos);
    RCC->AHBENR;
  }
}


void gpio_port_pin_get(uint32_t io, void ** port, uint32_t * pin)
{
  *pin = IO_GET_PIN(io);
  io = IO_GET_PORT(io);
  *port = (void*)((uintptr_t)GPIOA_BASE + (io * 0x0400UL));
}


void gpio_port_clock(uint32_t port)
{
  // Enable gpio clock
  switch (port) {
    case GPIOA_BASE:
      __HAL_RCC_GPIOA_CLK_ENABLE();
      break;
    case GPIOB_BASE:
      __HAL_RCC_GPIOB_CLK_ENABLE();
      break;
#ifdef __HAL_RCC_GPIOC_CLK_ENABLE
    case GPIOC_BASE:
      __HAL_RCC_GPIOC_CLK_ENABLE();
      break;
#endif
#ifdef __HAL_RCC_GPIOD_CLK_ENABLE
    case GPIOD_BASE:
      __HAL_RCC_GPIOD_CLK_ENABLE();
      break;
#endif
#ifdef __HAL_RCC_GPIOE_CLK_ENABLE
    case GPIOE_BASE:
      __HAL_RCC_GPIOE_CLK_ENABLE();
      break;
#endif
#ifdef __HAL_RCC_GPIOF_CLK_ENABLE
    case GPIOF_BASE:
      __HAL_RCC_GPIOF_CLK_ENABLE();
      break;
#endif
    default:
      break;
    }
}


static void GPIO_SetupPin(GPIO_TypeDef *regs, uint32_t pos, uint32_t mode, int pullup)
{
  gpio_port_clock((uint32_t)regs);

#if defined(STM32F1)
  // Configure GPIO
  uint32_t shift = (pos % 8) * 4, msk = 0xf << shift, cfg;

  if (mode == GPIO_INPUT) {
    cfg = pullup ? 0x8 : 0x4;
  } else if (mode == GPIO_OUTPUT) {
    cfg = 0x1; // push-pull, 0b00 | max speed 2 MHz, 0b01
  } else if (mode == (GPIO_OUTPUT | GPIO_OPEN_DRAIN)) {
    cfg = 0x5; // Open-drain, 0b01 | max speed 2 MHz, 0b01
  } else if (mode == GPIO_ANALOG) {
    cfg = 0x0;
  } else {
    // Alternate function
    if (mode & GPIO_OPEN_DRAIN)
      // output open-drain mode, 10MHz
      cfg = 0xd;
      // output open-drain mode, 50MHz
      //cfg = 0xF;
    else if (pullup > 0)
      // input pins use GPIO_INPUT mode on the stm32f1
      cfg = 0x8;
    else
      // output push-pull mode, 10MHz
      cfg = 0x9;
      // output push-pull mode, 50MHz
      //cfg = 0xB;
  }
  if (pos & 0x8)
    regs->CRH = (regs->CRH & ~msk) | (cfg << shift);
  else
    regs->CRL = (regs->CRL & ~msk) | (cfg << shift);

  if (pullup > 0)
    regs->BSRR = 1 << pos;
  else if (pullup < 0)
    regs->BSRR = 1 << (pos + 16);
#else
  uint32_t const bit_pos = 0x1 << pos;
  if (0 > pullup) pullup = LL_GPIO_PULL_DOWN;
  else if (0 < pullup) pullup = LL_GPIO_PULL_UP;
  else pullup = LL_GPIO_PULL_NO;

  if (mode == GPIO_INPUT) {
    LL_GPIO_SetPinMode(regs, bit_pos, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(regs, bit_pos, pullup);
  } else if (mode == GPIO_OUTPUT) {
    LL_GPIO_SetPinMode(regs, bit_pos, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(regs, bit_pos, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(regs, bit_pos, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_ResetOutputPin(regs, bit_pos);
  } else {
    mode = (mode >> 4) & 0xff;
    LL_GPIO_SetPinMode(regs, bit_pos, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinPull(regs, bit_pos, pullup);
    LL_GPIO_SetPinSpeed(regs, bit_pos, LL_GPIO_SPEED_FREQ_HIGH);
    if (pos & 0x8) {
      LL_GPIO_SetAFPin_8_15(regs, bit_pos, mode);
    } else {
      LL_GPIO_SetAFPin_0_7(regs, bit_pos, mode);
    }
  }
#endif
}


struct gpio_pin
GPIO_Setup(uint32_t gpio, uint32_t mode, int pullup)
{
  uint32_t pin = IO_GET_PIN(gpio);
  uint32_t io = IO_GET_PORT(gpio);
  GPIO_TypeDef *port = (void*)((uintptr_t)GPIOA_BASE + (io * 0x0400UL));

  GPIO_SetupPin(port, pin, mode, pullup);

#if STM32F1
  if (gpio == GPIO('A', 13) || gpio == GPIO('A', 14))
      // Disable SWD to free PA13, PA14
      //AFIO->MAPR = AFIO_MAPR_SWJ_CFG_DISABLE;
      __HAL_AFIO_REMAP_SWJ_DISABLE();
  else if ((gpio == GPIO('A', 15)) || (gpio == GPIO('B', 3)) || (gpio == GPIO('B', 4)))
      // Disable JTAG-DP
      __HAL_AFIO_REMAP_SWJ_NOJTAG();
#endif

  return (struct gpio_pin){.reg = port, .bit = (1 << pin)};
}


static void
servo_timer_init(TIM_HandleTypeDef *handle, TIM_TypeDef * timer)
{
  /* Peripheral clock enable */
  enable_pclock((uint32_t)timer);

  handle->Instance = timer;
  handle->Init.Prescaler = (2 * get_pclock_frequency((uint32_t)timer) / 1000000) - 1;
  handle->Init.CounterMode = TIM_COUNTERMODE_UP;
  handle->Init.Period = 20000 - 1;
  handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  handle->Init.RepetitionCounter = 0;
  handle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(handle);
}


static void
TIMx_Channel_Init(TIM_HandleTypeDef *handle, uint32_t channel, uint16_t val)
{
  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = val;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCNIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(handle, &sConfigOC, channel);
  HAL_TIM_PWM_Start(handle, channel);
}


struct pwm_pin {
  TIM_HandleTypeDef *handle;
  uint32_t channel;
};


struct pwm_pin
Servo_attach(TIM_HandleTypeDef *handle, uint16_t pin, uint8_t ch, uint8_t af, uint16_t outval)
{
  ch = ch << 2;
  GPIO_Setup(pin, af, 0);
  // Init PWM output
  TIMx_Channel_Init(handle, ch, outval);
  return (struct pwm_pin){.handle = handle, .channel = ch};
}


static FAST_CODE_1 void Servo_writeMicroseconds(struct pwm_pin pin, uint32_t us)
{
  /* limit input to range */
  //us = (us < SERVO_OUT_US_MIN) ? SERVO_OUT_US_MIN : (SERVO_OUT_US_MAX < us) ? SERVO_OUT_US_MAX : us;
  // set channel...
  __HAL_TIM_SET_COMPARE(pin.handle, pin.channel, us);
}


static TIM_HandleTypeDef tim2_handle; // PA0...PA3 => TIM2
static TIM_HandleTypeDef tim3_handle; // PA6, PA7, PB0, PB1 => TIM3
struct pwm_pin pwm_pins[NUM_CHANNELS];


static void pwm_output_configure(void)
{
#if STM32F0
  servo_timer_init(&tim2_handle, TIM2);
  pwm_pins[0] = Servo_attach(&tim2_handle, GPIO('A', 0), 0, GPIO_FUNCTION(2), SERVO_OUT_US_MIN);
  pwm_pins[1] = Servo_attach(&tim2_handle, GPIO('A', 1), 1, GPIO_FUNCTION(2), SERVO_OUT_US_MID);
  pwm_pins[2] = Servo_attach(&tim2_handle, GPIO('A', 2), 2, GPIO_FUNCTION(2), SERVO_OUT_US_MID);
  pwm_pins[3] = Servo_attach(&tim2_handle, GPIO('A', 3), 3, GPIO_FUNCTION(2), SERVO_OUT_US_MID);

  servo_timer_init(&tim3_handle, TIM3);
  pwm_pins[4] = Servo_attach(&tim3_handle, GPIO('A', 6), 0, GPIO_FUNCTION(1), SERVO_OUT_US_MID);
  pwm_pins[5] = Servo_attach(&tim3_handle, GPIO('A', 7), 1, GPIO_FUNCTION(1), SERVO_OUT_US_MID);
  pwm_pins[6] = Servo_attach(&tim3_handle, GPIO('B', 0), 2, GPIO_FUNCTION(1), SERVO_OUT_US_MID);
  pwm_pins[7] = Servo_attach(&tim3_handle, GPIO('B', 1), 3, GPIO_FUNCTION(1), SERVO_OUT_US_MID);

#elif STM32F1
#error "PWM timer config is not valid yet!"
#endif
}


static FAST_CODE_1 void pwm_output_set(uint16_t * rc_data, uint8_t len)
{
  static uint32_t pwm_adjustment_done_ms;
  uint32_t const now_ms = HAL_GetTick();
  if ((now_ms - pwm_adjustment_done_ms) < SERVO_UPDATE_INTERVAL_MS)
    return;
  pwm_adjustment_done_ms = now_ms;
  while (len--) {
    Servo_writeMicroseconds(pwm_pins[len], rc_data[len]);
  }
}


static FAST_CODE_1 void main_loop(void)
{
  /* Infinite loop */
  parse_byte_func_t parser;
  get_rc_data_func_t get_rc;
#if PROTO_CRSF
  parser = crsf_parse_byte;
  get_rc = crsf_get_rc_data;
#elif PROTO_SBUS
  parser = sbus_parse_byte;
  get_rc = sbus_get_rc_data;
#elif PROTO_GHST
  parser = ghst_parse_byte;
  get_rc = ghst_get_rc_data;
#endif
  uint16_t channels[NUM_CHANNELS];
  uint8_t data;

  led_set(LED_READY);

  while (1) {
    if (uart_receive_timeout(&data, 1, 5) == UART_OK) {
      if (parser(data)) {
        get_rc(channels, ARRAY_SIZE(channels));
        pwm_output_set(channels, ARRAY_SIZE(channels));
      }
    }
  }
}


static void copy_functions_to_ram(void)
{
  /* Load functions into ITCM RAM */
  extern uint8_t ram_code_start;
  extern uint8_t ram_code_end;
  extern uint8_t ram_code;
  /* Copy only if address is different */
  if (&ram_code != &ram_code_start)
    memcpy(&ram_code_start, &ram_code, (size_t) (&ram_code_end - &ram_code_start));

  /* Make sure the vectors are set correctly */
#if STM32F0
  LL_SYSCFG_SetRemapMemory(LL_SYSCFG_REMAP_SRAM);
#else
  SCB->VTOR = BL_FLASH_START;
#endif
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
#if DEBUG_BUILD
  //__asm__("BKPT");
#endif
  //copy_functions_to_ram();

  /* MCU Configuration--------------------------------------------------------*/
#if STM32F1
  __HAL_RCC_AFIO_CLK_ENABLE();
#endif
  __HAL_RCC_PWR_CLK_ENABLE();
  //__HAL_AFIO_REMAP_SWJ_DISABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* Init DWT if present */
#ifdef DWT_BASE
    if (dwt_init()) {
        Error_Handler();
    }
#endif
  __enable_irq();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  led_init();

  uint8_t state = 0;
  uint8_t cnt = 10;
  while (cnt--) {
    HAL_Delay(150);
    led_set(state ? LED_READY : LED_ERROR);
    state ^= 1;
  }

  // UART init
  uart_init(RX_BAUDRATE, RECEIVER_UART_RX, RECEIVER_UART_TX);

  pwm_output_configure();
  main_loop();
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  uint32_t FLatency = FLASH_LATENCY_1;

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
#if STM32F0
  /* 8MHz HSI, Activate PLL with HSI as source, MUL6 -> 6*8MHz = 48MHz */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_OFF;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; // Source is HSI / 2
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
#elif STM32F1
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
#endif
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_CFGR_PPRE_DIV1;
#if STM32F1
  RCC_ClkInitStruct.ClockType |= RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  FLatency = FLASH_LATENCY_2;
#endif
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLatency) != HAL_OK) {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }

  HAL_ResumeTick();

  SystemCoreClockUpdate();
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  led_set(LED_ERROR);
  while (1) {}
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
