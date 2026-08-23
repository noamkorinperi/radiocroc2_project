/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "radioroc2_daq.h"   /* pulls in radioroc2_regs.h + radioroc2.h */
#include "tmp102.h"          /* temperature sensor on I2C2 */
#include "usb_stream.h"      /* event streaming over the ST-Link VCP */
#include "radioroc2_ctrl.h"  /* shadow registers + setters */
#include "usb_cmd.h"         /* host command interface */
#include "rr2_test_clocks.h" /* scope test 1: CLK_SM_I2C + CK_READ */
#include "rr2_test_i2c.h"    /* scope test 2: Slow Control I2C bus */
#include <stdio.h>           /* optional: printf over SWO/VCP */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- Experiment window ------------------------------------------
 * Which channels the readout digitises, and which ones the ASIC is
 * configured for at boot. Everything outside this window is disabled,
 * so 60 unconnected inputs stop feeding the NOR trigger.
 *
 * The window is deliberately WIDER than the one instrumented channel.
 * Only RR2_EXP_SIGNAL_CH carries a SiPM; the channels after it are
 * dark, and their peak-detector codes in the same event are a live
 * sample of the readout chain's own baseline - trigger, temperature
 * and all. That is the pedestal, recorded next to every measurement
 * instead of once in a separate run, which is what lets a thermal
 * sweep be pedestal-corrected afterwards from its own data.
 *
 * The trigger that latched the event came from the signal channel, so
 * it is uncorrelated with the noise in these three - no selection bias,
 * unlike a pedestal harvested by lowering the threshold into the noise.
 * ------------------------------------------------------------------ */
#define RR2_EXP_SIGNAL_CH   0u   /* the channel with the SiPM on it    */
#define RR2_EXP_FIRST_CH    0u   /* first channel read out             */
#define RR2_EXP_NUM_CH      4u   /* signal + 3 dark baseline references */

/* Widening the array later means moving two of these three, and the one
   left behind is silent: the signal channel simply stops being read and
   the stream fills with baseline. Make the compiler notice instead.  */
_Static_assert((RR2_EXP_SIGNAL_CH >= RR2_EXP_FIRST_CH) &&
               (RR2_EXP_SIGNAL_CH <  RR2_EXP_FIRST_CH + RR2_EXP_NUM_CH),
               "signal channel sits outside the readout window");
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---- Bring-up / DAQ state -------------------------------------- */

/* Set by the NOR_T1OC EXTI: the ASIC started an acquisition.
   Cleared by the main loop once the event has been serviced.       */
volatile uint8_t  g_rr2_trigger_flag  = 0;
volatile uint32_t g_rr2_trigger_count = 0;

/* Set by the ERRORN_SC EXTI: the ASIC reported a Slow Control
   error (active low). Sticky - cleared only by software.           */
volatile uint8_t  g_rr2_sc_error = 0;

/* Result of the power-on configuration sequence. Inspect in the
   debugger: RR2_OK (0) means the ASIC accepted the full config.    */
volatile RR2_Status g_rr2_cfg_status = RR2_OK;
volatile uint8_t    g_rr2_online     = 0;   /* 1 = ASIC ACKed */

/* Most recent digitised event (~136 bytes, lives in .bss).         */
RR2_Event g_rr2_event;
volatile RR2_Status g_rr2_read_status = RR2_OK;
volatile uint32_t   g_rr2_events_ok   = 0;
volatile uint32_t   g_rr2_events_bad  = 0;

/* 0 means the DWT cycle counter is not running and readout delays
   fall back to a coarse loop. Safe, but slower. Check once at boot. */
volatile uint8_t    g_rr2_timing_ok   = 0;

/* ---- Temperature monitoring ------------------------------------ */
/* Last valid reading in milli-Celsius (25.0 C -> 25000).            */
volatile int32_t  g_temp_milli_c = 0;
volatile uint8_t  g_temp_online  = 0;   /* 1 = TMP102 ACKed */
static   uint32_t s_temp_next_ms = 0;   /* next poll deadline */

/* ---- Host link ------------------------------------------------- */
static uint32_t s_status_next_ms = 0;   /* status frame deadline */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void RR2_HW_ReleaseResets(void);
static void RR2_StartClocks(void);
static uint8_t RR2_Bringup(void);
static RR2_Status RR2_ApplyExperimentConfig(void);
static void RR2_ServiceEvent(void);
static void TMP_Poll(void);
static void USB_SendStatusPeriodic(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Flash prefetch buffer and ART accelerator.
   *
   * These two lines are the only thing that switches them on. HAL_Init()
   * does it too, but only behind PREFETCH_ENABLE and
   * ART_ACCELERATOR_ENABLE in stm32f7xx_hal_conf.h, and both are left at
   * the value CubeMX writes - which is 0U, i.e. off.
   *
   * Leaving them off there is deliberate. That file has a USER CODE
   * block around its header and nowhere else, so a define edited at line
   * 151 is rewritten by the next Generate Code with nothing to show for
   * it. Enabling from here instead survives regeneration, the same way
   * the link baud does in USART3_Init 2 - and it means hal_conf.h can be
   * regenerated freely without taking the setting with it.
   *
   * The prefetch buffer is what earns its keep: at 216 MHz the flash
   * needs seven wait states, and the buffer is what hides them.
   *
   * ART is honestly along for the ride. It accelerates instruction fetch
   * over ITCM only, and the linker maps FLASH at 0x08000000, which is
   * the AXIM interface - so in this configuration it does nothing. It is
   * enabled because it costs one instruction and would be correct if the
   * code ever moves.
   *
   * Safe next to the DAQ: every timing constraint there is a minimum
   * gated by DWT->CYCCNT, which counts core cycles regardless of how
   * fast instructions arrive. D-cache stays off deliberately - the DMA
   * writes straight to RAM. */
  __HAL_FLASH_ART_ENABLE();
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /* ---------------------------------------------------------------
     * RADIOROC2 power-on sequence.
     * Order matters:
     *   1) clk_sm_i2c must already be running before any I2C traffic,
     *      because the ASIC's I2C slave core is clocked by it.
     *   2) Only then release the resets.
     *   3) Only then talk to the chip.
     * ------------------------------------------------------------- */
    RR2_StartClocks();                    /* CLK_SM_I2C = 2 MHz           */
    RR2_HW_ReleaseResets();               /* de-assert active-low resets  */
    RR2_Init(&hi2c1);                     /* bind the Slow Control driver */
    /* Only OUT_AMUXLG is instrumented, on PA5 / ADC2_IN5. ADC1 (PA4) is
       still brought up by CubeMX above, but nothing reads it. */
    RR2_DAQ_Init(&hadc2);                 /* bind the ADC, enable DWT     */
    g_rr2_timing_ok = RR2_DAQ_IsTimingOk();

    /* Temperature sensor lives on its own bus, independent of the ASIC. */
    TMP102_Init(&hi2c2, TMP102_ADDR_GND);          /* 0x48 */
    g_temp_online = (TMP102_IsReady(3u) == HAL_OK) ? 1u : 0u;

    /* The shadow must hold sane defaults even if the ASIC never
     * answered, otherwise a later 'push' from the host would write
     * zeros everywhere. */
    RR2_Ctrl_ResetShadow();
    /* Scan all 16 possible chip IDs to find which one ACKs. */
    volatile uint8_t found_id = 0xFF;
    for (uint8_t id = 0; id < 16; id++) {
        uint8_t addr = (uint8_t)(((id << 3) | 0) << 1);   /* R0 of that chip id */
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr, 3, 100) == HAL_OK) {
            found_id = id;
            break;
        }
    }
    /* breakpoint here, read found_id */
    g_rr2_online = RR2_Bringup();

    if (g_rr2_online) {
        g_rr2_cfg_status = RR2_Ctrl_PushAll();

        if (g_rr2_cfg_status == RR2_OK) {
            g_rr2_cfg_status = RR2_Ctrl_SetThresholds(300u, 500u, 200u);
        }
        if (g_rr2_cfg_status == RR2_OK) {
            g_rr2_cfg_status = RR2_ApplyExperimentConfig();
        }
        RR2_DAQ_EndOfReadout();
    }

    /* Start the host link. Binary framing by default; switch to
     * USBSTREAM_FMT_TEXT if you want to watch it in a serial terminal. */
    USBStream_Init();
    /* USBStream_SetFormat(USBSTREAM_FMT_TEXT); */

    /* Host command interface. The shadow must mirror whatever we just
     * wrote to the ASIC, otherwise the first host edit would push stale
     * neighbouring bits. */
    USBCmd_Init();


    /* Breakpoint here during bring-up and inspect:
     *   g_rr2_online     -> 1 if the ASIC ACKed its Chip ID
     *   g_rr2_cfg_status -> RR2_OK if the whole config sequence passed */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
      {
        /* --- Scope tests, driven from the debugger -------------------- */
        /* Both are no-ops until their mode variable is set from Live
           Expressions, and both block until it is cleared again. Keep
           them ahead of the DAQ so nothing else drives the lines while
           a probe is on them. See rr2_test_clocks.h / rr2_test_i2c.h. */
        RR2_TestClocks_Task();
        RR2_TestI2C_Task();

        /* --- Slow Control error reported by the ASIC ------------------ */
        if (g_rr2_sc_error) {
            uint8_t sc_status = 0;
            g_rr2_sc_error = 0;
            /* R7 holds the error / parity flags of the I2C slave core.  */
            (void)RR2_ReadStatus(&sc_status);
            /* TODO: log or re-send the failed Slow Control transaction. */
        }

        /* --- Acquisition trigger from NOR_T1OC ----------------------- */
        if (g_rr2_trigger_flag) {
            g_rr2_trigger_flag = 0;
            RR2_ServiceEvent();
        }
        /* --- Temperature, polled about once per second ---------------- */
                TMP_Poll();

                /* --- Push queued bytes out and emit a periodic status frame --- */
                USBStream_Task();
                USB_SendStatusPeriodic();

                /* --- Commands from the host (parsing + ASIC writes) ----------- */
                USBCmd_Task();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* ==================================================================
 * RADIOROC2 low-level board helpers
 * ================================================================== */

/**
 * @brief Start the clock the ASIC's I2C slave core needs.
 *
 * CLK_SM_I2C (PE9 / TIM1_CH1) MUST run at exactly 20x the SCL
 * frequency: 100 kHz SCL -> 2 MHz here.
 *
 * CK_READ is deliberately NOT started here. It is burst generated in
 * software during readout - see radioroc2_daq.c.
 */
static void RR2_StartClocks(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* CLK_SM_I2C, 2 MHz */
}

/**
 * @brief Drive every reset line to its inactive state.
 *
 * All RSTN_* lines and RESET_N are active low, so "inactive" is HIGH.
 * HOLDEXT is active high and stays LOW (internal hold is used unless
 * selHoldExt is set in Slow Control address 65, subadd 12, bit 4).
 */
static void RR2_HW_ReleaseResets(void)
{
    /* Assert all resets briefly to force a known state ... */
    HAL_GPIO_WritePin(RESET_N_GPIO_Port,   RESET_N_Pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RSTN_I2C_GPIO_Port,  RSTN_I2C_Pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RSTN_SC_GPIO_Port,   RSTN_SC_Pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RSTN_READ_GPIO_Port, RSTN_READ_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HOLDEXT_GPIO_Port,   HOLDEXT_Pin,   GPIO_PIN_RESET);
    HAL_Delay(1);   /* datasheet asks for >20 ns; 1 ms is generous */

    /* ... then release them. */
    HAL_GPIO_WritePin(RESET_N_GPIO_Port,   RESET_N_Pin,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(RSTN_I2C_GPIO_Port,  RSTN_I2C_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(RSTN_SC_GPIO_Port,   RSTN_SC_Pin,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(RSTN_READ_GPIO_Port, RSTN_READ_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

/**
 * @brief Stage-1 bring-up check: does the ASIC answer on its Chip ID?
 * @retval 1 if the ASIC ACKed, 0 otherwise.
 */
static uint8_t RR2_Bringup(void)
{
    return (RR2_IsReady(3u) == HAL_OK) ? 1u : 0u;
}

/**
 * @brief Put the ASIC into the configuration the experiment runs in.
 *
 * Applied at boot rather than left to the operator, because the Slow
 * Control is volatile: a brown-out during a temperature soak otherwise
 * brings the chip back with 64 live channels on 20 ns shaping, and it
 * keeps streaming, and it looks exactly like data. Only uptime_ms going
 * backwards in the status frame gives it away.
 *
 * inDac is deliberately NOT set here. It is the one parameter the sweep
 * changes per temperature point, so it is left wherever the host put it
 * last - 0x80 out of the shadow reset.
 */
static RR2_Status RR2_ApplyExperimentConfig(void)
{
    RR2_Status st;

    /* Silence every channel, then bring back only the window. The 60
       unconnected inputs are floating, and their discriminators feed
       the same NOR trigger as the channel that matters.              */
    st = RR2_Ctrl_SetChannelEnabled(RR2_CH_ALL, 0u);
    if (st != RR2_OK) return st;

    for (uint8_t c = RR2_EXP_FIRST_CH;
         c < (uint8_t)(RR2_EXP_FIRST_CH + RR2_EXP_NUM_CH); ++c) {
        st = RR2_Ctrl_SetChannelEnabled(c, 1u);
        if (st != RR2_OK) return st;

        /* Everything in the window except the signal channel is there
           to report a baseline, not to trigger. Their inputs are
           unconnected and floating, and a spurious discriminator hit
           raises the same NOR trigger as a real gamma - which presents
           as a count rate that ignores the source, the one symptom
           first light is looking for. Shapers and peak detectors stay
           on; only the discriminators go. */
        if (c != RR2_EXP_SIGNAL_CH) {
            st = RR2_Ctrl_SetDiscriminators(c, 0u);
            if (st != RR2_OK) return st;
        }
    }

    /* Slow shaping, ~1.7 us peaking, hold delay stretched to match.
       Safe to run after the channel gating: the shaping setters touch
       only their own bits and leave the enables alone.               */
    return RR2_Ctrl_PresetCsI();
}

/**
 * @brief Digitise one event after a trigger.
 *
 * The ASIC's delay cell has to finish asserting "hold" before the read
 * register may be clocked, so wait that out first - for however long
 * the currently configured delay needs, which RR2_DAQ_WaitHold() works
 * out from the shadow.
 *
 * Only the experiment window is digitised. All 64 channels would cost
 * roughly 64 x 6 us = 400 us of dead time and 149 bytes on the wire per
 * event, against 25 us and 29 bytes for four - and the 60 channels left
 * out are disabled anyway, so their codes would carry nothing.
 */
static void RR2_ServiceEvent(void)
{
    RR2_DAQ_WaitHold();

    g_rr2_read_status = RR2_DAQ_ReadWindow(&g_rr2_event,
                                           RR2_EXP_FIRST_CH,
                                           RR2_EXP_NUM_CH);

    if (g_rr2_read_status == RR2_OK) {
        g_rr2_events_ok++;
        /* Queue for the host. Returns 0 if the ring was full, in which
           case the frame is dropped rather than stalling the DAQ. */
        (void)USBStream_SendEvent(&g_rr2_event, g_temp_milli_c, HAL_GetTick());
    } else {
        g_rr2_events_bad++;
    }
}

/**
 * @brief Sample the TMP102 roughly once per second.
 *
 * Kept in the main loop on purpose: a 3-byte I2C transaction at
 * 100 kHz costs about 400 us, which is far too long for an ISR.
 * The sensor free-runs at 4 Hz, so polling faster gains nothing.
 */
static void TMP_Poll(void)
{
    int32_t mc;

    if (!g_temp_online)                 return;
    if (HAL_GetTick() < s_temp_next_ms) return;

    s_temp_next_ms = HAL_GetTick() + 1000u;

    if (TMP102_ReadMilliC(&mc) == TMP102_OK) {
        g_temp_milli_c = mc;
    }
}

/**
 * @brief Emit a status frame once per second.
 *
 * Gives the host a heartbeat plus the counters needed to spot problems:
 * dropped frames mean the link cannot keep up, events_bad means the
 * readout itself is failing.
 */
static void USB_SendStatusPeriodic(void)
{
    USBStream_Status st;

    if (HAL_GetTick() < s_status_next_ms) return;
    s_status_next_ms = HAL_GetTick() + 1000u;

    st.uptime_ms      = HAL_GetTick();
    st.trigger_count  = g_rr2_trigger_count;
    st.events_ok      = g_rr2_events_ok;
    st.events_bad     = g_rr2_events_bad;
    st.frames_dropped = USBStream_GetDropped();
    st.temp_milli_c   = g_temp_milli_c;
    st.rr2_online     = g_rr2_online;
    st.temp_online    = g_temp_online;
    st.timing_ok      = g_rr2_timing_ok;
    st.cfg_status     = (uint8_t)g_rr2_cfg_status;
    st.read_status    = (uint8_t)g_rr2_read_status;

    (void)USBStream_SendStatus(&st);
}

/* ==================================================================
 * Interrupt callbacks
 * ================================================================== */

/**
 * @brief Shared EXTI callback for both ASIC status lines.
 *
 * NOR_T1OC  (PA0, EXTI0): falling edge = the ASIC started an
 *                         acquisition on the low time trigger.
 * ERRORN_SC (PD3, EXTI3): active-low Slow Control error flag.
 *
 * Keep this short - only set flags. The readout itself blocks for
 * hundreds of microseconds and must not run inside an ISR.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == NOR_T1OC_Pin) {
        g_rr2_trigger_flag = 1u;
        g_rr2_trigger_count++;
    }
    else if (GPIO_Pin == ERRORN_SC_Pin) {
        g_rr2_sc_error = 1u;
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
