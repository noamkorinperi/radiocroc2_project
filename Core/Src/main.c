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
#include "rr2_test_i2c.h"    /* Slow Control I2C health check */
#include <stdio.h>           /* optional: printf over SWO/VCP */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---------------------------------------------------------------
 * Which ASIC inputs actually carry a detector.
 *
 * The sensors are modular rather than soldered, so this is the one
 * place to edit when the arrangement changes - and it can also be
 * changed at runtime without reflashing, with "sel 3 9 20 41 55"
 * over the command link.
 *
 * The list does NOT have to be contiguous. Unselected channels are
 * clocked past without being digitised, and are disabled in the ASIC
 * so they cannot fire the trigger on their own noise.
 *
 * Leave it empty to start with all 64 enabled.
 * ------------------------------------------------------------- */
static const uint8_t RR2_DETECTOR_CHANNELS[] = { 0, 1, 2, 3, 4 };

/* ---------------------------------------------------------------
 * Manual scope walk-through, stepped with F8 from the debugger.
 *
 * Three stages, one per signal. Each puts exactly ONE signal on the
 * wire and parks the other two, so nothing on the screen can be
 * mistaken for the trace you are measuring.
 *
 * Set to 0 to get the plain firmware back: the stages and the three
 * breakpoint anchors disappear completely and main() goes straight
 * from bring-up into the DAQ loop.
 *
 * The block at the end of USER CODE 2 says where the breakpoints go.
 * ------------------------------------------------------------- */
#define RR2_MANUAL_DEBUG        1

/* Time limit on each of the two CPU-generated stages, in ms, and 0
 * for "no limit" on either.
 *
 * CK_READ is bounded so the walk-through always reaches the SCL stage
 * on its own. SCL is deliberately unbounded - it is the one under
 * investigation right now, so it stays on the wire for as long as you
 * leave the core running. Leave it with Suspend then F8, or by setting
 * g_dbg_stop to 1 from Live Expressions without halting at all.
 *
 * Both are editable live as g_dbg_pa1_ms / g_dbg_scl_ms. */
#define RR2_DBG_PA1_MS          10000u
#define RR2_DBG_SCL_MS          0u

/* Wall-clock time unaccounted for by SysTick that counts as "the
 * debugger stopped us". Ordinary loop jitter is microseconds, and a
 * Suspend followed by a Resume by hand is never under a few hundred
 * milliseconds, so there is a wide margin between the two. */
#define RR2_DBG_HALT_MS         100u

/* CK_READ pulses timed as one block. A single pulse is ~200 ns, so
 * reading the cycle counter around each one would cost more than the
 * pulse being measured. */
#define RR2_DBG_CKREAD_BLOCK    1000u

/* Slow Control frame timeout, and the idle gap left between frames
 * so the scope has something quiet to trigger against. */
#define RR2_DBG_I2C_TIMEOUT_MS  50u
#define RR2_DBG_I2C_GAP_MS      1u
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

/* Channels carrying a detector. Mirrors the DAQ mask, exposed here so
   it shows up next to the rest of the state in the debugger. */
volatile uint64_t g_rr2_channel_mask = 0;

/* Most recent digitised event (~260 bytes, lives in .bss).         */
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

/* ---- Manual scope walk-through --------------------------------- */
#if RR2_MANUAL_DEBUG
/* Stages completed so far. Also the quickest way to tell which of the
   three breakpoints you are currently sitting on.                   */
volatile uint8_t  g_dbg_stage  = 0u;

/* Controls - editable from Live Expressions while a stage runs.     */
volatile uint32_t g_dbg_pa1_ms = RR2_DBG_PA1_MS;   /* 0 = no limit   */
volatile uint32_t g_dbg_scl_ms = RR2_DBG_SCL_MS;   /* 0 = no limit   */
volatile uint8_t  g_dbg_stop   = 0u;               /* 1 = leave now  */
/* Why the last stage ended - 0 while one is still running.
   1 = the stage time limit elapsed, 2 = g_dbg_stop, 3 = Suspend. */
volatile uint8_t  g_dbg_exit_reason = 0u;

/* Stage 1 - PE9 / CLK_SM_I2C, read back out of the timer registers.
   Named for the pin it actually comes out of: this signal used to be
   on PA8 and the whole point of reading it back from the registers is
   that it stays honest when the board moves.                        */
volatile uint32_t g_dbg_pe9_hz      = 0u;
volatile uint8_t  g_dbg_pe9_running = 0u;  /* counter AND output on  */

/* Stage 2 - PA1 / CK_READ, timed with the DWT cycle counter.        */
volatile uint32_t g_dbg_pa1_period_ns = 0u;
volatile uint32_t g_dbg_pa1_freq_khz  = 0u;
volatile uint32_t g_dbg_pa1_pulses    = 0u;

/* Stage 3 - PB8 / SCL, timed per Slow Control frame.                */
volatile uint8_t  g_dbg_scl_status   = 0u; /* 0 OK, 1 ERROR, 3 NACK  */
volatile uint32_t g_dbg_scl_khz      = 0u;
volatile uint32_t g_dbg_scl_frame_us = 0u;
volatile uint32_t g_dbg_scl_frames   = 0u;
volatile uint32_t g_dbg_scl_ok       = 0u;
volatile uint32_t g_dbg_scl_fail     = 0u;
/* clk_sm_i2c : SCL, x10. The ASIC needs 20, so 200 is the target.   */
volatile uint32_t g_dbg_ratio_x10    = 0u;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void RR2_HW_ReleaseResets(void);
static void RR2_StartClocks(void);
static void RR2_ServiceEvent(void);
static void TMP_Poll(void);
static void USB_SendStatusPeriodic(void);
#if RR2_MANUAL_DEBUG
static void RR2_Dbg_Stage1_ClkSmI2c(void);
static void RR2_Dbg_Stage2_CkRead(void);
static void RR2_Dbg_Stage3_Scl(void);
#endif
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
  /* Flash ART accelerator and prefetch buffer.
   *
   * HAL_Init() has already switched both on from PREFETCH_ENABLE and
   * ART_ACCELERATOR_ENABLE in stm32f7xx_hal_conf.h, so these two lines
   * normally set bits that are already set. They are here because the
   * .ioc has no knob for either - CORTEX_M7 tracks CPU_ICache and
   * nothing else - so the next Generate Code rewrites both defines back
   * to 0U and the setting would vanish with nothing to show for it.
   * Same trap as the link baud; see USART3_Init 2. This block survives
   * regeneration, so the configuration self-heals.
   *
   * Safe next to the caches: ART only accelerates instruction fetch
   * from flash, and every timing constraint in the DAQ is gated by
   * DWT->CYCCNT, which counts core cycles regardless of fetch speed.
   * D-cache stays off deliberately - the DMA writes straight to RAM. */
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
    RR2_DAQ_Init(&hadc1, &hadc2);         /* bind ADCs, enable DWT timer  */
    g_rr2_timing_ok = RR2_DAQ_IsTimingOk();

    /* Temperature sensor lives on its own bus, independent of the ASIC. */
    TMP102_Init(&hi2c2, TMP102_ADDR_GND);          /* 0x48 */
    g_temp_online = (TMP102_IsReady(3u) == HAL_OK) ? 1u : 0u;

    /* The shadow must hold sane defaults even if the ASIC never
     * answered, otherwise a later 'push' from the host would write
     * zeros everywhere. */
    RR2_Ctrl_ResetShadow();

    /* Start the host link before the health check rather than after, so
     * the check has somewhere to print its report. Binary framing by
     * default; switch to USBSTREAM_FMT_TEXT to watch it in a terminal.
     * Text lines are sent verbatim in either format, so the report is
     * readable without switching. */
    USBStream_Init();
    /* USBStream_SetFormat(USBSTREAM_FMT_TEXT); */

    /* ---------------------------------------------------------------
     * Slow Control I2C health check.
     *
     * This is what decides whether there is a chip to configure, and it
     * is the only thing that can answer the CHIP_ID question: the id is
     * strapped in copper and IS the top half of the I2C address, so no
     * register holds it and the sweep is the only way to learn it. A
     * board strapped to anything other than RR2_CHIP_ID would otherwise
     * look completely dead on a perfectly healthy bus.
     *
     * It probes addresses only - never a data byte - so nothing here can
     * disturb the ASIC configuration. On a pass it points the driver at
     * the id it found. See rr2_test_i2c.h for the variables to read when
     * it fails.
     * ------------------------------------------------------------- */
    g_rr2_online = (RR2_TestI2C_Run() == RR2_TEST_PASS) ? 1u : 0u;

    /* Tell the readout which inputs to digitise. Done before PushAll so
     * the enable bits it writes already reflect the selection, instead
     * of enabling all 64 and switching most off again a moment later. */
    if (sizeof(RR2_DETECTOR_CHANNELS) > 0u) {
        (void)RR2_DAQ_SelectChannels(RR2_DETECTOR_CHANNELS,
                                     (uint8_t)sizeof(RR2_DETECTOR_CHANNELS));
    }
    g_rr2_channel_mask = RR2_DAQ_GetChannelMask();

    if (g_rr2_online) {
        /* Only the selected channels stay powered. An enabled channel
         * feeds the NOR trigger whether or not a detector hangs off it,
         * so leaving the empty ones on would trigger the DAQ on their
         * own noise. */
        g_rr2_cfg_status = RR2_Ctrl_ApplyChannelMask(g_rr2_channel_mask);

        if (g_rr2_cfg_status == RR2_OK) {
            g_rr2_cfg_status = RR2_Ctrl_PushAll();
        }
        if (g_rr2_cfg_status == RR2_OK) {
            g_rr2_cfg_status = RR2_Ctrl_SetThresholds(300u, 500u, 200u);
        }
        RR2_DAQ_EndOfReadout();
    }

    /* Host command interface. The shadow must mirror whatever we just
     * wrote to the ASIC, otherwise the first host edit would push stale
     * neighbouring bits. */
    USBCmd_Init();


    /* Inspect here during bring-up, in this order:
     *   g_test_verdict   -> RR2_TEST_PASS if the I2C link is healthy
     *   g_test_chipid    -> the CHIP_ID that answered
     *   g_test_reason    -> why it did not, when it did not
     *   g_rr2_cfg_status -> RR2_OK if the whole config sequence passed */

#if RR2_MANUAL_DEBUG
    /* ===============================================================
     * MANUAL SCOPE WALK-THROUGH - three stops, three signals.
     *
     * Put a breakpoint on each of the three lines marked BREAKPOINT
     * below (double-click the left margin), then just keep pressing
     * F8 (Resume):
     *
     *   F8 #1  ->  stops at BREAKPOINT 1    PE9  CLK_SM_I2C is live
     *   F8 #2  ->  stops at BREAKPOINT 2    PA1  CK_READ    has run
     *   F8 #3  ->  stops at BREAKPOINT 3    PB8  SCL        has run
     *   F8 #4  ->  normal firmware, straight into the DAQ loop
     *
     * WHY THE THREE STAGES DO NOT BEHAVE ALIKE
     * PE9 comes out of TIM1 in hardware, so it keeps toggling while
     * the core sits at a breakpoint. Park the probe on it and measure
     * it at any of the three stops, in your own time.
     * PA1 and PB8 are produced BY the CPU - bit-banged, and the I2C
     * peripheral being fed transactions - so they only exist while
     * the core is RUNNING, and the two are bounded differently:
     *
     *   PA1  runs 10 s and ends by itself, so F8 #3 always gets you
     *        through to the stage that matters.
     *   PB8  runs for as long as you leave the core running. Take all
     *        the time you need on SCL, then press Suspend and F8 - the
     *        stage notices the halt, because TIM2 keeps counting
     *        through it while SysTick does not. Setting g_dbg_stop to
     *        1 from Live Expressions does the same without halting.
     *
     * Swap either behaviour live with g_dbg_pa1_ms / g_dbg_scl_ms; 0
     * means unbounded. g_dbg_exit_reason says how a stage ended.
     * =============================================================== */
    RR2_Dbg_Stage1_ClkSmI2c();  /* PE9 up, the other two lines quiet */
    g_dbg_stage = 1;   /* <<<<< BREAKPOINT 1 - PE9 / CLK_SM_I2C <<<<< */

    /*RR2_Dbg_Stage2_CkRead();    /* PA1 pulses, g_dbg_pa1_ms long     */
    /*g_dbg_stage = 2;   /* <<<<< BREAKPOINT 2 - PA1 / CK_READ <<<<<<<< */

    RR2_Dbg_Stage3_Scl();       /* PB8 frames, until you stop it     */
    g_dbg_stage = 3;   /* <<<<< BREAKPOINT 3 - PB8 / SCL <<<<<<<<<<<< */
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
      {
        /* --- I2C health check, re-run on demand ---------------------- */
        /* A no-op until g_test_run or g_test_repeat is set from Live
           Expressions. Kept ahead of the DAQ so the sweep never has to
           share the bus with a readout. See rr2_test_i2c.h. */
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
 * CLK_SM_I2C (PA8 / TIM1_CH1) MUST run at exactly 20x the SCL
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
 * @brief Digitise one event after a trigger.
 *
 * The ASIC's delay cell has to finish asserting "hold" before the read
 * register may be clocked, so wait that out first.
 *
 * How long this takes depends on the channel selection, because the
 * ADC dominates: all 64 channels costs roughly 64 x 6 us = 400 us,
 * while five selected channels costs about 50 us - the clocking still
 * walks the read register, but only five pairs of conversions happen.
 * See RR2_DETECTOR_CHANNELS above and the "sel" command.
 */
static void RR2_ServiceEvent(void)
{
    RR2_DAQ_WaitHold();

    g_rr2_read_status = RR2_DAQ_ReadEvent(&g_rr2_event);

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

    st.channel_mask   = g_rr2_channel_mask;
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

/* ==================================================================
 * Manual scope walk-through
 *
 * Three stages, one per signal, run back to back before the DAQ loop
 * starts. Each stage puts exactly one signal on the wire and leaves
 * the others parked, so nothing else can be mistaken for the trace
 * being measured. The breakpoints that separate them live in main(),
 * next to the comment that explains the F8 sequence.
 * ================================================================== */
#if RR2_MANUAL_DEBUG

/**
 * @brief Cycle count to nanoseconds, at whatever the PLL actually set.
 *
 * SystemCoreClock rather than a hard-coded 216 MHz, so the numbers stay
 * honest if SystemClock_Config() is ever retuned. Saturates instead of
 * wrapping, which matters when a frame times out rather than answering.
 */
static uint32_t Dbg_CycToNs(uint32_t cycles)
{
    const uint32_t mhz = SystemCoreClock / 1000000u;

    if (mhz == 0u)                      return 0u;
    if (cycles > (0xFFFFFFFFu / 1000u)) return 0xFFFFFFFFu;

    return (cycles * 1000u) / mhz;
}

/**
 * @brief TIM1's own clock, which is not the APB2 bus clock.
 *
 * Whenever the APB2 prescaler is anything other than 1 the timer clock
 * is doubled. Here APB2 runs at 108 MHz and TIM1 therefore at 216 MHz,
 * which is what makes ARR = 107 come out as 2.000 MHz on PE9.
 */
static uint32_t Dbg_Tim1ClkHz(void)
{
    uint32_t hz = HAL_RCC_GetPCLK2Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) {
        hz *= 2u;
    }
    return hz;
}

/**
 * @brief TIM2's own clock. Same doubling rule as TIM1, on APB1.
 */
static uint32_t Dbg_Tim2ClkHz(void)
{
    uint32_t hz = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        hz *= 2u;
    }
    return hz;
}

/**
 * @brief A clock that keeps ticking while the debugger holds the core.
 *
 * This is what lets a stage notice that you pressed Suspend. SysTick
 * sits inside the core and stops dead in debug state, so HAL_GetTick()
 * cannot tell a halt from a fast loop. TIM2 is a peripheral, it is
 * 32-bit, nothing else in this project uses it, and with its DBGMCU
 * freeze bit clear it carries on counting through the halt.
 *
 * 1 us per tick, so the counter wraps once every ~71 minutes - long
 * enough that the difference of two readings is always meaningful.
 */
static TIM_HandleTypeDef s_dbg_wall;

static void Dbg_WallClock_Init(void)
{
    const uint32_t hz = Dbg_Tim2ClkHz();

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_DBGMCU_UNFREEZE_TIM2();   /* keep counting while halted */

    s_dbg_wall.Instance               = TIM2;
    s_dbg_wall.Init.Prescaler         = (hz / 1000000u) - 1u;   /* 1 MHz */
    s_dbg_wall.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_dbg_wall.Init.Period            = 0xFFFFFFFFu;
    s_dbg_wall.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_dbg_wall.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&s_dbg_wall) == HAL_OK) {
        HAL_TIM_Base_Start(&s_dbg_wall);   /* counter only, no interrupt */
    }
}

static uint32_t Dbg_WallClockUs(void)
{
    return __HAL_TIM_GET_COUNTER(&s_dbg_wall);
}

/* Per-stage bookkeeping for the two checks below. */
static uint32_t s_dbg_limit_ms     = 0u;
static uint32_t s_dbg_t_start_ms   = 0u;
static uint32_t s_dbg_last_tick_ms = 0u;
static uint32_t s_dbg_last_us      = 0u;

/**
 * @brief Arm the stage: clear the controls and start both clocks.
 */
static void Dbg_StageBegin(uint32_t limit_ms)
{
    g_dbg_stop        = 0u;
    g_dbg_exit_reason = 0u;

    s_dbg_limit_ms     = limit_ms;
    s_dbg_t_start_ms   = HAL_GetTick();
    s_dbg_last_tick_ms = s_dbg_t_start_ms;
    s_dbg_last_us      = Dbg_WallClockUs();
}

/**
 * @brief Time to leave the stage?
 *
 * Three ways out, and g_dbg_exit_reason says which one it was:
 *
 *   3  you pressed Suspend. Wall-clock time went by that SysTick never
 *      saw, which only happens in debug state - so the next F8 walks
 *      out of the stage and straight into the breakpoint after it.
 *   2  you set g_dbg_stop from Live Expressions, without halting.
 *   1  the stage time limit elapsed - 10 s by default, so a stage
 *      always ends on its own. Zero the limit to run unbounded.
 */
static uint8_t Dbg_StageExpired(void)
{
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t now_us = Dbg_WallClockUs();

    /* How much time the CPU lived through, against how much actually
       passed. They track each other to within loop jitter unless the
       core was standing still. */
    const uint32_t ran_ms  = now_ms - s_dbg_last_tick_ms;
    const uint32_t wall_ms = (now_us - s_dbg_last_us) / 1000u;

    s_dbg_last_tick_ms = now_ms;
    s_dbg_last_us      = now_us;

    if ((wall_ms > ran_ms) && ((wall_ms - ran_ms) >= RR2_DBG_HALT_MS)) {
        g_dbg_exit_reason = 3u;
        return 1u;
    }

    if (g_dbg_stop) {
        g_dbg_exit_reason = 2u;
        return 1u;
    }

    if (s_dbg_limit_ms == 0u) {
        return 0u;
    }

    if ((now_ms - s_dbg_t_start_ms) >= s_dbg_limit_ms) {
        g_dbg_exit_reason = 1u;
        return 1u;
    }
    return 0u;
}

/**
 * @brief Stage 1 - PE9 / CLK_SM_I2C, the ASIC's Slow Control clock.
 *
 * Nothing is bit-banged here. TIM1_CH1 drives PE9 in hardware, which is
 * exactly why this stage comes first: it is the one signal that
 * survives the breakpoint after it, so it can be studied with the core
 * halted, and it is the reference the other two are compared against.
 *
 * The DBGMCU freeze bits are clear out of reset, but say so explicitly.
 * A stale freeze left behind by another debug session would stop TIM1
 * the instant the breakpoint hits, and PE9 would look dead for a reason
 * that has nothing to do with the board.
 *
 * g_dbg_pe9_running is the sanity check worth reading first. TIM1 is an
 * advanced timer, so a running counter is not enough - the output also
 * needs its channel enabled and the master output enable set, and a
 * missing MOE is the classic reason an otherwise correct PWM setup
 * produces a flat line.
 *
 * Note what this stage can and cannot prove: it reads the timer back,
 * so a green result means the STM32 is emitting the clock. Whether it
 * arrives at the ASIC pin is a question only the probe can answer.
 */
static void RR2_Dbg_Stage1_ClkSmI2c(void)
{
    __HAL_DBGMCU_UNFREEZE_TIM1();   /* keep PE9 alive while halted */

    /* The reference that survives a halt, and therefore the thing
       that lets stages 2 and 3 notice you pressed Suspend. */
    Dbg_WallClock_Init();

    /* Park the other two signals so the scope sees this one alone.
       SCL idles high on its own once the bus is quiet. */
    HAL_GPIO_WritePin(CK_READ_GPIO_Port, CK_READ_Pin, GPIO_PIN_RESET);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    /* What the timer is programmed to emit, straight out of the
       registers - not what the comments elsewhere claim it emits. */
    const uint32_t psc = htim1.Instance->PSC + 1u;
    const uint32_t arr = htim1.Instance->ARR + 1u;

    g_dbg_pe9_hz = ((psc * arr) > 0u) ? (Dbg_Tim1ClkHz() / (psc * arr)) : 0u;

    g_dbg_pe9_running =
        (((htim1.Instance->CR1  & TIM_CR1_CEN)   != 0u) &&
         ((htim1.Instance->CCER & TIM_CCER_CC1E) != 0u) &&
         ((htim1.Instance->BDTR & TIM_BDTR_MOE)  != 0u)) ? 1u : 0u;
}

/**
 * @brief Stage 2 - PA1 / CK_READ, the bit-banged readout clock.
 *
 * Drives the production RR2_DAQ_ClockOnce() back to back, so what
 * reaches the scope is the same edge pattern a real event readout
 * produces rather than a simplified stand-in. The read pointer is
 * rewound once at the start for a defined beginning and then simply
 * clocked past the end of the register - this stage is about the shape
 * of the clock, not about the data being shifted out.
 *
 * g_dbg_pa1_period_ns is the number that decides whether the 100 ns
 * high / 100 ns low in radioroc2_daq.h are achievable at all. At
 * 216 MHz a 100 ns delay is about 21 cycles, the same order as one
 * HAL_GPIO_WritePin call, so the measured period usually lands well
 * above the requested 200 ns.
 */
static void RR2_Dbg_Stage2_CkRead(void)
{
    Dbg_StageBegin(g_dbg_pa1_ms);

    g_dbg_pa1_pulses = 0u;

    RR2_DAQ_ResetReadPointer();

    while (!Dbg_StageExpired()) {
        const uint32_t c0 = DWT->CYCCNT;

        for (uint32_t i = 0u; i < RR2_DBG_CKREAD_BLOCK; ++i) {
            RR2_DAQ_ClockOnce();
        }

        const uint32_t per_ns =
            Dbg_CycToNs((DWT->CYCCNT - c0) / RR2_DBG_CKREAD_BLOCK);

        g_dbg_pa1_period_ns = per_ns;
        g_dbg_pa1_freq_khz  = (per_ns > 0u) ? (1000000u / per_ns) : 0u;
        g_dbg_pa1_pulses   += RR2_DBG_CKREAD_BLOCK;
    }

    /* Leave the readout lines exactly as RR2_DAQ_Init() leaves them, so
       the DAQ can take over later without a stray edge. */
    HAL_GPIO_WritePin(CK_READ_GPIO_Port,   CK_READ_Pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RSTN_READ_GPIO_Port, RSTN_READ_Pin, GPIO_PIN_SET);
    RR2_DAQ_EndOfReadout();
}

/**
 * @brief Stage 3 - PB8 / SCL, the Slow Control I2C clock.
 *
 * Repeats one harmless single-byte write - R0, the sub-address latch,
 * with nothing following it - so the bus produces the same short frame
 * over and over and the picture on the scope stands still.
 *
 * g_dbg_ratio_x10 is the point of this stage. The ASIC requires
 * clk_sm_i2c = 20 x SCL, so 200 is the target and anything below it
 * means PE9 is too slow for the bus speed and the chip's I2C core
 * cannot keep up. It is measured against the PE9 frequency captured in
 * stage 1, which is why this stage runs last.
 *
 * The derived SCL reads a few percent low: it comes from timing the HAL
 * call, which carries some software overhead on top of the wire time.
 * Treat it as a cross-check on the scope, not a replacement.
 *
 * Unlike the CHIPID sweep this stage DOES put a data byte on the wire,
 * so trigger on SDA falling and read the 9th clock: that ACK bit is the
 * whole question of whether the ASIC is listening.
 */
static void RR2_Dbg_Stage3_Scl(void)
{
    Dbg_StageBegin(g_dbg_scl_ms);

    g_dbg_scl_ok     = 0u;
    g_dbg_scl_fail   = 0u;
    g_dbg_scl_frames = 0u;

    /* The ASIC's I2C slave core is clocked by PE9. Without it the chip
       cannot answer, and a perfectly healthy bus would read as dead. */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    while (!Dbg_StageExpired()) {
        uint8_t value = RR2_COM_SUB_DELAY;

        const uint32_t c0 = DWT->CYCCNT;
        const HAL_StatusTypeDef st =
            HAL_I2C_Master_Transmit(&hi2c1, RR2_HAL_ADDR(RR2_REG_ADDR_LSB),
                                    &value, 1u, RR2_DBG_I2C_TIMEOUT_MS);
        const uint32_t frame_ns = Dbg_CycToNs(DWT->CYCCNT - c0);

        g_dbg_scl_status = (uint8_t)st;
        g_dbg_scl_frames++;

        if (st == HAL_OK) {
            g_dbg_scl_ok++;

            /* Only a frame that ACKed says anything about bus timing. A
               NACK is mostly HAL timeout, and would drag the derived
               frequency down to nonsense. */
            if (frame_ns > 0u) {
                g_dbg_scl_frame_us = frame_ns / 1000u;

                /* 8 address bits + ACK, then 8 data bits + ACK. START
                   and STOP are not full clock periods and are left out
                   - counting them would bias the frequency high. */
                g_dbg_scl_khz = (18u * 1000000u) / frame_ns;

                if ((g_dbg_scl_khz > 0u) && (g_dbg_pe9_hz > 0u)) {
                    g_dbg_ratio_x10 =
                        ((g_dbg_pe9_hz / 1000u) * 10u) / g_dbg_scl_khz;
                }
            }
        } else {
            g_dbg_scl_fail++;
        }

        HAL_Delay(RR2_DBG_I2C_GAP_MS);
    }
}

#endif /* RR2_MANUAL_DEBUG */

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
