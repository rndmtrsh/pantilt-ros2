/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Stepper pan-tilt controller with discrete smoothing filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEP_FREQ         100000  // TIM3 ISR rate (Hz)
#define MAX_VEL           20000   // Max velocity (steps/s)
#define VEL_UPDATE_TICKS  10      // 100kHz / 10 = 10kHz velocity update
#define DIR_SETUP_TICKS   5       // 10µs ticks to wait after DIR change (50µs)
#define SERIAL_BUF_SIZE   256
#define VEL_SMOOTH_SHIFT  1       // 1/16 per update → time constant ~1.6ms
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;      // Step generation timebase

/* USER CODE BEGIN PV */
// Step generation (ISR context)
volatile uint32_t pan_period   = 0;
volatile uint32_t pan_counter  = 0;
volatile uint32_t tilt_period  = 0;
volatile uint32_t tilt_counter = 0;

// Velocity (current = smoothed; requested = target from commands)
volatile int32_t req_vel_pan      = 0;
volatile int32_t req_vel_tilt     = 0;
volatile int32_t current_vel_pan  = 0;
volatile int32_t current_vel_tilt = 0;

// Direction invert
volatile int8_t pan_dir_invert   = 0;
volatile int8_t tilt_dir_invert  = 1;

// Serial circular buffer
volatile uint8_t  serial_rx_buf[SERIAL_BUF_SIZE];
volatile uint16_t serial_rx_head = 0;
volatile uint16_t serial_rx_tail = 0;

// Command parsing buffer
char    cmd_buffer[64];
uint8_t cmd_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void step_isr(void);
void send_response(const char *msg);
void process_command(char *cmd);
void CDC_Receive_Handler(uint8_t* Buf, uint32_t Len);
void process_serial_data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void step_isr(void) {
    /* ---- Pan step pulse ---- */
    static uint8_t pan_dir_hold = 0;
    if (pan_dir_hold > 0) {
        pan_dir_hold--;
        GPIOB->BSRR = (uint32_t)GPIO_PIN_0 << 16;
    } else if (pan_period > 0) {
        if (pan_counter == 0)
            GPIOB->BSRR = GPIO_PIN_0;
        else if (pan_counter == 1)
            GPIOB->BSRR = (uint32_t)GPIO_PIN_0 << 16;
        if (++pan_counter >= pan_period) pan_counter = 0;
    }

    /* ---- Tilt step pulse ---- */
    static uint8_t tilt_dir_hold = 0;
    if (tilt_dir_hold > 0) {
        tilt_dir_hold--;
        GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
    } else if (tilt_period > 0) {
        if (tilt_counter == 0)
            GPIOA->BSRR = GPIO_PIN_6;
        else if (tilt_counter == 1)
            GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
        if (++tilt_counter >= tilt_period) tilt_counter = 0;
    }

    /* ---- 10kHz velocity update (every VEL_UPDATE_TICKS ticks) ---- */
    static uint16_t update_tick = 0;
    if (++update_tick < VEL_UPDATE_TICKS) return;
    update_tick = 0;

    /* -- Pan smooth & apply -- */
    {
        static uint8_t pan_dir_state = 0;
        int32_t cur = current_vel_pan;
        int32_t tgt = req_vel_pan;

        // Discrete P‑controller (exponential moving average)
        int32_t next = cur + ((tgt - cur) >> VEL_SMOOTH_SHIFT);
        if (next > MAX_VEL)  next = MAX_VEL;
        if (next < -MAX_VEL) next = -MAX_VEL;

        if (next != cur) {
            current_vel_pan = next;

            if (next == 0) {
                pan_period = 0;
                pan_counter = 0;
                GPIOB->BSRR = (uint32_t)GPIO_PIN_0 << 16;
            } else {
                uint8_t dir = ((next > 0) ^ pan_dir_invert) ? 1U : 0U;
                if (dir)
                    GPIOB->BSRR = GPIO_PIN_1;
                else
                    GPIOB->BSRR = (uint32_t)GPIO_PIN_1 << 16;
                if (dir != pan_dir_state) {
                    pan_dir_state = dir;
                    GPIOB->BSRR = (uint32_t)GPIO_PIN_0 << 16;
                    pan_counter = 0;
                    pan_dir_hold = DIR_SETUP_TICKS;
                }
                uint32_t abs_v = (next > 0) ? (uint32_t)next : (uint32_t)(-next);
                uint32_t p = (STEP_FREQ + (abs_v / 2U)) / abs_v;
                if (p < 2) p = 2;
                pan_period = p;
                if (pan_counter >= p) pan_counter = 0;
            }
        }
    }

    /* -- Tilt smooth & apply -- */
    {
        static uint8_t tilt_dir_state = 0;
        int32_t cur = current_vel_tilt;
        int32_t tgt = req_vel_tilt;

        int32_t next = cur + ((tgt - cur) >> VEL_SMOOTH_SHIFT);
        if (next > MAX_VEL)  next = MAX_VEL;
        if (next < -MAX_VEL) next = -MAX_VEL;

        if (next != cur) {
            current_vel_tilt = next;

            if (next == 0) {
                tilt_period = 0;
                tilt_counter = 0;
                GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
            } else {
                uint8_t dir = ((next > 0) ^ tilt_dir_invert) ? 1U : 0U;
                if (dir)
                    GPIOA->BSRR = GPIO_PIN_7;
                else
                    GPIOA->BSRR = (uint32_t)GPIO_PIN_7 << 16;
                if (dir != tilt_dir_state) {
                    tilt_dir_state = dir;
                    GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
                    tilt_counter = 0;
                    tilt_dir_hold = DIR_SETUP_TICKS;
                }
                uint32_t abs_v = (next > 0) ? (uint32_t)next : (uint32_t)(-next);
                uint32_t p = (STEP_FREQ + (abs_v / 2U)) / abs_v;
                if (p < 2) p = 2;
                tilt_period = p;
                if (tilt_counter >= p) tilt_counter = 0;
            }
        }
    }
}

void send_response(const char *msg) {
    // CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    (void)msg;
}

void process_command(char *cmd) {
    char resp[64];

    // --- Dukungan format gabungan ROS2: "P+1000,T-500" ---
    if (strchr(cmd, ',')) {
        char *token = strtok(cmd, ",");
        while (token != NULL) {
            // Hilangkan spasi awal jika ada (opsional)
            while (*token == ' ') token++;

            if (token[0] == 'P' || token[0] == 'p') {
                if (token[1] == '+' || token[1] == '-') {
                    int32_t vel = atoi(&token[1]);  // "+1000" → 1000, "-1000" → -1000
                    if (vel > MAX_VEL)  vel = MAX_VEL;
                    if (vel < -MAX_VEL) vel = -MAX_VEL;
                    req_vel_pan = vel;
                }
            }
            else if (token[0] == 'T' || token[0] == 't') {
                if (token[1] == '+' || token[1] == '-') {
                    int32_t vel = atoi(&token[1]);
                    if (vel > MAX_VEL)  vel = MAX_VEL;
                    if (vel < -MAX_VEL) vel = -MAX_VEL;
                    req_vel_tilt = vel;
                }
            }
            token = strtok(NULL, ",");
        }
        send_response("OK PT\r\n");
        return;
    }

    // --- Perintah tunggal (tetap dipertahankan untuk mode manual) ---
    if (cmd[0] == 'S' || cmd[0] == 's') {
        req_vel_pan  = 0; req_vel_tilt  = 0;
        current_vel_pan = 0; current_vel_tilt = 0;
        pan_period  = 0; tilt_period  = 0;
        GPIOB->BSRR = (uint32_t)GPIO_PIN_0 << 16;
        GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
        send_response("OK S\r\n");
        return;
    }

    if ((cmd[0] == 'I' || cmd[0] == 'i') && (cmd[1] == 'P' || cmd[1] == 'p')) {
        pan_dir_invert = !pan_dir_invert;
        snprintf(resp, sizeof(resp), "OK IP=%d\r\n", (int)pan_dir_invert);
        send_response(resp);
        return;
    }
    if ((cmd[0] == 'I' || cmd[0] == 'i') && (cmd[1] == 'T' || cmd[1] == 't')) {
        tilt_dir_invert = !tilt_dir_invert;
        snprintf(resp, sizeof(resp), "OK IT=%d\r\n", (int)tilt_dir_invert);
        send_response(resp);
        return;
    }

    if (cmd[0] == '?') {
        snprintf(resp, sizeof(resp), "P=%ld T=%ld IP=%d IT=%d\r\n",
                 (long)current_vel_pan, (long)current_vel_tilt,
                 (int)pan_dir_invert, (int)tilt_dir_invert);
        send_response(resp);
        return;
    }

    if (cmd[0] == 'P' || cmd[0] == 'p') {
        if (cmd[1] == '+' || cmd[1] == '-') {
            int32_t vel = atoi(&cmd[2]);
            if (vel < 0) vel = 0;
            if (vel > MAX_VEL) vel = MAX_VEL;
            req_vel_pan = (cmd[1] == '-') ? -vel : vel;
        } else {
            req_vel_pan = 0;
        }
        snprintf(resp, sizeof(resp), "OK P=%ld\r\n", (long)req_vel_pan);
        send_response(resp);
        return;
    }

    if (cmd[0] == 'T' || cmd[0] == 't') {
        if (cmd[1] == '+' || cmd[1] == '-') {
            int32_t vel = atoi(&cmd[2]);
            if (vel < 0) vel = 0;
            if (vel > MAX_VEL) vel = MAX_VEL;
            req_vel_tilt = (cmd[1] == '-') ? -vel : vel;
        } else {
            req_vel_tilt = 0;
        }
        snprintf(resp, sizeof(resp), "OK T=%ld\r\n", (long)req_vel_tilt);
        send_response(resp);
        return;
    }
}

void CDC_Receive_Handler(uint8_t* Buf, uint32_t Len) {
    for (uint32_t i = 0; i < Len; i++) {
        uint16_t next_head = (serial_rx_head + 1) % SERIAL_BUF_SIZE;
        if (next_head != serial_rx_tail) {
            serial_rx_buf[serial_rx_head] = Buf[i];
            serial_rx_head = next_head;
        }
    }
}

void process_serial_data(void) {
    while (serial_rx_tail != serial_rx_head) {
        uint8_t c = serial_rx_buf[serial_rx_tail];
        serial_rx_tail = (serial_rx_tail + 1) % SERIAL_BUF_SIZE;

        if (c == '\n' || c == '\r') {
            if (cmd_idx > 0) {
                cmd_buffer[cmd_idx] = '\0';
                process_command(cmd_buffer);
                cmd_idx = 0;
            }
        } else {
            if (cmd_idx < sizeof(cmd_buffer) - 1) {
                cmd_buffer[cmd_idx++] = c;
            } else {
                cmd_idx = 0;
            }
        }
    }
}
/* USER CODE END 0 */

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM3_Init();
    MX_USB_DEVICE_Init();

    HAL_TIM_Base_Start_IT(&htim3);

    while (1) {
        process_serial_data();
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();

    HAL_RCC_EnableCSS();
}

static void MX_TIM3_Init(void) {
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 84 - 1;           // 84 MHz / 84 = 1 MHz
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 10 - 1;              // 1 MHz / 10 = 100 kHz
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
        Error_Handler();
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}
