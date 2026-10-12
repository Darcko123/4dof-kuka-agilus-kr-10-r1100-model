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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usbd_cdc_if.h>

#include "ANSII_Codes.h"
#include "SSD1306.h"
#include "PCA9685.h"
#include "robotics.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#if KR10_TERMINAL_MENU
/** Estado del menú de usuario */
typedef enum {
	MENU_MAIN,          /**< Esperando una opción del menú principal */
	MENU_ANGLES,        /**< Capturando los ángulos articulares */
	MENU_SEQUENCE       /**< Secuencia de prueba en ejecución */
} Menu_State_t;
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ============================================================================
// SECUENCIA DE PRUEBA
// ============================================================================
#define DELAY_TEST_SEQUENCE     5000U   /**< Tiempo entre poses de la secuencia [ms] */
#define TEST_SEQUENCE_LENGTH    5U      /**< Número de poses de la secuencia */

// ============================================================================
// CONFIGURACIÓN DE SERVOS
// ============================================================================
#define SERVO_PWM_FREQUENCY     50U     /**< Frecuencia PWM para servos [Hz] */
#define SERVO_UPDATE_INTERVAL   20U     /**< Periodo de actualización del movimiento suave [ms] */
#define SERVO_MOVE_DURATION     1500U   /**< Duración de cada movimiento [ms] */
#define SERVO_ZERO_DEG          0.0f    /**< Ángulo del servo correspondiente a q = 0 (horizontal) [°] */

// ============================================================================
// INTERFAZ USB-CDC
// ============================================================================
#define CDC_TX_TIMEOUT          20U     /**< Espera máxima por mensaje si el host no lee [ms] */
#define RX_RING_SIZE            256U    /**< Bytes recibidos pendientes de procesar */
#define LINE_BUFFER_SIZE        32U     /**< Longitud máxima de una línea de la interfaz de terminal */
#define CMD_LINE_SIZE           96U     /**< Longitud máxima de una línea del protocolo de host */
#define OLED_TITLE_SIZE         20U     /**< Longitud del título mostrado en la barra de la OLED */
#define KR10_FW_VERSION         "1.1"   /**< Versión del firmware informada al host */
#define KR10_LINE_PREFIX        "KR10:" /**< Prefijo de toda línea emitida hacia el host */

/** Poner a 1 para compilar la interfaz de terminal del firmware (modo autónomo,
 *  sin PC conectado). La interfaz de Python sustituye a este menú, por lo que
 *  el valor por defecto es 0 y el firmware queda como agente de control.
 *  Se puede forzar por línea de comandos con -DKR10_TERMINAL_MENU=1. */
#ifndef KR10_TERMINAL_MENU
#define KR10_TERMINAL_MENU      0
#endif

// Estilos para la interfaz
#define HEADER_COLOR            COLOR_BRIGHT_CYAN
#define MENU_COLOR              COLOR_BRIGHT_GREEN
#define OPTION_COLOR            COLOR_CYAN
#define WARNING_COLOR           COLOR_BRIGHT_YELLOW
#define ERROR_COLOR             COLOR_BRIGHT_RED
#define SUCCESS_COLOR           COLOR_BRIGHT_GREEN
#define INFO_COLOR              COLOR_BRIGHT_BLUE
#define INPUT_COLOR             COLOR_BRIGHT_WHITE
#define VALUE_COLOR             COLOR_YELLOW
#define SEPARATOR_COLOR         COLOR_BRIGHT_MAGENTA

// Elementos gráficos de la terminal (UTF-8: cada carácter de recuadro ocupa 3 bytes)
#define BOX_TOP     "╔══════════════════════════════════════════════════╗"
#define BOX_BOTTOM  "╚══════════════════════════════════════════════════╝"
#define TERM_RULE   "──────────────────────────────────────────────────"
#define GAUGE_WIDTH             21U     /**< Ancho de la barra de posición articular [caracteres] */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN PV */
/** Canal del PCA9685 asignado a cada articulación (base, hombro, codo, muñeca) */
static const uint8_t JointChannel[ROBOTICS_NUM_JOINTS] = {0, 1, 2, 3};

/** Secuencia de prueba en espacio articular [rad], dentro de 0°-180° */
static const float TestSequence[TEST_SEQUENCE_LENGTH][ROBOTICS_NUM_JOINTS] = {
		{ ROBOTICS_DEG2RAD(  0.0f), ROBOTICS_DEG2RAD(  0.0f), ROBOTICS_DEG2RAD(  0.0f), ROBOTICS_DEG2RAD(  0.0f) },
		{ ROBOTICS_DEG2RAD( 45.0f), ROBOTICS_DEG2RAD( 30.0f), ROBOTICS_DEG2RAD( 30.0f), ROBOTICS_DEG2RAD(  0.0f) },
		{ ROBOTICS_DEG2RAD(135.0f), ROBOTICS_DEG2RAD( 30.0f), ROBOTICS_DEG2RAD( 30.0f), ROBOTICS_DEG2RAD(  0.0f) },
		{ ROBOTICS_DEG2RAD( 90.0f), ROBOTICS_DEG2RAD( 60.0f), ROBOTICS_DEG2RAD( 60.0f), ROBOTICS_DEG2RAD( 30.0f) },
		{ ROBOTICS_DEG2RAD( 90.0f), ROBOTICS_DEG2RAD( 90.0f), ROBOTICS_DEG2RAD(  0.0f), ROBOTICS_DEG2RAD(  0.0f) }
};

Robotics_Arm_t arm;                             /**< Estado cinemático del brazo (q, T, pose) */
Servo_Smooth_t servos[ROBOTICS_NUM_JOINTS];     /**< Movimiento suave de cada servo */

PCA9685_Status_t status_pca;
SSD1306_Status_t status_ssd;

char buffer[256];

extern USBD_HandleTypeDef hUsbDeviceFS;

/** Configuración articular de la posición HOME [rad] */
static const float HomePose[ROBOTICS_NUM_JOINTS] = { 0.0f, 0.0f, 0.0f, 0.0f };

#if KR10_TERMINAL_MENU
/** Nombre de cada articulación para la terminal, rellenado a 6 columnas
 *  a mano porque %-6s cuenta bytes y la 'ñ' ocupa 2 en UTF-8 */
static const char* const JointName[ROBOTICS_NUM_JOINTS] = { "Base  ", "Hombro", "Codo  ", "Muñeca" };
#endif

/** Buffer circular de recepción USB-CDC (productor: ISR USB, consumidor: lazo principal) */
static volatile uint8_t rxRing[RX_RING_SIZE];
static volatile uint32_t rxHead = 0;
static volatile uint32_t rxTail = 0;

/** Línea de entrada del protocolo en edición */
static char lineBuf[CMD_LINE_SIZE];
static uint32_t lineLen = 0;

#if KR10_TERMINAL_MENU
static Menu_State_t menuState = MENU_MAIN;

/** Captura manual de ángulos */
static uint32_t angleStep = 0;
static float angleInput[ROBOTICS_NUM_JOINTS];
#endif

/** Secuencia de prueba en curso (comandada por el host) */
static bool sequenceActive = false;
static uint32_t sequenceStep = 0;
static uint32_t lastMoveTick = 0;

#if !KR10_TERMINAL_MENU
/** Movimiento simple en curso: el lazo principal lo supervisa y avisa al host */
static bool moveActive = false;

/** Título mostrado en la barra de la OLED para el movimiento en curso */
static char oledTitle[OLED_TITLE_SIZE];
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
static float Arm_JointToServoAngle(float q);
static void Arm_Init(void);
static Robotics_Status_t Arm_MoveTo(const float q[ROBOTICS_NUM_JOINTS], uint32_t durationMs);
static bool Arm_Update(void);

static void CDC_Print(const char* str);
static void Display_ShowArm(const char* title);

#if !KR10_TERMINAL_MENU
/* --- Protocolo de host (interfaz de usuario en Python) ------------------- */
static void Display_ShowError(const char* text);
static void Host_SendLine(const char* line);
static void Host_SendFormat(const char* fmt, ...);
static void Host_SendState(void);
static void Host_SendLimits(void);
static void Host_SendSequenceState(void);
static void Host_SendOk(void);
static void Host_SendError(const char* reason);
static void Host_SendAck(const char* cmd);
static void Host_SendHello(void);
static void Host_SequenceStep(void);
static void Host_StopSequence(void);
static bool Host_StartMove(const float q[ROBOTICS_NUM_JOINTS], const char* title, const char* cmd);
static bool Host_ParseAngles(char* line, float q[ROBOTICS_NUM_JOINTS]);
static void Host_HandleByte(uint8_t c);
static void Host_ProcessLine(char* line);
static bool Host_StrEqArg(const char* arg, const char* cmd);
#else /* KR10_TERMINAL_MENU */

static void CDC_PrintColor(const char* color, const char* msg);
static void Menu_ShowHeader(const char* title);
static uint32_t Menu_GaugeIndex(float q, const Robotics_DH_Param_t* dh);
static void Menu_ShowMain(void);
static void Menu_ShowPrompt(void);
static void Menu_ShowPosition(void);
static void Menu_PrintArmState(void);
static void Menu_PrintReturn(void);
static void Menu_PromptAngle(void);
static void Menu_MoveBlocking(const float q[ROBOTICS_NUM_JOINTS], const char* title);
static void Menu_SequenceStep(void);
static void Menu_HandleByte(uint8_t c);
static void Menu_ProcessLine(char* line);
static void Menu_ProcessAngle(char* line);
#endif /* KR10_TERMINAL_MENU */
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

	status_ssd = SSD1306_Init(&hi2c3, 128, 64);

	// Sin PCA9685 el sistema se detiene: avisarlo en la OLED antes de Error_Handler()
	status_pca = PCA9685_Init(&hi2c1, SERVO_PWM_FREQUENCY);
	if(status_pca != PCA9685_OK)
	{
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		SSD1306_GotoXY(0, 0);
		SSD1306_Puts("ERROR PCA9685", &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
		Error_Handler();
	}

	HAL_Delay(2000);
	SSD1306_Clear();

	Arm_Init();

	Display_ShowArm("INICIO");

#if KR10_TERMINAL_MENU
	Menu_ShowMain();    // Si el host aún no abre el puerto, ENTER vuelve a mostrarlo
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		// Atender los bytes recibidos por USB-CDC
		while(rxTail != rxHead)
		{
			uint8_t c = rxRing[rxTail];
			rxTail = (rxTail + 1U) % RX_RING_SIZE;

#if KR10_TERMINAL_MENU
			Menu_HandleByte(c);
#else
			Host_HandleByte(c);
#endif
		}

		// Avanzar el movimiento en curso y, con la secuencia activa, cuando
		// termine y se cumpla el intervalo, comandar la siguiente pose
		bool idle = Arm_Update();

		#if !KR10_TERMINAL_MENU
		// Aviso al host de que un movimiento simple terminó
		if(moveActive && idle)
		{
			moveActive = false;
			Display_ShowArm(oledTitle);
			Host_SendOk();
			Host_SendState();
		}
#endif

		if(sequenceActive && idle && (HAL_GetTick() - lastMoveTick) >= DELAY_TEST_SEQUENCE)
		{
#if KR10_TERMINAL_MENU
			Menu_SequenceStep();
#else
			Host_SequenceStep();
#endif
		}
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Convierte un ángulo articular al ángulo de comando del servo.
 *
 * @details q = 0 es la posición horizontal, que corresponde a SERVO_ZERO_DEG,
 * de modo que el rango articular [0, pi] cubre los 0°-180° del servo. El
 * sentido de montaje de cada servo ya está modelado en la tabla DH
 * (campo direction), por lo que aquí no se invierte ningún canal.
 *
 * @param q Ángulo articular [rad].
 *
 * @return float Ángulo del servo [°].
 */
static float Arm_JointToServoAngle(float q)
{
    return SERVO_ZERO_DEG + ROBOTICS_RAD2DEG(q);
}

/**
 * @brief Inicializa el modelo cinemático y lleva el brazo a q = 0.
 *
 * @note Requiere el PCA9685 ya inicializado. Cualquier fallo detiene el
 * sistema en Error_Handler().
 */
static void Arm_Init(void)
{
    if(Robotics_Init(NULL) != ROBOTICS_OK)
    {
        Error_Handler();
    }

    if(Robotics_InitArm(&arm, NULL) != ROBOTICS_OK)
    {
        Error_Handler();
    }

    for(uint32_t i = 0; i < ROBOTICS_NUM_JOINTS; i++)
    {
        if(PCA9685_InitSmoothServo(&servos[i], JointChannel[i], Arm_JointToServoAngle(arm.q[i]), SERVO_UPDATE_INTERVAL) != PCA9685_OK)
        {
            Error_Handler();
        }
    }
}

/**
 * @brief Inicia un movimiento suave del brazo hacia una configuración articular.
 *
 * @details Valida los límites articulares y actualiza la cinemática directa
 * (arm.T y arm.pose) antes de mover los servos. Si la configuración no es
 * válida, el brazo no se mueve.
 *
 * @param q Ángulos articulares objetivo [rad].
 * @param durationMs Duración del movimiento [ms].
 *
 * @return Robotics_Status_t Estado de la operación (OK, OUT_OF_RANGE, ERROR, ...).
 */
static Robotics_Status_t Arm_MoveTo(const float q[ROBOTICS_NUM_JOINTS], uint32_t durationMs)
{
    Robotics_Status_t status;

    status = Robotics_SetJointAngles(&arm, q);
    if(status != ROBOTICS_OK)
    {
        return status;
    }

    for(uint32_t i = 0; i < ROBOTICS_NUM_JOINTS; i++)
    {
        if(PCA9685_SetSmoothAngle(&servos[i], Arm_JointToServoAngle(arm.q[i]), durationMs) != PCA9685_OK)
        {
            return ROBOTICS_ERROR;
        }
    }

    return ROBOTICS_OK;
}

/**
 * @brief Avanza el movimiento suave de todos los servos (llamar periódicamente).
 *
 * @return true si todos los servos alcanzaron su objetivo, false si alguno
 * sigue en movimiento.
 */
static bool Arm_Update(void)
{
    bool done = true;

    for(uint32_t i = 0; i < ROBOTICS_NUM_JOINTS; i++)
    {
        if(!PCA9685_UpdateSmoothServo(&servos[i]))
        {
            done = false;
        }

        if(servos[i].lastError != PCA9685_OK)
        {
            Error_Handler();
        }
    }

    return done;
}

// ============================================================================
// COMUNICACIÓN USB-CDC
// ============================================================================

/**
 * @brief Recibe los bytes del endpoint OUT y los encola para el lazo principal.
 *
 * @note Se ejecuta en el ISR USB: no debe transmitir ni bloquear. Si el
 * buffer circular se llena, los bytes sobrantes se descartan.
 */
void CDC_ReceiveCallback(uint8_t* Buf, uint32_t Len)
{
    for(uint32_t i = 0; i < Len; i++)
    {
        uint32_t next = (rxHead + 1U) % RX_RING_SIZE;
        if(next == rxTail)
        {
            break;
        }
        rxRing[rxHead] = Buf[i];
        rxHead = next;
    }
}

/**
 * @brief Envía una cadena por USB-CDC y espera a que termine de salir.
 *
 * @details CDC_Transmit_FS() transmite desde el buffer del llamador de forma
 * asíncrona y retorna USBD_BUSY si hay un envío en curso; esperar a TxState
 * permite reutilizar el buffer inmediatamente. Si el puerto no está
 * configurado o el host no lee, el mensaje se descarta.
 *
 * @param str Cadena terminada en '\0'.
 */
static void CDC_Print(const char* str)
{
    // ponytail: descarta el mensaje tras CDC_TX_TIMEOUT si el host no lee; usar cola TX si se requiere entrega garantizada
    if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return;
    }

    uint32_t start = HAL_GetTick();
    while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY)
    {
        if((HAL_GetTick() - start) >= CDC_TX_TIMEOUT)
        {
            return;
        }
    }

    USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    while(hcdc->TxState != 0U && (HAL_GetTick() - start) < CDC_TX_TIMEOUT)
    {
    }
}

#if KR10_TERMINAL_MENU
/**
 * @brief Envía un mensaje de una línea con color.
 *
 * @param color Código de color ANSI.
 * @param msg Mensaje (no debe ser el buffer global).
 */
static void CDC_PrintColor(const char* color, const char* msg)
{
    snprintf(buffer, sizeof(buffer), "  %s%s%s\r\n", color, msg, COLOR_RESET);
    CDC_Print(buffer);
}
#endif

// ============================================================================
// PANTALLA OLED
// ============================================================================

/**
 * @brief Muestra en la OLED los ángulos articulares y la pose del efector.
 *
 * @details Barra de título invertida y dos columnas: Q1-Q4 [°] a la
 * izquierda; X, Y, Z [mm] y pitch [°] del efector a la derecha.
 *
 * @param title Título de la barra superior (solo ASCII, máx. 17 caracteres).
 */
static void Display_ShowArm(const char* title)
{
    // Una fila por articulación; la columna derecha reutiliza las 4 filas
    static const char PoseName[ROBOTICS_NUM_JOINTS] = { 'X', 'Y', 'Z', 'P' };
    const float pose[ROBOTICS_NUM_JOINTS] = { arm.pose.x, arm.pose.y, arm.pose.z, ROBOTICS_RAD2DEG(arm.pose.pitch) };
    char line[20];

    SSD1306_Fill(SSD1306_COLOR_BLACK);

    SSD1306_DrawFilledRectangle(0, 0, SSD1306_GetWidth() - 1U, 11, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(2, 1);
    snprintf(line, sizeof(line), "%s", title);
    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_BLACK);

    SSD1306_DrawFastVLine(63, 14, 50, SSD1306_COLOR_WHITE);

    for(uint32_t i = 0; i < ROBOTICS_NUM_JOINTS; i++)
    {
        uint16_t y = 15U + (uint16_t)(i * 12U);

        SSD1306_GotoXY(0, y);
        snprintf(line, sizeof(line), "Q%lu%6.1f", i + 1U, ROBOTICS_RAD2DEG(arm.q[i]));
        SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);

        SSD1306_GotoXY(67, y);
        snprintf(line, sizeof(line), "%c%7.1f", PoseName[i], pose[i]);
        SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);
    }

    // El aviso de la terminal refleja el último envío real, no solo el de SSD1306_Init()
    status_ssd = SSD1306_UpdateScreen();
}

// ============================================================================
// PROTOCOLO DE HOST (USB-CDC, texto)
// ============================================================================
//
// El firmware actúa como agente de control: la interfaz de usuario vive en
// Python y se comunica con líneas de texto terminadas en CRLF. Todas las
// respuestas se prefijan con KR10: para que el host las distinga de restos de
// una sesión anterior y pueda ignorarlas con seguridad.
//
// Comandos (host -> firmware):
//   PING                      -> OK PONG <fw>
//   HELLO                     -> OK / ID / LIM / OLED ... / END
//   STATUS                    -> OK / Q i angulo ... / POSE x y z pitch yaw / OLED n / END
//   LIMITS                    -> OK / LIM i min max ... / END
//   MOVE q1 q2 q3 q4           -> OK (al terminar: OK + STATE)
//   HOME                      -> OK (al terminar: OK + STATE)
//   SEQ START|STOP            -> OK
//   SEQ STATUS                -> OK / SEQ i n / END
//   OLED titulo               -> OK
//   STOP                      -> OK (aborta movimiento en curso)
//
// Los ángulos viajan siempre en grados; la conversión a radianes y la
// validación de límites se hacen aquí, igual que en la interfaz de terminal.
// ============================================================================

#if !KR10_TERMINAL_MENU

/**
  * @brief Muestra en la OLED un mensaje de error y detiene el sistema.
  *
  * @param text Mensaje ASCII a mostrar.
  */
static void Display_ShowError(const char* text)
{
    SSD1306_Fill(SSD1306_COLOR_BLACK);
    SSD1306_GotoXY(0, 0);
    // SSD1306_Puts() no acepta const, pero solo lee el puntero
    SSD1306_Puts((char*)text, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
    Error_Handler();
}

/**
  * @brief Envía una línea terminada en CRLF hacia el host.
  *
  * @param line Cadena terminada en '\0' (sin CRLF).
  */
static void Host_SendLine(const char* line)
{
    uint32_t start = HAL_GetTick();

    if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return;
    }

    snprintf(buffer, sizeof(buffer), KR10_LINE_PREFIX "%s\r\n", line);
    CDC_Print(buffer);

    // No bloquear el lazo principal si el host no lee
    (void)start;
}

/**
  * @brief Envía una línea formateada hacia el host.
  *
  * @param fmt Formato printf (sin el prefijo ni el CRLF).
  * @param ... Argumentos del formato.
  */
static void Host_SendFormat(const char* fmt, ...)
{
    char line[CMD_LINE_SIZE];
    va_list args;

    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    Host_SendLine(line);
}

/**
  * @brief Compara el primer token de una línea con un comando.
  *
  * @details Se detiene en el primer espacio, por lo que "MOVE 0 0 0 0" casa
  * con "MOVE". La comparación no distingue mayúsculas porque los usuarios
  * escriben a mano; el resto del protocolo sí es sensible.
  *
  * @param arg Primer token de la línea recibida.
  * @param cmd Comando a comparar.
  *
  * @return true si coincide.
  */
static bool Host_StrEqArg(const char* arg, const char* cmd)
{
    while(*cmd != '\0')
    {
        char c = *arg++;

        if(c >= 'a' && c <= 'z')
        {
            c = (char)(c - ('a' - 'A'));
        }

        if(c != *cmd++)
        {
            return false;
        }
    }

    return (*arg == '\0' || *arg == ' ');
}

/**
  * @brief Envía la línea de confirmación OK.
  */
static void Host_SendOk(void)
{
    Host_SendLine("OK");
}

/**
  * @brief Envía un error seguido del fin de la respuesta.
  *
  * @param reason Código de error legible por el host.
  */
static void Host_SendError(const char* reason)
{
    Host_SendFormat("ERR %s", reason);
    Host_SendLine("END");
}

/**
  * @brief Envía el acuse de un comando y el final de la respuesta.
  *
  * @param cmd Nombre del comando atendido.
  */
static void Host_SendAck(const char* cmd)
{
    Host_SendFormat("OK %s", cmd);
    Host_SendLine("END");
}

/**
  * @brief Envía la descripción del robot (identidad y capacidades).
  */
static void Host_SendHello(void)
{
    Host_SendFormat("ID KR10 4DOF %s", KR10_FW_VERSION);
    Host_SendFormat("JOINTS %u", (unsigned)ROBOTICS_NUM_JOINTS);
    Host_SendFormat("MOVE_MS %u", (unsigned)SERVO_MOVE_DURATION);
    Host_SendFormat("SEQ_LEN %u", (unsigned)TEST_SEQUENCE_LENGTH);
    Host_SendFormat("SEQ_DELAY_MS %u", (unsigned)DELAY_TEST_SEQUENCE);
    Host_SendFormat("OLED %d", (status_ssd == SSD1306_OK) ? 1 : 0);
    Host_SendLine("END");
}

/**
  * @brief Envía la cinemática actual: ángulos articulares y pose del efector.
  */
static void Host_SendState(void)
{
    for(uint32_t i = 0; i < ROBOTICS_NUM_JOINTS; i++)
    {
        Host_SendFormat("Q %u %.2f", (unsigned)(i + 1U), ROBOTICS_RAD2DEG(arm.q[i]));
    }

    Host_SendFormat("POSE %.2f %.2f %.2f %.2f %.2f",
                    arm.pose.x, arm.pose.y, arm.pose.z,
                    ROBOTICS_RAD2DEG(arm.pose.pitch), ROBOTICS_RAD2DEG(arm.pose.yaw));

    Host_SendFormat("OLED %d", (status_ssd == SSD1306_OK) ? 1 : 0);
    Host_SendLine("END");
}

/**
  * @brief Envía los límites articulares en grados.
  */
static void Host_SendLimits(void)
{
    for(uint32_t i = 0; i < ROBOTICS_NUM_JOINTS; i++)
    {
        const Robotics_DH_Param_t* dh = &Robotics_DefaultDHTable[i];
        Host_SendFormat("LIM %u %.1f %.1f", (unsigned)(i + 1U),
                        ROBOTICS_RAD2DEG(dh->minAngle), ROBOTICS_RAD2DEG(dh->maxAngle));
    }
    Host_SendLine("END");
}

/**
  * @brief Envía el avance de la secuencia automática.
  */
static void Host_SendSequenceState(void)
{
    if(!sequenceActive)
    {
        Host_SendLine("SEQ IDLE");
    }
    else
    {
        Host_SendFormat("SEQ %lu %u", (unsigned long)sequenceStep, (unsigned)TEST_SEQUENCE_LENGTH);
    }
}

/**
  * @brief Extrae cuatro ángulos en grados de una línea de comando.
  *
  * @details Acepta ',' o '.' como separador decimal, igual que el menú de
  * terminal. Los valores se devuelven en radianes ya validados contra los
  * límites articulares, de modo que todo lo que acepte el host lo acepta
  * Robotics_SetJointAngles().
  *
  * @param line Línea recibida (terminada en '\0'), se modifica in situ.
  * @param[out] q Ángulos articulares resultantes [rad].
  *
  * @return true si los cuatro valores son válidos.
  */
static bool Host_ParseAngles(char* line, float q[ROBOTICS_NUM_JOINTS])
{
    uint32_t count = 0;
    char* p = line;

    while(*p != '\0' && count < ROBOTICS_NUM_JOINTS)
    {
        char* end;

        for(char* c = p; *c != '\0'; c++)
        {
            if(*c == ',')
            {
                *c = '.';
            }
        }

        float value = ROBOTICS_DEG2RAD(strtof(p, &end));
        if(end == p)
        {
            return false;
        }

        const Robotics_DH_Param_t* dh = &Robotics_DefaultDHTable[count];
        // La comparación negada también rechaza NaN
        if(!(value >= dh->minAngle && value <= dh->maxAngle))
        {
            return false;
        }

        q[count++] = value;
        p = end;

        while(*p == ' ' || *p == '\t')
        {
            p++;
        }
    }

    return (count == ROBOTICS_NUM_JOINTS);
}

/**
  * @brief Lanza un movimiento suave hacia una configuración articular.
  *
  * @details No bloquea: el lazo principal envía el OK y el estado cuando el
  * movimiento termina, de modo que el host puede seguir enviando comandos
  * (STOP, STATUS) mientras el brazo se mueve.
  *
  * @param q Ángulos articulares objetivo [rad].
  * @param title Título para la barra de la OLED.
  * @param cmd Nombre del comando para el acuse.
  *
  * @return true si el movimiento se ha iniciado correctamente.
  */
static bool Host_StartMove(const float q[ROBOTICS_NUM_JOINTS], const char* title, const char* cmd)
{
    if(sequenceActive)
    {
        Host_StopSequence();
    }

    if(Arm_MoveTo(q, SERVO_MOVE_DURATION) != ROBOTICS_OK)
    {
        Host_SendError("RANGE");
        return false;
    }

    snprintf(oledTitle, sizeof(oledTitle), "%s", title);
    moveActive = true;
    Host_SendAck(cmd);
    Display_ShowArm(oledTitle);
    return true;
}

/**
  * @brief Comanda la siguiente pose de la secuencia de prueba.
  */
static void Host_SequenceStep(void)
{
    snprintf(oledTitle, sizeof(oledTitle), "SEQ %lu/%u",
             (unsigned long)(sequenceStep + 1U), (unsigned)TEST_SEQUENCE_LENGTH);

    if(Arm_MoveTo(TestSequence[sequenceStep], SERVO_MOVE_DURATION) != ROBOTICS_OK)
    {
        Display_ShowError("SEQ RANGE");
    }

    // El estado se emite con el índice ya incrementado para que el host sepa
    // cuál es la pose que se acaba de comandar
    sequenceStep = (sequenceStep + 1U) % TEST_SEQUENCE_LENGTH;
    lastMoveTick = HAL_GetTick();

    Display_ShowArm(oledTitle);
    Host_SendSequenceState();
    Host_SendState();
}

/**
  * @brief Detiene la secuencia automática si está en curso.
  */
static void Host_StopSequence(void)
{
    if(sequenceActive)
    {
        sequenceActive = false;
        Display_ShowArm("DETENIDO");
    }
}

/**
  * @brief Arranca la secuencia automática desde la primera pose.
  */
static void Host_StartSequence(void)
{
    sequenceActive = true;
    sequenceStep = 0;
    lastMoveTick = HAL_GetTick() - DELAY_TEST_SEQUENCE;   // Primer paso inmediato
    moveActive = false;
    Host_SendAck("SEQ START");
}

/**
  * @brief Envía la vista OLED actual al host tras un comando OLED.
  *
  * @param title Título ya mostrado en la pantalla.
  */
static void Host_SetOledTitle(const char* title)
{
    snprintf(oledTitle, sizeof(oledTitle), "%s", title);
    Display_ShowArm(oledTitle);
    Host_SendAck("OLED");
}

/**
  * @brief Ensambla y despacha un comando recibido del host.
  *
  * @param line Línea recibida (terminada en '\0'), se modifica in situ.
  */
static void Host_ProcessLine(char* line)
{
    float q[ROBOTICS_NUM_JOINTS];

    // El comando es el primer token; el resto es su argumento
    char* arg = line;
    while(*arg != '\0' && *arg != ' ')
    {
        arg++;
    }
    if(*arg != '\0')
    {
        *arg++ = '\0';
        while(*arg == ' ')
        {
            arg++;
        }
    }

    if(Host_StrEqArg(line, "PING"))
    {
        Host_SendFormat("OK PONG %s", KR10_FW_VERSION);
        Host_SendLine("END");
    }
    else if(Host_StrEqArg(line, "HELLO"))
    {
        Host_SendOk();
        Host_SendHello();
    }
    else if(Host_StrEqArg(line, "STATUS"))
    {
        Host_SendOk();
        Host_SendState();
    }
    else if(Host_StrEqArg(line, "LIMITS"))
    {
        Host_SendOk();
        Host_SendLimits();
    }
    else if(Host_StrEqArg(line, "MOVE"))
    {
        if(!Host_ParseAngles(arg, q))
        {
            Host_SendError("ANGLE");
        }
        else
        {
            Host_StartMove(q, "MANUAL", "MOVE");
        }
    }
    else if(Host_StrEqArg(line, "HOME"))
    {
        Host_StartMove(HomePose, "HOME", "HOME");
    }
    else if(Host_StrEqArg(line, "SEQ"))
    {
        if(Host_StrEqArg(arg, "START"))
        {
            Host_StartSequence();
        }
        else if(Host_StrEqArg(arg, "STOP"))
        {
            Host_StopSequence();
            Host_SendAck("SEQ STOP");
        }
        else if(Host_StrEqArg(arg, "STATUS"))
        {
            Host_SendOk();
            Host_SendSequenceState();
            Host_SendLine("END");
        }
        else
        {
            Host_SendError("SEQ ARG");
        }
    }
    else if(Host_StrEqArg(line, "OLED"))
    {
        if(arg[0] == '\0')
        {
            Host_SendError("OLED ARG");
        }
        else
        {
            Host_SetOledTitle(arg);
        }
    }
    else if(Host_StrEqArg(line, "STOP"))
    {
        Host_StopSequence();

        if(moveActive)
        {
            // Fijar el objetivo en el ángulo actual cancela el movimiento suave
            // sin dejar los servos a medio camino de un destino arbitrario
            for(uint32_t i = 0; i < ROBOTICS_NUM_JOINTS; i++)
            {
                servos[i].targetAngle = servos[i].currentAngle;
                servos[i].isMoving = false;
            }
            moveActive = false;
            Display_ShowArm("DETENIDO");
        }

        Host_SendAck("STOP");
        Host_SendState();
    }
    else
    {
        Host_SendError("CMD");
    }
}

/**
  * @brief Ensambla líneas de comando: eco desactivado, CRLF = fin de línea.
  *
  * @param c Byte recibido por USB-CDC.
  */
static void Host_HandleByte(uint8_t c)
{
    static uint8_t prev = 0;

    // CRLF cuenta como un solo ENTER
    if(c == '\n' && prev == '\r')
    {
        prev = c;
        return;
    }
    prev = c;

    if(c == '\r' || c == '\n')
    {
        lineBuf[lineLen] = '\0';

        if(lineLen > 0U)
        {
            lineLen = 0;
            Host_ProcessLine(lineBuf);
        }
    }
    else if(c == '\b' || c == 127U)
    {
        if(lineLen > 0U)
        {
            lineLen--;
        }
    }
    else if(c >= ' ' && lineLen < (CMD_LINE_SIZE - 1U))
    {
        lineBuf[lineLen++] = (char)c;
    }
}

#endif /* !KR10_TERMINAL_MENU */

#if KR10_TERMINAL_MENU
// ============================================================================
// MENÚ DE USUARIO
// ============================================================================

/**
 * @brief Limpia la terminal y dibuja el encabezado y el título de la pantalla.
 *
 * @param title Título de la sección.
 */
static void Menu_ShowHeader(const char* title)
{
    snprintf(buffer, sizeof(buffer), "%s%s%s" BOX_TOP "\r\n", CLEAR_SCREEN, CURSOR_HOME, SEPARATOR_COLOR);
    CDC_Print(buffer);

    snprintf(buffer, sizeof(buffer), "║%s%s    KUKA AGILUS KR10  ·  Brazo robótico 4 GDL     %s%s║\r\n",
             HEADER_COLOR, COLOR_BOLD, COLOR_RESET, SEPARATOR_COLOR);
    CDC_Print(buffer);

    snprintf(buffer, sizeof(buffer), "║%s     Cinemática directa · Denavit-Hartenberg      %s║\r\n" BOX_BOTTOM "%s\r\n",
             INFO_COLOR, SEPARATOR_COLOR, COLOR_RESET);
    CDC_Print(buffer);

    if(status_ssd != SSD1306_OK)
    {
        CDC_PrintColor(WARNING_COLOR, "⚠ Pantalla OLED no disponible");
    }

    snprintf(buffer, sizeof(buffer), "\r\n  %s%s▸ %s%s\r\n  %s" TERM_RULE "%s\r\n",
             MENU_COLOR, COLOR_BOLD, title, COLOR_RESET, COLOR_DIM, COLOR_RESET);
    CDC_Print(buffer);
}

/**
 * @brief Muestra el menú principal con un resumen de la pose actual.
 */
static void Menu_ShowMain(void)
{
    static const char* const Option[] = {
        "Ejecutar secuencia automática",
        "Ingresar ángulos manualmente",
        "Ir a posición HOME",
        "Ver posición actual"
    };

    Menu_ShowHeader("MENÚ PRINCIPAL");

    for(uint32_t i = 0; i < sizeof(Option) / sizeof(Option[0]); i++)
    {
        snprintf(buffer, sizeof(buffer), "   %s%s[%lu]%s %s\r\n", OPTION_COLOR, COLOR_BOLD, i + 1U, COLOR_RESET, Option[i]);
        CDC_Print(buffer);
    }

    snprintf(buffer, sizeof(buffer), "  %s" TERM_RULE "%s\r\n", COLOR_DIM, COLOR_RESET);
    CDC_Print(buffer);

    snprintf(buffer, sizeof(buffer), "  %sEfector:%s X %s%.1f%s  Y %s%.1f%s  Z %s%.1f%s mm  Pitch %s%.1f°%s\r\n",
             COLOR_DIM, COLOR_RESET,
             VALUE_COLOR, arm.pose.x, COLOR_RESET,
             VALUE_COLOR, arm.pose.y, COLOR_RESET,
             VALUE_COLOR, arm.pose.z, COLOR_RESET,
             VALUE_COLOR, ROBOTICS_RAD2DEG(arm.pose.pitch), COLOR_RESET);
    CDC_Print(buffer);

    Menu_ShowPrompt();
}

/**
 * @brief Muestra el indicador de entrada del menú principal.
 */
static void Menu_ShowPrompt(void)
{
    snprintf(buffer, sizeof(buffer), "\r\n  %s%sKR10 ›%s Opción [1-4]: %s",
             COLOR_BRIGHT_MAGENTA, COLOR_BOLD, COLOR_RESET, INPUT_COLOR);
    CDC_Print(buffer);
}

/**
 * @brief Muestra la configuración articular y la pose actual del brazo.
 */
static void Menu_ShowPosition(void)
{
    Menu_ShowHeader("POSICIÓN ACTUAL");
    Menu_PrintArmState();
    Menu_PrintReturn();
}

/**
 * @brief Posición de q dentro de la barra de rango de su articulación.
 *
 * @param q Ángulo articular [rad].
 * @param dh Parámetros DH de la articulación (límites).
 *
 * @return uint32_t Índice 0..GAUGE_WIDTH-1.
 */
static uint32_t Menu_GaugeIndex(float q, const Robotics_DH_Param_t* dh)
{
    float t = (q - dh->minAngle) / (dh->maxAngle - dh->minAngle);
    return (uint32_t)(fminf(fmaxf(t, 0.0f), 1.0f) * (float)(GAUGE_WIDTH - 1U) + 0.5f);
}

/**
 * @brief Imprime la tabla de ángulos articulares (con barra de rango) y la
 * pose del efector (arm.q, arm.pose).
 */
static void Menu_PrintArmState(void)
{
    char gauge[GAUGE_WIDTH * 3U + 1U];      // Cada glifo ocupa 3 bytes en UTF-8

    snprintf(buffer, sizeof(buffer), "  %sArticulación Ángulo                Rango%s\r\n", INFO_COLOR, COLOR_RESET);
    CDC_Print(buffer);

    for(uint32_t i = 0; i < ROBOTICS_NUM_JOINTS; i++)
    {
        const Robotics_DH_Param_t* dh = &Robotics_DefaultDHTable[i];
        uint32_t pos  = Menu_GaugeIndex(arm.q[i], dh);
        uint32_t zero = Menu_GaugeIndex(0.0f, dh);
        char* p = gauge;

        for(uint32_t k = 0; k < GAUGE_WIDTH; k++)
        {
            const char* glyph = (k == pos) ? "●" : (k == zero) ? "┼" : "─";
            size_t n = strlen(glyph);
            memcpy(p, glyph, n);
            p += n;
        }
        *p = '\0';

        snprintf(buffer, sizeof(buffer), "  %sQ%lu%s %s  %s%7.1f°%s   %s%4.0f %s%s%s %.0f%s\r\n",
                 OPTION_COLOR, i + 1U, COLOR_RESET, JointName[i],
                 VALUE_COLOR, ROBOTICS_RAD2DEG(arm.q[i]), COLOR_RESET,
                 COLOR_DIM, ROBOTICS_RAD2DEG(dh->minAngle), COLOR_RESET, gauge, COLOR_DIM,
                 ROBOTICS_RAD2DEG(dh->maxAngle), COLOR_RESET);
        CDC_Print(buffer);
    }

    snprintf(buffer, sizeof(buffer), "\r\n  %sEfector final%s\r\n"
             "    X %s%9.2f%s mm      Pitch %s%7.2f°%s\r\n",
             INFO_COLOR, COLOR_RESET,
             VALUE_COLOR, arm.pose.x, COLOR_RESET, VALUE_COLOR, ROBOTICS_RAD2DEG(arm.pose.pitch), COLOR_RESET);
    CDC_Print(buffer);

    snprintf(buffer, sizeof(buffer), "    Y %s%9.2f%s mm      Yaw   %s%7.2f°%s\r\n"
             "    Z %s%9.2f%s mm\r\n",
             VALUE_COLOR, arm.pose.y, COLOR_RESET, VALUE_COLOR, ROBOTICS_RAD2DEG(arm.pose.yaw), COLOR_RESET,
             VALUE_COLOR, arm.pose.z, COLOR_RESET);
    CDC_Print(buffer);
}

/**
 * @brief Indica cómo volver al menú principal.
 */
static void Menu_PrintReturn(void)
{
    snprintf(buffer, sizeof(buffer), "\r\n  %s↵ ENTER para volver al menú%s", COLOR_DIM, COLOR_RESET);
    CDC_Print(buffer);
}

/**
 * @brief Solicita el ángulo de la articulación angleStep.
 */
static void Menu_PromptAngle(void)
{
    const Robotics_DH_Param_t* dh = &Robotics_DefaultDHTable[angleStep];

    snprintf(buffer, sizeof(buffer), "  %s[%lu/%u]%s Q%lu %s %s(%.0f° a %.0f°)%s %s›%s %s",
             COLOR_BRIGHT_MAGENTA, angleStep + 1U, ROBOTICS_NUM_JOINTS, COLOR_RESET,
             angleStep + 1U, JointName[angleStep],
             COLOR_DIM, ROBOTICS_RAD2DEG(dh->minAngle), ROBOTICS_RAD2DEG(dh->maxAngle), COLOR_RESET,
             COLOR_BRIGHT_MAGENTA, COLOR_RESET, INPUT_COLOR);
    CDC_Print(buffer);
}

/**
 * @brief Mueve el brazo a q, espera a que termine y reporta la pose alcanzada.
 *
 * @note Bloquea durante el movimiento; los bytes recibidos se siguen
 * encolando en el buffer circular.
 *
 * @param q Ángulos articulares objetivo [rad].
 * @param title Título para la pantalla OLED.
 */
static void Menu_MoveBlocking(const float q[ROBOTICS_NUM_JOINTS], const char* title)
{
    if(Arm_MoveTo(q, SERVO_MOVE_DURATION) != ROBOTICS_OK)
    {
        CDC_PrintColor(ERROR_COLOR, "✗ No se pudo ejecutar el movimiento (fuera de límites)");
        Menu_PrintReturn();
        return;
    }

    CDC_PrintColor(INFO_COLOR, "ℹ Moviendo robot...");
    Display_ShowArm(title);

    while(!Arm_Update())
    {
    }

    CDC_PrintColor(SUCCESS_COLOR, "✓ Movimiento completado\r\n");
    Menu_PrintArmState();
    Menu_PrintReturn();
}

/**
 * @brief Comanda la siguiente pose de la secuencia de prueba y redibuja el
 * panel de la secuencia (sin desplazar la terminal).
 */
static void Menu_SequenceStep(void)
{
    char title[20];
    char progress[TEST_SEQUENCE_LENGTH * 4U + 1U];  // "● " = 3 + 1 bytes por pose
    char* p = progress;

    if(Arm_MoveTo(TestSequence[sequenceStep], SERVO_MOVE_DURATION) != ROBOTICS_OK)
    {
        Error_Handler();
    }

    for(uint32_t k = 0; k < TEST_SEQUENCE_LENGTH; k++)
    {
        memcpy(p, (k <= sequenceStep) ? "● " : "○ ", 4);
        p += 4;
    }
    *p = '\0';

    Menu_ShowHeader("SECUENCIA AUTOMÁTICA");
    snprintf(buffer, sizeof(buffer), "  Pose %s%lu/%u%s   %s%s%s\r\n\r\n",
             VALUE_COLOR, sequenceStep + 1U, TEST_SEQUENCE_LENGTH, COLOR_RESET, SUCCESS_COLOR, progress, COLOR_RESET);
    CDC_Print(buffer);
    Menu_PrintArmState();

    snprintf(buffer, sizeof(buffer), "\r\n  %s↵ ENTER para detener%s", COLOR_DIM, COLOR_RESET);
    CDC_Print(buffer);

    snprintf(title, sizeof(title), "SECUENCIA %lu/%u", sequenceStep + 1U, TEST_SEQUENCE_LENGTH);
    Display_ShowArm(title);

    sequenceStep = (sequenceStep + 1U) % TEST_SEQUENCE_LENGTH;
    lastMoveTick = HAL_GetTick();
}

/**
 * @brief Edición de línea: eco, retroceso y despacho al pulsar ENTER.
 *
 * @details Un ENTER vacío redibuja el menú principal, cancela la captura de
 * ángulos o detiene la secuencia.
 *
 * @param c Byte recibido por USB-CDC.
 */
static void Menu_HandleByte(uint8_t c)
{
    static uint8_t prev = 0;
    char echo[2] = { (char)c, '\0' };

    // CRLF cuenta como un solo ENTER
    if(c == '\n' && prev == '\r')
    {
        prev = c;
        return;
    }
    prev = c;

    if(c == '\r' || c == '\n')
    {
        CDC_Print(COLOR_RESET "\r\n");
        lineBuf[lineLen] = '\0';

        if(lineLen > 0U || menuState != MENU_MAIN)
        {
            lineLen = 0;
            Menu_ProcessLine(lineBuf);
        }
        else
        {
            Menu_ShowMain();
        }
    }
    else if(c == '\b' || c == 127U)
    {
        if(lineLen > 0U)
        {
            lineLen--;
            CDC_Print("\b \b");
        }
    }
    else if(c >= ' ' && lineLen < (LINE_BUFFER_SIZE - 1U))
    {
        lineBuf[lineLen++] = (char)c;
        CDC_Print(echo);
    }
}

/**
 * @brief Procesa una línea completa según el estado del menú.
 *
 * @param line Línea recibida (terminada en '\0').
 */
static void Menu_ProcessLine(char* line)
{
    char option = (line[0] != '\0' && line[1] == '\0') ? line[0] : '\0';

    switch(menuState)
    {
    case MENU_MAIN:
        switch(option)
        {
        case '1':
            Menu_ShowHeader("SECUENCIA AUTOMÁTICA");
            CDC_PrintColor(INFO_COLOR, "ℹ Iniciando secuencia...");
            sequenceStep = 0;
            lastMoveTick = HAL_GetTick() - DELAY_TEST_SEQUENCE;    // Primer paso inmediato
            sequenceActive = true;     // El lazo principal la hace avanzar
            menuState = MENU_SEQUENCE;
            break;

        case '2':
            Menu_ShowHeader("INGRESO MANUAL DE ÁNGULOS");
            snprintf(buffer, sizeof(buffer), "  %sGrados; acepta '.' o ','. ENTER vacío cancela.%s\r\n\r\n",
                     COLOR_DIM, COLOR_RESET);
            CDC_Print(buffer);
            angleStep = 0;
            menuState = MENU_ANGLES;
            Menu_PromptAngle();
            break;

        case '3':
            Menu_ShowHeader("POSICIÓN HOME");
            Menu_MoveBlocking(HomePose, "HOME");
            break;

        case '4':
            Menu_ShowPosition();
            break;

        default:
            CDC_PrintColor(ERROR_COLOR, "✗ Opción inválida");
            Menu_ShowPrompt();
            break;
        }
        break;

    case MENU_ANGLES:
        if(line[0] == '\0')
        {
            menuState = MENU_MAIN;
            Menu_ShowMain();
        }
        else
        {
            Menu_ProcessAngle(line);
        }
        break;

    case MENU_SEQUENCE:
        // Cualquier línea (incluso vacía) detiene la secuencia
        menuState = MENU_MAIN;
        sequenceActive = false;
        Display_ShowArm("DETENIDO");
        Menu_ShowMain();
        break;
    }
}

/**
 * @brief Valida el ángulo ingresado [°] para la articulación angleStep y,
 * tras la cuarta articulación, mueve el brazo.
 *
 * @details Acepta '.' o ',' como separador decimal. El límite se verifica en
 * radianes igual que Robotics_CheckJointLimits(), de modo que todo valor
 * aceptado aquí también lo acepta la librería.
 *
 * @param line Línea recibida (terminada en '\0').
 */
static void Menu_ProcessAngle(char* line)
{
    const Robotics_DH_Param_t* dh = &Robotics_DefaultDHTable[angleStep];
    char* end;

    for(char* p = line; *p != '\0'; p++)
    {
        if(*p == ',')
        {
            *p = '.';
        }
    }

    float q = ROBOTICS_DEG2RAD(strtof(line, &end));
    while(*end == ' ')
    {
        end++;
    }

    // La comparación negada también rechaza NaN
    if(end == line || *end != '\0' || !(q >= dh->minAngle && q <= dh->maxAngle))
    {
        CDC_PrintColor(ERROR_COLOR, "✗ Valor inválido o fuera de rango");
        Menu_PromptAngle();
        return;
    }

    angleInput[angleStep++] = q;
    if(angleStep < ROBOTICS_NUM_JOINTS)
    {
        Menu_PromptAngle();
        return;
    }

    menuState = MENU_MAIN;
    Menu_ShowHeader("MOVIMIENTO MANUAL");
    Menu_MoveBlocking(angleInput, "MANUAL");
}

#endif /* KR10_TERMINAL_MENU */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	// LED parpadeando = sistema detenido. Sin interrupciones no hay HAL_Delay(),
	// por eso el retardo es un lazo; MX_GPIO_Init() cubre fallos previos a su llamada
	__disable_irq();
	MX_GPIO_Init();
	while (1)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		for(volatile uint32_t i = 0; i < 2000000U; i++)
		{
		}
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
