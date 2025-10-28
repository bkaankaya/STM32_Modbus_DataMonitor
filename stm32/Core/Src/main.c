/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Modbus RTU Slave - Temperature & Vref Monitoring (POLLING MODE)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODBUS_SLAVE_ID        1
#define MODBUS_FUNC_READ_HOLD  0x03
#define MODBUS_BUFFER_SIZE     64
#define MODBUS_TIMEOUT         100  // ms
#define MODBUS_FRAME_TIMEOUT   10   // ms (3.5 character time)
#define DEBOUNCE_DELAY 100 // ms

// Register adresleri
#define REG_TEMP_HIGH          0
#define REG_TEMP_LOW           1
#define REG_VREF_HIGH          2
#define REG_VREF_LOW           3
#define REG_BUTTON_STATE  4   // Buton durumunu tutacak register
#define TOTAL_REGISTERS   5


// LED Pinleri
#define LED_TX_PORT    GPIOD
#define LED_TX_PIN     GPIO_PIN_13  // Turuncu LED (LD3)
#define LED_RX_PORT    GPIOD
#define LED_RX_PIN     GPIO_PIN_15  // Mavi LED (LD6)
#define LED_BLINK_TIME 500          // ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
GPIO_PinState btn_State;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
volatile uint16_t adc_raw_temp = 0;
volatile uint16_t adc_raw_vref = 0;
volatile float vref_voltage = 0.0f;
volatile float temp_celsius = 0.0f;

// Debounce değişkenleri
uint32_t button_high_start = 0;
uint8_t button_debounce_active = 0;  // 0: beklemede, 1: HIGH takibi aktif
uint8_t button_stable_state = 0;     // 0: basılmamış, 1: basılı


// Modbus değişkenleri
uint8_t modbus_tx_buffer[MODBUS_BUFFER_SIZE];
uint8_t modbus_rx_buffer[MODBUS_BUFFER_SIZE];
uint16_t modbus_rx_index = 0;
uint32_t last_rx_time = 0;

uint16_t holding_registers[TOTAL_REGISTERS] = {0};

// LED timing
uint32_t led_tx_time = 0;
uint32_t led_rx_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void ADC_Read_All_Channels(void);
void Update_Holding_Registers(void);
uint16_t Calculate_CRC16(uint8_t *buffer, uint8_t length);
void Modbus_Process_Request(void);
void Float_To_Registers(float value, uint16_t *high_reg, uint16_t *low_reg);
void LED_TX_On(void);
void LED_RX_On(void);
void LED_Update(void);
uint8_t UART_Receive_Byte(uint8_t *data, uint32_t timeout);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  TX LED'ini yak
  * @retval None
  */
void LED_TX_On(void)
{
    HAL_GPIO_WritePin(LED_TX_PORT, LED_TX_PIN, GPIO_PIN_SET);
    led_tx_time = HAL_GetTick();
}

/**
  * @brief  RX LED'ini yak
  * @retval None
  */
void LED_RX_On(void)
{
    HAL_GPIO_WritePin(LED_RX_PORT, LED_RX_PIN, GPIO_PIN_SET);
    led_rx_time = HAL_GetTick(); // Mavi led en son ne zaman yakılı bunu takip eder daha sonra satır 136 daki kodda bu bilgi kullanılır
}

/**
  * @brief  LED'leri güncelle (süre dolduysa söndür)
  * @retval None
  */
void LED_Update(void)
{
    uint32_t current_time = HAL_GetTick();

    if (led_tx_time > 0 && (current_time - led_tx_time) >= LED_BLINK_TIME)
    {
        HAL_GPIO_WritePin(LED_TX_PORT, LED_TX_PIN, GPIO_PIN_RESET);
        led_tx_time = 0;
    }

    if (led_rx_time > 0 && (current_time - led_rx_time) >= LED_BLINK_TIME)
    {
        HAL_GPIO_WritePin(LED_RX_PORT, LED_RX_PIN, GPIO_PIN_RESET);
        led_rx_time = 0;
    }
}

/**
  * @brief  UART'tan polling ile tek byte oku
  * @param  data: Okunan veri
  * @param  timeout: Timeout süresi (ms)
  * @retval 1: Başarılı, 0: Timeout
  */
uint8_t UART_Receive_Byte(uint8_t *data, uint32_t timeout)
{
    if (HAL_UART_Receive(&huart4, data, 1, timeout) == HAL_OK)
    {
        LED_RX_On();  // Her RX'te mavi LED yanar
        return 1;
    }
    return 0;
}

/**
  * @brief  ADC'den iki channel'ı polling ile oku
  * @retval None
  */
void ADC_Read_All_Channels(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    HAL_ADC_Stop(&hadc1);

    // Channel 1: Temperature Sensor
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK)
    {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        {
            adc_raw_temp = HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }

    HAL_Delay(2);

    // Channel 2: Vref Internal
    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK)
    {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        {
            adc_raw_vref = HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }

    if (adc_raw_vref > 100 && adc_raw_vref < 4095)
    {
        vref_voltage = (1.21f * 4095.0f) / (float)adc_raw_vref;
        float temp_voltage = ((float)adc_raw_temp * vref_voltage) / 4095.0f;
        temp_celsius = ((temp_voltage - 0.76f) / 0.0025f) + 25.0f;
    }
}

/**
  * @brief  Buton debouncing algoritması (100ms boyunca HIGH kalırsa geçerli)
  * @retval None
  */
void Button_Debounce_Update(void)
{
    GPIO_PinState raw_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    uint32_t current_time = HAL_GetTick();

    if (raw_state == GPIO_PIN_SET)
    {
        // HIGH tespit edildi
        if (!button_debounce_active)
        {
            // İlk kez HIGH'a geçti, zaman sayacını başlat
            button_debounce_active = 1;
            button_high_start = current_time;
        }
        else
        {
            // Zaten HIGH takibi aktif, süre doldu mu kontrol et
            if ((current_time - button_high_start) >= DEBOUNCE_DELAY)
            {
                button_stable_state = 1; // 100ms boyunca HIGH kaldı
            }
        }
    }
    else
    {
        // LOW tespit edildi → sıfırla
        button_debounce_active = 0;
        button_high_start = 0;
        button_stable_state = 0;
    }

    // Holding register'ı güncelle
    holding_registers[REG_BUTTON_STATE] = button_stable_state;
    // Butonun debounce edilmiş hali (button_stable_state) Modbus holding register dizisine yazılıyor.

}


/**
  * @brief  Float'ı 2 adet 16-bit register'a dönüştür
  * @param  value: Float değer
  * @param  high_reg: Yüksek word (MSB)
  * @param  low_reg: Düşük word (LSB)
  * @retval None
  */
void Float_To_Registers(float value, uint16_t *high_reg, uint16_t *low_reg)
{
    union {
        float f;
        uint32_t u;
    } data;

    data.f = value;
    *high_reg = (uint16_t)((data.u >> 16) & 0xFFFF);
    *low_reg = (uint16_t)(data.u & 0xFFFF);
}

/**
  * @brief  Holding register'ları güncelle
  * @retval None
  */
void Update_Holding_Registers(void)
{
    Float_To_Registers(temp_celsius,
                      &holding_registers[REG_TEMP_HIGH], // REG_TEMP_HIGH (sıcaklık float üst 16 bit)
                      &holding_registers[REG_TEMP_LOW]); // REG_TEMP_LOW  (sıcaklık float alt 16 bit)

    Float_To_Registers(vref_voltage,
                      &holding_registers[REG_VREF_HIGH], // REG_VREF_HIGH (vref float üst 16 bit)
                      &holding_registers[REG_VREF_LOW]); // REG_VREF_LOW  (vref float alt 16 bit)
}

/**
  * @brief  CRC-16 hesaplama (Modbus standardı)
  * @param  buffer: Veri buffer'ı
  * @param  length: Veri uzunluğu
  * @retval CRC-16 değeri
  */
uint16_t Calculate_CRC16(uint8_t *buffer, uint8_t length)
{
    uint16_t crc = 0xFFFF;
    uint8_t i, j;

    for (i = 0; i < length; i++)
    {
        crc ^= (uint16_t)buffer[i]; // Bit düzeyinde XOR ile işlem yapılır

        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/**
  * @brief  Modbus isteğini işle ve yanıt gönder
  * @retval None
  */
void Modbus_Process_Request(void)
{
    if (modbus_rx_index < 8)
    {
        modbus_rx_index = 0;
        return;
    }

    // CRC kontrolü
    uint16_t received_crc = ((uint16_t)modbus_rx_buffer[modbus_rx_index - 1] << 8) |
                            modbus_rx_buffer[modbus_rx_index - 2];
    uint16_t calculated_crc = Calculate_CRC16(modbus_rx_buffer, modbus_rx_index - 2);

    if (received_crc != calculated_crc)
    {
        modbus_rx_index = 0;
        return;
    }

    // Slave ID kontrolü
    if (modbus_rx_buffer[0] != MODBUS_SLAVE_ID)
    {// Eğer gelen mesajın ID'si benim ID'm değilse cevap verme
        modbus_rx_index = 0;
        return;
    }

    uint8_t function_code = modbus_rx_buffer[1];

    if (function_code == MODBUS_FUNC_READ_HOLD)
    {
        uint16_t start_address = ((uint16_t)modbus_rx_buffer[2] << 8) | modbus_rx_buffer[3];
        uint16_t num_registers = ((uint16_t)modbus_rx_buffer[4] << 8) | modbus_rx_buffer[5];

        // Adres kontrolü
        if (start_address + num_registers > TOTAL_REGISTERS || num_registers == 0)
        {
            // Exception response
            modbus_tx_buffer[0] = MODBUS_SLAVE_ID;
            modbus_tx_buffer[1] = function_code | 0x80;
            modbus_tx_buffer[2] = 0x02;

            uint16_t crc = Calculate_CRC16(modbus_tx_buffer, 3);
            modbus_tx_buffer[3] = (uint8_t)(crc & 0xFF);
            modbus_tx_buffer[4] = (uint8_t)(crc >> 8);

            HAL_UART_Transmit(&huart4, modbus_tx_buffer, 5, MODBUS_TIMEOUT);
            LED_TX_On();  // TX LED yak

            modbus_rx_index = 0;
            return;
        }

        // Normal response
        modbus_tx_buffer[0] = MODBUS_SLAVE_ID;
        modbus_tx_buffer[1] = function_code;
        modbus_tx_buffer[2] = num_registers * 2;

        uint8_t index = 3;
        for (uint16_t i = 0; i < num_registers; i++)
        {
            uint16_t reg_value = holding_registers[start_address + i];
            modbus_tx_buffer[index++] = (uint8_t)(reg_value >> 8);
            modbus_tx_buffer[index++] = (uint8_t)(reg_value & 0xFF);
        }

        uint16_t crc = Calculate_CRC16(modbus_tx_buffer, index);
        modbus_tx_buffer[index++] = (uint8_t)(crc & 0xFF);
        modbus_tx_buffer[index++] = (uint8_t)(crc >> 8);

        HAL_UART_Transmit(&huart4, modbus_tx_buffer, index, MODBUS_TIMEOUT);
        LED_TX_On();  // TX LED yak
    }
    else
    {
        // Desteklenmeyen fonksiyon
        modbus_tx_buffer[0] = MODBUS_SLAVE_ID;
        modbus_tx_buffer[1] = function_code | 0x80;
        modbus_tx_buffer[2] = 0x01;

        uint16_t crc = Calculate_CRC16(modbus_tx_buffer, 3);
        modbus_tx_buffer[3] = (uint8_t)(crc & 0xFF);
        modbus_tx_buffer[4] = (uint8_t)(crc >> 8);

        HAL_UART_Transmit(&huart4, modbus_tx_buffer, 5, MODBUS_TIMEOUT);
        LED_TX_On();  // TX LED yak
    }

    modbus_rx_index = 0;
}

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
  MX_ADC1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  // İlk ADC okuması
  ADC_Read_All_Channels();
  Update_Holding_Registers();

  // LED'leri söndür
  HAL_GPIO_WritePin(LED_TX_PORT, LED_TX_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RX_PORT, LED_RX_PIN, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_adc_update = 0;
  uint8_t rx_byte = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  btn_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);


	  if(btn_State == GPIO_PIN_SET){
		  holding_registers[REG_BUTTON_STATE] = 1;
		 // HAL_DELAY(100); gelse mi ki acaba???
	  }
	  else {
		  holding_registers[REG_BUTTON_STATE] = 0;
	  }

    // LED'leri güncelle
    LED_Update();

    // Her 500ms'de bir ADC oku
    if (HAL_GetTick() - last_adc_update >= 500)
    {
        ADC_Read_All_Channels();
        Update_Holding_Registers();
        last_adc_update = HAL_GetTick();
    }

    // Polling ile UART oku (timeout = 1ms)
    if (UART_Receive_Byte(&rx_byte, 1))
    {
        modbus_rx_buffer[modbus_rx_index++] = rx_byte;
        last_rx_time = HAL_GetTick();

        // Buffer taşmasını önle
        if (modbus_rx_index >= MODBUS_BUFFER_SIZE)
        {
            modbus_rx_index = 0;
        }
    }

    // Frame timeout kontrolü (3.5 character time)
    if (modbus_rx_index > 0 && (HAL_GetTick() - last_rx_time) >= MODBUS_FRAME_TIMEOUT)
    {
        // Frame tamamlandı, işle
        Modbus_Process_Request();
        modbus_rx_index = 0;
    }
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
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
