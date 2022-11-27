/**
  ******************************************************************************
  * @file    Wifi/WiFi_HTTP_Server/src/main.c
  * @author  MCD Application Team
  * @brief   This file provides main program functions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"
#include "math.h"
#include "stdio.h"
#include "stm32l4xx_hal_adc.h"
#include "stm32l4xx_hal_adc_ex.h"
#include "stm32l4xx_hal.h"

#ifdef __ICCARM__
#include <LowLevelIOInterface.h>
#endif

/* Private defines -----------------------------------------------------------*/
#define PORT           80

#define TERMINAL_USE


#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000
#define SOCKET                 0


#ifdef  TERMINAL_USE
#define LOG(a) printf a
#else
#define LOG(a)
#endif

#define SSID_SIZE     100
#define PASSWORD_SIZE 100
#define USER_CONF_MAGIC                 0x0123456789ABCDEFuLL

/* Private typedef------------------------------------------------------------*/

typedef struct {
  char ssid[SSID_SIZE];
  char password[PASSWORD_SIZE];
  uint8_t security;
} wifi_config_t;

typedef struct {
  uint64_t      wifi_config_magic;        /**< The USER_CONF_MAGIC magic word signals that the wifi config
                                               (wifi_config_t) is present in Flash. */
  wifi_config_t wifi_config;
} user_config_t;


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if defined (TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#endif /* TERMINAL_USE */
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
int sin_samples[50];
int sin2_samples[40];
int sin3_samples[25];
uint8_t temp;
int time_p2 = 0;

/* configuration storage in Flash memory */
#if defined(__ICCARM__)
/* IAR */
extern void __ICFEDIT_region_FIXED_LOC_start__;
const  user_config_t    *lUserConfigPtr = &__ICFEDIT_region_FIXED_LOC_start__;
#elif defined(__CC_ARM)
/* Keil / armcc */
user_config_t __uninited_region_start__ __attribute__((section("UNINIT_FIXED_LOC"), zero_init));
const  user_config_t    *lUserConfigPtr = &__uninited_region_start__;
#elif defined(__GNUC__)
/* GNU compiler */
user_config_t __uninited_region_start__ __attribute__((section("UNINIT_FIXED_LOC")));
const  user_config_t    *lUserConfigPtr = &__uninited_region_start__;
#endif

static volatile uint8_t button_flag = 0;
static user_config_t user_config;

static  uint8_t http[1024];
static  uint8_t  IP_Addr[4];
static  int     LedState = 0; 

/* Private function prototypes -----------------------------------------------*/
#if defined (TERMINAL_USE)
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
#endif /* TERMINAL_USE */

static void SystemClock_Config(void);
static WIFI_Status_t SendWebPage(uint8_t ledIsOn, uint8_t temperature,float voltage);
static int wifi_server(void);
static int wifi_start(void);
static int wifi_connect(void);
static bool WebServerProcess(void);
static void Button_ISR(void);
static void Button_Reset(void);
static uint8_t Button_WaitForPush(uint32_t delay);
static void MX_DAC1_Init(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_DMA_Init(void);

/* Private functions ---------------------------------------------------------*/
void sin_generate(){
	for(int i=0; i<50; i++){
		sin_samples[i] = (int)(2000*(1+arm_sin_f32((2*3.1415926/50)*i)));
		//TIM frequency is 60MHZ, counter period is 1200, so the time step for a single sample
		//time is 0.02ms. If we take 50 sample times for one period, the period will be 1ms, frequency
		//will be 1000HZ
	}
}

void sin2_generate(){
	for(int i=0; i<40; i++){
			sin2_samples[i] = (int)(2000*(1+arm_sin_f32((2*3.1415926/40)*i)));
			//TIM frequency is 60MHZ, counter period is 1200, so the time step for a single sample
			//time is 0.02ms. If we take 40 sample times for one period, the period will be 0.8ms, frequency
			//will be 1250HZ
		}
}

void sin3_generate(){
	for(int i=0; i<25; i++){
			sin3_samples[i] = (int)(2000*(1+arm_sin_f32((2*3.1415926/25)*i)));
			//TIM frequency is 60MHZ, counter period is 1200, so the time step for a single sample
			//time is 0.02ms. If we take 25 sample times for one period, the period will be 0.5ms, frequency
			//will be 2000HZ
		}
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	if(htim->Instance == TIM4){
//		WebServerProcess();
//	}
//}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
uint16_t *TS_CAL1 = ((uint16_t*) ((uint32_t) 0x1FFF75A8));
uint16_t *TS_CAL2 = ((uint16_t*) ((uint32_t) 0x1FFF75CA));
uint16_t *VREFINT = ((uint16_t*) ((uint32_t) 0x1FFF75AA));
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
ADC_ChannelConfTypeDef sConfig1;
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  //MX_ADC1_Init();
 // MX_TIM4_Init();
  /* Configure LED2 */
  BSP_LED_Init(LED2);
  /* USER push button is used to ask if reconfiguration is needed */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	sConfig1.Channel = ADC_CHANNEL_VREFINT;
	sConfig1.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	sConfig1.SingleDiff = ADC_SINGLE_ENDED;
	sConfig1.OffsetNumber = ADC_OFFSET_NONE;
	sConfig1.Rank = 1;
	//HAL_ADC_Init(&hadc1);
  /* WIFI Web Server demonstration */
#if defined (TERMINAL_USE)
  /* Initialize all configured peripherals */
  hDiscoUart.Instance = DISCOVERY_COM1;
  hDiscoUart.Init.BaudRate = 115200;
  hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
  hDiscoUart.Init.StopBits = UART_STOPBITS_1;
  hDiscoUart.Init.Parity = UART_PARITY_NONE;
  hDiscoUart.Init.Mode = UART_MODE_TX_RX;
  hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
  hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;


  BSP_COM_Init(COM1, &hDiscoUart);
  BSP_TSENSOR_Init();

  printf("\n****** WIFI Web Server demonstration ******\n\n");

#endif /* TERMINAL_USE */
  HAL_TIM_Base_Start(&htim2);
  sin_generate();
  sin2_generate();
  sin3_generate();
  HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
  wifi_server();
  uint32_t i = 0;
  while (1)
  {
  i++;
  HAL_Delay(1000);
  }
}

/**
  * @brief  Send HTML page
  * @param  None
  * @retval None
  */


static int wifi_start(void)
{
  uint8_t  MAC_Addr[6];

 /*Initialize and use WIFI module */
  if(WIFI_Init() ==  WIFI_STATUS_OK)
  {
    printf("eS-WiFi Initialized.\r\n");
    if(WIFI_GetMAC_Address(MAC_Addr) == WIFI_STATUS_OK)
    {
      LOG(("eS-WiFi module MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\r\n",
               MAC_Addr[0],
               MAC_Addr[1],
               MAC_Addr[2],
               MAC_Addr[3],
               MAC_Addr[4],
               MAC_Addr[5]));
    }
    else
    {
      LOG(("> ERROR : CANNOT get MAC address\r\n"));
      return -1;
    }
  }
  else
  {
    return -1;
  }
  return 0;
}



int wifi_connect(void)
{
  wifi_start();
  
  memset(&user_config, 0, sizeof(user_config));
  memcpy(&user_config, lUserConfigPtr, sizeof(user_config));
  if (user_config.wifi_config_magic == USER_CONF_MAGIC)
  {
    /* WiFi configuration is already in Flash. Ask if we want to change it */
    printf("Already configured SSID: %s security: %d\r\n",
           user_config.wifi_config.ssid, user_config.wifi_config.security);
    printf("Press board User button (blue) within 5 seconds if you want to change the configuration.\r\n");
    Button_Reset();
    if (Button_WaitForPush(5000))
    {
      /* we want to change the configuration already stored in Flash memory */
      memset(&user_config, 0, sizeof(user_config));
    }
  }

  if (user_config.wifi_config_magic != USER_CONF_MAGIC)
  {
    printf("\r\nEnter WiFi SSID : ");
    //gets(user_config.wifi_config.ssid);
    scanf("%s", &user_config.wifi_config.ssid);
    //char test_ssid[SSID_SIZE] = "BELL222-V";
//    user_config.wifi_config.ssid = test_ssid;


    printf("\r\nWe receive the SSID : ");
    LOG(("\r\nYou have entered %s as SSID.\r\n", user_config.wifi_config.ssid));

    char c;
    do
    {
        printf("\rEnter Security Mode (0 - Open, 1 - WEP, 2 - WPA, 3 - WPA2): ");
        c = getchar();
    }
    while ( (c < '0')  || (c > '3'));
    user_config.wifi_config.security = c - '0';
    LOG(("\r\nYou have entered %d as the security mode.\r\n", user_config.wifi_config.security));

    if (user_config.wifi_config.security != 0)
    {
      printf("\r\nEnter WiFi password : ");
      //gets(user_config.wifi_config.password);
      scanf("%s", &user_config.wifi_config.password);
    }
    user_config.wifi_config_magic = USER_CONF_MAGIC;
    FLASH_Erase_Size((uint32_t)lUserConfigPtr, sizeof(user_config));
    FLASH_Write((uint32_t)lUserConfigPtr, (uint32_t*)&user_config, sizeof(user_config));
  }

  printf("\r\nConnecting to %s\r\n", user_config.wifi_config.ssid);
  WIFI_Ecn_t security;
  switch (user_config.wifi_config.security)
  {
    case 0:
      security = WIFI_ECN_OPEN;
      break;
    case 1:
      security = WIFI_ECN_WEP;
      break;
    case 2:
      security =  WIFI_ECN_WPA_PSK;
      break;
    case 3:
    default:
      security =  WIFI_ECN_WPA2_PSK;
      break;
  }
  if (WIFI_Connect(user_config.wifi_config.ssid, user_config.wifi_config.password, security) == WIFI_STATUS_OK)
  {
    if(WIFI_GetIP_Address(IP_Addr) == WIFI_STATUS_OK)
    {
      LOG(("eS-WiFi module connected: got IP Address : %d.%d.%d.%d\r\n",
               IP_Addr[0],
               IP_Addr[1],
               IP_Addr[2],
               IP_Addr[3]));
    }
    else
    {
      LOG((" ERROR : es-wifi module CANNOT get IP address\r\n"));
      return -1;
    }
  }
  else
  {
     LOG(("ERROR : es-wifi module NOT connected\r\n"));
     return -1;
  }
  return 0;
}

int wifi_server(void)
{
  bool StopServer = false;

  LOG(("\r\nRunning HTML Server test\r\n"));
  if (wifi_connect()!=0) return -1;


  if (WIFI_STATUS_OK!=WIFI_StartServer(SOCKET, WIFI_TCP_PROTOCOL, 1, "", PORT))
  {
    LOG(("ERROR: Cannot start server.\r\n"));
  }

  LOG(("Server is running and waiting for an HTTP  Client connection to %d.%d.%d.%d\r\n",IP_Addr[0],IP_Addr[1],IP_Addr[2],IP_Addr[3]));

  do
  {
    uint8_t RemoteIP[4];
    uint16_t RemotePort;

    LOG(("Waiting connection to http://%d.%d.%d.%d\r\n",IP_Addr[0],IP_Addr[1],IP_Addr[2],IP_Addr[3]));
    while (WIFI_STATUS_OK != WIFI_WaitServerConnection(SOCKET,1000,RemoteIP,&RemotePort))
    {
        LOG(("."));
    }

    LOG(("\r\nClient connected %d.%d.%d.%d:%d\r\n",RemoteIP[0],RemoteIP[1],RemoteIP[2],RemoteIP[3],RemotePort));

    StopServer = WebServerProcess();

    if (WIFI_CloseServerConnection(SOCKET) != WIFI_STATUS_OK)
    {
      LOG(("ERROR: failed to close current Server connection\r\n"));
      return -1;
    }
    //HAL_Delay(1000);
  }
  while(StopServer == false);

  if (WIFI_STATUS_OK!=WIFI_StopServer(SOCKET))
  {
    LOG(("ERROR: Cannot stop server.\r\n"));
  }

  LOG(("Server is stop\r\n"));
  return 0;
}


static bool WebServerProcess(void)
{
	HAL_TIM_Base_Start(&htim2);
//	HAL_TIM_Base_Start_IT(&htim4);
	sin_generate();
	sin2_generate();
	sin3_generate();
  //uint8_t temp;
  uint16_t  respLen;
  static   uint8_t resp[1024];
  bool    stopserver=false;
  //SendWebPage(LedState, temp);
  float vol = 0, temperature = 0.0;
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_ConfigChannel(&hadc1, &sConfig1);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  vol = 3.0f * (*VREFINT) / HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  //LedState = 0;
  if (WIFI_STATUS_OK == WIFI_ReceiveData(SOCKET, resp, 1000, &respLen, WIFI_READ_TIMEOUT))
  {
   LOG(("get %d byte from server\r\n",respLen));

   if( respLen > 0)
   {
      if(strstr((char *)resp, "GET")) /* GET: put web page */
      {
        temp = (int) BSP_TSENSOR_ReadTemp();
        if (temp >32 && LedState == 0){
			if(temp > 32 && temp < 39){
				HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
				HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,sin_samples,50,DAC_ALIGN_12B_R);
				LedState = 0;
			}
			else if(temp > 38 && temp < 46){
				HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
				HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,sin2_samples,40,DAC_ALIGN_12B_R);
				LedState = 0;
			}
			else if(temp > 45){
				HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
				HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,sin3_samples,25,DAC_ALIGN_12B_R);
				LedState = 0;
			}
        }
        else{
        	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        }
        if(SendWebPage(LedState, temp,vol) != WIFI_STATUS_OK)
        {
          LOG(("> ERROR : Cannot send web page\r\n"));
        }
        else
        {
          LOG(("Send page after  GET command\r\n"));
        }
       }
       else if(strstr((char *)resp, "POST"))/* POST: received info */
       {
         LOG(("Post request\r\n"));

         if(strstr((char *)resp, "radio"))
         {
           if(strstr((char *)resp, "radio=0"))
           {
             LedState = 0;
             BSP_LED_On(LED2);
           }
           else if(strstr((char *)resp, "radio=1"))
           {
             LedState = 1;
             BSP_LED_Off(LED2);
           }
           temp = (int) BSP_TSENSOR_ReadTemp();
         }
         if(strstr((char *)resp, "stop_server"))
         {
           if(strstr((char *)resp, "stop_server=0"))
           {
             stopserver = false;
           }
           else if(strstr((char *)resp, "stop_server=1"))
           {
             stopserver = true;
           }
         }
         temp = (int) BSP_TSENSOR_ReadTemp();
         if(SendWebPage(LedState, temp,vol) != WIFI_STATUS_OK)
         {
           LOG(("> ERROR : Cannot send web page\r\n"));
         }
         else
         {
           LOG(("Send Page after POST command\r\n"));
         }
       }
     }
  }
  else
  {
    LOG(("Client close connection\r\n"));
  }
  return stopserver;

 }

/**
  * @brief  Send HTML page
  * @param  None
  * @retval None
  */
static WIFI_Status_t SendWebPage(uint8_t ledIsOn, uint8_t temperature,float voltage)
{
  uint8_t  temp[50];
  uint16_t SentDataLength;
  WIFI_Status_t ret;

  /* construct web page content */
  strcpy((char *)http, (char *)"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n");
  strcat((char *)http, (char *)"<html>\r\n<body>\r\n");
  strcat((char *)http, (char *)"<title>STM32 Web Server</title>\r\n");
  strcat((char *)http, (char *)"<h2>ECSE 444 FINAL PROJECT - GROUP 6 </h2>\r\n");
  strcat((char *)http, (char *)"<br /><hr>\r\n");
  strcat((char *)http, (char *)"<p><form method=\"POST\"><strong>Temp: <input type=\"text\" value=\"");
  sprintf((char *)temp, "%d", temperature);
  strcat((char *)http, (char *)temp);
  strcat((char *)http, (char *)"\"> <sup>O</sup>C");
  strcat((char*)http,(char*)"<p><form method=\"POST\"><strong>Voltage: <input type=\"text\" value=\"");
  memset(temp,0,sizeof(temp));
  sprintf((char*)temp,"%f",voltage);
  strcat((char*)http,(char*)temp);
  strcat((char*)http,(char*)"\"> V");

  if (ledIsOn)
  {
    strcat((char *)http, (char *)"<p><input type=\"radio\" name=\"radio\" value=\"0\" >Speaker on");
    strcat((char *)http, (char *)"<br><input type=\"radio\" name=\"radio\" value=\"1\" checked>Speaker off");
  }
  else
  {
    strcat((char *)http, (char *)"<p><input type=\"radio\" name=\"radio\" value=\"0\" checked>Speaker on");
    strcat((char *)http, (char *)"<br><input type=\"radio\" name=\"radio\" value=\"1\" >Speaker off");
  }

  strcat((char *)http, (char *)"</strong><p><input type=\"submit\"></form></span>");
  strcat((char *)http, (char *)"</body>\r\n</html>\r\n");

  ret = WIFI_SendData(0, (uint8_t *)http, strlen((char *)http), &SentDataLength, WIFI_WRITE_TIMEOUT);

  if((ret == WIFI_STATUS_OK) && (SentDataLength != strlen((char *)http)))
  {
    ret = WIFI_STATUS_ERROR;
  }

  return ret;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
//static void SystemClock_Config(void)
//{
//  RCC_ClkInitTypeDef RCC_ClkInitStruct;
//  RCC_OscInitTypeDef RCC_OscInitStruct;
//
//  /* MSI is enabled after System reset, activate PLL with MSI as source */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
//  RCC_OscInitStruct.PLL.PLLM = 1;
//  RCC_OscInitStruct.PLL.PLLN = 40;
//  RCC_OscInitStruct.PLL.PLLR = 2;
//  RCC_OscInitStruct.PLL.PLLP = 7;
//  RCC_OscInitStruct.PLL.PLLQ = 4;
//  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    /* Initialization Error */
//    while(1);
//  }
//
//  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
//     clocks dividers */
//  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
//  {
//    /* Initialization Error */
//    while(1);
//  }
//}

//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//  RCC_OscInitStruct.MSICalibrationValue = 0;
//  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
//  RCC_OscInitStruct.PLL.PLLM = 1;
//  RCC_OscInitStruct.PLL.PLLN = 60;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
//  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief Reset button state
  *        To be called before Button_WaitForPush()
  */
void Button_Reset()
{
  button_flag = 0;
}

/**
  * @brief Waiting for button to be pushed
  */
uint8_t Button_WaitForPush(uint32_t delay)
{
  uint32_t time_out = HAL_GetTick() + delay;

  do
  {
    if (button_flag > 0)
    {
      return button_flag;
    }
    HAL_Delay(100);
  }
  while (HAL_GetTick() < time_out);

  return 0;
}

#if defined (TERMINAL_USE)
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}


#ifdef __ICCARM__
/**
  * @brief  
  * @param  
  * @retval 
  */
size_t __read(int handle, unsigned char * buffer, size_t size)
{
  int nChars = 0;

  /* handle ? */

  for (/* Empty */; size > 0; --size)
  {
    uint8_t ch = 0;
    while (HAL_OK != HAL_UART_Receive(&hDiscoUart, (uint8_t *)&ch, 1, 30000))
    {
      ;
    }

    *buffer++ = ch;
    ++nChars;
  }

  return nChars;
}
#elif defined(__CC_ARM) || defined(__GNUC__)
/**
  * @brief  Retargets the C library scanf function to the USART.
  * @param  None
  * @retval None
  */
GETCHAR_PROTOTYPE
{
  /* Place your implementation of fgetc here */
  /* e.g. read a character on USART and loop until the end of read */
  uint8_t ch = 0;
  while (HAL_OK != HAL_UART_Receive(&hDiscoUart, (uint8_t *)&ch, 1, 30000))
  {
    ;
  }
  return ch;
}
#endif /* defined(__CC_ARM)  */
#endif /* TERMINAL_USE */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif


/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
int status_pin = 1;
int count = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	  {
	    case (USER_BUTTON_PIN):
	    {
	      Button_ISR();
	      break;
	    }
	    case (GPIO_PIN_1):
	    {
	      SPI_WIFI_ISR();
	      break;
	    }
	    default:
	    {
	      break;
		}
	  }
	if(GPIO_Pin == GPIO_PIN_13){
	if(status_pin == 0){
		HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);
		status_pin = 1;
		count++;
	}
	else{
		HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
		status_pin = 0;
	}
//	if(fmod(count, 3) == 0){
//		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
//		HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,sin_samples,50,DAC_ALIGN_12B_R);
//	}
//	else if(fmod(count, 3) == 1){
//		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
//		HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,sin2_samples,40,DAC_ALIGN_12B_R);
//	}
//	else if(fmod(count, 3) == 2){
//		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
//		HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,sin3_samples,25,DAC_ALIGN_12B_R);
//	}
}
}

/**
  * @brief  SPI3 line detection callback.
  * @param  None
  * @retval None
  */
void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}

/**
  * @brief Update button ISR status
  */
static void Button_ISR(void)
{
  button_flag++;
}

static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 24000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}



/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

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
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
    PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
    PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
    PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
    PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC1 clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE();

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC1_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}
#define ADC_JSQR_FIELDS  ((ADC_JSQR_JL | ADC_JSQR_JEXTSEL | ADC_JSQR_JEXTEN |\
                           ADC_JSQR_JSQ1  | ADC_JSQR_JSQ2 |\
                           ADC_JSQR_JSQ3 | ADC_JSQR_JSQ4 ))
#define ADC_CALIBRATION_TIMEOUT         (296960UL)   /*!< ADC calibration time-out value (unit: CPU cycles) */
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc, uint32_t SingleDiff)
{
  HAL_StatusTypeDef tmp_hal_status;
  __IO uint32_t wait_loop_index = 0UL;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  assert_param(IS_ADC_SINGLE_DIFFERENTIAL(SingleDiff));

  /* Process locked */
  __HAL_LOCK(hadc);

  /* Calibration prerequisite: ADC must be disabled. */

  /* Disable the ADC (if not already disabled) */
  tmp_hal_status = ADC_Disable(hadc);

  /* Check if ADC is effectively disabled */
  if (tmp_hal_status == HAL_OK)
  {
    /* Set ADC state */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
                      HAL_ADC_STATE_BUSY_INTERNAL);

    /* Start ADC calibration in mode single-ended or differential */
    LL_ADC_StartCalibration(hadc->Instance, SingleDiff);

    /* Wait for calibration completion */
    while (LL_ADC_IsCalibrationOnGoing(hadc->Instance) != 0UL)
    {
      wait_loop_index++;
      if (wait_loop_index >= ADC_CALIBRATION_TIMEOUT)
      {
        /* Update ADC state machine to error */
        ADC_STATE_CLR_SET(hadc->State,
                          HAL_ADC_STATE_BUSY_INTERNAL,
                          HAL_ADC_STATE_ERROR_INTERNAL);

        /* Process unlocked */
        __HAL_UNLOCK(hadc);

        return HAL_ERROR;
      }
    }

    /* Set ADC state */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_BUSY_INTERNAL,
                      HAL_ADC_STATE_READY);
  }
  else
  {
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

    /* Note: No need to update variable "tmp_hal_status" here: already set    */
    /*       to state "HAL_ERROR" by function disabling the ADC.              */
  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}
uint32_t HAL_ADCEx_Calibration_GetValue(ADC_HandleTypeDef *hadc, uint32_t SingleDiff)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  assert_param(IS_ADC_SINGLE_DIFFERENTIAL(SingleDiff));

  /* Return the selected ADC calibration value */
  return LL_ADC_GetCalibrationFactor(hadc->Instance, SingleDiff);
}
HAL_StatusTypeDef HAL_ADCEx_Calibration_SetValue(ADC_HandleTypeDef *hadc, uint32_t SingleDiff,
                                                 uint32_t CalibrationFactor)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  uint32_t tmp_adc_is_conversion_on_going_regular;
  uint32_t tmp_adc_is_conversion_on_going_injected;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  assert_param(IS_ADC_SINGLE_DIFFERENTIAL(SingleDiff));
  assert_param(IS_ADC_CALFACT(CalibrationFactor));

  /* Process locked */
  __HAL_LOCK(hadc);

  /* Verification of hardware constraints before modifying the calibration    */
  /* factors register: ADC must be enabled, no conversion on going.           */
  tmp_adc_is_conversion_on_going_regular = LL_ADC_REG_IsConversionOngoing(hadc->Instance);
  tmp_adc_is_conversion_on_going_injected = LL_ADC_INJ_IsConversionOngoing(hadc->Instance);

  if ((LL_ADC_IsEnabled(hadc->Instance) != 0UL)
      && (tmp_adc_is_conversion_on_going_regular == 0UL)
      && (tmp_adc_is_conversion_on_going_injected == 0UL)
     )
  {
    /* Set the selected ADC calibration value */
    LL_ADC_SetCalibrationFactor(hadc->Instance, SingleDiff, CalibrationFactor);
  }
  else
  {
    /* Update ADC state machine */
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
    /* Update ADC error code */
    SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);

    /* Update ADC state machine to error */
    tmp_hal_status = HAL_ERROR;
  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;
  uint32_t tmp_config_injected_queue;
#if defined(ADC_MULTIMODE_SUPPORT)
  uint32_t tmp_multimode_config = LL_ADC_GetMultimode(__LL_ADC_COMMON_INSTANCE(hadc->Instance));
#endif

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  if (LL_ADC_INJ_IsConversionOngoing(hadc->Instance) != 0UL)
  {
    return HAL_BUSY;
  }
  else
  {
    /* In case of software trigger detection enabled, JQDIS must be set
      (which can be done only if ADSTART and JADSTART are both cleared).
       If JQDIS is not set at that point, returns an error
       - since software trigger detection is disabled. User needs to
       resort to HAL_ADCEx_DisableInjectedQueue() API to set JQDIS.
       - or (if JQDIS is intentionally reset) since JEXTEN = 0 which means
         the queue is empty */
    tmp_config_injected_queue = READ_BIT(hadc->Instance->CFGR, ADC_CFGR_JQDIS);

    if ((READ_BIT(hadc->Instance->JSQR, ADC_JSQR_JEXTEN) == 0UL)
        && (tmp_config_injected_queue == 0UL)
       )
    {
      SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
      return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(hadc);

    /* Enable the ADC peripheral */
    tmp_hal_status = ADC_Enable(hadc);

    /* Start conversion if ADC is effectively enabled */
    if (tmp_hal_status == HAL_OK)
    {
      /* Check if a regular conversion is ongoing */
      if ((hadc->State & HAL_ADC_STATE_REG_BUSY) != 0UL)
      {
        /* Reset ADC error code field related to injected conversions only */
        CLEAR_BIT(hadc->ErrorCode, HAL_ADC_ERROR_JQOVF);
      }
      else
      {
        /* Set ADC error code to none */
        ADC_CLEAR_ERRORCODE(hadc);
      }

      /* Set ADC state                                                        */
      /* - Clear state bitfield related to injected group conversion results  */
      /* - Set state bitfield related to injected operation                   */
      ADC_STATE_CLR_SET(hadc->State,
                        HAL_ADC_STATE_READY | HAL_ADC_STATE_INJ_EOC,
                        HAL_ADC_STATE_INJ_BUSY);

#if defined(ADC_MULTIMODE_SUPPORT)
      /* Reset HAL_ADC_STATE_MULTIMODE_SLAVE bit
        - if ADC instance is master or if multimode feature is not available
        - if multimode setting is disabled (ADC instance slave in independent mode) */
      if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
          || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
         )
      {
        CLEAR_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
      }
#endif

      /* Clear ADC group injected group conversion flag */
      /* (To ensure of no unknown state from potential previous ADC operations) */
      __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_JEOC | ADC_FLAG_JEOS));

      /* Process unlocked */
      /* Unlock before starting ADC conversions: in case of potential         */
      /* interruption, to let the process to ADC IRQ Handler.                 */
      __HAL_UNLOCK(hadc);

      /* Enable conversion of injected group, if automatic injected conversion  */
      /* is disabled.                                                           */
      /* If software start has been selected, conversion starts immediately.    */
      /* If external trigger has been selected, conversion will start at next   */
      /* trigger event.                                                         */
      /* Case of multimode enabled (when multimode feature is available):       */
      /* if ADC is slave,                                                       */
      /*    - ADC is enabled only (conversion is not started),                  */
      /*    - if multimode only concerns regular conversion, ADC is enabled     */
      /*     and conversion is started.                                         */
      /* If ADC is master or independent,                                       */
      /*    - ADC is enabled and conversion is started.                         */
#if defined(ADC_MULTIMODE_SUPPORT)
      if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
          || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
          || (tmp_multimode_config == LL_ADC_MULTI_DUAL_REG_SIMULT)
          || (tmp_multimode_config == LL_ADC_MULTI_DUAL_REG_INTERL)
         )
      {
        /* ADC instance is not a multimode slave instance with multimode injected conversions enabled */
        if (LL_ADC_INJ_GetTrigAuto(hadc->Instance) == LL_ADC_INJ_TRIG_INDEPENDENT)
        {
          LL_ADC_INJ_StartConversion(hadc->Instance);
        }
      }
      else
      {
        /* ADC instance is not a multimode slave instance with multimode injected conversions enabled */
        SET_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
      }
#else
      if (LL_ADC_INJ_GetTrigAuto(hadc->Instance) == LL_ADC_INJ_TRIG_INDEPENDENT)
      {
        /* Start ADC group injected conversion */
        LL_ADC_INJ_StartConversion(hadc->Instance);
      }
#endif

    }
    else
    {
      /* Process unlocked */
      __HAL_UNLOCK(hadc);
    }

    /* Return function status */
    return tmp_hal_status;
  }
}

/**
  * @brief  Stop conversion of injected channels. Disable ADC peripheral if
  *         no regular conversion is on going.
  * @note   If ADC must be disabled and if conversion is on going on
  *         regular group, function HAL_ADC_Stop must be used to stop both
  *         injected and regular groups, and disable the ADC.
  * @note   If injected group mode auto-injection is enabled,
  *         function HAL_ADC_Stop must be used.
  * @note   In case of multimode enabled (when multimode feature is available),
  *         HAL_ADCEx_InjectedStop() must be called for ADC master first, then for ADC slave.
  *         For ADC master, conversion is stopped and ADC is disabled.
  *         For ADC slave, ADC is disabled only (conversion stop of ADC master
  *         has already stopped conversion of ADC slave).
  * @param hadc ADC handle.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_InjectedStop(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Process locked */
  __HAL_LOCK(hadc);

  /* 1. Stop potential conversion on going on injected group only. */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_INJECTED_GROUP);

  /* Disable ADC peripheral if injected conversions are effectively stopped   */
  /* and if no conversion on regular group is on-going                       */
  if (tmp_hal_status == HAL_OK)
  {
    if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
    {
      /* 2. Disable the ADC peripheral */
      tmp_hal_status = ADC_Disable(hadc);

      /* Check if ADC is effectively disabled */
      if (tmp_hal_status == HAL_OK)
      {
        /* Set ADC state */
        ADC_STATE_CLR_SET(hadc->State,
                          HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
                          HAL_ADC_STATE_READY);
      }
    }
    /* Conversion on injected group is stopped, but ADC not disabled since    */
    /* conversion on regular group is still running.                          */
    else
    {
      /* Set ADC state */
      CLEAR_BIT(hadc->State, HAL_ADC_STATE_INJ_BUSY);
    }
  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}

/**
  * @brief  Wait for injected group conversion to be completed.
  * @param hadc ADC handle
  * @param Timeout Timeout value in millisecond.
  * @note   Depending on hadc->Init.EOCSelection, JEOS or JEOC is
  *         checked and cleared depending on AUTDLY bit status.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_InjectedPollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout)
{
  uint32_t tickstart;
  uint32_t tmp_Flag_End;
  uint32_t tmp_adc_inj_is_trigger_source_sw_start;
  uint32_t tmp_adc_reg_is_trigger_source_sw_start;
  uint32_t tmp_cfgr;
#if defined(ADC_MULTIMODE_SUPPORT)
  const ADC_TypeDef *tmpADC_Master;
  uint32_t tmp_multimode_config = LL_ADC_GetMultimode(__LL_ADC_COMMON_INSTANCE(hadc->Instance));
#endif

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* If end of sequence selected */
  if (hadc->Init.EOCSelection == ADC_EOC_SEQ_CONV)
  {
    tmp_Flag_End = ADC_FLAG_JEOS;
  }
  else /* end of conversion selected */
  {
    tmp_Flag_End = ADC_FLAG_JEOC;
  }

  /* Get timeout */
  tickstart = HAL_GetTick();

  /* Wait until End of Conversion or Sequence flag is raised */
  while ((hadc->Instance->ISR & tmp_Flag_End) == 0UL)
  {
    /* Check if timeout is disabled (set to infinite wait) */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0UL))
      {
        /* New check to avoid false timeout detection in case of preemption */
        if ((hadc->Instance->ISR & tmp_Flag_End) == 0UL)
        {
          /* Update ADC state machine to timeout */
          SET_BIT(hadc->State, HAL_ADC_STATE_TIMEOUT);

          /* Process unlocked */
          __HAL_UNLOCK(hadc);

          return HAL_TIMEOUT;
        }
      }
    }
  }

  /* Retrieve ADC configuration */
  tmp_adc_inj_is_trigger_source_sw_start = LL_ADC_INJ_IsTriggerSourceSWStart(hadc->Instance);
  tmp_adc_reg_is_trigger_source_sw_start = LL_ADC_REG_IsTriggerSourceSWStart(hadc->Instance);
  /* Get relevant register CFGR in ADC instance of ADC master or slave  */
  /* in function of multimode state (for devices with multimode         */
  /* available).                                                        */
#if defined(ADC_MULTIMODE_SUPPORT)
  if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
      || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
      || (tmp_multimode_config == LL_ADC_MULTI_DUAL_REG_SIMULT)
      || (tmp_multimode_config == LL_ADC_MULTI_DUAL_REG_INTERL)
     )
  {
    tmp_cfgr = READ_REG(hadc->Instance->CFGR);
  }
  else
  {
    tmpADC_Master = __LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance);
    tmp_cfgr = READ_REG(tmpADC_Master->CFGR);
  }
#else
  tmp_cfgr = READ_REG(hadc->Instance->CFGR);
#endif

  /* Update ADC state machine */
  SET_BIT(hadc->State, HAL_ADC_STATE_INJ_EOC);

  /* Determine whether any further conversion upcoming on group injected      */
  /* by external trigger or by automatic injected conversion                  */
  /* from group regular.                                                      */
  if ((tmp_adc_inj_is_trigger_source_sw_start != 0UL)            ||
      ((READ_BIT(tmp_cfgr, ADC_CFGR_JAUTO) == 0UL)      &&
       ((tmp_adc_reg_is_trigger_source_sw_start != 0UL)  &&
        (READ_BIT(tmp_cfgr, ADC_CFGR_CONT) == 0UL))))
  {
    /* Check whether end of sequence is reached */
    if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOS))
    {
      /* Particular case if injected contexts queue is enabled:             */
      /* when the last context has been fully processed, JSQR is reset      */
      /* by the hardware. Even if no injected conversion is planned to come */
      /* (queue empty, triggers are ignored), it can start again            */
      /* immediately after setting a new context (JADSTART is still set).   */
      /* Therefore, state of HAL ADC injected group is kept to busy.        */
      if (READ_BIT(tmp_cfgr, ADC_CFGR_JQM) == 0UL)
      {
        /* Set ADC state */
        CLEAR_BIT(hadc->State, HAL_ADC_STATE_INJ_BUSY);

        if ((hadc->State & HAL_ADC_STATE_REG_BUSY) == 0UL)
        {
          SET_BIT(hadc->State, HAL_ADC_STATE_READY);
        }
      }
    }
  }

  /* Clear polled flag */
  if (tmp_Flag_End == ADC_FLAG_JEOS)
  {
    /* Clear end of sequence JEOS flag of injected group if low power feature */
    /* "LowPowerAutoWait " is disabled, to not interfere with this feature.   */
    /* For injected groups, no new conversion will start before JEOS is       */
    /* cleared.                                                               */
    if (READ_BIT(tmp_cfgr, ADC_CFGR_AUTDLY) == 0UL)
    {
      __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_JEOC | ADC_FLAG_JEOS));
    }
  }
  else
  {
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC);
  }

  /* Return API HAL status */
  return HAL_OK;
}

/**
  * @brief  Enable ADC, start conversion of injected group with interruption.
  * @note   Interruptions enabled in this function according to initialization
  *         setting : JEOC (end of conversion) or JEOS (end of sequence)
  * @note   Case of multimode enabled (when multimode feature is enabled):
  *         HAL_ADCEx_InjectedStart_IT() API must be called for ADC slave first,
  *         then for ADC master.
  *         For ADC slave, ADC is enabled only (conversion is not started).
  *         For ADC master, ADC is enabled and multimode conversion is started.
  * @param hadc ADC handle.
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;
  uint32_t tmp_config_injected_queue;
#if defined(ADC_MULTIMODE_SUPPORT)
  uint32_t tmp_multimode_config = LL_ADC_GetMultimode(__LL_ADC_COMMON_INSTANCE(hadc->Instance));
#endif

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  if (LL_ADC_INJ_IsConversionOngoing(hadc->Instance) != 0UL)
  {
    return HAL_BUSY;
  }
  else
  {
    /* In case of software trigger detection enabled, JQDIS must be set
      (which can be done only if ADSTART and JADSTART are both cleared).
       If JQDIS is not set at that point, returns an error
       - since software trigger detection is disabled. User needs to
       resort to HAL_ADCEx_DisableInjectedQueue() API to set JQDIS.
       - or (if JQDIS is intentionally reset) since JEXTEN = 0 which means
         the queue is empty */
    tmp_config_injected_queue = READ_BIT(hadc->Instance->CFGR, ADC_CFGR_JQDIS);

    if ((READ_BIT(hadc->Instance->JSQR, ADC_JSQR_JEXTEN) == 0UL)
        && (tmp_config_injected_queue == 0UL)
       )
    {
      SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
      return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(hadc);

    /* Enable the ADC peripheral */
    tmp_hal_status = ADC_Enable(hadc);

    /* Start conversion if ADC is effectively enabled */
    if (tmp_hal_status == HAL_OK)
    {
      /* Check if a regular conversion is ongoing */
      if ((hadc->State & HAL_ADC_STATE_REG_BUSY) != 0UL)
      {
        /* Reset ADC error code field related to injected conversions only */
        CLEAR_BIT(hadc->ErrorCode, HAL_ADC_ERROR_JQOVF);
      }
      else
      {
        /* Set ADC error code to none */
        ADC_CLEAR_ERRORCODE(hadc);
      }

      /* Set ADC state                                                        */
      /* - Clear state bitfield related to injected group conversion results  */
      /* - Set state bitfield related to injected operation                   */
      ADC_STATE_CLR_SET(hadc->State,
                        HAL_ADC_STATE_READY | HAL_ADC_STATE_INJ_EOC,
                        HAL_ADC_STATE_INJ_BUSY);

#if defined(ADC_MULTIMODE_SUPPORT)
      /* Reset HAL_ADC_STATE_MULTIMODE_SLAVE bit
        - if ADC instance is master or if multimode feature is not available
        - if multimode setting is disabled (ADC instance slave in independent mode) */
      if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
          || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
         )
      {
        CLEAR_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
      }
#endif

      /* Clear ADC group injected group conversion flag */
      /* (To ensure of no unknown state from potential previous ADC operations) */
      __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_JEOC | ADC_FLAG_JEOS));

      /* Process unlocked */
      /* Unlock before starting ADC conversions: in case of potential         */
      /* interruption, to let the process to ADC IRQ Handler.                 */
      __HAL_UNLOCK(hadc);

      /* Enable ADC Injected context queue overflow interrupt if this feature   */
      /* is enabled.                                                            */
      if ((hadc->Instance->CFGR & ADC_CFGR_JQM) != 0UL)
      {
        __HAL_ADC_ENABLE_IT(hadc, ADC_FLAG_JQOVF);
      }

      /* Enable ADC end of conversion interrupt */
      switch (hadc->Init.EOCSelection)
      {
        case ADC_EOC_SEQ_CONV:
          __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOC);
          __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOS);
          break;
        /* case ADC_EOC_SINGLE_CONV */
        default:
          __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOS);
          __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);
          break;
      }

      /* Enable conversion of injected group, if automatic injected conversion  */
      /* is disabled.                                                           */
      /* If software start has been selected, conversion starts immediately.    */
      /* If external trigger has been selected, conversion will start at next   */
      /* trigger event.                                                         */
      /* Case of multimode enabled (when multimode feature is available):       */
      /* if ADC is slave,                                                       */
      /*    - ADC is enabled only (conversion is not started),                  */
      /*    - if multimode only concerns regular conversion, ADC is enabled     */
      /*     and conversion is started.                                         */
      /* If ADC is master or independent,                                       */
      /*    - ADC is enabled and conversion is started.                         */
#if defined(ADC_MULTIMODE_SUPPORT)
      if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
          || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
          || (tmp_multimode_config == LL_ADC_MULTI_DUAL_REG_SIMULT)
          || (tmp_multimode_config == LL_ADC_MULTI_DUAL_REG_INTERL)
         )
      {
        /* ADC instance is not a multimode slave instance with multimode injected conversions enabled */
        if (LL_ADC_INJ_GetTrigAuto(hadc->Instance) == LL_ADC_INJ_TRIG_INDEPENDENT)
        {
          LL_ADC_INJ_StartConversion(hadc->Instance);
        }
      }
      else
      {
        /* ADC instance is not a multimode slave instance with multimode injected conversions enabled */
        SET_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
      }
#else
      if (LL_ADC_INJ_GetTrigAuto(hadc->Instance) == LL_ADC_INJ_TRIG_INDEPENDENT)
      {
        /* Start ADC group injected conversion */
        LL_ADC_INJ_StartConversion(hadc->Instance);
      }
#endif

    }
    else
    {
      /* Process unlocked */
      __HAL_UNLOCK(hadc);
    }

    /* Return function status */
    return tmp_hal_status;
  }
}

/**
  * @brief  Stop conversion of injected channels, disable interruption of
  *         end-of-conversion. Disable ADC peripheral if no regular conversion
  *         is on going.
  * @note   If ADC must be disabled and if conversion is on going on
  *         regular group, function HAL_ADC_Stop must be used to stop both
  *         injected and regular groups, and disable the ADC.
  * @note   If injected group mode auto-injection is enabled,
  *         function HAL_ADC_Stop must be used.
  * @note   Case of multimode enabled (when multimode feature is available):
  *         HAL_ADCEx_InjectedStop_IT() API must be called for ADC master first,
  *         then for ADC slave.
  *         For ADC master, conversion is stopped and ADC is disabled.
  *         For ADC slave, ADC is disabled only (conversion stop of ADC master
  *         has already stopped conversion of ADC slave).
  * @note   In case of auto-injection mode, HAL_ADC_Stop() must be used.
  * @param hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Process locked */
  __HAL_LOCK(hadc);

  /* 1. Stop potential conversion on going on injected group only. */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_INJECTED_GROUP);

  /* Disable ADC peripheral if injected conversions are effectively stopped   */
  /* and if no conversion on the other group (regular group) is intended to   */
  /* continue.                                                                */
  if (tmp_hal_status == HAL_OK)
  {
    /* Disable ADC end of conversion interrupt for injected channels */
    __HAL_ADC_DISABLE_IT(hadc, (ADC_IT_JEOC | ADC_IT_JEOS | ADC_FLAG_JQOVF));

    if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
    {
      /* 2. Disable the ADC peripheral */
      tmp_hal_status = ADC_Disable(hadc);

      /* Check if ADC is effectively disabled */
      if (tmp_hal_status == HAL_OK)
      {
        /* Set ADC state */
        ADC_STATE_CLR_SET(hadc->State,
                          HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
                          HAL_ADC_STATE_READY);
      }
    }
    /* Conversion on injected group is stopped, but ADC not disabled since    */
    /* conversion on regular group is still running.                          */
    else
    {
      /* Set ADC state */
      CLEAR_BIT(hadc->State, HAL_ADC_STATE_INJ_BUSY);
    }
  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}

#if defined(ADC_MULTIMODE_SUPPORT)
/**
  * @brief  Enable ADC, start MultiMode conversion and transfer regular results through DMA.
  * @note   Multimode must have been previously configured using
  *         HAL_ADCEx_MultiModeConfigChannel() function.
  *         Interruptions enabled in this function:
  *          overrun, DMA half transfer, DMA transfer complete.
  *         Each of these interruptions has its dedicated callback function.
  * @note   State field of Slave ADC handle is not updated in this configuration:
  *          user should not rely on it for information related to Slave regular
  *         conversions.
  * @param hadc ADC handle of ADC master (handle of ADC slave must not be used)
  * @param pData Destination Buffer address.
  * @param Length Length of data to be transferred from ADC peripheral to memory (in bytes).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length)
{
  HAL_StatusTypeDef tmp_hal_status;
  ADC_HandleTypeDef tmphadcSlave;
  ADC_Common_TypeDef *tmpADC_Common;

  /* Check the parameters */
  assert_param(IS_ADC_MULTIMODE_MASTER_INSTANCE(hadc->Instance));
  assert_param(IS_FUNCTIONAL_STATE(hadc->Init.ContinuousConvMode));
  assert_param(IS_ADC_EXTTRIG_EDGE(hadc->Init.ExternalTrigConvEdge));
  assert_param(IS_FUNCTIONAL_STATE(hadc->Init.DMAContinuousRequests));

  if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) != 0UL)
  {
    return HAL_BUSY;
  }
  else
  {
    /* Process locked */
    __HAL_LOCK(hadc);

    /* Temporary handle minimum initialization */
    __HAL_ADC_RESET_HANDLE_STATE(&tmphadcSlave);
    ADC_CLEAR_ERRORCODE(&tmphadcSlave);

    /* Set a temporary handle of the ADC slave associated to the ADC master   */
    ADC_MULTI_SLAVE(hadc, &tmphadcSlave);

    if (tmphadcSlave.Instance == NULL)
    {
      /* Set ADC state */
      SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);

      /* Process unlocked */
      __HAL_UNLOCK(hadc);

      return HAL_ERROR;
    }

    /* Enable the ADC peripherals: master and slave (in case if not already   */
    /* enabled previously)                                                    */
    tmp_hal_status = ADC_Enable(hadc);
    if (tmp_hal_status == HAL_OK)
    {
      tmp_hal_status = ADC_Enable(&tmphadcSlave);
    }

    /* Start multimode conversion of ADCs pair */
    if (tmp_hal_status == HAL_OK)
    {
      /* Set ADC state */
      ADC_STATE_CLR_SET(hadc->State,
                        (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP),
                        HAL_ADC_STATE_REG_BUSY);

      /* Set ADC error code to none */
      ADC_CLEAR_ERRORCODE(hadc);

      /* Set the DMA transfer complete callback */
      hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;

      /* Set the DMA half transfer complete callback */
      hadc->DMA_Handle->XferHalfCpltCallback = ADC_DMAHalfConvCplt;

      /* Set the DMA error callback */
      hadc->DMA_Handle->XferErrorCallback = ADC_DMAError ;

      /* Pointer to the common control register  */
      tmpADC_Common = __LL_ADC_COMMON_INSTANCE(hadc->Instance);

      /* Manage ADC and DMA start: ADC overrun interruption, DMA start, ADC     */
      /* start (in case of SW start):                                           */

      /* Clear regular group conversion flag and overrun flag */
      /* (To ensure of no unknown state from potential previous ADC operations) */
      __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

      /* Process unlocked */
      /* Unlock before starting ADC conversions: in case of potential         */
      /* interruption, to let the process to ADC IRQ Handler.                 */
      __HAL_UNLOCK(hadc);

      /* Enable ADC overrun interrupt */
      __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

      /* Start the DMA channel */
      tmp_hal_status = HAL_DMA_Start_IT(hadc->DMA_Handle, (uint32_t)&tmpADC_Common->CDR, (uint32_t)pData, Length);

      /* Enable conversion of regular group.                                    */
      /* If software start has been selected, conversion starts immediately.    */
      /* If external trigger has been selected, conversion will start at next   */
      /* trigger event.                                                         */
      /* Start ADC group regular conversion */
      LL_ADC_REG_StartConversion(hadc->Instance);
    }
    else
    {
      /* Process unlocked */
      __HAL_UNLOCK(hadc);
    }

    /* Return function status */
    return tmp_hal_status;
  }
}

/**
  * @brief  Stop multimode ADC conversion, disable ADC DMA transfer, disable ADC peripheral.
  * @note   Multimode is kept enabled after this function. MultiMode DMA bits
  *         (MDMA and DMACFG bits of common CCR register) are maintained. To disable
  *         Multimode (set with HAL_ADCEx_MultiModeConfigChannel()), ADC must be
  *         reinitialized using HAL_ADC_Init() or HAL_ADC_DeInit(), or the user can
  *         resort to HAL_ADCEx_DisableMultiMode() API.
  * @note   In case of DMA configured in circular mode, function
  *         HAL_ADC_Stop_DMA() must be called after this function with handle of
  *         ADC slave, to properly disable the DMA channel.
  * @param hadc ADC handle of ADC master (handle of ADC slave must not be used)
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_MultiModeStop_DMA(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;
  uint32_t tickstart;
  ADC_HandleTypeDef tmphadcSlave;
  uint32_t tmphadcSlave_conversion_on_going;
  HAL_StatusTypeDef tmphadcSlave_disable_status;

  /* Check the parameters */
  assert_param(IS_ADC_MULTIMODE_MASTER_INSTANCE(hadc->Instance));

  /* Process locked */
  __HAL_LOCK(hadc);


  /* 1. Stop potential multimode conversion on going, on regular and injected groups */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_REGULAR_INJECTED_GROUP);

  /* Disable ADC peripheral if conversions are effectively stopped */
  if (tmp_hal_status == HAL_OK)
  {
    /* Temporary handle minimum initialization */
    __HAL_ADC_RESET_HANDLE_STATE(&tmphadcSlave);
    ADC_CLEAR_ERRORCODE(&tmphadcSlave);

    /* Set a temporary handle of the ADC slave associated to the ADC master   */
    ADC_MULTI_SLAVE(hadc, &tmphadcSlave);

    if (tmphadcSlave.Instance == NULL)
    {
      /* Update ADC state machine to error */
      SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);

      /* Process unlocked */
      __HAL_UNLOCK(hadc);

      return HAL_ERROR;
    }

    /* Procedure to disable the ADC peripheral: wait for conversions          */
    /* effectively stopped (ADC master and ADC slave), then disable ADC       */

    /* 1. Wait for ADC conversion completion for ADC master and ADC slave */
    tickstart = HAL_GetTick();

    tmphadcSlave_conversion_on_going = LL_ADC_REG_IsConversionOngoing((&tmphadcSlave)->Instance);
    while ((LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 1UL)
           || (tmphadcSlave_conversion_on_going == 1UL)
          )
    {
      if ((HAL_GetTick() - tickstart) > ADC_STOP_CONVERSION_TIMEOUT)
      {
        /* New check to avoid false timeout detection in case of preemption */
        tmphadcSlave_conversion_on_going = LL_ADC_REG_IsConversionOngoing((&tmphadcSlave)->Instance);
        if ((LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 1UL)
            || (tmphadcSlave_conversion_on_going == 1UL)
           )
        {
          /* Update ADC state machine to error */
          SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

          /* Process unlocked */
          __HAL_UNLOCK(hadc);

          return HAL_ERROR;
        }
      }

      tmphadcSlave_conversion_on_going = LL_ADC_REG_IsConversionOngoing((&tmphadcSlave)->Instance);
    }

    /* Disable the DMA channel (in case of DMA in circular mode or stop       */
    /* while DMA transfer is on going)                                        */
    /* Note: DMA channel of ADC slave should be stopped after this function   */
    /*       with HAL_ADC_Stop_DMA() API.                                     */
    tmp_hal_status = HAL_DMA_Abort(hadc->DMA_Handle);

    /* Check if DMA channel effectively disabled */
    if (tmp_hal_status == HAL_ERROR)
    {
      /* Update ADC state machine to error */
      SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_DMA);
    }

    /* Disable ADC overrun interrupt */
    __HAL_ADC_DISABLE_IT(hadc, ADC_IT_OVR);

    /* 2. Disable the ADC peripherals: master and slave */
    /* Update "tmp_hal_status" only if DMA channel disabling passed, to keep in */
    /* memory a potential failing status.                                     */
    if (tmp_hal_status == HAL_OK)
    {
      tmphadcSlave_disable_status = ADC_Disable(&tmphadcSlave);
      if ((ADC_Disable(hadc) == HAL_OK)           &&
          (tmphadcSlave_disable_status == HAL_OK))
      {
        tmp_hal_status = HAL_OK;
      }
    }
    else
    {
      /* In case of error, attempt to disable ADC master and slave without status assert */
      (void) ADC_Disable(hadc);
      (void) ADC_Disable(&tmphadcSlave);
    }

    /* Set ADC state (ADC master) */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
                      HAL_ADC_STATE_READY);
  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}

/**
  * @brief  Return the last ADC Master and Slave regular conversions results when in multimode configuration.
  * @param hadc ADC handle of ADC Master (handle of ADC Slave must not be used)
  * @retval The converted data values.
  */
uint32_t HAL_ADCEx_MultiModeGetValue(ADC_HandleTypeDef *hadc)
{
  const ADC_Common_TypeDef *tmpADC_Common;

  /* Check the parameters */
  assert_param(IS_ADC_MULTIMODE_MASTER_INSTANCE(hadc->Instance));

  /* Prevent unused argument(s) compilation warning if no assert_param check */
  /* and possible no usage in __LL_ADC_COMMON_INSTANCE() below               */
  UNUSED(hadc);

  /* Pointer to the common control register  */
  tmpADC_Common = __LL_ADC_COMMON_INSTANCE(hadc->Instance);

  /* Return the multi mode conversion value */
  return tmpADC_Common->CDR;
}
#endif /* ADC_MULTIMODE_SUPPORT */

/**
  * @brief  Get ADC injected group conversion result.
  * @note   Reading register JDRx automatically clears ADC flag JEOC
  *         (ADC group injected end of unitary conversion).
  * @note   This function does not clear ADC flag JEOS
  *         (ADC group injected end of sequence conversion)
  *         Occurrence of flag JEOS rising:
  *          - If sequencer is composed of 1 rank, flag JEOS is equivalent
  *            to flag JEOC.
  *          - If sequencer is composed of several ranks, during the scan
  *            sequence flag JEOC only is raised, at the end of the scan sequence
  *            both flags JEOC and EOS are raised.
  *         Flag JEOS must not be cleared by this function because
  *         it would not be compliant with low power features
  *         (feature low power auto-wait, not available on all STM32 families).
  *         To clear this flag, either use function:
  *         in programming model IT: @ref HAL_ADC_IRQHandler(), in programming
  *         model polling: @ref HAL_ADCEx_InjectedPollForConversion()
  *         or @ref __HAL_ADC_CLEAR_FLAG(&hadc, ADC_FLAG_JEOS).
  * @param hadc ADC handle
  * @param InjectedRank the converted ADC injected rank.
  *          This parameter can be one of the following values:
  *            @arg @ref ADC_INJECTED_RANK_1 ADC group injected rank 1
  *            @arg @ref ADC_INJECTED_RANK_2 ADC group injected rank 2
  *            @arg @ref ADC_INJECTED_RANK_3 ADC group injected rank 3
  *            @arg @ref ADC_INJECTED_RANK_4 ADC group injected rank 4
  * @retval ADC group injected conversion data
  */
uint32_t HAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef *hadc, uint32_t InjectedRank)
{
  uint32_t tmp_jdr;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  assert_param(IS_ADC_INJECTED_RANK(InjectedRank));

  /* Get ADC converted value */
  switch (InjectedRank)
  {
    case ADC_INJECTED_RANK_4:
      tmp_jdr = hadc->Instance->JDR4;
      break;
    case ADC_INJECTED_RANK_3:
      tmp_jdr = hadc->Instance->JDR3;
      break;
    case ADC_INJECTED_RANK_2:
      tmp_jdr = hadc->Instance->JDR2;
      break;
    case ADC_INJECTED_RANK_1:
    default:
      tmp_jdr = hadc->Instance->JDR1;
      break;
  }

  /* Return ADC converted value */
  return tmp_jdr;
}

/**
  * @brief  Injected conversion complete callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
__weak void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADCEx_InjectedConvCpltCallback must be implemented in the user file.
  */
}

/**
  * @brief  Injected context queue overflow callback.
  * @note   This callback is called if injected context queue is enabled
            (parameter "QueueInjectedContext" in injected channel configuration)
            and if a new injected context is set when queue is full (maximum 2
            contexts).
  * @param hadc ADC handle
  * @retval None
  */
__weak void HAL_ADCEx_InjectedQueueOverflowCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADCEx_InjectedQueueOverflowCallback must be implemented in the user file.
  */
}

/**
  * @brief  Analog watchdog 2 callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
__weak void HAL_ADCEx_LevelOutOfWindow2Callback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADCEx_LevelOutOfWindow2Callback must be implemented in the user file.
  */
}

/**
  * @brief  Analog watchdog 3 callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
__weak void HAL_ADCEx_LevelOutOfWindow3Callback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADCEx_LevelOutOfWindow3Callback must be implemented in the user file.
  */
}


/**
  * @brief  End Of Sampling callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
__weak void HAL_ADCEx_EndOfSamplingCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADCEx_EndOfSamplingCallback must be implemented in the user file.
  */
}

/**
  * @brief  Stop ADC conversion of regular group (and injected channels in
  *         case of auto_injection mode), disable ADC peripheral if no
  *         conversion is on going on injected group.
  * @param hadc ADC handle
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADCEx_RegularStop(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Process locked */
  __HAL_LOCK(hadc);

  /* 1. Stop potential regular conversion on going */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_REGULAR_GROUP);

  /* Disable ADC peripheral if regular conversions are effectively stopped
     and if no injected conversions are on-going */
  if (tmp_hal_status == HAL_OK)
  {
    /* Clear HAL_ADC_STATE_REG_BUSY bit */
    CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);

    if (LL_ADC_INJ_IsConversionOngoing(hadc->Instance) == 0UL)
    {
      /* 2. Disable the ADC peripheral */
      tmp_hal_status = ADC_Disable(hadc);

      /* Check if ADC is effectively disabled */
      if (tmp_hal_status == HAL_OK)
      {
        /* Set ADC state */
        ADC_STATE_CLR_SET(hadc->State,
                          HAL_ADC_STATE_INJ_BUSY,
                          HAL_ADC_STATE_READY);
      }
    }
    /* Conversion on injected group is stopped, but ADC not disabled since    */
    /* conversion on regular group is still running.                          */
    else
    {
      SET_BIT(hadc->State, HAL_ADC_STATE_INJ_BUSY);
    }
  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}


/**
  * @brief  Stop ADC conversion of ADC groups regular and injected,
  *         disable interrution of end-of-conversion,
  *         disable ADC peripheral if no conversion is on going
  *         on injected group.
  * @param hadc ADC handle
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADCEx_RegularStop_IT(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Process locked */
  __HAL_LOCK(hadc);

  /* 1. Stop potential regular conversion on going */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_REGULAR_GROUP);

  /* Disable ADC peripheral if conversions are effectively stopped
    and if no injected conversion is on-going */
  if (tmp_hal_status == HAL_OK)
  {
    /* Clear HAL_ADC_STATE_REG_BUSY bit */
    CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);

    /* Disable all regular-related interrupts */
    __HAL_ADC_DISABLE_IT(hadc, (ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR));

    /* 2. Disable ADC peripheral if no injected conversions are on-going */
    if (LL_ADC_INJ_IsConversionOngoing(hadc->Instance) == 0UL)
    {
      tmp_hal_status = ADC_Disable(hadc);
      /* if no issue reported */
      if (tmp_hal_status == HAL_OK)
      {
        /* Set ADC state */
        ADC_STATE_CLR_SET(hadc->State,
                          HAL_ADC_STATE_INJ_BUSY,
                          HAL_ADC_STATE_READY);
      }
    }
    else
    {
      SET_BIT(hadc->State, HAL_ADC_STATE_INJ_BUSY);
    }
  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}

/**
  * @brief  Stop ADC conversion of regular group (and injected group in
  *         case of auto_injection mode), disable ADC DMA transfer, disable
  *         ADC peripheral if no conversion is on going
  *         on injected group.
  * @note   HAL_ADCEx_RegularStop_DMA() function is dedicated to single-ADC mode only.
  *         For multimode (when multimode feature is available),
  *         HAL_ADCEx_RegularMultiModeStop_DMA() API must be used.
  * @param hadc ADC handle
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADCEx_RegularStop_DMA(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Process locked */
  __HAL_LOCK(hadc);

  /* 1. Stop potential regular conversion on going */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_REGULAR_GROUP);

  /* Disable ADC peripheral if conversions are effectively stopped
     and if no injected conversion is on-going */
  if (tmp_hal_status == HAL_OK)
  {
    /* Clear HAL_ADC_STATE_REG_BUSY bit */
    CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);

    /* Disable ADC DMA (ADC DMA configuration ADC_CFGR_DMACFG is kept) */
    CLEAR_BIT(hadc->Instance->CFGR, ADC_CFGR_DMAEN);

    /* Disable the DMA channel (in case of DMA in circular mode or stop while */
    /* while DMA transfer is on going)                                        */
    tmp_hal_status = HAL_DMA_Abort(hadc->DMA_Handle);

    /* Check if DMA channel effectively disabled */
    if (tmp_hal_status != HAL_OK)
    {
      /* Update ADC state machine to error */
      SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_DMA);
    }

    /* Disable ADC overrun interrupt */
    __HAL_ADC_DISABLE_IT(hadc, ADC_IT_OVR);

    /* 2. Disable the ADC peripheral */
    /* Update "tmp_hal_status" only if DMA channel disabling passed,          */
    /* to keep in memory a potential failing status.                          */
    if (LL_ADC_INJ_IsConversionOngoing(hadc->Instance) == 0UL)
    {
      if (tmp_hal_status == HAL_OK)
      {
        tmp_hal_status = ADC_Disable(hadc);
      }
      else
      {
        (void)ADC_Disable(hadc);
      }

      /* Check if ADC is effectively disabled */
      if (tmp_hal_status == HAL_OK)
      {
        /* Set ADC state */
        ADC_STATE_CLR_SET(hadc->State,
                          HAL_ADC_STATE_INJ_BUSY,
                          HAL_ADC_STATE_READY);
      }
    }
    else
    {
      SET_BIT(hadc->State, HAL_ADC_STATE_INJ_BUSY);
    }
  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}

#if defined(ADC_MULTIMODE_SUPPORT)
/**
  * @brief  Stop DMA-based multimode ADC conversion, disable ADC DMA transfer, disable ADC peripheral if no injected conversion is on-going.
  * @note   Multimode is kept enabled after this function. Multimode DMA bits
  *         (MDMA and DMACFG bits of common CCR register) are maintained. To disable
  *         multimode (set with HAL_ADCEx_MultiModeConfigChannel()), ADC must be
  *         reinitialized using HAL_ADC_Init() or HAL_ADC_DeInit(), or the user can
  *         resort to HAL_ADCEx_DisableMultiMode() API.
  * @note   In case of DMA configured in circular mode, function
  *         HAL_ADCEx_RegularStop_DMA() must be called after this function with handle of
  *         ADC slave, to properly disable the DMA channel.
  * @param hadc ADC handle of ADC master (handle of ADC slave must not be used)
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_RegularMultiModeStop_DMA(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;
  uint32_t tickstart;
  ADC_HandleTypeDef tmphadcSlave;
  uint32_t tmphadcSlave_conversion_on_going;

  /* Check the parameters */
  assert_param(IS_ADC_MULTIMODE_MASTER_INSTANCE(hadc->Instance));

  /* Process locked */
  __HAL_LOCK(hadc);


  /* 1. Stop potential multimode conversion on going, on regular groups */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_REGULAR_GROUP);

  /* Disable ADC peripheral if conversions are effectively stopped */
  if (tmp_hal_status == HAL_OK)
  {
    /* Clear HAL_ADC_STATE_REG_BUSY bit */
    CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);

    /* Temporary handle minimum initialization */
    __HAL_ADC_RESET_HANDLE_STATE(&tmphadcSlave);
    ADC_CLEAR_ERRORCODE(&tmphadcSlave);

    /* Set a temporary handle of the ADC slave associated to the ADC master   */
    ADC_MULTI_SLAVE(hadc, &tmphadcSlave);

    if (tmphadcSlave.Instance == NULL)
    {
      /* Update ADC state machine to error */
      SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);

      /* Process unlocked */
      __HAL_UNLOCK(hadc);

      return HAL_ERROR;
    }

    /* Procedure to disable the ADC peripheral: wait for conversions          */
    /* effectively stopped (ADC master and ADC slave), then disable ADC       */

    /* 1. Wait for ADC conversion completion for ADC master and ADC slave */
    tickstart = HAL_GetTick();

    tmphadcSlave_conversion_on_going = LL_ADC_REG_IsConversionOngoing((&tmphadcSlave)->Instance);
    while ((LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 1UL)
           || (tmphadcSlave_conversion_on_going == 1UL)
          )
    {
      if ((HAL_GetTick() - tickstart) > ADC_STOP_CONVERSION_TIMEOUT)
      {
        /* New check to avoid false timeout detection in case of preemption */
        tmphadcSlave_conversion_on_going = LL_ADC_REG_IsConversionOngoing((&tmphadcSlave)->Instance);
        if ((LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 1UL)
            || (tmphadcSlave_conversion_on_going == 1UL)
           )
        {
          /* Update ADC state machine to error */
          SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

          /* Process unlocked */
          __HAL_UNLOCK(hadc);

          return HAL_ERROR;
        }
      }

      tmphadcSlave_conversion_on_going = LL_ADC_REG_IsConversionOngoing((&tmphadcSlave)->Instance);
    }

    /* Disable the DMA channel (in case of DMA in circular mode or stop       */
    /* while DMA transfer is on going)                                        */
    /* Note: DMA channel of ADC slave should be stopped after this function   */
    /* with HAL_ADCEx_RegularStop_DMA() API.                                  */
    tmp_hal_status = HAL_DMA_Abort(hadc->DMA_Handle);

    /* Check if DMA channel effectively disabled */
    if (tmp_hal_status != HAL_OK)
    {
      /* Update ADC state machine to error */
      SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_DMA);
    }

    /* Disable ADC overrun interrupt */
    __HAL_ADC_DISABLE_IT(hadc, ADC_IT_OVR);

    /* 2. Disable the ADC peripherals: master and slave if no injected        */
    /*   conversion is on-going.                                              */
    /* Update "tmp_hal_status" only if DMA channel disabling passed, to keep in */
    /* memory a potential failing status.                                     */
    if (tmp_hal_status == HAL_OK)
    {
      if (LL_ADC_INJ_IsConversionOngoing(hadc->Instance) == 0UL)
      {
        tmp_hal_status =  ADC_Disable(hadc);
        if (tmp_hal_status == HAL_OK)
        {
          if (LL_ADC_INJ_IsConversionOngoing((&tmphadcSlave)->Instance) == 0UL)
          {
            tmp_hal_status =  ADC_Disable(&tmphadcSlave);
          }
        }
      }

      if (tmp_hal_status == HAL_OK)
      {
        /* Both Master and Slave ADC's could be disabled. Update Master State */
        /* Clear HAL_ADC_STATE_INJ_BUSY bit, set HAL_ADC_STATE_READY bit */
        ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY, HAL_ADC_STATE_READY);
      }
      else
      {
        /* injected (Master or Slave) conversions are still on-going,
           no Master State change */
      }
    }
  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}
#endif /* ADC_MULTIMODE_SUPPORT */

/**
  * @}
  */

/** @defgroup ADCEx_Exported_Functions_Group2 ADC Extended Peripheral Control functions
  * @brief    ADC Extended Peripheral Control functions
  *
@verbatim
 ===============================================================================
             ##### Peripheral Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure channels on injected group
      (+) Configure multimode when multimode feature is available
      (+) Enable or Disable Injected Queue
      (+) Disable ADC voltage regulator
      (+) Enter ADC deep-power-down mode

@endverbatim
  * @{
  */

/**
  * @brief  Configure a channel to be assigned to ADC group injected.
  * @note   Possibility to update parameters on the fly:
  *         This function initializes injected group, following calls to this
  *         function can be used to reconfigure some parameters of structure
  *         "ADC_InjectionConfTypeDef" on the fly, without resetting the ADC.
  *         The setting of these parameters is conditioned to ADC state:
  *         Refer to comments of structure "ADC_InjectionConfTypeDef".
  * @note   In case of usage of internal measurement channels:
  *         Vbat/VrefInt/TempSensor.
  *         These internal paths can be disabled using function
  *         HAL_ADC_DeInit().
  * @note   Caution: For Injected Context Queue use, a context must be fully
  *         defined before start of injected conversion. All channels are configured
  *         consecutively for the same ADC instance. Therefore, the number of calls to
  *         HAL_ADCEx_InjectedConfigChannel() must be equal to the value of parameter
  *         InjectedNbrOfConversion for each context.
  *  - Example 1: If 1 context is intended to be used (or if there is no use of the
  *    Injected Queue Context feature) and if the context contains 3 injected ranks
  *    (InjectedNbrOfConversion = 3), HAL_ADCEx_InjectedConfigChannel() must be
  *    called once for each channel (i.e. 3 times) before starting a conversion.
  *    This function must not be called to configure a 4th injected channel:
  *    it would start a new context into context queue.
  *  - Example 2: If 2 contexts are intended to be used and each of them contains
  *    3 injected ranks (InjectedNbrOfConversion = 3),
  *    HAL_ADCEx_InjectedConfigChannel() must be called once for each channel and
  *    for each context (3 channels x 2 contexts = 6 calls). Conversion can
  *    start once the 1st context is set, that is after the first three
  *    HAL_ADCEx_InjectedConfigChannel() calls. The 2nd context can be set on the fly.
  * @param hadc ADC handle
  * @param sConfigInjected Structure of ADC injected group and ADC channel for
  *         injected group.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef *hadc, ADC_InjectionConfTypeDef *sConfigInjected)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  uint32_t tmpOffsetShifted;
  uint32_t tmp_config_internal_channel;
  uint32_t tmp_adc_is_conversion_on_going_regular;
  uint32_t tmp_adc_is_conversion_on_going_injected;
  __IO uint32_t wait_loop_index = 0;

  uint32_t tmp_JSQR_ContextQueueBeingBuilt = 0U;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  assert_param(IS_ADC_SAMPLE_TIME(sConfigInjected->InjectedSamplingTime));
  assert_param(IS_ADC_SINGLE_DIFFERENTIAL(sConfigInjected->InjectedSingleDiff));
  assert_param(IS_FUNCTIONAL_STATE(sConfigInjected->AutoInjectedConv));
  assert_param(IS_FUNCTIONAL_STATE(sConfigInjected->QueueInjectedContext));
  assert_param(IS_ADC_EXTTRIGINJEC_EDGE(sConfigInjected->ExternalTrigInjecConvEdge));
  assert_param(IS_ADC_EXTTRIGINJEC(hadc, sConfigInjected->ExternalTrigInjecConv));
  assert_param(IS_ADC_OFFSET_NUMBER(sConfigInjected->InjectedOffsetNumber));
  assert_param(IS_ADC_RANGE(ADC_GET_RESOLUTION(hadc), sConfigInjected->InjectedOffset));
  assert_param(IS_FUNCTIONAL_STATE(sConfigInjected->InjecOversamplingMode));

  if (hadc->Init.ScanConvMode != ADC_SCAN_DISABLE)
  {
    assert_param(IS_ADC_INJECTED_RANK(sConfigInjected->InjectedRank));
    assert_param(IS_ADC_INJECTED_NB_CONV(sConfigInjected->InjectedNbrOfConversion));
    assert_param(IS_FUNCTIONAL_STATE(sConfigInjected->InjectedDiscontinuousConvMode));
  }


  /* if JOVSE is set, the value of the OFFSETy_EN bit in ADCx_OFRy register is
     ignored (considered as reset) */
  assert_param(!((sConfigInjected->InjectedOffsetNumber != ADC_OFFSET_NONE) && (sConfigInjected->InjecOversamplingMode == ENABLE)));

  /* JDISCEN and JAUTO bits can't be set at the same time  */
  assert_param(!((sConfigInjected->InjectedDiscontinuousConvMode == ENABLE) && (sConfigInjected->AutoInjectedConv == ENABLE)));

  /*  DISCEN and JAUTO bits can't be set at the same time */
  assert_param(!((hadc->Init.DiscontinuousConvMode == ENABLE) && (sConfigInjected->AutoInjectedConv == ENABLE)));

  /* Verification of channel number */
  if (sConfigInjected->InjectedSingleDiff != ADC_DIFFERENTIAL_ENDED)
  {
    assert_param(IS_ADC_CHANNEL(hadc, sConfigInjected->InjectedChannel));
  }
  else
  {
    assert_param(IS_ADC_DIFF_CHANNEL(hadc, sConfigInjected->InjectedChannel));
  }

  /* Process locked */
  __HAL_LOCK(hadc);

  /* Configuration of injected group sequencer:                               */
  /* Hardware constraint: Must fully define injected context register JSQR    */
  /* before make it entering into injected sequencer queue.                   */
  /*                                                                          */
  /* - if scan mode is disabled:                                              */
  /*    * Injected channels sequence length is set to 0x00: 1 channel         */
  /*      converted (channel on injected rank 1)                              */
  /*      Parameter "InjectedNbrOfConversion" is discarded.                   */
  /*    * Injected context register JSQR setting is simple: register is fully */
  /*      defined on one call of this function (for injected rank 1) and can  */
  /*      be entered into queue directly.                                     */
  /* - if scan mode is enabled:                                               */
  /*    * Injected channels sequence length is set to parameter               */
  /*      "InjectedNbrOfConversion".                                          */
  /*    * Injected context register JSQR setting more complex: register is    */
  /*      fully defined over successive calls of this function, for each      */
  /*      injected channel rank. It is entered into queue only when all       */
  /*      injected ranks have been set.                                       */
  /*   Note: Scan mode is not present by hardware on this device, but used    */
  /*   by software for alignment over all STM32 devices.                      */

  if ((hadc->Init.ScanConvMode == ADC_SCAN_DISABLE)  ||
      (sConfigInjected->InjectedNbrOfConversion == 1U))
  {
    /* Configuration of context register JSQR:                                */
    /*  - number of ranks in injected group sequencer: fixed to 1st rank      */
    /*    (scan mode disabled, only rank 1 used)                              */
    /*  - external trigger to start conversion                                */
    /*  - external trigger polarity                                           */
    /*  - channel set to rank 1 (scan mode disabled, only rank 1 can be used) */

    if (sConfigInjected->InjectedRank == ADC_INJECTED_RANK_1)
    {
      /* Enable external trigger if trigger selection is different of         */
      /* software start.                                                      */
      /* Note: This configuration keeps the hardware feature of parameter     */
      /*       ExternalTrigInjecConvEdge "trigger edge none" equivalent to    */
      /*       software start.                                                */
      if (sConfigInjected->ExternalTrigInjecConv != ADC_INJECTED_SOFTWARE_START)
      {
        tmp_JSQR_ContextQueueBeingBuilt = (ADC_JSQR_RK(sConfigInjected->InjectedChannel, ADC_INJECTED_RANK_1)
                                           | (sConfigInjected->ExternalTrigInjecConv & ADC_JSQR_JEXTSEL)
                                           | sConfigInjected->ExternalTrigInjecConvEdge
                                          );
      }
      else
      {
        tmp_JSQR_ContextQueueBeingBuilt = (ADC_JSQR_RK(sConfigInjected->InjectedChannel, ADC_INJECTED_RANK_1));
      }

      MODIFY_REG(hadc->Instance->JSQR, ADC_JSQR_FIELDS, tmp_JSQR_ContextQueueBeingBuilt);
      /* For debug and informative reasons, hadc handle saves JSQR setting */
      hadc->InjectionConfig.ContextQueue = tmp_JSQR_ContextQueueBeingBuilt;

    }
  }
  else
  {
    /* Case of scan mode enabled, several channels to set into injected group */
    /* sequencer.                                                             */
    /*                                                                        */
    /* Procedure to define injected context register JSQR over successive     */
    /* calls of this function, for each injected channel rank:                */
    /* 1. Start new context and set parameters related to all injected        */
    /*    channels: injected sequence length and trigger.                     */

    /* if hadc->InjectionConfig.ChannelCount is equal to 0, this is the first */
    /*   call of the context under setting                                    */
    if (hadc->InjectionConfig.ChannelCount == 0U)
    {
      /* Initialize number of channels that will be configured on the context */
      /*  being built                                                         */
      hadc->InjectionConfig.ChannelCount = sConfigInjected->InjectedNbrOfConversion;
      /* Handle hadc saves the context under build up over each HAL_ADCEx_InjectedConfigChannel()
         call, this context will be written in JSQR register at the last call.
         At this point, the context is merely reset  */
      hadc->InjectionConfig.ContextQueue = 0x00000000U;

      /* Configuration of context register JSQR:                              */
      /*  - number of ranks in injected group sequencer                       */
      /*  - external trigger to start conversion                              */
      /*  - external trigger polarity                                         */

      /* Enable external trigger if trigger selection is different of         */
      /* software start.                                                      */
      /* Note: This configuration keeps the hardware feature of parameter     */
      /*       ExternalTrigInjecConvEdge "trigger edge none" equivalent to    */
      /*       software start.                                                */
      if (sConfigInjected->ExternalTrigInjecConv != ADC_INJECTED_SOFTWARE_START)
      {
        tmp_JSQR_ContextQueueBeingBuilt = ((sConfigInjected->InjectedNbrOfConversion - 1U)
                                           | (sConfigInjected->ExternalTrigInjecConv & ADC_JSQR_JEXTSEL)
                                           | sConfigInjected->ExternalTrigInjecConvEdge
                                          );
      }
      else
      {
        tmp_JSQR_ContextQueueBeingBuilt = ((sConfigInjected->InjectedNbrOfConversion - 1U));
      }

    }

    /* 2. Continue setting of context under definition with parameter       */
    /*    related to each channel: channel rank sequence                    */
    /* Clear the old JSQx bits for the selected rank */
    tmp_JSQR_ContextQueueBeingBuilt &= ~ADC_JSQR_RK(ADC_SQR3_SQ10, sConfigInjected->InjectedRank);

    /* Set the JSQx bits for the selected rank */
    tmp_JSQR_ContextQueueBeingBuilt |= ADC_JSQR_RK(sConfigInjected->InjectedChannel, sConfigInjected->InjectedRank);

    /* Decrease channel count  */
    hadc->InjectionConfig.ChannelCount--;

    /* 3. tmp_JSQR_ContextQueueBeingBuilt is fully built for this HAL_ADCEx_InjectedConfigChannel()
          call, aggregate the setting to those already built during the previous
          HAL_ADCEx_InjectedConfigChannel() calls (for the same context of course)  */
    hadc->InjectionConfig.ContextQueue |= tmp_JSQR_ContextQueueBeingBuilt;

    /* 4. End of context setting: if this is the last channel set, then write context
        into register JSQR and make it enter into queue                   */
    if (hadc->InjectionConfig.ChannelCount == 0U)
    {
      MODIFY_REG(hadc->Instance->JSQR, ADC_JSQR_FIELDS, hadc->InjectionConfig.ContextQueue);
    }
  }

  /* Parameters update conditioned to ADC state:                              */
  /* Parameters that can be updated when ADC is disabled or enabled without   */
  /* conversion on going on injected group:                                   */
  /*  - Injected context queue: Queue disable (active context is kept) or     */
  /*    enable (context decremented, up to 2 contexts queued)                 */
  /*  - Injected discontinuous mode: can be enabled only if auto-injected     */
  /*    mode is disabled.                                                     */
  if (LL_ADC_INJ_IsConversionOngoing(hadc->Instance) == 0UL)
  {
    /* If auto-injected mode is disabled: no constraint                       */
    if (sConfigInjected->AutoInjectedConv == DISABLE)
    {
      MODIFY_REG(hadc->Instance->CFGR,
                 ADC_CFGR_JQM | ADC_CFGR_JDISCEN,
                 ADC_CFGR_INJECT_CONTEXT_QUEUE((uint32_t)sConfigInjected->QueueInjectedContext)           |
                 ADC_CFGR_INJECT_DISCCONTINUOUS((uint32_t)sConfigInjected->InjectedDiscontinuousConvMode));
    }
    /* If auto-injected mode is enabled: Injected discontinuous setting is    */
    /* discarded.                                                             */
    else
    {
      MODIFY_REG(hadc->Instance->CFGR,
                 ADC_CFGR_JQM | ADC_CFGR_JDISCEN,
                 ADC_CFGR_INJECT_CONTEXT_QUEUE((uint32_t)sConfigInjected->QueueInjectedContext));
    }

  }

  /* Parameters update conditioned to ADC state:                              */
  /* Parameters that can be updated when ADC is disabled or enabled without   */
  /* conversion on going on regular and injected groups:                      */
  /*  - Automatic injected conversion: can be enabled if injected group       */
  /*    external triggers are disabled.                                       */
  /*  - Channel sampling time                                                 */
  /*  - Channel offset                                                        */
  tmp_adc_is_conversion_on_going_regular = LL_ADC_REG_IsConversionOngoing(hadc->Instance);
  tmp_adc_is_conversion_on_going_injected = LL_ADC_INJ_IsConversionOngoing(hadc->Instance);

  if ((tmp_adc_is_conversion_on_going_regular == 0UL)
      && (tmp_adc_is_conversion_on_going_injected == 0UL)
     )
  {
    /* If injected group external triggers are disabled (set to injected      */
    /* software start): no constraint                                         */
    if ((sConfigInjected->ExternalTrigInjecConv == ADC_INJECTED_SOFTWARE_START)
        || (sConfigInjected->ExternalTrigInjecConvEdge == ADC_EXTERNALTRIGINJECCONV_EDGE_NONE))
    {
      if (sConfigInjected->AutoInjectedConv == ENABLE)
      {
        SET_BIT(hadc->Instance->CFGR, ADC_CFGR_JAUTO);
      }
      else
      {
        CLEAR_BIT(hadc->Instance->CFGR, ADC_CFGR_JAUTO);
      }
    }
    /* If Automatic injected conversion was intended to be set and could not  */
    /* due to injected group external triggers enabled, error is reported.    */
    else
    {
      if (sConfigInjected->AutoInjectedConv == ENABLE)
      {
        /* Update ADC state machine to error */
        SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);

        tmp_hal_status = HAL_ERROR;
      }
      else
      {
        CLEAR_BIT(hadc->Instance->CFGR, ADC_CFGR_JAUTO);
      }
    }

    if (sConfigInjected->InjecOversamplingMode == ENABLE)
    {
      assert_param(IS_ADC_OVERSAMPLING_RATIO(sConfigInjected->InjecOversampling.Ratio));
      assert_param(IS_ADC_RIGHT_BIT_SHIFT(sConfigInjected->InjecOversampling.RightBitShift));

      /*  JOVSE must be reset in case of triggered regular mode  */
      assert_param(!(READ_BIT(hadc->Instance->CFGR2, ADC_CFGR2_ROVSE | ADC_CFGR2_TROVS) == (ADC_CFGR2_ROVSE | ADC_CFGR2_TROVS)));

      /* Configuration of Injected Oversampler:                                 */
      /*  - Oversampling Ratio                                                  */
      /*  - Right bit shift                                                     */

      /* Enable OverSampling mode */
      MODIFY_REG(hadc->Instance->CFGR2,
                 ADC_CFGR2_JOVSE |
                 ADC_CFGR2_OVSR  |
                 ADC_CFGR2_OVSS,
                 ADC_CFGR2_JOVSE                                  |
                 sConfigInjected->InjecOversampling.Ratio         |
                 sConfigInjected->InjecOversampling.RightBitShift
                );
    }
    else
    {
      /* Disable Regular OverSampling */
      CLEAR_BIT(hadc->Instance->CFGR2, ADC_CFGR2_JOVSE);
    }

#if defined(ADC_SMPR1_SMPPLUS)
    /* Manage specific case of sampling time 3.5 cycles replacing 2.5 cyles */
    if (sConfigInjected->InjectedSamplingTime == ADC_SAMPLETIME_3CYCLES_5)
    {
      /* Set sampling time of the selected ADC channel */
      LL_ADC_SetChannelSamplingTime(hadc->Instance, sConfigInjected->InjectedChannel, LL_ADC_SAMPLINGTIME_2CYCLES_5);

      /* Set ADC sampling time common configuration */
      LL_ADC_SetSamplingTimeCommonConfig(hadc->Instance, LL_ADC_SAMPLINGTIME_COMMON_3C5_REPL_2C5);
    }
    else
    {
      /* Set sampling time of the selected ADC channel */
      LL_ADC_SetChannelSamplingTime(hadc->Instance, sConfigInjected->InjectedChannel, sConfigInjected->InjectedSamplingTime);

      /* Set ADC sampling time common configuration */
      LL_ADC_SetSamplingTimeCommonConfig(hadc->Instance, LL_ADC_SAMPLINGTIME_COMMON_DEFAULT);
    }
#else
    /* Set sampling time of the selected ADC channel */
    LL_ADC_SetChannelSamplingTime(hadc->Instance, sConfigInjected->InjectedChannel, sConfigInjected->InjectedSamplingTime);
#endif

    /* Configure the offset: offset enable/disable, channel, offset value */

    /* Shift the offset with respect to the selected ADC resolution. */
    /* Offset has to be left-aligned on bit 11, the LSB (right bits) are set to 0 */
    tmpOffsetShifted = ADC_OFFSET_SHIFT_RESOLUTION(hadc, sConfigInjected->InjectedOffset);

    if (sConfigInjected->InjectedOffsetNumber != ADC_OFFSET_NONE)
    {
      /* Set ADC selected offset number */
      LL_ADC_SetOffset(hadc->Instance, sConfigInjected->InjectedOffsetNumber, sConfigInjected->InjectedChannel,
                       tmpOffsetShifted);

    }
    else
    {
      /* Scan each offset register to check if the selected channel is targeted. */
      /* If this is the case, the corresponding offset number is disabled.       */
      if (__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_1))
          == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfigInjected->InjectedChannel))
      {
        LL_ADC_SetOffsetState(hadc->Instance, LL_ADC_OFFSET_1, LL_ADC_OFFSET_DISABLE);
      }
      if (__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_2))
          == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfigInjected->InjectedChannel))
      {
        LL_ADC_SetOffsetState(hadc->Instance, LL_ADC_OFFSET_2, LL_ADC_OFFSET_DISABLE);
      }
      if (__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_3))
          == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfigInjected->InjectedChannel))
      {
        LL_ADC_SetOffsetState(hadc->Instance, LL_ADC_OFFSET_3, LL_ADC_OFFSET_DISABLE);
      }
      if (__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_4))
          == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfigInjected->InjectedChannel))
      {
        LL_ADC_SetOffsetState(hadc->Instance, LL_ADC_OFFSET_4, LL_ADC_OFFSET_DISABLE);
      }
    }

  }

  /* Parameters update conditioned to ADC state:                              */
  /* Parameters that can be updated only when ADC is disabled:                */
  /*  - Single or differential mode                                           */
  if (LL_ADC_IsEnabled(hadc->Instance) == 0UL)
  {
    /* Set mode single-ended or differential input of the selected ADC channel */
    LL_ADC_SetChannelSingleDiff(hadc->Instance, sConfigInjected->InjectedChannel, sConfigInjected->InjectedSingleDiff);

    /* Configuration of differential mode */
    /* Note: ADC channel number masked with value "0x1F" to ensure shift value within 32 bits range */
    if (sConfigInjected->InjectedSingleDiff == ADC_DIFFERENTIAL_ENDED)
    {
      /* Set sampling time of the selected ADC channel */
      LL_ADC_SetChannelSamplingTime(hadc->Instance,
                                    (uint32_t)(__LL_ADC_DECIMAL_NB_TO_CHANNEL((__LL_ADC_CHANNEL_TO_DECIMAL_NB((uint32_t)sConfigInjected->InjectedChannel)
                                                                               + 1UL) & 0x1FUL)), sConfigInjected->InjectedSamplingTime);
    }

  }

  /* Management of internal measurement channels: Vbat/VrefInt/TempSensor   */
  /* internal measurement paths enable: If internal channel selected,       */
  /* enable dedicated internal buffers and path.                            */
  /* Note: these internal measurement paths can be disabled using           */
  /* HAL_ADC_DeInit().                                                      */

  if (__LL_ADC_IS_CHANNEL_INTERNAL(sConfigInjected->InjectedChannel))
  {
    tmp_config_internal_channel = LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance));

    /* If the requested internal measurement path has already been enabled,   */
    /* bypass the configuration processing.                                   */
    if ((sConfigInjected->InjectedChannel == ADC_CHANNEL_TEMPSENSOR)
        && ((tmp_config_internal_channel & LL_ADC_PATH_INTERNAL_TEMPSENSOR) == 0UL))
    {
      if (ADC_TEMPERATURE_SENSOR_INSTANCE(hadc))
      {
        LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance),
                                       LL_ADC_PATH_INTERNAL_TEMPSENSOR | tmp_config_internal_channel);

        /* Delay for temperature sensor stabilization time */
        /* Wait loop initialization and execution */
        /* Note: Variable divided by 2 to compensate partially              */
        /*       CPU processing cycles, scaling in us split to not          */
        /*       exceed 32 bits register capacity and handle low frequency. */
        wait_loop_index = ((LL_ADC_DELAY_TEMPSENSOR_STAB_US / 10UL) * (((SystemCoreClock / (100000UL * 2UL)) + 1UL) + 1UL));
        while (wait_loop_index != 0UL)
        {
          wait_loop_index--;
        }
      }
    }
    else if ((sConfigInjected->InjectedChannel == ADC_CHANNEL_VBAT)
             && ((tmp_config_internal_channel & LL_ADC_PATH_INTERNAL_VBAT) == 0UL))
    {
      if (ADC_BATTERY_VOLTAGE_INSTANCE(hadc))
      {
        LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance),
                                       LL_ADC_PATH_INTERNAL_VBAT | tmp_config_internal_channel);
      }
    }
    else if ((sConfigInjected->InjectedChannel == ADC_CHANNEL_VREFINT)
             && ((tmp_config_internal_channel & LL_ADC_PATH_INTERNAL_VREFINT) == 0UL))
    {
      if (ADC_VREFINT_INSTANCE(hadc))
      {
        LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance),
                                       LL_ADC_PATH_INTERNAL_VREFINT | tmp_config_internal_channel);
      }
    }
    else
    {
      /* nothing to do */
    }
  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}

#if defined(ADC_MULTIMODE_SUPPORT)
/**
  * @brief  Enable ADC multimode and configure multimode parameters
  * @note   Possibility to update parameters on the fly:
  *         This function initializes multimode parameters, following
  *         calls to this function can be used to reconfigure some parameters
  *         of structure "ADC_MultiModeTypeDef" on the fly, without resetting
  *         the ADCs.
  *         The setting of these parameters is conditioned to ADC state.
  *         For parameters constraints, see comments of structure
  *         "ADC_MultiModeTypeDef".
  * @note   To move back configuration from multimode to single mode, ADC must
  *         be reset (using function HAL_ADC_Init() ).
  * @param hadc Master ADC handle
  * @param multimode Structure of ADC multimode configuration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *hadc, ADC_MultiModeTypeDef *multimode)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  ADC_Common_TypeDef *tmpADC_Common;
  ADC_HandleTypeDef tmphadcSlave;
  uint32_t tmphadcSlave_conversion_on_going;

  /* Check the parameters */
  assert_param(IS_ADC_MULTIMODE_MASTER_INSTANCE(hadc->Instance));
  assert_param(IS_ADC_MULTIMODE(multimode->Mode));
  if (multimode->Mode != ADC_MODE_INDEPENDENT)
  {
    assert_param(IS_ADC_DMA_ACCESS_MULTIMODE(multimode->DMAAccessMode));
    assert_param(IS_ADC_SAMPLING_DELAY(multimode->TwoSamplingDelay));
  }

  /* Process locked */
  __HAL_LOCK(hadc);

  /* Temporary handle minimum initialization */
  __HAL_ADC_RESET_HANDLE_STATE(&tmphadcSlave);
  ADC_CLEAR_ERRORCODE(&tmphadcSlave);

  ADC_MULTI_SLAVE(hadc, &tmphadcSlave);

  if (tmphadcSlave.Instance == NULL)
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);

    /* Process unlocked */
    __HAL_UNLOCK(hadc);

    return HAL_ERROR;
  }

  /* Parameters update conditioned to ADC state:                              */
  /* Parameters that can be updated when ADC is disabled or enabled without   */
  /* conversion on going on regular group:                                    */
  /*  - Multimode DMA configuration                                           */
  /*  - Multimode DMA mode                                                    */
  tmphadcSlave_conversion_on_going = LL_ADC_REG_IsConversionOngoing((&tmphadcSlave)->Instance);
  if ((LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
      && (tmphadcSlave_conversion_on_going == 0UL))
  {
    /* Pointer to the common control register */
    tmpADC_Common = __LL_ADC_COMMON_INSTANCE(hadc->Instance);

    /* If multimode is selected, configure all multimode parameters.          */
    /* Otherwise, reset multimode parameters (can be used in case of          */
    /* transition from multimode to independent mode).                        */
    if (multimode->Mode != ADC_MODE_INDEPENDENT)
    {
      MODIFY_REG(tmpADC_Common->CCR, ADC_CCR_MDMA | ADC_CCR_DMACFG,
                 multimode->DMAAccessMode |
                 ADC_CCR_MULTI_DMACONTREQ((uint32_t)hadc->Init.DMAContinuousRequests));
      if (__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE(__LL_ADC_COMMON_INSTANCE(hadc->Instance)) == 0UL)
      {
        MODIFY_REG(tmpADC_Common->CCR,
                   ADC_CCR_DUAL |
                   ADC_CCR_DELAY,
                   multimode->Mode |
                   multimode->TwoSamplingDelay
                  );
      }
    }
    else{
      CLEAR_BIT(tmpADC_Common->CCR, ADC_CCR_MDMA | ADC_CCR_DMACFG);
      if (__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE(__LL_ADC_COMMON_INSTANCE(hadc->Instance)) == 0UL) CLEAR_BIT(tmpADC_Common->CCR, ADC_CCR_DUAL | ADC_CCR_DELAY);
    }
  }
  else{
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
    tmp_hal_status = HAL_ERROR;
  }
  __HAL_UNLOCK(hadc);
  return tmp_hal_status;
}
#endif
HAL_StatusTypeDef HAL_ADCEx_EnableInjectedQueue(ADC_HandleTypeDef *hadc){
  HAL_StatusTypeDef tmp_hal_status;
  uint32_t tmp_adc_is_conversion_on_going_regular;
  uint32_t tmp_adc_is_conversion_on_going_injected;
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  tmp_adc_is_conversion_on_going_regular = LL_ADC_REG_IsConversionOngoing(hadc->Instance);
  tmp_adc_is_conversion_on_going_injected = LL_ADC_INJ_IsConversionOngoing(hadc->Instance);
  if ((tmp_adc_is_conversion_on_going_regular == 0UL)&& (tmp_adc_is_conversion_on_going_injected == 0UL)){
    CLEAR_BIT(hadc->Instance->CFGR, ADC_CFGR_JQDIS);
    CLEAR_BIT(hadc->State, HAL_ADC_STATE_INJ_JQOVF);
    tmp_hal_status = HAL_OK;
  }else tmp_hal_status = HAL_ERROR;
  return tmp_hal_status;
}
HAL_StatusTypeDef HAL_ADCEx_DisableInjectedQueue(ADC_HandleTypeDef *hadc){
  HAL_StatusTypeDef tmp_hal_status;
  uint32_t tmp_adc_is_conversion_on_going_regular;
  uint32_t tmp_adc_is_conversion_on_going_injected;
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  tmp_adc_is_conversion_on_going_regular = LL_ADC_REG_IsConversionOngoing(hadc->Instance);
  tmp_adc_is_conversion_on_going_injected = LL_ADC_INJ_IsConversionOngoing(hadc->Instance);
  if ((tmp_adc_is_conversion_on_going_regular == 0UL) && (tmp_adc_is_conversion_on_going_injected == 0UL)){
    LL_ADC_INJ_SetQueueMode(hadc->Instance, LL_ADC_INJ_QUEUE_DISABLE);
    tmp_hal_status = HAL_OK;
  }else tmp_hal_status = HAL_ERROR;
  return tmp_hal_status;
}
HAL_StatusTypeDef HAL_ADCEx_DisableVoltageRegulator(ADC_HandleTypeDef *hadc){
  HAL_StatusTypeDef tmp_hal_status;
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  if (LL_ADC_IsEnabled(hadc->Instance) == 0UL){
    LL_ADC_DisableInternalRegulator(hadc->Instance);
    tmp_hal_status = HAL_OK;
  }else tmp_hal_status = HAL_ERROR;
  return tmp_hal_status;
}
HAL_StatusTypeDef HAL_ADCEx_EnterADCDeepPowerDownMode(ADC_HandleTypeDef *hadc){
  HAL_StatusTypeDef tmp_hal_status;
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  if (LL_ADC_IsEnabled(hadc->Instance) == 0UL){
    LL_ADC_EnableDeepPowerDown(hadc->Instance);
    tmp_hal_status = HAL_OK;
  }else tmp_hal_status = HAL_ERROR;
  return tmp_hal_status;
}

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/** @defgroup ADC_Private_Constants ADC Private Constants
  * @{
  */

#define ADC_CFGR_FIELDS_1  ((ADC_CFGR_RES    | ADC_CFGR_ALIGN   |\
                             ADC_CFGR_CONT   | ADC_CFGR_OVRMOD  |\
                             ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM |\
                             ADC_CFGR_EXTEN  | ADC_CFGR_EXTSEL))   /*!< ADC_CFGR fields of parameters that can be updated when no regular conversion is on-going */

/* Timeout values for ADC operations (enable settling time,                   */
/*   disable settling time, ...).                                             */
/*   Values defined to be higher than worst cases: low clock frequency,       */
/*   maximum prescalers.                                                      */
#define ADC_ENABLE_TIMEOUT              (2UL)    /*!< ADC enable time-out value  */
#define ADC_DISABLE_TIMEOUT             (2UL)    /*!< ADC disable time-out value */

/* Timeout to wait for current conversion on going to be completed.           */
/* Timeout fixed to longest ADC conversion possible, for 1 channel:           */
/*   - maximum sampling time (640.5 adc_clk)                                  */
/*   - ADC resolution (Tsar 12 bits= 12.5 adc_clk)                            */
/*   - System clock / ADC clock <= 4096 (hypothesis of maximum clock ratio)   */
/*   - ADC oversampling ratio 256                                             */
/*   Calculation: 653 * 4096 * 256 CPU clock cycles max                       */
/* Unit: cycles of CPU clock.                                                 */
#define ADC_CONVERSION_TIME_MAX_CPU_CYCLES (653UL * 4096UL * 256UL)
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  uint32_t tmpCFGR;
  uint32_t tmp_adc_reg_is_conversion_on_going;
  __IO uint32_t wait_loop_index = 0UL;
  uint32_t tmp_adc_is_conversion_on_going_regular;
  uint32_t tmp_adc_is_conversion_on_going_injected;

  /* Check ADC handle */
  if (hadc == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  assert_param(IS_ADC_CLOCKPRESCALER(hadc->Init.ClockPrescaler));
  assert_param(IS_ADC_RESOLUTION(hadc->Init.Resolution));
#if defined(ADC_CFGR_DFSDMCFG) &&defined(DFSDM1_Channel0)
  assert_param(IS_ADC_DFSDMCFG_MODE(hadc));
#endif
  assert_param(IS_ADC_DATA_ALIGN(hadc->Init.DataAlign));
  assert_param(IS_ADC_SCAN_MODE(hadc->Init.ScanConvMode));
  assert_param(IS_FUNCTIONAL_STATE(hadc->Init.ContinuousConvMode));
  assert_param(IS_ADC_EXTTRIG_EDGE(hadc->Init.ExternalTrigConvEdge));
  assert_param(IS_ADC_EXTTRIG(hadc, hadc->Init.ExternalTrigConv));
  assert_param(IS_FUNCTIONAL_STATE(hadc->Init.DMAContinuousRequests));
  assert_param(IS_ADC_EOC_SELECTION(hadc->Init.EOCSelection));
  assert_param(IS_ADC_OVERRUN(hadc->Init.Overrun));
  assert_param(IS_FUNCTIONAL_STATE(hadc->Init.LowPowerAutoWait));
  assert_param(IS_FUNCTIONAL_STATE(hadc->Init.OversamplingMode));

  if (hadc->Init.ScanConvMode != ADC_SCAN_DISABLE)
  {
    assert_param(IS_ADC_REGULAR_NB_CONV(hadc->Init.NbrOfConversion));
    assert_param(IS_FUNCTIONAL_STATE(hadc->Init.DiscontinuousConvMode));

    if (hadc->Init.DiscontinuousConvMode == ENABLE)
    {
      assert_param(IS_ADC_REGULAR_DISCONT_NUMBER(hadc->Init.NbrOfDiscConversion));
    }
  }

  /* DISCEN and CONT bits cannot be set at the same time */
  assert_param(!((hadc->Init.DiscontinuousConvMode == ENABLE) && (hadc->Init.ContinuousConvMode == ENABLE)));

  /* Actions performed only if ADC is coming from state reset:                */
  /* - Initialization of ADC MSP                                              */
  if (hadc->State == HAL_ADC_STATE_RESET)
  {
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    /* Init the ADC Callback settings */
    hadc->ConvCpltCallback              = HAL_ADC_ConvCpltCallback;                 /* Legacy weak callback */
    hadc->ConvHalfCpltCallback          = HAL_ADC_ConvHalfCpltCallback;             /* Legacy weak callback */
    hadc->LevelOutOfWindowCallback      = HAL_ADC_LevelOutOfWindowCallback;         /* Legacy weak callback */
    hadc->ErrorCallback                 = HAL_ADC_ErrorCallback;                    /* Legacy weak callback */
    hadc->InjectedConvCpltCallback      = HAL_ADCEx_InjectedConvCpltCallback;       /* Legacy weak callback */
    hadc->InjectedQueueOverflowCallback = HAL_ADCEx_InjectedQueueOverflowCallback;  /* Legacy weak callback */
    hadc->LevelOutOfWindow2Callback     = HAL_ADCEx_LevelOutOfWindow2Callback;      /* Legacy weak callback */
    hadc->LevelOutOfWindow3Callback     = HAL_ADCEx_LevelOutOfWindow3Callback;      /* Legacy weak callback */
    hadc->EndOfSamplingCallback         = HAL_ADCEx_EndOfSamplingCallback;          /* Legacy weak callback */

    if (hadc->MspInitCallback == NULL)
    {
      hadc->MspInitCallback = HAL_ADC_MspInit; /* Legacy weak MspInit  */
    }

    /* Init the low level hardware */
    hadc->MspInitCallback(hadc);
#else
    /* Init the low level hardware */
    HAL_ADC_MspInit(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */

    /* Set ADC error code to none */
    ADC_CLEAR_ERRORCODE(hadc);

    /* Initialize Lock */
    hadc->Lock = HAL_UNLOCKED;
  }

  /* - Exit from deep-power-down mode and ADC voltage regulator enable        */
  if (LL_ADC_IsDeepPowerDownEnabled(hadc->Instance) != 0UL)
  {
    /* Disable ADC deep power down mode */
    LL_ADC_DisableDeepPowerDown(hadc->Instance);

    /* System was in deep power down mode, calibration must
     be relaunched or a previously saved calibration factor
     re-applied once the ADC voltage regulator is enabled */
  }

  if (LL_ADC_IsInternalRegulatorEnabled(hadc->Instance) == 0UL)
  {
    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(hadc->Instance);

    /* Note: Variable divided by 2 to compensate partially              */
    /*       CPU processing cycles, scaling in us split to not          */
    /*       exceed 32 bits register capacity and handle low frequency. */
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / 10UL) * ((SystemCoreClock / (100000UL * 2UL)) + 1UL));
    while (wait_loop_index != 0UL)
    {
      wait_loop_index--;
    }
  }

  /* Verification that ADC voltage regulator is correctly enabled, whether    */
  /* or not ADC is coming from state reset (if any potential problem of       */
  /* clocking, voltage regulator would not be enabled).                       */
  if (LL_ADC_IsInternalRegulatorEnabled(hadc->Instance) == 0UL)
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

    /* Set ADC error code to ADC peripheral internal error */
    SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);

    tmp_hal_status = HAL_ERROR;
  }

  /* Configuration of ADC parameters if previous preliminary actions are      */
  /* correctly completed and if there is no conversion on going on regular    */
  /* group (ADC may already be enabled at this point if HAL_ADC_Init() is     */
  /* called to update a parameter on the fly).                                */
  tmp_adc_reg_is_conversion_on_going = LL_ADC_REG_IsConversionOngoing(hadc->Instance);

  if (((hadc->State & HAL_ADC_STATE_ERROR_INTERNAL) == 0UL)
      && (tmp_adc_reg_is_conversion_on_going == 0UL)
     )
  {
    /* Set ADC state */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_REG_BUSY,
                      HAL_ADC_STATE_BUSY_INTERNAL);

    /* Configuration of common ADC parameters                                 */

    /* Parameters update conditioned to ADC state:                            */
    /* Parameters that can be updated only when ADC is disabled:              */
    /*  - clock configuration                                                 */
    if (LL_ADC_IsEnabled(hadc->Instance) == 0UL)
    {
      if (__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE(__LL_ADC_COMMON_INSTANCE(hadc->Instance)) == 0UL)
      {
        /* Reset configuration of ADC common register CCR:                      */
        /*                                                                      */
        /*   - ADC clock mode and ACC prescaler (CKMODE and PRESC bits)are set  */
        /*     according to adc->Init.ClockPrescaler. It selects the clock      */
        /*    source and sets the clock division factor.                        */
        /*                                                                      */
        /* Some parameters of this register are not reset, since they are set   */
        /* by other functions and must be kept in case of usage of this         */
        /* function on the fly (update of a parameter of ADC_InitTypeDef        */
        /* without needing to reconfigure all other ADC groups/channels         */
        /* parameters):                                                         */
        /*   - when multimode feature is available, multimode-related           */
        /*     parameters: MDMA, DMACFG, DELAY, DUAL (set by API                */
        /*     HAL_ADCEx_MultiModeConfigChannel() )                             */
        /*   - internal measurement paths: Vbat, temperature sensor, Vref       */
        /*     (set into HAL_ADC_ConfigChannel() or                             */
        /*     HAL_ADCEx_InjectedConfigChannel() )                              */
        LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(hadc->Instance), hadc->Init.ClockPrescaler);
      }
    }

    /* Configuration of ADC:                                                  */
    /*  - resolution                               Init.Resolution            */
    /*  - data alignment                           Init.DataAlign             */
    /*  - external trigger to start conversion     Init.ExternalTrigConv      */
    /*  - external trigger polarity                Init.ExternalTrigConvEdge  */
    /*  - continuous conversion mode               Init.ContinuousConvMode    */
    /*  - overrun                                  Init.Overrun               */
    /*  - discontinuous mode                       Init.DiscontinuousConvMode */
    /*  - discontinuous mode channel count         Init.NbrOfDiscConversion   */
    tmpCFGR  = (ADC_CFGR_CONTINUOUS((uint32_t)hadc->Init.ContinuousConvMode)           |
                hadc->Init.Overrun                                                     |
                hadc->Init.DataAlign                                                   |
                hadc->Init.Resolution                                                  |
                ADC_CFGR_REG_DISCONTINUOUS((uint32_t)hadc->Init.DiscontinuousConvMode));

    if (hadc->Init.DiscontinuousConvMode == ENABLE)
    {
      tmpCFGR |= ADC_CFGR_DISCONTINUOUS_NUM(hadc->Init.NbrOfDiscConversion);
    }

    /* Enable external trigger if trigger selection is different of software  */
    /* start.                                                                 */
    /* Note: This configuration keeps the hardware feature of parameter       */
    /*       ExternalTrigConvEdge "trigger edge none" equivalent to           */
    /*       software start.                                                  */
    if (hadc->Init.ExternalTrigConv != ADC_SOFTWARE_START)
    {
      tmpCFGR |= ((hadc->Init.ExternalTrigConv & ADC_CFGR_EXTSEL)
                  | hadc->Init.ExternalTrigConvEdge
                 );
    }

    /* Update Configuration Register CFGR */
    MODIFY_REG(hadc->Instance->CFGR, ADC_CFGR_FIELDS_1, tmpCFGR);

    /* Parameters update conditioned to ADC state:                            */
    /* Parameters that can be updated when ADC is disabled or enabled without */
    /* conversion on going on regular and injected groups:                    */
    /*  - DMA continuous request          Init.DMAContinuousRequests          */
    /*  - LowPowerAutoWait feature        Init.LowPowerAutoWait               */
    /*  - Oversampling parameters         Init.Oversampling                   */
    tmp_adc_is_conversion_on_going_regular = LL_ADC_REG_IsConversionOngoing(hadc->Instance);
    tmp_adc_is_conversion_on_going_injected = LL_ADC_INJ_IsConversionOngoing(hadc->Instance);
    if ((tmp_adc_is_conversion_on_going_regular == 0UL)
        && (tmp_adc_is_conversion_on_going_injected == 0UL)
       )
    {
      tmpCFGR = (ADC_CFGR_DFSDM(hadc)                                            |
                 ADC_CFGR_AUTOWAIT((uint32_t)hadc->Init.LowPowerAutoWait)        |
                 ADC_CFGR_DMACONTREQ((uint32_t)hadc->Init.DMAContinuousRequests));

      MODIFY_REG(hadc->Instance->CFGR, ADC_CFGR_FIELDS_2, tmpCFGR);

      if (hadc->Init.OversamplingMode == ENABLE)
      {
        assert_param(IS_ADC_OVERSAMPLING_RATIO(hadc->Init.Oversampling.Ratio));
        assert_param(IS_ADC_RIGHT_BIT_SHIFT(hadc->Init.Oversampling.RightBitShift));
        assert_param(IS_ADC_TRIGGERED_OVERSAMPLING_MODE(hadc->Init.Oversampling.TriggeredMode));
        assert_param(IS_ADC_REGOVERSAMPLING_MODE(hadc->Init.Oversampling.OversamplingStopReset));

        /* Configuration of Oversampler:                                      */
        /*  - Oversampling Ratio                                              */
        /*  - Right bit shift                                                 */
        /*  - Triggered mode                                                  */
        /*  - Oversampling mode (continued/resumed)                           */
        MODIFY_REG(hadc->Instance->CFGR2,
                   ADC_CFGR2_OVSR  |
                   ADC_CFGR2_OVSS  |
                   ADC_CFGR2_TROVS |
                   ADC_CFGR2_ROVSM,
                   ADC_CFGR2_ROVSE                       |
                   hadc->Init.Oversampling.Ratio         |
                   hadc->Init.Oversampling.RightBitShift |
                   hadc->Init.Oversampling.TriggeredMode |
                   hadc->Init.Oversampling.OversamplingStopReset
                  );
      }
      else
      {
        /* Disable ADC oversampling scope on ADC group regular */
        CLEAR_BIT(hadc->Instance->CFGR2, ADC_CFGR2_ROVSE);
      }

    }

    /* Configuration of regular group sequencer:                              */
    /* - if scan mode is disabled, regular channels sequence length is set to */
    /*   0x00: 1 channel converted (channel on regular rank 1)                */
    /*   Parameter "NbrOfConversion" is discarded.                            */
    /*   Note: Scan mode is not present by hardware on this device, but       */
    /*   emulated by software for alignment over all STM32 devices.           */
    /* - if scan mode is enabled, regular channels sequence length is set to  */
    /*   parameter "NbrOfConversion".                                         */

    if (hadc->Init.ScanConvMode == ADC_SCAN_ENABLE)
    {
      /* Set number of ranks in regular group sequencer */
      MODIFY_REG(hadc->Instance->SQR1, ADC_SQR1_L, (hadc->Init.NbrOfConversion - (uint8_t)1));
    }
    else
    {
      CLEAR_BIT(hadc->Instance->SQR1, ADC_SQR1_L);
    }

    /* Initialize the ADC state */
    /* Clear HAL_ADC_STATE_BUSY_INTERNAL bit, set HAL_ADC_STATE_READY bit */
    ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_BUSY_INTERNAL, HAL_ADC_STATE_READY);
  }
  else
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

    tmp_hal_status = HAL_ERROR;
  }

  /* Return function status */
  return tmp_hal_status;
}

/**
  * @brief  Deinitialize the ADC peripheral registers to their default reset
  *         values, with deinitialization of the ADC MSP.
  * @note   For devices with several ADCs: reset of ADC common registers is done
  *         only if all ADCs sharing the same common group are disabled.
  *         (function "HAL_ADC_MspDeInit()" is also called under the same conditions:
  *         all ADC instances use the same core clock at RCC level, disabling
  *         the core clock reset all ADC instances).
  *         If this is not the case, reset of these common parameters reset is
  *         bypassed without error reporting: it can be the intended behavior in
  *         case of reset of a single ADC while the other ADCs sharing the same
  *         common group is still running.
  * @note   By default, HAL_ADC_DeInit() set ADC in mode deep power-down:
  *         this saves more power by reducing leakage currents
  *         and is particularly interesting before entering MCU low-power modes.
  * @param hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

  /* Check ADC handle */
  if (hadc == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Set ADC state */
  SET_BIT(hadc->State, HAL_ADC_STATE_BUSY_INTERNAL);

  /* Stop potential conversion on going */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_REGULAR_INJECTED_GROUP);

  /* Disable ADC peripheral if conversions are effectively stopped            */
  /* Flush register JSQR: reset the queue sequencer when injected             */
  /* queue sequencer is enabled and ADC disabled.                             */
  /* The software and hardware triggers of the injected sequence are both     */
  /* internally disabled just after the completion of the last valid          */
  /* injected sequence.                                                       */
  SET_BIT(hadc->Instance->CFGR, ADC_CFGR_JQM);

  /* Disable ADC peripheral if conversions are effectively stopped */
  if (tmp_hal_status == HAL_OK)
  {
    /* Disable the ADC peripheral */
    tmp_hal_status = ADC_Disable(hadc);

    /* Check if ADC is effectively disabled */
    if (tmp_hal_status == HAL_OK)
    {
      /* Change ADC state */
      hadc->State = HAL_ADC_STATE_READY;
    }
  }

  /* Note: HAL ADC deInit is done independently of ADC conversion stop        */
  /*       and disable return status. In case of status fail, attempt to      */
  /*       perform deinitialization anyway and it is up user code in          */
  /*       in HAL_ADC_MspDeInit() to reset the ADC peripheral using           */
  /*       system RCC hard reset.                                             */

  /* ========== Reset ADC registers ========== */
  /* Reset register IER */
  __HAL_ADC_DISABLE_IT(hadc, (ADC_IT_AWD3  | ADC_IT_AWD2 | ADC_IT_AWD1 |
                              ADC_IT_JQOVF | ADC_IT_OVR  |
                              ADC_IT_JEOS  | ADC_IT_JEOC |
                              ADC_IT_EOS   | ADC_IT_EOC  |
                              ADC_IT_EOSMP | ADC_IT_RDY));

  /* Reset register ISR */
  __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_AWD3  | ADC_FLAG_AWD2 | ADC_FLAG_AWD1 |
                              ADC_FLAG_JQOVF | ADC_FLAG_OVR  |
                              ADC_FLAG_JEOS  | ADC_FLAG_JEOC |
                              ADC_FLAG_EOS   | ADC_FLAG_EOC  |
                              ADC_FLAG_EOSMP | ADC_FLAG_RDY));

  /* Reset register CR */
  /* Bits ADC_CR_JADSTP, ADC_CR_ADSTP, ADC_CR_JADSTART, ADC_CR_ADSTART,
     ADC_CR_ADCAL, ADC_CR_ADDIS and ADC_CR_ADEN are in access mode "read-set":
     no direct reset applicable.
     Update CR register to reset value where doable by software */
  CLEAR_BIT(hadc->Instance->CR, ADC_CR_ADVREGEN | ADC_CR_ADCALDIF);
  SET_BIT(hadc->Instance->CR, ADC_CR_DEEPPWD);

  /* Reset register CFGR */
  CLEAR_BIT(hadc->Instance->CFGR, ADC_CFGR_FIELDS);
  SET_BIT(hadc->Instance->CFGR, ADC_CFGR_JQDIS);

  /* Reset register CFGR2 */
  CLEAR_BIT(hadc->Instance->CFGR2, ADC_CFGR2_ROVSM  | ADC_CFGR2_TROVS   | ADC_CFGR2_OVSS |
            ADC_CFGR2_OVSR  | ADC_CFGR2_JOVSE | ADC_CFGR2_ROVSE);

  /* Reset register SMPR1 */
  CLEAR_BIT(hadc->Instance->SMPR1, ADC_SMPR1_FIELDS);

  /* Reset register SMPR2 */
  CLEAR_BIT(hadc->Instance->SMPR2, ADC_SMPR2_SMP18 | ADC_SMPR2_SMP17 | ADC_SMPR2_SMP16 |
            ADC_SMPR2_SMP15 | ADC_SMPR2_SMP14 | ADC_SMPR2_SMP13 |
            ADC_SMPR2_SMP12 | ADC_SMPR2_SMP11 | ADC_SMPR2_SMP10);

  /* Reset register TR1 */
  CLEAR_BIT(hadc->Instance->TR1, ADC_TR1_HT1 | ADC_TR1_LT1);

  /* Reset register TR2 */
  CLEAR_BIT(hadc->Instance->TR2, ADC_TR2_HT2 | ADC_TR2_LT2);

  /* Reset register TR3 */
  CLEAR_BIT(hadc->Instance->TR3, ADC_TR3_HT3 | ADC_TR3_LT3);

  /* Reset register SQR1 */
  CLEAR_BIT(hadc->Instance->SQR1, ADC_SQR1_SQ4 | ADC_SQR1_SQ3 | ADC_SQR1_SQ2 |
            ADC_SQR1_SQ1 | ADC_SQR1_L);

  /* Reset register SQR2 */
  CLEAR_BIT(hadc->Instance->SQR2, ADC_SQR2_SQ9 | ADC_SQR2_SQ8 | ADC_SQR2_SQ7 |
            ADC_SQR2_SQ6 | ADC_SQR2_SQ5);

  /* Reset register SQR3 */
  CLEAR_BIT(hadc->Instance->SQR3, ADC_SQR3_SQ14 | ADC_SQR3_SQ13 | ADC_SQR3_SQ12 |
            ADC_SQR3_SQ11 | ADC_SQR3_SQ10);

  /* Reset register SQR4 */
  CLEAR_BIT(hadc->Instance->SQR4, ADC_SQR4_SQ16 | ADC_SQR4_SQ15);

  /* Register JSQR was reset when the ADC was disabled */

  /* Reset register DR */
  /* bits in access mode read only, no direct reset applicable*/

  /* Reset register OFR1 */
  CLEAR_BIT(hadc->Instance->OFR1, ADC_OFR1_OFFSET1_EN | ADC_OFR1_OFFSET1_CH | ADC_OFR1_OFFSET1);
  /* Reset register OFR2 */
  CLEAR_BIT(hadc->Instance->OFR2, ADC_OFR2_OFFSET2_EN | ADC_OFR2_OFFSET2_CH | ADC_OFR2_OFFSET2);
  /* Reset register OFR3 */
  CLEAR_BIT(hadc->Instance->OFR3, ADC_OFR3_OFFSET3_EN | ADC_OFR3_OFFSET3_CH | ADC_OFR3_OFFSET3);
  /* Reset register OFR4 */
  CLEAR_BIT(hadc->Instance->OFR4, ADC_OFR4_OFFSET4_EN | ADC_OFR4_OFFSET4_CH | ADC_OFR4_OFFSET4);

  /* Reset registers JDR1, JDR2, JDR3, JDR4 */
  /* bits in access mode read only, no direct reset applicable*/

  /* Reset register AWD2CR */
  CLEAR_BIT(hadc->Instance->AWD2CR, ADC_AWD2CR_AWD2CH);

  /* Reset register AWD3CR */
  CLEAR_BIT(hadc->Instance->AWD3CR, ADC_AWD3CR_AWD3CH);

  /* Reset register DIFSEL */
  CLEAR_BIT(hadc->Instance->DIFSEL, ADC_DIFSEL_DIFSEL);

  /* Reset register CALFACT */
  CLEAR_BIT(hadc->Instance->CALFACT, ADC_CALFACT_CALFACT_D | ADC_CALFACT_CALFACT_S);


  /* ========== Reset common ADC registers ========== */

  /* Software is allowed to change common parameters only when all the other
     ADCs are disabled.   */
  if (__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE(__LL_ADC_COMMON_INSTANCE(hadc->Instance)) == 0UL)
  {
    /* Reset configuration of ADC common register CCR:
      - clock mode: CKMODE, PRESCEN
      - multimode related parameters (when this feature is available): MDMA,
        DMACFG, DELAY, DUAL (set by HAL_ADCEx_MultiModeConfigChannel() API)
      - internal measurement paths: Vbat, temperature sensor, Vref (set into
        HAL_ADC_ConfigChannel() or HAL_ADCEx_InjectedConfigChannel() )
    */
    ADC_CLEAR_COMMON_CONTROL_REGISTER(hadc);

    /* ========== Hard reset ADC peripheral ========== */
    /* Performs a global reset of the entire ADC peripherals instances        */
    /* sharing the same common ADC instance: ADC state is forced to           */
    /* a similar state as after device power-on.                              */
    /* Note: A possible implementation is to add RCC bus reset of ADC         */
    /* (for example, using macro                                              */
    /*  __HAL_RCC_ADC..._FORCE_RESET()/..._RELEASE_RESET()/..._CLK_DISABLE()) */
    /* in function "void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)":         */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    if (hadc->MspDeInitCallback == NULL)
    {
      hadc->MspDeInitCallback = HAL_ADC_MspDeInit; /* Legacy weak MspDeInit  */
    }

    /* DeInit the low level hardware */
    hadc->MspDeInitCallback(hadc);
#else
    /* DeInit the low level hardware */
    HAL_ADC_MspDeInit(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  /* Set ADC error code to none */
  ADC_CLEAR_ERRORCODE(hadc);

  /* Reset injected channel configuration parameters */
  hadc->InjectionConfig.ContextQueue = 0;
  hadc->InjectionConfig.ChannelCount = 0;

  /* Set ADC state */
  hadc->State = HAL_ADC_STATE_RESET;

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User ADC Callback
  *         To be used instead of the weak predefined callback
  * @param  hadc Pointer to a ADC_HandleTypeDef structure that contains
  *                the configuration information for the specified ADC.
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_ADC_CONVERSION_COMPLETE_CB_ID      ADC conversion complete callback ID
  *          @arg @ref HAL_ADC_CONVERSION_HALF_CB_ID          ADC conversion DMA half-transfer callback ID
  *          @arg @ref HAL_ADC_LEVEL_OUT_OF_WINDOW_1_CB_ID    ADC analog watchdog 1 callback ID
  *          @arg @ref HAL_ADC_ERROR_CB_ID                    ADC error callback ID
  *          @arg @ref HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID  ADC group injected conversion complete callback ID
  *          @arg @ref HAL_ADC_INJ_QUEUE_OVEFLOW_CB_ID        ADC group injected context queue overflow callback ID
  *          @arg @ref HAL_ADC_LEVEL_OUT_OF_WINDOW_2_CB_ID    ADC analog watchdog 2 callback ID
  *          @arg @ref HAL_ADC_LEVEL_OUT_OF_WINDOW_3_CB_ID    ADC analog watchdog 3 callback ID
  *          @arg @ref HAL_ADC_END_OF_SAMPLING_CB_ID          ADC end of sampling callback ID
  *          @arg @ref HAL_ADC_MSPINIT_CB_ID                  ADC Msp Init callback ID
  *          @arg @ref HAL_ADC_MSPDEINIT_CB_ID                ADC Msp DeInit callback ID
  *          @arg @ref HAL_ADC_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_ADC_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_RegisterCallback(ADC_HandleTypeDef *hadc, HAL_ADC_CallbackIDTypeDef CallbackID,
                                           pADC_CallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hadc->ErrorCode |= HAL_ADC_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  if ((hadc->State & HAL_ADC_STATE_READY) != 0UL)
  {
    switch (CallbackID)
    {
      case HAL_ADC_CONVERSION_COMPLETE_CB_ID :
        hadc->ConvCpltCallback = pCallback;
        break;

      case HAL_ADC_CONVERSION_HALF_CB_ID :
        hadc->ConvHalfCpltCallback = pCallback;
        break;

      case HAL_ADC_LEVEL_OUT_OF_WINDOW_1_CB_ID :
        hadc->LevelOutOfWindowCallback = pCallback;
        break;

      case HAL_ADC_ERROR_CB_ID :
        hadc->ErrorCallback = pCallback;
        break;

      case HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID :
        hadc->InjectedConvCpltCallback = pCallback;
        break;

      case HAL_ADC_INJ_QUEUE_OVEFLOW_CB_ID :
        hadc->InjectedQueueOverflowCallback = pCallback;
        break;

      case HAL_ADC_LEVEL_OUT_OF_WINDOW_2_CB_ID :
        hadc->LevelOutOfWindow2Callback = pCallback;
        break;

      case HAL_ADC_LEVEL_OUT_OF_WINDOW_3_CB_ID :
        hadc->LevelOutOfWindow3Callback = pCallback;
        break;

      case HAL_ADC_END_OF_SAMPLING_CB_ID :
        hadc->EndOfSamplingCallback = pCallback;
        break;

      case HAL_ADC_MSPINIT_CB_ID :
        hadc->MspInitCallback = pCallback;
        break;

      case HAL_ADC_MSPDEINIT_CB_ID :
        hadc->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hadc->ErrorCode |= HAL_ADC_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status = HAL_ERROR;
        break;
    }
  }
  else if (HAL_ADC_STATE_RESET == hadc->State)
  {
    switch (CallbackID)
    {
      case HAL_ADC_MSPINIT_CB_ID :
        hadc->MspInitCallback = pCallback;
        break;

      case HAL_ADC_MSPDEINIT_CB_ID :
        hadc->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hadc->ErrorCode |= HAL_ADC_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status = HAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hadc->ErrorCode |= HAL_ADC_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  return status;
}

/**
  * @brief  Unregister a ADC Callback
  *         ADC callback is redirected to the weak predefined callback
  * @param  hadc Pointer to a ADC_HandleTypeDef structure that contains
  *                the configuration information for the specified ADC.
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_ADC_CONVERSION_COMPLETE_CB_ID      ADC conversion complete callback ID
  *          @arg @ref HAL_ADC_CONVERSION_HALF_CB_ID          ADC conversion DMA half-transfer callback ID
  *          @arg @ref HAL_ADC_LEVEL_OUT_OF_WINDOW_1_CB_ID    ADC analog watchdog 1 callback ID
  *          @arg @ref HAL_ADC_ERROR_CB_ID                    ADC error callback ID
  *          @arg @ref HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID  ADC group injected conversion complete callback ID
  *          @arg @ref HAL_ADC_INJ_QUEUE_OVEFLOW_CB_ID        ADC group injected context queue overflow callback ID
  *          @arg @ref HAL_ADC_LEVEL_OUT_OF_WINDOW_2_CB_ID    ADC analog watchdog 2 callback ID
  *          @arg @ref HAL_ADC_LEVEL_OUT_OF_WINDOW_3_CB_ID    ADC analog watchdog 3 callback ID
  *          @arg @ref HAL_ADC_END_OF_SAMPLING_CB_ID          ADC end of sampling callback ID
  *          @arg @ref HAL_ADC_MSPINIT_CB_ID                  ADC Msp Init callback ID
  *          @arg @ref HAL_ADC_MSPDEINIT_CB_ID                ADC Msp DeInit callback ID
  *          @arg @ref HAL_ADC_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_ADC_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_UnRegisterCallback(ADC_HandleTypeDef *hadc, HAL_ADC_CallbackIDTypeDef CallbackID)
{
  HAL_StatusTypeDef status = HAL_OK;

  if ((hadc->State & HAL_ADC_STATE_READY) != 0UL)
  {
    switch (CallbackID)
    {
      case HAL_ADC_CONVERSION_COMPLETE_CB_ID :
        hadc->ConvCpltCallback = HAL_ADC_ConvCpltCallback;
        break;

      case HAL_ADC_CONVERSION_HALF_CB_ID :
        hadc->ConvHalfCpltCallback = HAL_ADC_ConvHalfCpltCallback;
        break;

      case HAL_ADC_LEVEL_OUT_OF_WINDOW_1_CB_ID :
        hadc->LevelOutOfWindowCallback = HAL_ADC_LevelOutOfWindowCallback;
        break;

      case HAL_ADC_ERROR_CB_ID :
        hadc->ErrorCallback = HAL_ADC_ErrorCallback;
        break;

      case HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID :
        hadc->InjectedConvCpltCallback = HAL_ADCEx_InjectedConvCpltCallback;
        break;

      case HAL_ADC_INJ_QUEUE_OVEFLOW_CB_ID :
        hadc->InjectedQueueOverflowCallback = HAL_ADCEx_InjectedQueueOverflowCallback;
        break;

      case HAL_ADC_LEVEL_OUT_OF_WINDOW_2_CB_ID :
        hadc->LevelOutOfWindow2Callback = HAL_ADCEx_LevelOutOfWindow2Callback;
        break;

      case HAL_ADC_LEVEL_OUT_OF_WINDOW_3_CB_ID :
        hadc->LevelOutOfWindow3Callback = HAL_ADCEx_LevelOutOfWindow3Callback;
        break;

      case HAL_ADC_END_OF_SAMPLING_CB_ID :
        hadc->EndOfSamplingCallback = HAL_ADCEx_EndOfSamplingCallback;
        break;

      case HAL_ADC_MSPINIT_CB_ID :
        hadc->MspInitCallback = HAL_ADC_MspInit; /* Legacy weak MspInit              */
        break;

      case HAL_ADC_MSPDEINIT_CB_ID :
        hadc->MspDeInitCallback = HAL_ADC_MspDeInit; /* Legacy weak MspDeInit            */
        break;

      default :
        /* Update the error code */
        hadc->ErrorCode |= HAL_ADC_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else if (HAL_ADC_STATE_RESET == hadc->State)
  {
    switch (CallbackID)
    {
      case HAL_ADC_MSPINIT_CB_ID :
        hadc->MspInitCallback = HAL_ADC_MspInit;                   /* Legacy weak MspInit              */
        break;

      case HAL_ADC_MSPDEINIT_CB_ID :
        hadc->MspDeInitCallback = HAL_ADC_MspDeInit;               /* Legacy weak MspDeInit            */
        break;

      default :
        /* Update the error code */
        hadc->ErrorCode |= HAL_ADC_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hadc->ErrorCode |= HAL_ADC_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  return status;
}

#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup ADC_Exported_Functions_Group2 ADC Input and Output operation functions
  * @brief    ADC IO operation functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Start conversion of regular group.
      (+) Stop conversion of regular group.
      (+) Poll for conversion complete on regular group.
      (+) Poll for conversion event.
      (+) Get result of regular channel conversion.
      (+) Start conversion of regular group and enable interruptions.
      (+) Stop conversion of regular group and disable interruptions.
      (+) Handle ADC interrupt request
      (+) Start conversion of regular group and enable DMA transfer.
      (+) Stop conversion of regular group and disable ADC DMA transfer.
@endverbatim
  * @{
  */

/**
  * @brief  Enable ADC, start conversion of regular group.
  * @note   Interruptions enabled in this function: None.
  * @note   Case of multimode enabled (when multimode feature is available):
  *           if ADC is Slave, ADC is enabled but conversion is not started,
  *           if ADC is master, ADC is enabled and multimode conversion is started.
  * @param hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;
#if defined(ADC_MULTIMODE_SUPPORT)
  const ADC_TypeDef *tmpADC_Master;
  uint32_t tmp_multimode_config = LL_ADC_GetMultimode(__LL_ADC_COMMON_INSTANCE(hadc->Instance));
#endif

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Perform ADC enable and conversion start if no conversion is on going */
  if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
  {
    /* Process locked */
    __HAL_LOCK(hadc);

    /* Enable the ADC peripheral */
    tmp_hal_status = ADC_Enable(hadc);

    /* Start conversion if ADC is effectively enabled */
    if (tmp_hal_status == HAL_OK)
    {
      /* Set ADC state                                                        */
      /* - Clear state bitfield related to regular group conversion results   */
      /* - Set state bitfield related to regular operation                    */
      ADC_STATE_CLR_SET(hadc->State,
                        HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP,
                        HAL_ADC_STATE_REG_BUSY);

#if defined(ADC_MULTIMODE_SUPPORT)
      /* Reset HAL_ADC_STATE_MULTIMODE_SLAVE bit
        - if ADC instance is master or if multimode feature is not available
        - if multimode setting is disabled (ADC instance slave in independent mode) */
      if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
          || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
         )
      {
        CLEAR_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
      }
#endif

      /* Set ADC error code */
      /* Check if a conversion is on going on ADC group injected */
      if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY))
      {
        /* Reset ADC error code fields related to regular conversions only */
        CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
      }
      else
      {
        /* Reset all ADC error code fields */
        ADC_CLEAR_ERRORCODE(hadc);
      }

      /* Clear ADC group regular conversion flag and overrun flag               */
      /* (To ensure of no unknown state from potential previous ADC operations) */
      __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

      /* Process unlocked */
      /* Unlock before starting ADC conversions: in case of potential         */
      /* interruption, to let the process to ADC IRQ Handler.                 */
      __HAL_UNLOCK(hadc);

      /* Enable conversion of regular group.                                  */
      /* If software start has been selected, conversion starts immediately.  */
      /* If external trigger has been selected, conversion will start at next */
      /* trigger event.                                                       */
      /* Case of multimode enabled (when multimode feature is available):     */
      /*  - if ADC is slave and dual regular conversions are enabled, ADC is  */
      /*    enabled only (conversion is not started),                         */
      /*  - if ADC is master, ADC is enabled and conversion is started.       */
#if defined(ADC_MULTIMODE_SUPPORT)
      if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
          || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
          || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_SIMULT)
          || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_ALTERN)
         )
      {
        /* ADC instance is not a multimode slave instance with multimode regular conversions enabled */
        if (READ_BIT(hadc->Instance->CFGR, ADC_CFGR_JAUTO) != 0UL)
        {
          ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);
        }

        /* Start ADC group regular conversion */
        LL_ADC_REG_StartConversion(hadc->Instance);
      }
      else
      {
        /* ADC instance is a multimode slave instance with multimode regular conversions enabled */
        SET_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
        /* if Master ADC JAUTO bit is set, update Slave State in setting
           HAL_ADC_STATE_INJ_BUSY bit and in resetting HAL_ADC_STATE_INJ_EOC bit */
        tmpADC_Master = __LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance);
        if (READ_BIT(tmpADC_Master->CFGR, ADC_CFGR_JAUTO) != 0UL)
        {
          ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);
        }

      }
#else
      if (READ_BIT(hadc->Instance->CFGR, ADC_CFGR_JAUTO) != 0UL)
      {
        ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);
      }

      /* Start ADC group regular conversion */
      LL_ADC_REG_StartConversion(hadc->Instance);
#endif
    }
    else
    {
      /* Process unlocked */
      __HAL_UNLOCK(hadc);
    }
  }
  else
  {
    tmp_hal_status = HAL_BUSY;
  }

  /* Return function status */
  return tmp_hal_status;
}

/**
  * @brief  Stop ADC conversion of regular group (and injected channels in
  *         case of auto_injection mode), disable ADC peripheral.
  * @note:  ADC peripheral disable is forcing stop of potential
  *         conversion on injected group. If injected group is under use, it
  *         should be preliminarily stopped using HAL_ADCEx_InjectedStop function.
  * @param hadc ADC handle
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Process locked */
  __HAL_LOCK(hadc);

  /* 1. Stop potential conversion on going, on ADC groups regular and injected */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_REGULAR_INJECTED_GROUP);

  /* Disable ADC peripheral if conversions are effectively stopped */
  if (tmp_hal_status == HAL_OK)
  {
    /* 2. Disable the ADC peripheral */
    tmp_hal_status = ADC_Disable(hadc);

    /* Check if ADC is effectively disabled */
    if (tmp_hal_status == HAL_OK)
    {
      /* Set ADC state */
      ADC_STATE_CLR_SET(hadc->State,
                        HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
                        HAL_ADC_STATE_READY);
    }
  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}

/**
  * @brief  Wait for regular group conversion to be completed.
  * @note   ADC conversion flags EOS (end of sequence) and EOC (end of
  *         conversion) are cleared by this function, with an exception:
  *         if low power feature "LowPowerAutoWait" is enabled, flags are
  *         not cleared to not interfere with this feature until data register
  *         is read using function HAL_ADC_GetValue().
  * @note   This function cannot be used in a particular setup: ADC configured
  *         in DMA mode and polling for end of each conversion (ADC init
  *         parameter "EOCSelection" set to ADC_EOC_SINGLE_CONV).
  *         In this case, DMA resets the flag EOC and polling cannot be
  *         performed on each conversion. Nevertheless, polling can still
  *         be performed on the complete sequence (ADC init
  *         parameter "EOCSelection" set to ADC_EOC_SEQ_CONV).
  * @param hadc ADC handle
  * @param Timeout Timeout value in millisecond.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout)
{
  uint32_t tickstart;
  uint32_t tmp_Flag_End;
  uint32_t tmp_cfgr;
#if defined(ADC_MULTIMODE_SUPPORT)
  const ADC_TypeDef *tmpADC_Master;
  uint32_t tmp_multimode_config = LL_ADC_GetMultimode(__LL_ADC_COMMON_INSTANCE(hadc->Instance));
#endif

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* If end of conversion selected to end of sequence conversions */
  if (hadc->Init.EOCSelection == ADC_EOC_SEQ_CONV)
  {
    tmp_Flag_End = ADC_FLAG_EOS;
  }
  /* If end of conversion selected to end of unitary conversion */
  else /* ADC_EOC_SINGLE_CONV */
  {
    /* Verification that ADC configuration is compliant with polling for      */
    /* each conversion:                                                       */
    /* Particular case is ADC configured in DMA mode and ADC sequencer with   */
    /* several ranks and polling for end of each conversion.                  */
    /* For code simplicity sake, this particular case is generalized to       */
    /* ADC configured in DMA mode and and polling for end of each conversion. */
#if defined(ADC_MULTIMODE_SUPPORT)
    if ((tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
        || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_SIMULT)
        || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_ALTERN)
       )
    {
      /* Check ADC DMA mode in independent mode on ADC group regular */
      if (READ_BIT(hadc->Instance->CFGR, ADC_CFGR_DMAEN) != 0UL)
      {
        SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
        return HAL_ERROR;
      }
      else
      {
        tmp_Flag_End = (ADC_FLAG_EOC);
      }
    }
    else
    {
      /* Check ADC DMA mode in multimode on ADC group regular */
      if (LL_ADC_GetMultiDMATransfer(__LL_ADC_COMMON_INSTANCE(hadc->Instance)) != LL_ADC_MULTI_REG_DMA_EACH_ADC)
      {
        SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
        return HAL_ERROR;
      }
      else
      {
        tmp_Flag_End = (ADC_FLAG_EOC);
      }
    }
#else
    /* Check ADC DMA mode */
    if (READ_BIT(hadc->Instance->CFGR, ADC_CFGR_DMAEN) != 0UL)
    {
      SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
      return HAL_ERROR;
    }
    else
    {
      tmp_Flag_End = (ADC_FLAG_EOC);
    }
#endif
  }

  /* Get tick count */
  tickstart = HAL_GetTick();

  /* Wait until End of unitary conversion or sequence conversions flag is raised */
  while ((hadc->Instance->ISR & tmp_Flag_End) == 0UL)
  {
    /* Check if timeout is disabled (set to infinite wait) */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0UL))
      {
        /* New check to avoid false timeout detection in case of preemption */
        if ((hadc->Instance->ISR & tmp_Flag_End) == 0UL)
        {
          /* Update ADC state machine to timeout */
          SET_BIT(hadc->State, HAL_ADC_STATE_TIMEOUT);

          /* Process unlocked */
          __HAL_UNLOCK(hadc);

          return HAL_TIMEOUT;
        }
      }
    }
  }

  /* Update ADC state machine */
  SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);

  /* Determine whether any further conversion upcoming on group regular       */
  /* by external trigger, continuous mode or scan sequence on going.          */
  if ((LL_ADC_REG_IsTriggerSourceSWStart(hadc->Instance) != 0UL)
      && (hadc->Init.ContinuousConvMode == DISABLE)
     )
  {
    /* Check whether end of sequence is reached */
    if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS))
    {
      /* Set ADC state */
      CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);

      if ((hadc->State & HAL_ADC_STATE_INJ_BUSY) == 0UL)
      {
        SET_BIT(hadc->State, HAL_ADC_STATE_READY);
      }
    }
  }

  /* Get relevant register CFGR in ADC instance of ADC master or slave        */
  /* in function of multimode state (for devices with multimode               */
  /* available).                                                              */
#if defined(ADC_MULTIMODE_SUPPORT)
  if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
      || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
      || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_SIMULT)
      || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_ALTERN)
     )
  {
    /* Retrieve handle ADC CFGR register */
    tmp_cfgr = READ_REG(hadc->Instance->CFGR);
  }
  else
  {
    /* Retrieve Master ADC CFGR register */
    tmpADC_Master = __LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance);
    tmp_cfgr = READ_REG(tmpADC_Master->CFGR);
  }
#else
  /* Retrieve handle ADC CFGR register */
  tmp_cfgr = READ_REG(hadc->Instance->CFGR);
#endif

  /* Clear polled flag */
  if (tmp_Flag_End == ADC_FLAG_EOS)
  {
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOS);
  }
  else
  {
    /* Clear end of conversion EOC flag of regular group if low power feature */
    /* "LowPowerAutoWait " is disabled, to not interfere with this feature    */
    /* until data register is read using function HAL_ADC_GetValue().         */
    if (READ_BIT(tmp_cfgr, ADC_CFGR_AUTDLY) == 0UL)
    {
      __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS));
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Poll for ADC event.
  * @param hadc ADC handle
  * @param EventType the ADC event type.
  *          This parameter can be one of the following values:
  *            @arg @ref ADC_EOSMP_EVENT  ADC End of Sampling event
  *            @arg @ref ADC_AWD1_EVENT   ADC Analog watchdog 1 event (main analog watchdog, present on all STM32 devices)
  *            @arg @ref ADC_AWD2_EVENT   ADC Analog watchdog 2 event (additional analog watchdog, not present on all STM32 families)
  *            @arg @ref ADC_AWD3_EVENT   ADC Analog watchdog 3 event (additional analog watchdog, not present on all STM32 families)
  *            @arg @ref ADC_OVR_EVENT    ADC Overrun event
  *            @arg @ref ADC_JQOVF_EVENT  ADC Injected context queue overflow event
  * @param Timeout Timeout value in millisecond.
  * @note   The relevant flag is cleared if found to be set, except for ADC_FLAG_OVR.
  *         Indeed, the latter is reset only if hadc->Init.Overrun field is set
  *         to ADC_OVR_DATA_OVERWRITTEN. Otherwise, data register may be potentially overwritten
  *         by a new converted data as soon as OVR is cleared.
  *         To reset OVR flag once the preserved data is retrieved, the user can resort
  *         to macro __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_PollForEvent(ADC_HandleTypeDef *hadc, uint32_t EventType, uint32_t Timeout)
{
  uint32_t tickstart;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  assert_param(IS_ADC_EVENT_TYPE(EventType));

  /* Get tick count */
  tickstart = HAL_GetTick();

  /* Check selected event flag */
  while (__HAL_ADC_GET_FLAG(hadc, EventType) == 0UL)
  {
    /* Check if timeout is disabled (set to infinite wait) */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0UL))
      {
        /* New check to avoid false timeout detection in case of preemption */
        if (__HAL_ADC_GET_FLAG(hadc, EventType) == 0UL)
        {
          /* Update ADC state machine to timeout */
          SET_BIT(hadc->State, HAL_ADC_STATE_TIMEOUT);

          /* Process unlocked */
          __HAL_UNLOCK(hadc);

          return HAL_TIMEOUT;
        }
      }
    }
  }

  switch (EventType)
  {
    /* End Of Sampling event */
    case ADC_EOSMP_EVENT:
      /* Set ADC state */
      SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOSMP);

      /* Clear the End Of Sampling flag */
      __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOSMP);

      break;

    /* Analog watchdog (level out of window) event */
    /* Note: In case of several analog watchdog enabled, if needed to know      */
    /* which one triggered and on which ADCx, test ADC state of analog watchdog */
    /* flags HAL_ADC_STATE_AWD1/2/3 using function "HAL_ADC_GetState()".        */
    /* For example:                                                             */
    /*  " if ((HAL_ADC_GetState(hadc1) & HAL_ADC_STATE_AWD1) != 0UL) "          */
    /*  " if ((HAL_ADC_GetState(hadc1) & HAL_ADC_STATE_AWD2) != 0UL) "          */
    /*  " if ((HAL_ADC_GetState(hadc1) & HAL_ADC_STATE_AWD3) != 0UL) "          */

    /* Check analog watchdog 1 flag */
    case ADC_AWD_EVENT:
      /* Set ADC state */
      SET_BIT(hadc->State, HAL_ADC_STATE_AWD1);

      /* Clear ADC analog watchdog flag */
      __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_AWD1);

      break;

    /* Check analog watchdog 2 flag */
    case ADC_AWD2_EVENT:
      /* Set ADC state */
      SET_BIT(hadc->State, HAL_ADC_STATE_AWD2);

      /* Clear ADC analog watchdog flag */
      __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_AWD2);

      break;

    /* Check analog watchdog 3 flag */
    case ADC_AWD3_EVENT:
      /* Set ADC state */
      SET_BIT(hadc->State, HAL_ADC_STATE_AWD3);

      /* Clear ADC analog watchdog flag */
      __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_AWD3);

      break;

    /* Injected context queue overflow event */
    case ADC_JQOVF_EVENT:
      /* Set ADC state */
      SET_BIT(hadc->State, HAL_ADC_STATE_INJ_JQOVF);

      /* Set ADC error code to Injected context queue overflow */
      SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_JQOVF);

      /* Clear ADC Injected context queue overflow flag */
      __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JQOVF);

      break;

    /* Overrun event */
    default: /* Case ADC_OVR_EVENT */
      /* If overrun is set to overwrite previous data, overrun event is not     */
      /* considered as an error.                                                */
      /* (cf ref manual "Managing conversions without using the DMA and without */
      /* overrun ")                                                             */
      if (hadc->Init.Overrun == ADC_OVR_DATA_PRESERVED)
      {
        /* Set ADC state */
        SET_BIT(hadc->State, HAL_ADC_STATE_REG_OVR);

        /* Set ADC error code to overrun */
        SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_OVR);
      }
      else
      {
        /* Clear ADC Overrun flag only if Overrun is set to ADC_OVR_DATA_OVERWRITTEN
           otherwise, data register is potentially overwritten by new converted data as soon
           as OVR is cleared. */
        __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
      }
      break;
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Enable ADC, start conversion of regular group with interruption.
  * @note   Interruptions enabled in this function according to initialization
  *         setting : EOC (end of conversion), EOS (end of sequence),
  *         OVR overrun.
  *         Each of these interruptions has its dedicated callback function.
  * @note   Case of multimode enabled (when multimode feature is available):
  *         HAL_ADC_Start_IT() must be called for ADC Slave first, then for
  *         ADC Master.
  *         For ADC Slave, ADC is enabled only (conversion is not started).
  *         For ADC Master, ADC is enabled and multimode conversion is started.
  * @note   To guarantee a proper reset of all interruptions once all the needed
  *         conversions are obtained, HAL_ADC_Stop_IT() must be called to ensure
  *         a correct stop of the IT-based conversions.
  * @note   By default, HAL_ADC_Start_IT() does not enable the End Of Sampling
  *         interruption. If required (e.g. in case of oversampling with trigger
  *         mode), the user must:
  *          1. first clear the EOSMP flag if set with macro __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOSMP)
  *          2. then enable the EOSMP interrupt with macro __HAL_ADC_ENABLE_IT(hadc, ADC_IT_EOSMP)
  *          before calling HAL_ADC_Start_IT().
  * @param hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;
#if defined(ADC_MULTIMODE_SUPPORT)
  const ADC_TypeDef *tmpADC_Master;
  uint32_t tmp_multimode_config = LL_ADC_GetMultimode(__LL_ADC_COMMON_INSTANCE(hadc->Instance));
#endif

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Perform ADC enable and conversion start if no conversion is on going */
  if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
  {
    /* Process locked */
    __HAL_LOCK(hadc);

    /* Enable the ADC peripheral */
    tmp_hal_status = ADC_Enable(hadc);

    /* Start conversion if ADC is effectively enabled */
    if (tmp_hal_status == HAL_OK)
    {
      /* Set ADC state                                                        */
      /* - Clear state bitfield related to regular group conversion results   */
      /* - Set state bitfield related to regular operation                    */
      ADC_STATE_CLR_SET(hadc->State,
                        HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP,
                        HAL_ADC_STATE_REG_BUSY);

#if defined(ADC_MULTIMODE_SUPPORT)
      /* Reset HAL_ADC_STATE_MULTIMODE_SLAVE bit
        - if ADC instance is master or if multimode feature is not available
        - if multimode setting is disabled (ADC instance slave in independent mode) */
      if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
          || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
         )
      {
        CLEAR_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
      }
#endif

      /* Set ADC error code */
      /* Check if a conversion is on going on ADC group injected */
      if ((hadc->State & HAL_ADC_STATE_INJ_BUSY) != 0UL)
      {
        /* Reset ADC error code fields related to regular conversions only */
        CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
      }
      else
      {
        /* Reset all ADC error code fields */
        ADC_CLEAR_ERRORCODE(hadc);
      }

      /* Clear ADC group regular conversion flag and overrun flag               */
      /* (To ensure of no unknown state from potential previous ADC operations) */
      __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

      /* Process unlocked */
      /* Unlock before starting ADC conversions: in case of potential         */
      /* interruption, to let the process to ADC IRQ Handler.                 */
      __HAL_UNLOCK(hadc);

      /* Disable all interruptions before enabling the desired ones */
      __HAL_ADC_DISABLE_IT(hadc, (ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR));

      /* Enable ADC end of conversion interrupt */
      switch (hadc->Init.EOCSelection)
      {
        case ADC_EOC_SEQ_CONV:
          __HAL_ADC_ENABLE_IT(hadc, ADC_IT_EOS);
          break;
        /* case ADC_EOC_SINGLE_CONV */
        default:
          __HAL_ADC_ENABLE_IT(hadc, ADC_IT_EOC);
          break;
      }

      /* Enable ADC overrun interrupt */
      /* If hadc->Init.Overrun is set to ADC_OVR_DATA_PRESERVED, only then is
         ADC_IT_OVR enabled; otherwise data overwrite is considered as normal
         behavior and no CPU time is lost for a non-processed interruption */
      if (hadc->Init.Overrun == ADC_OVR_DATA_PRESERVED)
      {
        __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);
      }

      /* Enable conversion of regular group.                                  */
      /* If software start has been selected, conversion starts immediately.  */
      /* If external trigger has been selected, conversion will start at next */
      /* trigger event.                                                       */
      /* Case of multimode enabled (when multimode feature is available):     */
      /*  - if ADC is slave and dual regular conversions are enabled, ADC is  */
      /*    enabled only (conversion is not started),                         */
      /*  - if ADC is master, ADC is enabled and conversion is started.       */
#if defined(ADC_MULTIMODE_SUPPORT)
      if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
          || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
          || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_SIMULT)
          || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_ALTERN)
         )
      {
        /* ADC instance is not a multimode slave instance with multimode regular conversions enabled */
        if (READ_BIT(hadc->Instance->CFGR, ADC_CFGR_JAUTO) != 0UL)
        {
          ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);

          /* Enable as well injected interruptions in case
           HAL_ADCEx_InjectedStart_IT() has not been called beforehand. This
           allows to start regular and injected conversions when JAUTO is
           set with a single call to HAL_ADC_Start_IT() */
          switch (hadc->Init.EOCSelection)
          {
            case ADC_EOC_SEQ_CONV:
              __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOC);
              __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOS);
              break;
            /* case ADC_EOC_SINGLE_CONV */
            default:
              __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOS);
              __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);
              break;
          }
        }

        /* Start ADC group regular conversion */
        LL_ADC_REG_StartConversion(hadc->Instance);
      }
      else
      {
        /* ADC instance is a multimode slave instance with multimode regular conversions enabled */
        SET_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
        /* if Master ADC JAUTO bit is set, Slave injected interruptions
           are enabled nevertheless (for same reason as above) */
        tmpADC_Master = __LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance);
        if (READ_BIT(tmpADC_Master->CFGR, ADC_CFGR_JAUTO) != 0UL)
        {
          /* First, update Slave State in setting HAL_ADC_STATE_INJ_BUSY bit
             and in resetting HAL_ADC_STATE_INJ_EOC bit */
          ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);
          /* Next, set Slave injected interruptions */
          switch (hadc->Init.EOCSelection)
          {
            case ADC_EOC_SEQ_CONV:
              __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOC);
              __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOS);
              break;
            /* case ADC_EOC_SINGLE_CONV */
            default:
              __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOS);
              __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);
              break;
          }
        }
      }
#else
      /* ADC instance is not a multimode slave instance with multimode regular conversions enabled */
      if (READ_BIT(hadc->Instance->CFGR, ADC_CFGR_JAUTO) != 0UL)
      {
        ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);

        /* Enable as well injected interruptions in case
         HAL_ADCEx_InjectedStart_IT() has not been called beforehand. This
         allows to start regular and injected conversions when JAUTO is
         set with a single call to HAL_ADC_Start_IT() */
        switch (hadc->Init.EOCSelection)
        {
          case ADC_EOC_SEQ_CONV:
            __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOC);
            __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOS);
            break;
          /* case ADC_EOC_SINGLE_CONV */
          default:
            __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOS);
            __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);
            break;
        }
      }

      /* Start ADC group regular conversion */
      LL_ADC_REG_StartConversion(hadc->Instance);
#endif
    }
    else
    {
      /* Process unlocked */
      __HAL_UNLOCK(hadc);
    }

  }
  else
  {
    tmp_hal_status = HAL_BUSY;
  }

  /* Return function status */
  return tmp_hal_status;
}

/**
  * @brief  Stop ADC conversion of regular group (and injected group in
  *         case of auto_injection mode), disable interrution of
  *         end-of-conversion, disable ADC peripheral.
  * @param hadc ADC handle
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Process locked */
  __HAL_LOCK(hadc);

  /* 1. Stop potential conversion on going, on ADC groups regular and injected */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_REGULAR_INJECTED_GROUP);

  /* Disable ADC peripheral if conversions are effectively stopped */
  if (tmp_hal_status == HAL_OK)
  {
    /* Disable ADC end of conversion interrupt for regular group */
    /* Disable ADC overrun interrupt */
    __HAL_ADC_DISABLE_IT(hadc, (ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR));

    /* 2. Disable the ADC peripheral */
    tmp_hal_status = ADC_Disable(hadc);

    /* Check if ADC is effectively disabled */
    if (tmp_hal_status == HAL_OK)
    {
      /* Set ADC state */
      ADC_STATE_CLR_SET(hadc->State,
                        HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
                        HAL_ADC_STATE_READY);
    }
  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}

/**
  * @brief  Enable ADC, start conversion of regular group and transfer result through DMA.
  * @note   Interruptions enabled in this function:
  *         overrun (if applicable), DMA half transfer, DMA transfer complete.
  *         Each of these interruptions has its dedicated callback function.
  * @note   Case of multimode enabled (when multimode feature is available): HAL_ADC_Start_DMA()
  *         is designed for single-ADC mode only. For multimode, the dedicated
  *         HAL_ADCEx_MultiModeStart_DMA() function must be used.
  * @param hadc ADC handle
  * @param pData Destination Buffer address.
  * @param Length Number of data to be transferred from ADC peripheral to memory
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length)
{
  HAL_StatusTypeDef tmp_hal_status;
#if defined(ADC_MULTIMODE_SUPPORT)
  uint32_t tmp_multimode_config = LL_ADC_GetMultimode(__LL_ADC_COMMON_INSTANCE(hadc->Instance));
#endif

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Perform ADC enable and conversion start if no conversion is on going */
  if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
  {
    /* Process locked */
    __HAL_LOCK(hadc);

#if defined(ADC_MULTIMODE_SUPPORT)
    /* Ensure that multimode regular conversions are not enabled.   */
    /* Otherwise, dedicated API HAL_ADCEx_MultiModeStart_DMA() must be used.  */
    if ((ADC_IS_INDEPENDENT(hadc) != RESET)
        || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
        || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_SIMULT)
        || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_ALTERN)
       )
#endif /* ADC_MULTIMODE_SUPPORT */
    {
      /* Enable the ADC peripheral */
      tmp_hal_status = ADC_Enable(hadc);

      /* Start conversion if ADC is effectively enabled */
      if (tmp_hal_status == HAL_OK)
      {
        /* Set ADC state                                                        */
        /* - Clear state bitfield related to regular group conversion results   */
        /* - Set state bitfield related to regular operation                    */
        ADC_STATE_CLR_SET(hadc->State,
                          HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP,
                          HAL_ADC_STATE_REG_BUSY);

#if defined(ADC_MULTIMODE_SUPPORT)
        /* Reset HAL_ADC_STATE_MULTIMODE_SLAVE bit
          - if ADC instance is master or if multimode feature is not available
          - if multimode setting is disabled (ADC instance slave in independent mode) */
        if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
            || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
           )
        {
          CLEAR_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
        }
#endif

        /* Check if a conversion is on going on ADC group injected */
        if ((hadc->State & HAL_ADC_STATE_INJ_BUSY) != 0UL)
        {
          /* Reset ADC error code fields related to regular conversions only */
          CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
        }
        else
        {
          /* Reset all ADC error code fields */
          ADC_CLEAR_ERRORCODE(hadc);
        }

        /* Set the DMA transfer complete callback */
        hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;

        /* Set the DMA half transfer complete callback */
        hadc->DMA_Handle->XferHalfCpltCallback = ADC_DMAHalfConvCplt;

        /* Set the DMA error callback */
        hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;


        /* Manage ADC and DMA start: ADC overrun interruption, DMA start,     */
        /* ADC start (in case of SW start):                                   */

        /* Clear regular group conversion flag and overrun flag               */
        /* (To ensure of no unknown state from potential previous ADC         */
        /* operations)                                                        */
        __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

        /* Process unlocked */
        /* Unlock before starting ADC conversions: in case of potential         */
        /* interruption, to let the process to ADC IRQ Handler.                 */
        __HAL_UNLOCK(hadc);

        /* With DMA, overrun event is always considered as an error even if
           hadc->Init.Overrun is set to ADC_OVR_DATA_OVERWRITTEN. Therefore,
           ADC_IT_OVR is enabled. */
        __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

        /* Enable ADC DMA mode */
        SET_BIT(hadc->Instance->CFGR, ADC_CFGR_DMAEN);

        /* Start the DMA channel */
        tmp_hal_status = HAL_DMA_Start_IT(hadc->DMA_Handle, (uint32_t)&hadc->Instance->DR, (uint32_t)pData, Length);

        /* Enable conversion of regular group.                                  */
        /* If software start has been selected, conversion starts immediately.  */
        /* If external trigger has been selected, conversion will start at next */
        /* trigger event.                                                       */
        /* Start ADC group regular conversion */
        LL_ADC_REG_StartConversion(hadc->Instance);
      }
      else
      {
        /* Process unlocked */
        __HAL_UNLOCK(hadc);
      }

    }
#if defined(ADC_MULTIMODE_SUPPORT)
    else
    {
      tmp_hal_status = HAL_ERROR;
      /* Process unlocked */
      __HAL_UNLOCK(hadc);
    }
#endif
  }
  else
  {
    tmp_hal_status = HAL_BUSY;
  }

  /* Return function status */
  return tmp_hal_status;
}

/**
  * @brief  Stop ADC conversion of regular group (and injected group in
  *         case of auto_injection mode), disable ADC DMA transfer, disable
  *         ADC peripheral.
  * @note:  ADC peripheral disable is forcing stop of potential
  *         conversion on ADC group injected. If ADC group injected is under use, it
  *         should be preliminarily stopped using HAL_ADCEx_InjectedStop function.
  * @note   Case of multimode enabled (when multimode feature is available):
  *         HAL_ADC_Stop_DMA() function is dedicated to single-ADC mode only.
  *         For multimode, the dedicated HAL_ADCEx_MultiModeStop_DMA() API must be used.
  * @param hadc ADC handle
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef tmp_hal_status;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Process locked */
  __HAL_LOCK(hadc);

  /* 1. Stop potential ADC group regular conversion on going */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_REGULAR_INJECTED_GROUP);

  /* Disable ADC peripheral if conversions are effectively stopped */
  if (tmp_hal_status == HAL_OK)
  {
    /* Disable ADC DMA (ADC DMA configuration of continuous requests is kept) */
    CLEAR_BIT(hadc->Instance->CFGR, ADC_CFGR_DMAEN);

    /* Disable the DMA channel (in case of DMA in circular mode or stop       */
    /* while DMA transfer is on going)                                        */
    if (hadc->DMA_Handle->State == HAL_DMA_STATE_BUSY)
    {
      tmp_hal_status = HAL_DMA_Abort(hadc->DMA_Handle);

      /* Check if DMA channel effectively disabled */
      if (tmp_hal_status != HAL_OK)
      {
        /* Update ADC state machine to error */
        SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_DMA);
      }
    }

    /* Disable ADC overrun interrupt */
    __HAL_ADC_DISABLE_IT(hadc, ADC_IT_OVR);

    /* 2. Disable the ADC peripheral */
    /* Update "tmp_hal_status" only if DMA channel disabling passed,          */
    /* to keep in memory a potential failing status.                          */
    if (tmp_hal_status == HAL_OK)
    {
      tmp_hal_status = ADC_Disable(hadc);
    }
    else
    {
      (void)ADC_Disable(hadc);
    }

    /* Check if ADC is effectively disabled */
    if (tmp_hal_status == HAL_OK)
    {
      /* Set ADC state */
      ADC_STATE_CLR_SET(hadc->State,
                        HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
                        HAL_ADC_STATE_READY);
    }

  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}

/**
  * @brief  Get ADC regular group conversion result.
  * @note   Reading register DR automatically clears ADC flag EOC
  *         (ADC group regular end of unitary conversion).
  * @note   This function does not clear ADC flag EOS
  *         (ADC group regular end of sequence conversion).
  *         Occurrence of flag EOS rising:
  *          - If sequencer is composed of 1 rank, flag EOS is equivalent
  *            to flag EOC.
  *          - If sequencer is composed of several ranks, during the scan
  *            sequence flag EOC only is raised, at the end of the scan sequence
  *            both flags EOC and EOS are raised.
  *         To clear this flag, either use function:
  *         in programming model IT: @ref HAL_ADC_IRQHandler(), in programming
  *         model polling: @ref HAL_ADC_PollForConversion()
  *         or @ref __HAL_ADC_CLEAR_FLAG(&hadc, ADC_FLAG_EOS).
  * @param hadc ADC handle
  * @retval ADC group regular conversion data
  */
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Note: EOC flag is not cleared here by software because automatically     */
  /*       cleared by hardware when reading register DR.                      */

  /* Return ADC converted value */
  return hadc->Instance->DR;
}

/**
  * @brief  Handle ADC interrupt request.
  * @param hadc ADC handle
  * @retval None
  */
void HAL_ADC_IRQHandler(ADC_HandleTypeDef *hadc)
{
  uint32_t overrun_error = 0UL; /* flag set if overrun occurrence has to be considered as an error */
  uint32_t tmp_isr = hadc->Instance->ISR;
  uint32_t tmp_ier = hadc->Instance->IER;
  uint32_t tmp_adc_inj_is_trigger_source_sw_start;
  uint32_t tmp_adc_reg_is_trigger_source_sw_start;
  uint32_t tmp_cfgr;
#if defined(ADC_MULTIMODE_SUPPORT)
  const ADC_TypeDef *tmpADC_Master;
  uint32_t tmp_multimode_config = LL_ADC_GetMultimode(__LL_ADC_COMMON_INSTANCE(hadc->Instance));
#endif

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  assert_param(IS_ADC_EOC_SELECTION(hadc->Init.EOCSelection));

  /* ========== Check End of Sampling flag for ADC group regular ========== */
  if (((tmp_isr & ADC_FLAG_EOSMP) == ADC_FLAG_EOSMP) && ((tmp_ier & ADC_IT_EOSMP) == ADC_IT_EOSMP))
  {
    /* Update state machine on end of sampling status if not in error state */
    if ((hadc->State & HAL_ADC_STATE_ERROR_INTERNAL) == 0UL)
    {
      /* Set ADC state */
      SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOSMP);
    }

    /* End Of Sampling callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->EndOfSamplingCallback(hadc);
#else
    HAL_ADCEx_EndOfSamplingCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */

    /* Clear regular group conversion flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOSMP);
  }

  /* ====== Check ADC group regular end of unitary conversion sequence conversions ===== */
  if ((((tmp_isr & ADC_FLAG_EOC) == ADC_FLAG_EOC) && ((tmp_ier & ADC_IT_EOC) == ADC_IT_EOC)) ||
      (((tmp_isr & ADC_FLAG_EOS) == ADC_FLAG_EOS) && ((tmp_ier & ADC_IT_EOS) == ADC_IT_EOS)))
  {
    /* Update state machine on conversion status if not in error state */
    if ((hadc->State & HAL_ADC_STATE_ERROR_INTERNAL) == 0UL)
    {
      /* Set ADC state */
      SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);
    }

    /* Determine whether any further conversion upcoming on group regular     */
    /* by external trigger, continuous mode or scan sequence on going         */
    /* to disable interruption.                                               */
    if (LL_ADC_REG_IsTriggerSourceSWStart(hadc->Instance) != 0UL)
    {
      /* Get relevant register CFGR in ADC instance of ADC master or slave    */
      /* in function of multimode state (for devices with multimode           */
      /* available).                                                          */
#if defined(ADC_MULTIMODE_SUPPORT)
      if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
          || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
          || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_SIMULT)
          || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_ALTERN)
         )
      {
        /* check CONT bit directly in handle ADC CFGR register */
        tmp_cfgr = READ_REG(hadc->Instance->CFGR);
      }
      else
      {
        /* else need to check Master ADC CONT bit */
        tmpADC_Master = __LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance);
        tmp_cfgr = READ_REG(tmpADC_Master->CFGR);
      }
#else
      tmp_cfgr = READ_REG(hadc->Instance->CFGR);
#endif

      /* Carry on if continuous mode is disabled */
      if (READ_BIT(tmp_cfgr, ADC_CFGR_CONT) != ADC_CFGR_CONT)
      {
        /* If End of Sequence is reached, disable interrupts */
        if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS))
        {
          /* Allowed to modify bits ADC_IT_EOC/ADC_IT_EOS only if bit         */
          /* ADSTART==0 (no conversion on going)                              */
          if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
          {
            /* Disable ADC end of sequence conversion interrupt */
            /* Note: Overrun interrupt was enabled with EOC interrupt in      */
            /* HAL_Start_IT(), but is not disabled here because can be used   */
            /* by overrun IRQ process below.                                  */
            __HAL_ADC_DISABLE_IT(hadc, ADC_IT_EOC | ADC_IT_EOS);

            /* Set ADC state */
            CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);

            if ((hadc->State & HAL_ADC_STATE_INJ_BUSY) == 0UL)
            {
              SET_BIT(hadc->State, HAL_ADC_STATE_READY);
            }
          }
          else
          {
            /* Change ADC state to error state */
            SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

            /* Set ADC error code to ADC peripheral internal error */
            SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
          }
        }
      }
    }

    /* Conversion complete callback */
    /* Note: Into callback function "HAL_ADC_ConvCpltCallback()",             */
    /*       to determine if conversion has been triggered from EOC or EOS,   */
    /*       possibility to use:                                              */
    /*        " if ( __HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_EOS)) "               */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->ConvCpltCallback(hadc);
#else
    HAL_ADC_ConvCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */

    /* Clear regular group conversion flag */
    /* Note: in case of overrun set to ADC_OVR_DATA_PRESERVED, end of         */
    /*       conversion flags clear induces the release of the preserved data.*/
    /*       Therefore, if the preserved data value is needed, it must be     */
    /*       read preliminarily into HAL_ADC_ConvCpltCallback().              */
    __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS));
  }

  /* ====== Check ADC group injected end of unitary conversion sequence conversions ===== */
  if ((((tmp_isr & ADC_FLAG_JEOC) == ADC_FLAG_JEOC) && ((tmp_ier & ADC_IT_JEOC) == ADC_IT_JEOC)) ||
      (((tmp_isr & ADC_FLAG_JEOS) == ADC_FLAG_JEOS) && ((tmp_ier & ADC_IT_JEOS) == ADC_IT_JEOS)))
  {
    /* Update state machine on conversion status if not in error state */
    if ((hadc->State & HAL_ADC_STATE_ERROR_INTERNAL) == 0UL)
    {
      /* Set ADC state */
      SET_BIT(hadc->State, HAL_ADC_STATE_INJ_EOC);
    }

    /* Retrieve ADC configuration */
    tmp_adc_inj_is_trigger_source_sw_start = LL_ADC_INJ_IsTriggerSourceSWStart(hadc->Instance);
    tmp_adc_reg_is_trigger_source_sw_start = LL_ADC_REG_IsTriggerSourceSWStart(hadc->Instance);
    /* Get relevant register CFGR in ADC instance of ADC master or slave  */
    /* in function of multimode state (for devices with multimode         */
    /* available).                                                        */
#if defined(ADC_MULTIMODE_SUPPORT)
    if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
        || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
        || (tmp_multimode_config == LL_ADC_MULTI_DUAL_REG_SIMULT)
        || (tmp_multimode_config == LL_ADC_MULTI_DUAL_REG_INTERL)
       )
    {
      tmp_cfgr = READ_REG(hadc->Instance->CFGR);
    }
    else
    {
      tmpADC_Master = __LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance);
      tmp_cfgr = READ_REG(tmpADC_Master->CFGR);
    }
#else
    tmp_cfgr = READ_REG(hadc->Instance->CFGR);
#endif

    /* Disable interruption if no further conversion upcoming by injected     */
    /* external trigger or by automatic injected conversion with regular      */
    /* group having no further conversion upcoming (same conditions as        */
    /* regular group interruption disabling above),                           */
    /* and if injected scan sequence is completed.                            */
    if (tmp_adc_inj_is_trigger_source_sw_start != 0UL)
    {
      if ((READ_BIT(tmp_cfgr, ADC_CFGR_JAUTO) == 0UL) ||
          ((tmp_adc_reg_is_trigger_source_sw_start != 0UL) &&
           (READ_BIT(tmp_cfgr, ADC_CFGR_CONT) == 0UL)))
      {
        /* If End of Sequence is reached, disable interrupts */
        if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOS))
        {
          /* Particular case if injected contexts queue is enabled:             */
          /* when the last context has been fully processed, JSQR is reset      */
          /* by the hardware. Even if no injected conversion is planned to come */
          /* (queue empty, triggers are ignored), it can start again            */
          /* immediately after setting a new context (JADSTART is still set).   */
          /* Therefore, state of HAL ADC injected group is kept to busy.        */
          if (READ_BIT(tmp_cfgr, ADC_CFGR_JQM) == 0UL)
          {
            /* Allowed to modify bits ADC_IT_JEOC/ADC_IT_JEOS only if bit       */
            /* JADSTART==0 (no conversion on going)                             */
            if (LL_ADC_INJ_IsConversionOngoing(hadc->Instance) == 0UL)
            {
              /* Disable ADC end of sequence conversion interrupt  */
              __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOC | ADC_IT_JEOS);

              /* Set ADC state */
              CLEAR_BIT(hadc->State, HAL_ADC_STATE_INJ_BUSY);

              if ((hadc->State & HAL_ADC_STATE_REG_BUSY) == 0UL)
              {
                SET_BIT(hadc->State, HAL_ADC_STATE_READY);
              }
            }
            else
            {
              /* Update ADC state machine to error */
              SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

              /* Set ADC error code to ADC peripheral internal error */
              SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
            }
          }
        }
      }
    }

    /* Injected Conversion complete callback */
    /* Note:  HAL_ADCEx_InjectedConvCpltCallback can resort to
              if (__HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_JEOS)) or
              if (__HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_JEOC)) to determine whether
              interruption has been triggered by end of conversion or end of
              sequence.    */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->InjectedConvCpltCallback(hadc);
#else
    HAL_ADCEx_InjectedConvCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */

    /* Clear injected group conversion flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
  }

  /* ========== Check Analog watchdog 1 flag ========== */
  if (((tmp_isr & ADC_FLAG_AWD1) == ADC_FLAG_AWD1) && ((tmp_ier & ADC_IT_AWD1) == ADC_IT_AWD1))
  {
    /* Set ADC state */
    SET_BIT(hadc->State, HAL_ADC_STATE_AWD1);

    /* Level out of window 1 callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->LevelOutOfWindowCallback(hadc);
#else
    HAL_ADC_LevelOutOfWindowCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */

    /* Clear ADC analog watchdog flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_AWD1);
  }

  /* ========== Check analog watchdog 2 flag ========== */
  if (((tmp_isr & ADC_FLAG_AWD2) == ADC_FLAG_AWD2) && ((tmp_ier & ADC_IT_AWD2) == ADC_IT_AWD2))
  {
    /* Set ADC state */
    SET_BIT(hadc->State, HAL_ADC_STATE_AWD2);

    /* Level out of window 2 callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->LevelOutOfWindow2Callback(hadc);
#else
    HAL_ADCEx_LevelOutOfWindow2Callback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */

    /* Clear ADC analog watchdog flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_AWD2);
  }

  /* ========== Check analog watchdog 3 flag ========== */
  if (((tmp_isr & ADC_FLAG_AWD3) == ADC_FLAG_AWD3) && ((tmp_ier & ADC_IT_AWD3) == ADC_IT_AWD3))
  {
    /* Set ADC state */
    SET_BIT(hadc->State, HAL_ADC_STATE_AWD3);

    /* Level out of window 3 callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->LevelOutOfWindow3Callback(hadc);
#else
    HAL_ADCEx_LevelOutOfWindow3Callback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */

    /* Clear ADC analog watchdog flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_AWD3);
  }

  /* ========== Check Overrun flag ========== */
  if (((tmp_isr & ADC_FLAG_OVR) == ADC_FLAG_OVR) && ((tmp_ier & ADC_IT_OVR) == ADC_IT_OVR))
  {
    /* If overrun is set to overwrite previous data (default setting),        */
    /* overrun event is not considered as an error.                           */
    /* (cf ref manual "Managing conversions without using the DMA and without */
    /* overrun ")                                                             */
    /* Exception for usage with DMA overrun event always considered as an     */
    /* error.                                                                 */
    if (hadc->Init.Overrun == ADC_OVR_DATA_PRESERVED)
    {
      overrun_error = 1UL;
    }
    else
    {
      /* Check DMA configuration */
#if defined(ADC_MULTIMODE_SUPPORT)
      if (tmp_multimode_config != LL_ADC_MULTI_INDEPENDENT)
      {
        /* Multimode (when feature is available) is enabled,
           Common Control Register MDMA bits must be checked. */
        if (LL_ADC_GetMultiDMATransfer(__LL_ADC_COMMON_INSTANCE(hadc->Instance)) != LL_ADC_MULTI_REG_DMA_EACH_ADC)
        {
          overrun_error = 1UL;
        }
      }
      else
#endif
      {
        /* Multimode not set or feature not available or ADC independent */
        if ((hadc->Instance->CFGR & ADC_CFGR_DMAEN) != 0UL)
        {
          overrun_error = 1UL;
        }
      }
    }

    if (overrun_error == 1UL)
    {
      /* Change ADC state to error state */
      SET_BIT(hadc->State, HAL_ADC_STATE_REG_OVR);

      /* Set ADC error code to overrun */
      SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_OVR);

      /* Error callback */
      /* Note: In case of overrun, ADC conversion data is preserved until     */
      /*       flag OVR is reset.                                             */
      /*       Therefore, old ADC conversion data can be retrieved in         */
      /*       function "HAL_ADC_ErrorCallback()".                            */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
      hadc->ErrorCallback(hadc);
#else
      HAL_ADC_ErrorCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
    }

    /* Clear ADC overrun flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
  }

  /* ========== Check Injected context queue overflow flag ========== */
  if (((tmp_isr & ADC_FLAG_JQOVF) == ADC_FLAG_JQOVF) && ((tmp_ier & ADC_IT_JQOVF) == ADC_IT_JQOVF))
  {
    /* Change ADC state to overrun state */
    SET_BIT(hadc->State, HAL_ADC_STATE_INJ_JQOVF);

    /* Set ADC error code to Injected context queue overflow */
    SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_JQOVF);

    /* Clear the Injected context queue overflow flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JQOVF);

    /* Injected context queue overflow callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->InjectedQueueOverflowCallback(hadc);
#else
    HAL_ADCEx_InjectedQueueOverflowCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

}

/**
  * @brief  Conversion complete callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
__weak void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvCpltCallback must be implemented in the user file.
   */
}

/**
  * @brief  Conversion DMA half-transfer callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
__weak void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvHalfCpltCallback must be implemented in the user file.
  */
}

/**
  * @brief  Analog watchdog 1 callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
__weak void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_LevelOutOfWindowCallback must be implemented in the user file.
  */
}

/**
  * @brief  ADC error callback in non-blocking mode
  *         (ADC conversion with interruption or transfer by DMA).
  * @note   In case of error due to overrun when using ADC with DMA transfer
  *         (HAL ADC handle parameter "ErrorCode" to state "HAL_ADC_ERROR_OVR"):
  *         - Reinitialize the DMA using function "HAL_ADC_Stop_DMA()".
  *         - If needed, restart a new ADC conversion using function
  *           "HAL_ADC_Start_DMA()"
  *           (this function is also clearing overrun flag)
  * @param hadc ADC handle
  * @retval None
  */
__weak void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ErrorCallback must be implemented in the user file.
  */
}

/**
  * @}
  */

/** @defgroup ADC_Exported_Functions_Group3 Peripheral Control functions
  * @brief    Peripheral Control functions
  *
@verbatim
 ===============================================================================
             ##### Peripheral Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure channels on regular group
      (+) Configure the analog watchdog

@endverbatim
  * @{
  */

/**
  * @brief  Configure a channel to be assigned to ADC group regular.
  * @note   In case of usage of internal measurement channels:
  *         Vbat/VrefInt/TempSensor.
  *         These internal paths can be disabled using function
  *         HAL_ADC_DeInit().
  * @note   Possibility to update parameters on the fly:
  *         This function initializes channel into ADC group regular,
  *         following calls to this function can be used to reconfigure
  *         some parameters of structure "ADC_ChannelConfTypeDef" on the fly,
  *         without resetting the ADC.
  *         The setting of these parameters is conditioned to ADC state:
  *         Refer to comments of structure "ADC_ChannelConfTypeDef".
  * @param hadc ADC handle
  * @param sConfig Structure of ADC channel assigned to ADC group regular.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *sConfig)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  uint32_t tmpOffsetShifted;
  uint32_t tmp_config_internal_channel;
  __IO uint32_t wait_loop_index = 0UL;
  uint32_t tmp_adc_is_conversion_on_going_regular;
  uint32_t tmp_adc_is_conversion_on_going_injected;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  assert_param(IS_ADC_REGULAR_RANK(sConfig->Rank));
  assert_param(IS_ADC_SAMPLE_TIME(sConfig->SamplingTime));
  assert_param(IS_ADC_SINGLE_DIFFERENTIAL(sConfig->SingleDiff));
  assert_param(IS_ADC_OFFSET_NUMBER(sConfig->OffsetNumber));
  assert_param(IS_ADC_RANGE(ADC_GET_RESOLUTION(hadc), sConfig->Offset));

  /* if ROVSE is set, the value of the OFFSETy_EN bit in ADCx_OFRy register is
     ignored (considered as reset) */
  assert_param(!((sConfig->OffsetNumber != ADC_OFFSET_NONE) && (hadc->Init.OversamplingMode == ENABLE)));

  /* Verification of channel number */
  if (sConfig->SingleDiff != ADC_DIFFERENTIAL_ENDED)
  {
    assert_param(IS_ADC_CHANNEL(hadc, sConfig->Channel));
  }
  else
  {
    assert_param(IS_ADC_DIFF_CHANNEL(hadc, sConfig->Channel));
  }

  /* Process locked */
  __HAL_LOCK(hadc);

  /* Parameters update conditioned to ADC state:                              */
  /* Parameters that can be updated when ADC is disabled or enabled without   */
  /* conversion on going on regular group:                                    */
  /*  - Channel number                                                        */
  /*  - Channel rank                                                          */
  if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
  {
#if !defined (USE_FULL_ASSERT)
    /* Correspondence for compatibility with legacy definition of             */
    /* sequencer ranks in direct number format. This correspondence can       */
    /* be done only on ranks 1 to 5 due to literal values.                    */
    /* Note: Sequencer ranks in direct number format are no more used         */
    /*       and are detected by activating USE_FULL_ASSERT feature.          */
    if (sConfig->Rank <= 5U)
    {
      switch (sConfig->Rank)
      {
        case 2U:
          sConfig->Rank = ADC_REGULAR_RANK_2;
          break;
        case 3U:
          sConfig->Rank = ADC_REGULAR_RANK_3;
          break;
        case 4U:
          sConfig->Rank = ADC_REGULAR_RANK_4;
          break;
        case 5U:
          sConfig->Rank = ADC_REGULAR_RANK_5;
          break;
        /* case 1U */
        default:
          sConfig->Rank = ADC_REGULAR_RANK_1;
          break;
      }
    }
#endif

    /* Set ADC group regular sequence: channel on the selected scan sequence rank */
    LL_ADC_REG_SetSequencerRanks(hadc->Instance, sConfig->Rank, sConfig->Channel);

    /* Parameters update conditioned to ADC state:                              */
    /* Parameters that can be updated when ADC is disabled or enabled without   */
    /* conversion on going on regular group:                                    */
    /*  - Channel sampling time                                                 */
    /*  - Channel offset                                                        */
    tmp_adc_is_conversion_on_going_regular = LL_ADC_REG_IsConversionOngoing(hadc->Instance);
    tmp_adc_is_conversion_on_going_injected = LL_ADC_INJ_IsConversionOngoing(hadc->Instance);
    if ((tmp_adc_is_conversion_on_going_regular == 0UL)
        && (tmp_adc_is_conversion_on_going_injected == 0UL)
       )
    {
#if defined(ADC_SMPR1_SMPPLUS)
      /* Manage specific case of sampling time 3.5 cycles replacing 2.5 cyles */
      if (sConfig->SamplingTime == ADC_SAMPLETIME_3CYCLES_5)
      {
        /* Set sampling time of the selected ADC channel */
        LL_ADC_SetChannelSamplingTime(hadc->Instance, sConfig->Channel, LL_ADC_SAMPLINGTIME_2CYCLES_5);

        /* Set ADC sampling time common configuration */
        LL_ADC_SetSamplingTimeCommonConfig(hadc->Instance, LL_ADC_SAMPLINGTIME_COMMON_3C5_REPL_2C5);
      }
      else
      {
        /* Set sampling time of the selected ADC channel */
        LL_ADC_SetChannelSamplingTime(hadc->Instance, sConfig->Channel, sConfig->SamplingTime);

        /* Set ADC sampling time common configuration */
        LL_ADC_SetSamplingTimeCommonConfig(hadc->Instance, LL_ADC_SAMPLINGTIME_COMMON_DEFAULT);
      }
#else
      /* Set sampling time of the selected ADC channel */
      LL_ADC_SetChannelSamplingTime(hadc->Instance, sConfig->Channel, sConfig->SamplingTime);
#endif

      /* Configure the offset: offset enable/disable, channel, offset value */

      /* Shift the offset with respect to the selected ADC resolution. */
      /* Offset has to be left-aligned on bit 11, the LSB (right bits) are set to 0 */
      tmpOffsetShifted = ADC_OFFSET_SHIFT_RESOLUTION(hadc, (uint32_t)sConfig->Offset);

      if (sConfig->OffsetNumber != ADC_OFFSET_NONE)
      {
        /* Set ADC selected offset number */
        LL_ADC_SetOffset(hadc->Instance, sConfig->OffsetNumber, sConfig->Channel, tmpOffsetShifted);

      }
      else
      {
        /* Scan each offset register to check if the selected channel is targeted. */
        /* If this is the case, the corresponding offset number is disabled.       */
        if (__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_1))
            == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfig->Channel))
        {
          LL_ADC_SetOffsetState(hadc->Instance, LL_ADC_OFFSET_1, LL_ADC_OFFSET_DISABLE);
        }
        if (__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_2))
            == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfig->Channel))
        {
          LL_ADC_SetOffsetState(hadc->Instance, LL_ADC_OFFSET_2, LL_ADC_OFFSET_DISABLE);
        }
        if (__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_3))
            == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfig->Channel))
        {
          LL_ADC_SetOffsetState(hadc->Instance, LL_ADC_OFFSET_3, LL_ADC_OFFSET_DISABLE);
        }
        if (__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_4))
            == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfig->Channel))
        {
          LL_ADC_SetOffsetState(hadc->Instance, LL_ADC_OFFSET_4, LL_ADC_OFFSET_DISABLE);
        }
      }
    }

    /* Parameters update conditioned to ADC state:                              */
    /* Parameters that can be updated only when ADC is disabled:                */
    /*  - Single or differential mode                                           */
    if (LL_ADC_IsEnabled(hadc->Instance) == 0UL)
    {
      /* Set mode single-ended or differential input of the selected ADC channel */
      LL_ADC_SetChannelSingleDiff(hadc->Instance, sConfig->Channel, sConfig->SingleDiff);

      /* Configuration of differential mode */
      if (sConfig->SingleDiff == ADC_DIFFERENTIAL_ENDED)
      {
        /* Set sampling time of the selected ADC channel */
        /* Note: ADC channel number masked with value "0x1F" to ensure shift value within 32 bits range */
        LL_ADC_SetChannelSamplingTime(hadc->Instance,
                                      (uint32_t)(__LL_ADC_DECIMAL_NB_TO_CHANNEL((__LL_ADC_CHANNEL_TO_DECIMAL_NB((uint32_t)sConfig->Channel) + 1UL) & 0x1FUL)),
                                      sConfig->SamplingTime);
      }

    }

    /* Management of internal measurement channels: Vbat/VrefInt/TempSensor.  */
    /* If internal channel selected, enable dedicated internal buffers and    */
    /* paths.                                                                 */
    /* Note: these internal measurement paths can be disabled using           */
    /* HAL_ADC_DeInit().                                                      */

    if (__LL_ADC_IS_CHANNEL_INTERNAL(sConfig->Channel))
    {
      tmp_config_internal_channel = LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance));

      /* If the requested internal measurement path has already been enabled, */
      /* bypass the configuration processing.                                 */
      if ((sConfig->Channel == ADC_CHANNEL_TEMPSENSOR)
          && ((tmp_config_internal_channel & LL_ADC_PATH_INTERNAL_TEMPSENSOR) == 0UL))
      {
        if (ADC_TEMPERATURE_SENSOR_INSTANCE(hadc))
        {
          LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance),
                                         LL_ADC_PATH_INTERNAL_TEMPSENSOR | tmp_config_internal_channel);

          /* Delay for temperature sensor stabilization time */
          /* Wait loop initialization and execution */
          /* Note: Variable divided by 2 to compensate partially              */
          /*       CPU processing cycles, scaling in us split to not          */
          /*       exceed 32 bits register capacity and handle low frequency. */
          wait_loop_index = ((LL_ADC_DELAY_TEMPSENSOR_STAB_US / 10UL) * ((SystemCoreClock / (100000UL * 2UL)) + 1UL));
          while (wait_loop_index != 0UL)
          {
            wait_loop_index--;
          }
        }
      }
      else if ((sConfig->Channel == ADC_CHANNEL_VBAT) && ((tmp_config_internal_channel & LL_ADC_PATH_INTERNAL_VBAT) == 0UL))
      {
        if (ADC_BATTERY_VOLTAGE_INSTANCE(hadc))
        {
          LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance),
                                         LL_ADC_PATH_INTERNAL_VBAT | tmp_config_internal_channel);
        }
      }
      else if ((sConfig->Channel == ADC_CHANNEL_VREFINT)
               && ((tmp_config_internal_channel & LL_ADC_PATH_INTERNAL_VREFINT) == 0UL))
      {
        if (ADC_VREFINT_INSTANCE(hadc))
        {
          LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance),
                                         LL_ADC_PATH_INTERNAL_VREFINT | tmp_config_internal_channel);
        }
      }
      else
      {
        /* nothing to do */
      }
    }
  }

  /* If a conversion is on going on regular group, no update on regular       */
  /* channel could be done on neither of the channel configuration structure  */
  /* parameters.                                                              */
  else
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);

    tmp_hal_status = HAL_ERROR;
  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}

/**
  * @brief  Configure the analog watchdog.
  * @note   Possibility to update parameters on the fly:
  *         This function initializes the selected analog watchdog, successive
  *         calls to this function can be used to reconfigure some parameters
  *         of structure "ADC_AnalogWDGConfTypeDef" on the fly, without resetting
  *         the ADC.
  *         The setting of these parameters is conditioned to ADC state.
  *         For parameters constraints, see comments of structure
  *         "ADC_AnalogWDGConfTypeDef".
  * @note   On this STM32 series, analog watchdog thresholds cannot be modified
  *         while ADC conversion is on going.
  * @param hadc ADC handle
  * @param AnalogWDGConfig Structure of ADC analog watchdog configuration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_AnalogWDGConfig(ADC_HandleTypeDef *hadc, ADC_AnalogWDGConfTypeDef *AnalogWDGConfig)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  uint32_t tmpAWDHighThresholdShifted;
  uint32_t tmpAWDLowThresholdShifted;
  uint32_t tmp_adc_is_conversion_on_going_regular;
  uint32_t tmp_adc_is_conversion_on_going_injected;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  assert_param(IS_ADC_ANALOG_WATCHDOG_NUMBER(AnalogWDGConfig->WatchdogNumber));
  assert_param(IS_ADC_ANALOG_WATCHDOG_MODE(AnalogWDGConfig->WatchdogMode));
  assert_param(IS_FUNCTIONAL_STATE(AnalogWDGConfig->ITMode));

  if ((AnalogWDGConfig->WatchdogMode == ADC_ANALOGWATCHDOG_SINGLE_REG)     ||
      (AnalogWDGConfig->WatchdogMode == ADC_ANALOGWATCHDOG_SINGLE_INJEC)   ||
      (AnalogWDGConfig->WatchdogMode == ADC_ANALOGWATCHDOG_SINGLE_REGINJEC))
  {
    assert_param(IS_ADC_CHANNEL(hadc, AnalogWDGConfig->Channel));
  }

  /* Verify thresholds range */
  if (hadc->Init.OversamplingMode == ENABLE)
  {
    /* Case of oversampling enabled: depending on ratio and shift configuration,
       analog watchdog thresholds can be higher than ADC resolution.
       Verify if thresholds are within maximum thresholds range. */
    assert_param(IS_ADC_RANGE(ADC_RESOLUTION_12B, AnalogWDGConfig->HighThreshold));
    assert_param(IS_ADC_RANGE(ADC_RESOLUTION_12B, AnalogWDGConfig->LowThreshold));
  }
  else
  {
    /* Verify if thresholds are within the selected ADC resolution */
    assert_param(IS_ADC_RANGE(ADC_GET_RESOLUTION(hadc), AnalogWDGConfig->HighThreshold));
    assert_param(IS_ADC_RANGE(ADC_GET_RESOLUTION(hadc), AnalogWDGConfig->LowThreshold));
  }

  /* Process locked */
  __HAL_LOCK(hadc);

  /* Parameters update conditioned to ADC state:                              */
  /* Parameters that can be updated when ADC is disabled or enabled without   */
  /* conversion on going on ADC groups regular and injected:                  */
  /*  - Analog watchdog channels                                              */
  /*  - Analog watchdog thresholds                                            */
  tmp_adc_is_conversion_on_going_regular = LL_ADC_REG_IsConversionOngoing(hadc->Instance);
  tmp_adc_is_conversion_on_going_injected = LL_ADC_INJ_IsConversionOngoing(hadc->Instance);
  if ((tmp_adc_is_conversion_on_going_regular == 0UL)
      && (tmp_adc_is_conversion_on_going_injected == 0UL)
     )
  {
    /* Analog watchdog configuration */
    if (AnalogWDGConfig->WatchdogNumber == ADC_ANALOGWATCHDOG_1)
    {
      /* Configuration of analog watchdog:                                    */
      /*  - Set the analog watchdog enable mode: one or overall group of      */
      /*    channels, on groups regular and-or injected.                      */
      switch (AnalogWDGConfig->WatchdogMode)
      {
        case ADC_ANALOGWATCHDOG_SINGLE_REG:
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, LL_ADC_AWD1, __LL_ADC_ANALOGWD_CHANNEL_GROUP(AnalogWDGConfig->Channel,
                                          LL_ADC_GROUP_REGULAR));
          break;

        case ADC_ANALOGWATCHDOG_SINGLE_INJEC:
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, LL_ADC_AWD1, __LL_ADC_ANALOGWD_CHANNEL_GROUP(AnalogWDGConfig->Channel,
                                          LL_ADC_GROUP_INJECTED));
          break;

        case ADC_ANALOGWATCHDOG_SINGLE_REGINJEC:
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, LL_ADC_AWD1, __LL_ADC_ANALOGWD_CHANNEL_GROUP(AnalogWDGConfig->Channel,
                                          LL_ADC_GROUP_REGULAR_INJECTED));
          break;

        case ADC_ANALOGWATCHDOG_ALL_REG:
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_REG);
          break;

        case ADC_ANALOGWATCHDOG_ALL_INJEC:
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_INJ);
          break;

        case ADC_ANALOGWATCHDOG_ALL_REGINJEC:
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_REG_INJ);
          break;

        default: /* ADC_ANALOGWATCHDOG_NONE */
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, LL_ADC_AWD1, LL_ADC_AWD_DISABLE);
          break;
      }

      /* Shift the offset in function of the selected ADC resolution:         */
      /* Thresholds have to be left-aligned on bit 11, the LSB (right bits)   */
      /* are set to 0                                                         */
      tmpAWDHighThresholdShifted = ADC_AWD1THRESHOLD_SHIFT_RESOLUTION(hadc, AnalogWDGConfig->HighThreshold);
      tmpAWDLowThresholdShifted  = ADC_AWD1THRESHOLD_SHIFT_RESOLUTION(hadc, AnalogWDGConfig->LowThreshold);

      /* Set ADC analog watchdog thresholds value of both thresholds high and low */
      LL_ADC_ConfigAnalogWDThresholds(hadc->Instance, AnalogWDGConfig->WatchdogNumber, tmpAWDHighThresholdShifted,
                                      tmpAWDLowThresholdShifted);

      /* Update state, clear previous result related to AWD1 */
      CLEAR_BIT(hadc->State, HAL_ADC_STATE_AWD1);

      /* Clear flag ADC analog watchdog */
      /* Note: Flag cleared Clear the ADC Analog watchdog flag to be ready  */
      /* to use for HAL_ADC_IRQHandler() or HAL_ADC_PollForEvent()          */
      /* (in case left enabled by previous ADC operations).                 */
      LL_ADC_ClearFlag_AWD1(hadc->Instance);

      /* Configure ADC analog watchdog interrupt */
      if (AnalogWDGConfig->ITMode == ENABLE)
      {
        LL_ADC_EnableIT_AWD1(hadc->Instance);
      }
      else
      {
        LL_ADC_DisableIT_AWD1(hadc->Instance);
      }
    }
    /* Case of ADC_ANALOGWATCHDOG_2 or ADC_ANALOGWATCHDOG_3 */
    else
    {
      switch (AnalogWDGConfig->WatchdogMode)
      {
        case ADC_ANALOGWATCHDOG_SINGLE_REG:
        case ADC_ANALOGWATCHDOG_SINGLE_INJEC:
        case ADC_ANALOGWATCHDOG_SINGLE_REGINJEC:
          /* Update AWD by bitfield to keep the possibility to monitor        */
          /* several channels by successive calls of this function.           */
          if (AnalogWDGConfig->WatchdogNumber == ADC_ANALOGWATCHDOG_2)
          {
            SET_BIT(hadc->Instance->AWD2CR, (1UL << (__LL_ADC_CHANNEL_TO_DECIMAL_NB(AnalogWDGConfig->Channel) & 0x1FUL)));
          }
          else
          {
            SET_BIT(hadc->Instance->AWD3CR, (1UL << (__LL_ADC_CHANNEL_TO_DECIMAL_NB(AnalogWDGConfig->Channel) & 0x1FUL)));
          }
          break;

        case ADC_ANALOGWATCHDOG_ALL_REG:
        case ADC_ANALOGWATCHDOG_ALL_INJEC:
        case ADC_ANALOGWATCHDOG_ALL_REGINJEC:
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, AnalogWDGConfig->WatchdogNumber, LL_ADC_AWD_ALL_CHANNELS_REG_INJ);
          break;

        default: /* ADC_ANALOGWATCHDOG_NONE */
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, AnalogWDGConfig->WatchdogNumber, LL_ADC_AWD_DISABLE);
          break;
      }

      /* Shift the thresholds in function of the selected ADC resolution      */
      /* have to be left-aligned on bit 7, the LSB (right bits) are set to 0  */
      tmpAWDHighThresholdShifted = ADC_AWD23THRESHOLD_SHIFT_RESOLUTION(hadc, AnalogWDGConfig->HighThreshold);
      tmpAWDLowThresholdShifted  = ADC_AWD23THRESHOLD_SHIFT_RESOLUTION(hadc, AnalogWDGConfig->LowThreshold);

      /* Set ADC analog watchdog thresholds value of both thresholds high and low */
      LL_ADC_ConfigAnalogWDThresholds(hadc->Instance, AnalogWDGConfig->WatchdogNumber, tmpAWDHighThresholdShifted,
                                      tmpAWDLowThresholdShifted);

      if (AnalogWDGConfig->WatchdogNumber == ADC_ANALOGWATCHDOG_2)
      {
        /* Update state, clear previous result related to AWD2 */
        CLEAR_BIT(hadc->State, HAL_ADC_STATE_AWD2);

        /* Clear flag ADC analog watchdog */
        /* Note: Flag cleared Clear the ADC Analog watchdog flag to be ready  */
        /* to use for HAL_ADC_IRQHandler() or HAL_ADC_PollForEvent()          */
        /* (in case left enabled by previous ADC operations).                 */
        LL_ADC_ClearFlag_AWD2(hadc->Instance);

        /* Configure ADC analog watchdog interrupt */
        if (AnalogWDGConfig->ITMode == ENABLE)
        {
          LL_ADC_EnableIT_AWD2(hadc->Instance);
        }
        else
        {
          LL_ADC_DisableIT_AWD2(hadc->Instance);
        }
      }
      /* (AnalogWDGConfig->WatchdogNumber == ADC_ANALOGWATCHDOG_3) */
      else
      {
        /* Update state, clear previous result related to AWD3 */
        CLEAR_BIT(hadc->State, HAL_ADC_STATE_AWD3);

        /* Clear flag ADC analog watchdog */
        /* Note: Flag cleared Clear the ADC Analog watchdog flag to be ready  */
        /* to use for HAL_ADC_IRQHandler() or HAL_ADC_PollForEvent()          */
        /* (in case left enabled by previous ADC operations).                 */
        LL_ADC_ClearFlag_AWD3(hadc->Instance);

        /* Configure ADC analog watchdog interrupt */
        if (AnalogWDGConfig->ITMode == ENABLE)
        {
          LL_ADC_EnableIT_AWD3(hadc->Instance);
        }
        else
        {
          LL_ADC_DisableIT_AWD3(hadc->Instance);
        }
      }
    }

  }
  /* If a conversion is on going on ADC group regular or injected, no update  */
  /* could be done on neither of the AWD configuration structure parameters.  */
  else
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);

    tmp_hal_status = HAL_ERROR;
  }
  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}


/**
  * @}
  */

/** @defgroup ADC_Exported_Functions_Group4 Peripheral State functions
  *  @brief    ADC Peripheral State functions
  *
@verbatim
 ===============================================================================
            ##### Peripheral state and errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions to get in run-time the status of the
    peripheral.
      (+) Check the ADC state
      (+) Check the ADC error code

@endverbatim
  * @{
  */

/**
  * @brief  Return the ADC handle state.
  * @note   ADC state machine is managed by bitfields, ADC status must be
  *         compared with states bits.
  *         For example:
  *           " if ((HAL_ADC_GetState(hadc1) & HAL_ADC_STATE_REG_BUSY) != 0UL) "
  *           " if ((HAL_ADC_GetState(hadc1) & HAL_ADC_STATE_AWD1) != 0UL) "
  * @param hadc ADC handle
  * @retval ADC handle state (bitfield on 32 bits)
  */
uint32_t HAL_ADC_GetState(ADC_HandleTypeDef *hadc)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Return ADC handle state */
  return hadc->State;
}

/**
  * @brief  Return the ADC error code.
  * @param hadc ADC handle
  * @retval ADC error code (bitfield on 32 bits)
  */
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef *hadc)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  return hadc->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup ADC_Private_Functions ADC Private Functions
  * @{
  */

/**
  * @brief  Stop ADC conversion.
  * @param hadc ADC handle
  * @param ConversionGroup ADC group regular and/or injected.
  *          This parameter can be one of the following values:
  *            @arg @ref ADC_REGULAR_GROUP           ADC regular conversion type.
  *            @arg @ref ADC_INJECTED_GROUP          ADC injected conversion type.
  *            @arg @ref ADC_REGULAR_INJECTED_GROUP  ADC regular and injected conversion type.
  * @retval HAL status.
  */
HAL_StatusTypeDef ADC_ConversionStop(ADC_HandleTypeDef *hadc, uint32_t ConversionGroup)
{
  uint32_t tickstart;
  uint32_t Conversion_Timeout_CPU_cycles = 0UL;
  uint32_t conversion_group_reassigned = ConversionGroup;
  uint32_t tmp_ADC_CR_ADSTART_JADSTART;
  uint32_t tmp_adc_is_conversion_on_going_regular;
  uint32_t tmp_adc_is_conversion_on_going_injected;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  assert_param(IS_ADC_CONVERSION_GROUP(ConversionGroup));

  /* Verification if ADC is not already stopped (on regular and injected      */
  /* groups) to bypass this function if not needed.                           */
  tmp_adc_is_conversion_on_going_regular = LL_ADC_REG_IsConversionOngoing(hadc->Instance);
  tmp_adc_is_conversion_on_going_injected = LL_ADC_INJ_IsConversionOngoing(hadc->Instance);
  if ((tmp_adc_is_conversion_on_going_regular != 0UL)
      || (tmp_adc_is_conversion_on_going_injected != 0UL)
     )
  {
    /* Particular case of continuous auto-injection mode combined with        */
    /* auto-delay mode.                                                       */
    /* In auto-injection mode, regular group stop ADC_CR_ADSTP is used (not   */
    /* injected group stop ADC_CR_JADSTP).                                    */
    /* Procedure to be followed: Wait until JEOS=1, clear JEOS, set ADSTP=1   */
    /* (see reference manual).                                                */
    if (((hadc->Instance->CFGR & ADC_CFGR_JAUTO) != 0UL)
        && (hadc->Init.ContinuousConvMode == ENABLE)
        && (hadc->Init.LowPowerAutoWait == ENABLE)
       )
    {
      /* Use stop of regular group */
      conversion_group_reassigned = ADC_REGULAR_GROUP;

      /* Wait until JEOS=1 (maximum Timeout: 4 injected conversions) */
      while (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOS) == 0UL)
      {
        if (Conversion_Timeout_CPU_cycles >= (ADC_CONVERSION_TIME_MAX_CPU_CYCLES * 4UL))
        {
          /* Update ADC state machine to error */
          SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

          /* Set ADC error code to ADC peripheral internal error */
          SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);

          return HAL_ERROR;
        }
        Conversion_Timeout_CPU_cycles ++;
      }

      /* Clear JEOS */
      __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOS);
    }

    /* Stop potential conversion on going on ADC group regular */
    if (conversion_group_reassigned != ADC_INJECTED_GROUP)
    {
      /* Software is allowed to set ADSTP only when ADSTART=1 and ADDIS=0 */
      if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) != 0UL)
      {
        if (LL_ADC_IsDisableOngoing(hadc->Instance) == 0UL)
        {
          /* Stop ADC group regular conversion */
          LL_ADC_REG_StopConversion(hadc->Instance);
        }
      }
    }

    /* Stop potential conversion on going on ADC group injected */
    if (conversion_group_reassigned != ADC_REGULAR_GROUP)
    {
      /* Software is allowed to set JADSTP only when JADSTART=1 and ADDIS=0 */
      if (LL_ADC_INJ_IsConversionOngoing(hadc->Instance) != 0UL)
      {
        if (LL_ADC_IsDisableOngoing(hadc->Instance) == 0UL)
        {
          /* Stop ADC group injected conversion */
          LL_ADC_INJ_StopConversion(hadc->Instance);
        }
      }
    }

    /* Selection of start and stop bits with respect to the regular or injected group */
    switch (conversion_group_reassigned)
    {
      case ADC_REGULAR_INJECTED_GROUP:
        tmp_ADC_CR_ADSTART_JADSTART = (ADC_CR_ADSTART | ADC_CR_JADSTART);
        break;
      case ADC_INJECTED_GROUP:
        tmp_ADC_CR_ADSTART_JADSTART = ADC_CR_JADSTART;
        break;
      /* Case ADC_REGULAR_GROUP only*/
      default:
        tmp_ADC_CR_ADSTART_JADSTART = ADC_CR_ADSTART;
        break;
    }

    /* Wait for conversion effectively stopped */
    tickstart = HAL_GetTick();

    while ((hadc->Instance->CR & tmp_ADC_CR_ADSTART_JADSTART) != 0UL)
    {
      if ((HAL_GetTick() - tickstart) > ADC_STOP_CONVERSION_TIMEOUT)
      {
        /* New check to avoid false timeout detection in case of preemption */
        if ((hadc->Instance->CR & tmp_ADC_CR_ADSTART_JADSTART) != 0UL)
        {
          /* Update ADC state machine to error */
          SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

          /* Set ADC error code to ADC peripheral internal error */
          SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);

          return HAL_ERROR;
        }
      }
    }

  }

  /* Return HAL status */
  return HAL_OK;
}

/**
  * @brief  Enable the selected ADC.
  * @note   Prerequisite condition to use this function: ADC must be disabled
  *         and voltage regulator must be enabled (done into HAL_ADC_Init()).
  * @param hadc ADC handle
  * @retval HAL status.
  */
HAL_StatusTypeDef ADC_Enable(ADC_HandleTypeDef *hadc)
{
  uint32_t tickstart;

  /* ADC enable and wait for ADC ready (in case of ADC is disabled or         */
  /* enabling phase not yet completed: flag ADC ready not yet set).           */
  /* Timeout implemented to not be stuck if ADC cannot be enabled (possible   */
  /* causes: ADC clock not running, ...).                                     */
  if (LL_ADC_IsEnabled(hadc->Instance) == 0UL)
  {
    /* Check if conditions to enable the ADC are fulfilled */
    if ((hadc->Instance->CR & (ADC_CR_ADCAL | ADC_CR_JADSTP | ADC_CR_ADSTP | ADC_CR_JADSTART | ADC_CR_ADSTART
                               | ADC_CR_ADDIS | ADC_CR_ADEN)) != 0UL)
    {
      /* Update ADC state machine to error */
      SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

      /* Set ADC error code to ADC peripheral internal error */
      SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);

      return HAL_ERROR;
    }

    /* Enable the ADC peripheral */
    LL_ADC_Enable(hadc->Instance);

    /* Wait for ADC effectively enabled */
    tickstart = HAL_GetTick();

    while (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_RDY) == 0UL)
    {
      /*  If ADEN bit is set less than 4 ADC clock cycles after the ADCAL bit
          has been cleared (after a calibration), ADEN bit is reset by the
          calibration logic.
          The workaround is to continue setting ADEN until ADRDY is becomes 1.
          Additionally, ADC_ENABLE_TIMEOUT is defined to encompass this
          4 ADC clock cycle duration */
      /* Note: Test of ADC enabled required due to hardware constraint to     */
      /*       not enable ADC if already enabled.                             */
      if (LL_ADC_IsEnabled(hadc->Instance) == 0UL)
      {
        LL_ADC_Enable(hadc->Instance);
      }

      if ((HAL_GetTick() - tickstart) > ADC_ENABLE_TIMEOUT)
      {
        /* New check to avoid false timeout detection in case of preemption */
        if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_RDY) == 0UL)
        {
          /* Update ADC state machine to error */
          SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

          /* Set ADC error code to ADC peripheral internal error */
          SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);

          return HAL_ERROR;
        }
      }
    }
  }

  /* Return HAL status */
  return HAL_OK;
}

/**
  * @brief  Disable the selected ADC.
  * @note   Prerequisite condition to use this function: ADC conversions must be
  *         stopped.
  * @param hadc ADC handle
  * @retval HAL status.
  */
HAL_StatusTypeDef ADC_Disable(ADC_HandleTypeDef *hadc)
{
  uint32_t tickstart;
  const uint32_t tmp_adc_is_disable_on_going = LL_ADC_IsDisableOngoing(hadc->Instance);

  /* Verification if ADC is not already disabled:                             */
  /* Note: forbidden to disable ADC (set bit ADC_CR_ADDIS) if ADC is already  */
  /*       disabled.                                                          */
  if ((LL_ADC_IsEnabled(hadc->Instance) != 0UL)
      && (tmp_adc_is_disable_on_going == 0UL)
     )
  {
    /* Check if conditions to disable the ADC are fulfilled */
    if ((hadc->Instance->CR & (ADC_CR_JADSTART | ADC_CR_ADSTART | ADC_CR_ADEN)) == ADC_CR_ADEN)
    {
      /* Disable the ADC peripheral */
      LL_ADC_Disable(hadc->Instance);
      __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOSMP | ADC_FLAG_RDY));
    }
    else
    {
      /* Update ADC state machine to error */
      SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

      /* Set ADC error code to ADC peripheral internal error */
      SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);

      return HAL_ERROR;
    }

    /* Wait for ADC effectively disabled */
    /* Get tick count */
    tickstart = HAL_GetTick();

    while ((hadc->Instance->CR & ADC_CR_ADEN) != 0UL)
    {
      if ((HAL_GetTick() - tickstart) > ADC_DISABLE_TIMEOUT)
      {
        /* New check to avoid false timeout detection in case of preemption */
        if ((hadc->Instance->CR & ADC_CR_ADEN) != 0UL)
        {
          /* Update ADC state machine to error */
          SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

          /* Set ADC error code to ADC peripheral internal error */
          SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);

          return HAL_ERROR;
        }
      }
    }
  }

  /* Return HAL status */
  return HAL_OK;
}

/**
  * @brief  DMA transfer complete callback.
  * @param hdma pointer to DMA handle.
  * @retval None
  */
void ADC_DMAConvCplt(DMA_HandleTypeDef *hdma)
{
  /* Retrieve ADC handle corresponding to current DMA handle */
  ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Update state machine on conversion status if not in error state */
  if ((hadc->State & (HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_DMA)) == 0UL)
  {
    /* Set ADC state */
    SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);

    /* Determine whether any further conversion upcoming on group regular     */
    /* by external trigger, continuous mode or scan sequence on going         */
    /* to disable interruption.                                               */
    /* Is it the end of the regular sequence ? */
    if ((hadc->Instance->ISR & ADC_FLAG_EOS) != 0UL)
    {
      /* Are conversions software-triggered ? */
      if (LL_ADC_REG_IsTriggerSourceSWStart(hadc->Instance) != 0UL)
      {
        /* Is CONT bit set ? */
        if (READ_BIT(hadc->Instance->CFGR, ADC_CFGR_CONT) == 0UL)
        {
          /* CONT bit is not set, no more conversions expected */
          CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);
          if ((hadc->State & HAL_ADC_STATE_INJ_BUSY) == 0UL)
          {
            SET_BIT(hadc->State, HAL_ADC_STATE_READY);
          }
        }
      }
    }
    else
    {
      /* DMA End of Transfer interrupt was triggered but conversions sequence
         is not over. If DMACFG is set to 0, conversions are stopped. */
      if (READ_BIT(hadc->Instance->CFGR, ADC_CFGR_DMACFG) == 0UL)
      {
        /* DMACFG bit is not set, conversions are stopped. */
        CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);
        if ((hadc->State & HAL_ADC_STATE_INJ_BUSY) == 0UL)
        {
          SET_BIT(hadc->State, HAL_ADC_STATE_READY);
        }
      }
    }

    /* Conversion complete callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->ConvCpltCallback(hadc);
#else
    HAL_ADC_ConvCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }
  else /* DMA and-or internal error occurred */
  {
    if ((hadc->State & HAL_ADC_STATE_ERROR_INTERNAL) != 0UL)
    {
      /* Call HAL ADC Error Callback function */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
      hadc->ErrorCallback(hadc);
#else
      HAL_ADC_ErrorCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
    }
    else
    {
      /* Call ADC DMA error callback */
      hadc->DMA_Handle->XferErrorCallback(hdma);
    }
  }
}

/**
  * @brief  DMA half transfer complete callback.
  * @param hdma pointer to DMA handle.
  * @retval None
  */
void ADC_DMAHalfConvCplt(DMA_HandleTypeDef *hdma)
{
  /* Retrieve ADC handle corresponding to current DMA handle */
  ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Half conversion callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
  hadc->ConvHalfCpltCallback(hadc);
#else
  HAL_ADC_ConvHalfCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA error callback.
  * @param hdma pointer to DMA handle.
  * @retval None
  */
void ADC_DMAError(DMA_HandleTypeDef *hdma)
{
  /* Retrieve ADC handle corresponding to current DMA handle */
  ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Set ADC state */
  SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_DMA);

  /* Set ADC error code to DMA error */
  SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_DMA);

  /* Error callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
  hadc->ErrorCallback(hadc);
#else
  HAL_ADC_ErrorCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
}
