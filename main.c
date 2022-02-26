/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "DDSvariables.h"
#include "FREQUENCYvariables.h"
#include "STM32variables.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void PrepairVariablesBand1(void);
void PrepairVariablesBand2(void);
void PrepairVariablesBand3(void);
void PrepairVariablesBand4(void);
void Startup(void);
void Create_register(void);
void Create_instruction(void);
void Create_register_instruction_word(void);
void Create_instruction_word_FTW(void);
void Write_register_full(void);
void Write_register_FTW_full(void);
void Write_register_full(void);
void Write_register_FTW_full(void);


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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  char text1 [] = "W4INST\r\n";
  char text2 [] = "W4PPS\r\n";
  char text3 [] = "STRTUP\r\n";
  char text4 [] = "BSTART\r\n";

  SecondStart;

  __INLINE void Create_register()
  {

  // void Register_adress_config() {

    Register_adress[0]  = Ser_config;           // 0x0000
    Register_adress[1]  = Ser_opt2_update;      // 0x0005
    Register_adress[2]  = Pdown_pdown;          // 0x0010
    Register_adress[3]  = Pdown_reset1;         // 0x0012
    Register_adress[4]  = Pdwon_reset2;         // 0x0013
    Register_adress[5]  = Sysclk_ndivider;      // 0x0020
    Register_adress[6]  = Sysclk_pllparameters; // 0x0022
    Register_adress[7]  = Cal_dac1;             // 0x040B
    Register_adress[8]  = Cal_dac2;             // 0x040C
    Register_adress[9]  = Spur_A1;              // 0x0500
    Register_adress[10] = Spur_A2;              // 0x0501
    Register_adress[11] = Spur_A3;              // 0x0503
    Register_adress[12] = Spur_A4;              // 0x0504
    Register_adress[13] = Spur_B1;              // 0x0505
    Register_adress[14] = Spur_B2;              // 0x0506
    Register_adress[15] = Spur_B3;              // 0x0508
    Register_adress[16] = Spur_B4;              // 0x0509


  ////////////////////////////////////////////////////////////////////////////////////////

  // void Register_serial_config() {

    Register_data[0] |= SDO_active1       << 0; // bit 0
    Register_data[0] |= LSB_first1        << 1; // bit 1
    Register_data[0] |= Soft_reset1       << 2; // bit 2
    Register_data[0] |= Long_instruction1 << 3; // bit 3
    Register_data[0] |= Long_instruction2 << 4; // bit 4
    Register_data[0] |= Soft_reset2       << 5; // bit 5
    Register_data[0] |= LSB_first2        << 6; // bit 6
    Register_data[0] |= SDO_active2       << 7; // bit 7


  // void Register_IOupdate() {
    Register_data[1] |= Register_update      << 0; // bit 0

  // void Register_powerdown_enable() {

    Register_data[2] |= Powerdown_digital      << 0; // bit 0
    Register_data[2] |= Powerdown_full         << 1; // bit 1
    Register_data[2] |= Not_used_1             << 2; // bit 2
    Register_data[2] |= Not_used_2             << 3; // bit 3
    Register_data[2] |= Powerdown_sysclock_pll << 4; // bit 4
    Register_data[2] |= Enable_output_doubler  << 5; // bit 5
    Register_data[2] |= Enable_CMOS            << 6; // bit 6
    Register_data[2] |= Powerdown_HSTL         << 7; // bit 7


  // void Register_DDS_reset() {

    Register_data[3] = Reset_DDS;                // bit 0


  // void Register_powerdown_reset() {

    Register_data[4] |= Not_used_3         << 0; // bit 0
    Register_data[4] |= Reset_sdivider     << 1; // bit 1
    Register_data[4] |= Not_used_4         << 2; // bit 2
    Register_data[4] |= Reset_sdivider2    << 3; // bit 3
    Register_data[4] |= Not_used_5         << 4; // bit 4
    Register_data[4] |= Not_used_6         << 5; // bit 5
    Register_data[4] |= Not_used_7         << 6; // bit 6
    Register_data[4] |= Powerdown_fund_DDS << 7; // bit 7


  // void Register_n_divider() {

    Register_data[5] |= N_divider0         << 0; // bit 0
    Register_data[5] |= N_divider1         << 1; // bit 1
    Register_data[5] |= N_divider2         << 2; // bit 2
    Register_data[5] |= N_divider3         << 3; // bit 3
    Register_data[5] |= N_divider4         << 4; // bit 4
    Register_data[5] |= Not_used_8         << 5; // bit 5
    Register_data[5] |= Not_used_9         << 6; // bit 6
    Register_data[5] |= Not_used_10        << 7; // bit 7


  // void Register_pll_parameters() {

    Register_data[6] |= Charge_pump1       << 0; // bit 0
    Register_data[6] |= Charge_pump2       << 1; // bit 1
    Register_data[6] |= VCO_range          << 2; // bit 2
    Register_data[6] |= Reference_2x       << 3; // bit 3
    Register_data[6] |= Not_used_11        << 4; // bit 4
    Register_data[6] |= Not_used_12        << 5; // bit 5
    Register_data[6] |= Not_used_13        << 6; // bit 6
    Register_data[6] |= VCO_autorange      << 7; // bit 7


  // void Register_calibration() {

    Register_data[7] |= DAC_scale_current1;       // byte 0
    Register_data[8] |= DAC_scale_current2;       // byte 1 bit 0


  // void Register_spurA() {

    Register_data[9] |= SpurA_harmonic0    << 0; // bit 0
    Register_data[9] |= SpurA_harmonic1    << 1; // bit 1
    Register_data[9] |= SpurA_harmonic2    << 2; // bit 2
    Register_data[9] |= SpurA_harmonic3    << 3; // bit 3
    Register_data[9] |= Not_used_14        << 4; // bit 4
    Register_data[9] |= Not_used_15        << 5; // bit 5
    Register_data[9] |= AmplitudeA_gainx2  << 6; // bit 6
    Register_data[9] |= Enable_HSRA        << 7; // bit 7


  // void Register_spurA_magnetude() {

    Register_data[10] |= SpurA_magnetude;          // byte 0


  // void Register_spurA_phase() {

    Register_data[11]  |= SpurA_phase;            // byte 0
    Register_data[12] |= SpurA_phase_bit8;        // byte 1 bit 0


  // void Register_spurB() {

    Register_data[13] |= SpurB_harmonic0    << 0; // bit 0
    Register_data[13] |= SpurB_harmonic1    << 1; // bit 1
    Register_data[13] |= SpurB_harmonic2    << 2; // bit 2
    Register_data[13] |= SpurB_harmonic3    << 3; // bit 3
    Register_data[13] |= Not_used_16        << 4; // bit 4
    Register_data[13] |= Not_used_17        << 5; // bit 5
    Register_data[13] |= AmplitudeB_gainx2  << 6; // bit 6
    Register_data[13] |= Enable_HRSB        << 7; // bit 7


  // void Register_spurB_magnetude() {

    Register_data[14] |= SpurB_magnetude;          // byte 0


  // void Register_spurB_phase() {

    Register_data[15] |= SpurB_phase;              // byte 0
    Register_data[16] |= SpurB_phase_bit8;         // byte 1 bit 0
  }

  __INLINE void Create_instruction()
  {
       Instruction_header = Read_write
                          + Transfer_bytes_register;
  }

  __INLINE void Create_register_instruction_word()
  {
       for (z = 0; z < Register_amount; z++)
       {
       Register_adress[z] = Instruction_header
                           + Register_adress[z];
       }
  }

  __INLINE void Create_instruction_word_FTW()
  {
       for (z = 0; z < Register_amount_FTW; z++)
       {
       Register_adress_FTW[z] = Instruction_header
                              + Register_adress_FTW[z];
       }
  }

  __INLINE void Write_register_full()
  {

       for (z = 0; z < Register_amount; z++)
       {
  //     uint8_t bytes[3] = { Register_adress[z] << 16, Register_adress[z] << 8, Register_data[z] };

  //     SPI.transfer(bytes[0]); // 4,298uS Gehele instructie
  //     SPI.transfer(bytes[1]); // 0,425uS voor 8-bits
  //     SPI.transfer(bytes[2]);
  //     SERIAL.write(bytes[0]); // For testing output
  //     SERIAL.write(bytes[1]);
  //     SERIAL.write(bytes[2]);
    }
  }

  __INLINE void Write_register_FTW_full()
  {
       for (z = 0; z < Register_amount_FTW; z++)
       {
  //     uint8_t bytes[3] = { Register_adress_FTW[z] << 16, Register_adress_FTW[z] << 8, Register_data_FTW[z] }; // Total of 8 bytes

  //     SPI.transfer(bytes[0]);
  //     SPI.transfer(bytes[1]);
  //     SPI.transfer(bytes[2]);
  //     SERIAL.write(bytes[0]); // For testing output
  //     SERIAL.write(bytes[1]);
  //     SERIAL.write(bytes[2]);
       }
  }

  __INLINE void PrepairVariablesBand1()
           {
                Band1ModeTimerCounter = 0;
                Position_Array_Band1 = 0x00;
                ModeCounterBand1 = 0x00;

                Start_Array_Band1 = Mode_Start_Array[ModeBand1];
                End_Array_Band1 = Mode_Start_Array[ModeBand1] + Mode_Lenght_Array[ModeBand1];
                 Symbol_Duration_Band1 = Mode_Symbol_Duration[ModeBand1];
                 ModeCounterBand1 = ModeRepeat [ModeBand1]; // Moet nog geimplemeteerd worden
           }

  __INLINE void PrepairVariablesBand2()
           {
                Band2ModeTimerCounter = 0;
                Position_Array_Band2 = 0x00;
                ModeCounterBand2 = 0x00;

                Start_Array_Band2 = Mode_Start_Array[ModeBand2];
                End_Array_Band2 = Mode_Start_Array[ModeBand2] + Mode_Lenght_Array[ModeBand2];
                Symbol_Duration_Band2 = Mode_Symbol_Duration[ModeBand2];
                ModeCounterBand2 = ModeRepeat [ModeBand2];
           }

  __INLINE void PrepairVariablesBand3()
           {
                Band3ModeTimerCounter = 0;
                Position_Array_Band3 = 0x00;
                 ModeCounterBand3 = 0x00;

                Start_Array_Band3 = Mode_Start_Array[ModeBand3];
                End_Array_Band3 = Mode_Start_Array[ModeBand3] + Mode_Lenght_Array[ModeBand3];
                Symbol_Duration_Band3 = Mode_Symbol_Duration[ModeBand3];
                ModeCounterBand3 = ModeRepeat[ModeBand3];
           }

  __INLINE void PrepairVariablesBand4()
           {
                Band4ModeTimerCounter = 0;
                 Position_Array_Band4 = 0x00;
                ModeCounterBand4 = 0x00;

                Start_Array_Band4 = Mode_Start_Array[ModeBand4];
                End_Array_Band4 = Mode_Start_Array[ModeBand4] + Mode_Lenght_Array[ModeBand4];
                Symbol_Duration_Band4 = Mode_Symbol_Duration[ModeBand4];
                ModeCounterBand4 = ModeRepeat [ModeBand4];
           }




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	     StartupDone = 1;
	     if (StartupDone == 0) // Run once to initialize DDS
	     {
	    	Create_register();
	    	Create_instruction();
	    	Create_register_instruction_word();
	    	Create_instruction_word_FTW();
	    	CS1on;
	    	Write_register_full();
	    	Write_register_FTW_full();
	    	UPD1on; // 1,3uS
	    	UPD1off;
	    	CS1off;
	    	CS2on;
	    	Write_register_full();
	    	Write_register_FTW_full();
	        UPD2on;
	        UPD2off;
	    	CS2off;

	        WaitForInstruction = 1;
	        StartupDone = 1;
	        HAL_UART_Transmit(&huart6, text3, sizeof(text3), HAL_MAX_DELAY);
	     }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



      HAL_UART_Transmit(&huart6, text1, sizeof(text1), HAL_MAX_DELAY);

	    STM32outputON; // AtmegaCanSendData                             // digitalWrite(STM32output, HIGH); // AtmegaCanSendData

    do                                                                   // do
    {                                                                    // {
	    HAL_UART_Receive(&huart1, STM32ReadBuffer, buffsize, HAL_MAX_DELAY);	     //    if (STM32.available()>0)
	                                                                     //    {
	                                                                     //       STM32.readBytes(STM32ReadBuffer, buffsize);

	    AtmegaHasNoData = STM32input; // ATMEGAouput                     //       AtmegaHasNoData = digitalRead(STM32input); // ATMEGAouput

	    if (AtmegaHasNoData == LOW)                                      //       if (AtmegaHasNoData == LOW)
	    {                                                                //       {
           STM32outputOFF; // AtmegaCanSendData                          //          digitalWrite(STM32output, LOW); // AtmegaCanSendData

           STM32ReadState   = 1;                                         //          STM32ReadState   = 1;
           WaitForInstruction = 0;                                       //          WaitForInstruction = 0;
           AtmegaHasNoData = 0;                                          //          AtmegaHasNoData = 0;
	    }                                                                //       }
	                                                                     //    }
	} while (WaitForInstruction == 1); // Activatie voor inkomende data  // } while (WaitForInstruction == 1); // Activatie voor inkomende data


	     while (STM32ReadState == 1) // Here all the serial decoding and programming
	     {

	    	   if (STM32ReadBuffer[0] == AdresSTM32) // Take the 3rd byte
	           {
	              ModeBand1 = STM32ReadBuffer[2]; // Example 0x02 (PI4)
	              ModeBand2 = STM32ReadBuffer[3];
	              ModeBand3 = STM32ReadBuffer[4];
	              ModeBand4 = STM32ReadBuffer[5];
	           }

	           WaitFor1PPS = 1;
	           STM32ReadState = 0;
	     }

	     // First a bit calculation to prepare
	     // This is in streaming mode eventually can end with register 0x005 for software I/O update
	     // Here the actual sending of: 2 bytes instruction byte + adress byte (fixed)
	     //                             6 bytes FTW value
	     //                             This is a total of 8 bytes = 48 bits
	     //
	     // Write only an streaming mode = 0x0000 + 0x3000 = 0x3000
	     // With MSB first adres is counting lower. So start adress = 0x01AB (otherwise 0x01A6)
	     // The instruction word will be 0x31AB
	     //
	     // Array structure is : mode = band[z], tone[y], frequency[x]


	      Repeat                = 1;

	      SecondTimer           = 0;
	      MaximumTime           = 58;

	      PrepairVariablesBand1(); // LET OP: ATMEGA geeft CW
	      PrepairVariablesBand2();
	      PrepairVariablesBand3();
	      PrepairVariablesBand4();

	      //////// CW           WSPR         QRSS         PI4
	      if ((ModeBand1 || ModeBand2 || ModeBand3 || ModeBand4) == WSPR) // 2 minutes
	      {
	         if (ModeBand1 != WSPR && QRSS)
             {
                 ModeCounterBand1 = ModeCounterBand1 * 2;
             }

             if (ModeBand2 != WSPR && QRSS)
             {
                ModeCounterBand2 = ModeCounterBand2 * 2;
             }

             if (ModeBand3 != WSPR && QRSS)
             {
                ModeCounterBand3 = ModeCounterBand3 * 2;
             }

             if (ModeBand4 != WSPR && QRSS)
             {
                ModeCounterBand4 = ModeCounterBand4 * 2;
             }

             MaximumTime = 119;
	      }

	      //////// CW           WSPR         QRSS         PI4
	      if ((ModeBand1 || ModeBand2 || ModeBand3 || ModeBand4) == QRSS) // 10 minutes
          {
             if (ModeBand1 != WSPR && QRSS)
             {
                ModeCounterBand1 = ModeCounterBand1 * 5;
             }

             if (ModeBand2 != WSPR && QRSS)
             {
                ModeCounterBand2 = ModeCounterBand2 * 5;
             }

             if (ModeBand3 != WSPR && QRSS)
             {
                ModeCounterBand3 = ModeCounterBand3 * 5;
             }

             if (ModeBand4 != WSPR && QRSS)
             {
                ModeCounterBand4 = ModeCounterBand4 * 5;
             }

             MaximumTime = 599;
          }


      HAL_UART_Transmit(&huart6, text2, sizeof(text2), HAL_MAX_DELAY);

	     while (WaitFor1PPS == 1)
	     {
//	           PPSdetect = PPSinput;

	           if (PPSinput == 0) // Input is inverted
	           {

//	              PPSdetect = 1;
	              z = 1;
	              WaitFor1PPS = 0;
	           }
	     }

	            SecondStart;
	            SPIsequenceStart;
	            SymbolDurationStart;
	            HAL_UART_Transmit(&huart6, text4, sizeof(text4), HAL_MAX_DELAY);
	      do
	      {
/*	    	     uint8_t Band1Mode[6][176]        = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                 uint8_t Frequency_Start_Array[]  = {0x00,
	    	     uint8_t Band1[22][8]             = {{0x31, 0xAB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

	             Data1 = Band1[freq1][0];
				 Data2 = Band2[freq2][0];
				 Data3 = Band3[freq3][0];
				 Data4 = Band4[freq4][0];
*/
	    	     freq1 = Band1Mode[ModeBand1][Position_Array_Band1] + Frequency_Start_Array[ModeBand1];
	    	     freq2 = Band2Mode[ModeBand2][Position_Array_Band2] + Frequency_Start_Array[ModeBand2];
	    	     freq3 = Band3Mode[ModeBand3][Position_Array_Band3] + Frequency_Start_Array[ModeBand3];
	    	     freq4 = Band4Mode[ModeBand4][Position_Array_Band4] + Frequency_Start_Array[ModeBand4];

	//             SERIAL.println(SecondTimer); Met deze erbij werkt hij wel

	       } while (SecondTimer < MaximumTime); //  ModeBand1counter moet nog geimplemteerd worden

	      SecondStop;
	      SPIsequenceStop;
	      SymbolDurationStop;
	      z = 0;
	      WaitForInstruction = 1;
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
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 57600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, UPD2_Pin|CS2_Pin|UPD1_Pin|CS1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STM32output_GPIO_Port, STM32output_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UPD2_Pin CS2_Pin UPD1_Pin CS1_Pin */
  GPIO_InitStruct.Pin = UPD2_Pin|CS2_Pin|UPD1_Pin|CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : STM32output_Pin */
  GPIO_InitStruct.Pin = STM32output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(STM32output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STM32input_Pin */
  GPIO_InitStruct.Pin = STM32input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STM32input_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PPS_Pin */
  GPIO_InitStruct.Pin = PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PPS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == PPS_Pin)
    {
    	LEDtoggle;
    	HAL_GPIO_EXTI_IRQHandler(PPS_Pin);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // TX Done .. Do Something ...
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi1)
{
    // TX Done .. Do Something ...
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
     if (htim->Instance == TIM2)  // Timetick 1 second
     {
	    SecondTimer++;

	    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
     }

     if (htim->Instance == TIM3) // Timetick at least 4x faster then TIM4
     {
        switch (z)
        {
        case 1:
               CS1on;

               HAL_SPI_Transmit(&hspi1, &Band1[freq1][0], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band1[freq1][1], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band1[freq1][2], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band1[freq1][3], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band1[freq1][4], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band1[freq1][5], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band1[freq1][6], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band1[freq1][7], 1, 100);

               UPD1on;
               UPD1off;
               CS1off;
               z = 2;
        break;

        case 2:
               CS2on;

               HAL_SPI_Transmit(&hspi1, &Band2[freq2][0], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band2[freq2][1], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band2[freq2][2], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band2[freq2][3], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band2[freq2][4], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band2[freq2][5], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band2[freq2][6], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band2[freq2][7], 1, 100);

               UPD2on;
               UPD2off;
               CS2off;
               z = 3;
        break;

        case 3:
               CS1on;

               HAL_SPI_Transmit(&hspi1, &Band3[freq3][0], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band3[freq3][1], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band3[freq3][2], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band3[freq3][3], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band3[freq3][4], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band3[freq3][5], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band3[freq3][6], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band3[freq3][7], 1, 100);

               UPD1on;
               UPD1off;
               CS1off;
               z = 4;
        break;

        case 4:
               CS2on;

               HAL_SPI_Transmit(&hspi1, &Band4[freq4][0], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band4[freq4][1], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band4[freq4][2], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band4[freq4][3], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band4[freq4][4], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band4[freq4][5], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band4[freq4][6], 1, 100);
               HAL_SPI_Transmit(&hspi1, &Band4[freq4][7], 1, 100);

               UPD2on;
               UPD2off;
               CS2off;
               z = 1;
        break;

       default:

        break;
        }
      __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
     }

     if (htim->Instance == TIM4) // Ticker is 0,1ms
     {     // Interrupt Counter need also to be set 0
        Band1ModeTimerCounter++;
        Band2ModeTimerCounter++;
        Band3ModeTimerCounter++;
        Band4ModeTimerCounter++;

        if (Band1ModeTimerCounter > Symbol_Duration_Band1)
        {
           Position_Array_Band1++;

           if (Position_Array_Band1 > End_Array_Band1)
           {
              Position_Array_Band1 = 0x00;
           }

           Band1ModeTimerCounter = 0;
        }

        if (Band2ModeTimerCounter > Symbol_Duration_Band2)
        {
           Position_Array_Band2++;

           if (Position_Array_Band2 > End_Array_Band2)
           {
              Position_Array_Band2 = 0x00;
           }

           Band2ModeTimerCounter = 0;
        }

        if (Band3ModeTimerCounter > Symbol_Duration_Band3)
        {
           Position_Array_Band3++;

           if (Position_Array_Band3 > End_Array_Band3)
           {
              Position_Array_Band3 = 0x00;
           }

           Band3ModeTimerCounter = 0;
        }

        if (Band4ModeTimerCounter > Symbol_Duration_Band4)
        {
           Position_Array_Band4++;

           if (Position_Array_Band4 > End_Array_Band4)
           {
              Position_Array_Band4 = 0x00;
           }

           Band4ModeTimerCounter = 0;
        }
      __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
     }
}
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
