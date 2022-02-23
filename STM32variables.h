/*
 * STM32variables.h
 *
 *  Created on: 20 feb. 2022
 *      Author: Gebruiker
 */

#ifndef SRC_STM32VARIABLES_H_
#define SRC_STM32VARIABLES_H_

#define AdresDDS        0x01 // DDS unit
#define AdresSTM32      0x02 // Co-processor
#define AdresATMEGA     0x03 // Base
#define AdresGPS        0x04 // No external instructions needed

#define AdresUNO0       0x05 // Lora unit
#define AdresUNOLORA0   0x06 // Lora unit

#define AdresSERIAL     0x07 // For checking/debugging
#define AdresLCD        0x27 // Not mounted

#define AdresUNO1A      0x10 // IOT-box base Uno
#define AdresUNOLORA1   0x11 // IOT-box Lora unit
#define AdresUNONRF1    0x12 // IOT-box NRF unit
#define AdresUNOLCD     0x27 // IOT-box LCD

#define AdresESP1B      0x15 // IOT-box base ESP
#define AdresESPLOCAL   0x16 // IOT-box for Local instructions
#define AdresESPETH     0x17 // IOT-box WiFi
#define AdresESPINT     0x18 // IOT-box Cable
#define AdresESPSERIAL  0x19 // For checking/debugging

#define AdresUNO2       0x20 // Handheld-box base
#define AdresUNONRF2    0x21 // Handheld-box NRF unit

#define Notice          0x2A // For feedback tekst

#define SecondsStart         TIM2->CR1 = 1 // Start 32-bit TIM2
#define SecondsStop          TIM2->CR1 = 0 // Stop  32-bit TIM2
#define SPIsequenceStart     TIM3->CR1 = 1 // Start 16-bit TIM3
#define SPIsequenceStop      TIM3->CR1 = 0 // Stop  16-bit TIM3
#define SymbolDurationStart  TIM4->CR1 = 1 // Start 16-bit TIM4
#define SymbolDurationStop   TIM4->CR1 = 0 // Stop  16-bit TIM4

#define CS1on                GPIOA -> ODR |=  CS1_Pin
#define CS1off               GPIOA -> ODR &= ~CS1_Pin
#define CS2on                GPIOA -> ODR |=  CS2_Pin
#define CS2off               GPIOA -> ODR &= ~CS2_Pin
#define UPD1on               GPIOA -> ODR |=  UPD1_Pin
#define UPD1off              GPIOA -> ODR &= ~UPD1_Pin
#define UPD2on               GPIOA -> ODR |=  UPD2_Pin
#define UPD2off              GPIOA -> ODR &= ~UPD2_Pin

#define STM32outputON        GPIOB -> ODR |=  STM32output_Pin
#define STM32outputOFF       GPIOB -> ODR &= ~STM32output_Pin

#define LEDon                GPIOA -> ODR |=  LED_Pin
#define LEDoff               GPIOA -> ODR &= ~LED_Pin

#define STM32input           GPIOC -> IDR &   STM32input_Pin
#define PPSinput             GPIOA -> IDR &  ~PPS_Pin

#define LOW                  0
#define HIGH                 1

      uint8_t TEST   = 0;
      uint8_t CONFIG = 0;
      uint8_t SEQUENCE = 0;
      uint8_t BAND   = 0;
      uint8_t MODE   = 0;

      uint8_t test =0;
      uint8_t config = 0;
      uint8_t sequence = 0;
      uint8_t Band = 0;
      uint8_t Mode = 0;
      uint8_t toon = 0;

      uint16_t Instruction_word_FTW;

      uint8_t x = 0;
      uint8_t y = 0;
      uint8_t z = 1;

      uint8_t buffsize   = 8;
      uint8_t STM32ReadState   = LOW;
      uint8_t STM32WriteState  = LOW;
      uint8_t STM32ReadBuffer[] = {0,0,0,0,0,0,0,0}; //    [buffsize];
      uint8_t STM32WriteBuffer[] = {0,0,0,0,0,0,0,0}; //    [buffsize];

      uint8_t SPIreadbuffer [] = {0};
      uint8_t SPIwritebuffer [] = {0};

      uint8_t StartupDone = 0;
      uint8_t WaitForInstruction = 0;
      uint8_t AtmegaHasNoData = 0;
      uint8_t WaitFor1PPS = 0;
      uint8_t PPSdetect = 0;
      uint8_t BeaconStart = 0;
      uint8_t SecondTimer = 0;

#endif /* SRC_STM32VARIABLES_H_ */
