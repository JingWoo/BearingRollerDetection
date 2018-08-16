/**
  ******************************************************************************
  * @file    TIM/TIM_PWMOutput/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include<stdio.h>  
#include<stdlib.h>  
typedef struct node  
{  
    uint64_t MoveStepNum;  
    struct node *link;  
}sRoller,*pRoller; 
sRoller * creatRollerList(uint8_t n);

typedef enum
{
  CNotStarted= 0U,
  CElectroMagnetStart = 1U,
  CDM542MotorPositiveTurn = 2U,
  CElectroMagnetRelease = 3U,
  CDM542MotorReversal = 4U
} TransitionalInstitutionsStatus;

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor TIMx instance used and associated
   resources */
/* Definition for TIMx clock resources */
#define TIMx                           TIM3
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM3_CLK_ENABLE()

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM3_IRQn
#define TIMx_IRQHandler                TIM3_IRQHandler

/* Definition for USARTx clock resources */
#define USARTx                           USART6
#define USARTx_CLK_ENABLE()              __USART6_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART6_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART6_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_6
#define USARTx_TX_GPIO_PORT              GPIOC
#define USARTx_TX_AF                     GPIO_AF8_USART6
#define USARTx_RX_PIN                    GPIO_PIN_7
#define USARTx_RX_GPIO_PORT              GPIOC
#define USARTx_RX_AF                     GPIO_AF8_USART6

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_STREAM              DMA2_Stream6
#define USARTx_RX_DMA_STREAM              DMA2_Stream1
#define USARTx_TX_DMA_CHANNEL             DMA_CHANNEL_5
#define USARTx_RX_DMA_CHANNEL             DMA_CHANNEL_5


/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA2_Stream6_IRQn
#define USARTx_DMA_RX_IRQn                DMA2_Stream1_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA2_Stream6_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA2_Stream1_IRQHandler

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART6_IRQn
#define USARTx_IRQHandler                USART6_IRQHandler

/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE
  
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */
//ÖùÃæ¼ì²â²½½øµç»úÅäÖÃ
#define AWO_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)// µç¶¯»úµÄµçÁ÷»á±»ÇÐ¶Ï µç¶¯»úµÄ±£³ÖÁ¦ÏûÊ§ ÄÜ×ª¶¯µç¶¯»úµÄÊä³öÖá
#define AWO_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET) //µç¶¯»úµÄµçÁ÷¿ªÊ¼¹©µç µç¶¯»ú»Ö¸´±£³ÖÁ¦
#define AWO_TOGGLE HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4)

#define CS_ON    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET) // Îª»ù±¾²½¾à½Ç
#define CS_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET) //Îª²½¾à½ÇÉè¶¨¿ª¹ØSW1µÄÉè¶¨  
#define CS_TOGGLE HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6)

#define ACDOFF_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)//µçÁ÷ÏÂ½µ¹¦ÄÜ½â³ý
#define ACDOFF_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET) //Ò»°ãÉè¶¨ÎªOFF£
#define ACDOFF_TOGGLE HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5)

#define MotorP_HIGH  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define MotorP_LOW   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define MotorPToggle HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3)

#define MotorN_HIGH  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)
#define MotorN_LOW   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MotorNToggle HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_6)

//ÖùÃæ¼ì²âÏßÕóÏà»úÅäÖÃ
#define CameraTriggerRise  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET) //´¥·¢Ïà»úÉÏÉýÑØ
#define CameraTriggerFall  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET) //´¥·¢Ïà»úÏÂ½µÑØ
#define CameraTriggerToggle HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_3)

//Öù£¨×¶£©Ãæ¼ì²âÏß¹âÔ´¿ØÖÆ
#define LightSourceControl_ON   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);//Ïß¹âÔ´ÁÁ
#define LightSourceControl_OFF   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);//Ïß¹âÔ´Ãð
#define LightSourceControlToggle HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);

//µç´ÅÌú¿ØÖÆ
//PWM
/* Definition for TIMx clock resources */
#define ElectroMagnetTIM2CH1PWM                     TIM2
#define ElectroMagnetTIM2_CLK_ENABLE()              __HAL_RCC_TIM2_CLK_ENABLE()
/* Definition for TIMx Channel Pins */
#define ElectroMagnetTIM2_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define ElectroMagnetTIM2_GPIO_PORT_CHANNEL1        GPIOA
#define ElectroMagnetTIM2_GPIO_PIN_CHANNEL1         GPIO_PIN_15
#define ElectroMagnetTIM2_GPIO_AF_CHANNEL1          GPIO_AF1_TIM2
//INA
#define ElectroMagnetINA_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
#define ElectroMagnetINA_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
#define ElectroMagnetINAToggle HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
//INB
#define ElectroMagnetINB_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
#define ElectroMagnetINB_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
#define ElectroMagnetINBToggle HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

//»¬Ì¨µç»ú¿ØÖÆDM542
//Plus
#define DM542Plus_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
#define DM542Plus_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
#define DM542PlusToggle HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
//Dir
#define DM542Dir_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
#define DM542Dir_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
#define DM542DirToggle HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
//Enable
#define DM542Enable_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
#define DM542Enable_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
#define DM542EnableToggle HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);

//ÏÂÁÏ»¬Ì¨µç»ú¿ØÖÆDM541
//Plus
#define DM541Plus_ON    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
#define DM541Plus_OFF   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
#define DM541PlusToggle HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
//Dir
#define DM541Dir_ON    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
#define DM541Dir_OFF   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
#define DM541DirToggle HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_7);
////Enable 
#define DM541Enable_ON    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
#define DM541Enable_OFF   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
#define DM541EnableToggle HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);

//Í¬²½´«ËÍ´øµç»úMA860H
//Plus
#define MA860HPlus_ON    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);
#define MA860HPlus_OFF   HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_RESET);
#define MA860HPlusToggle HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_3);
//Dir
#define MA860HDir_ON    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
#define MA860HDir_OFF   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
#define MA860HDirToggle HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
//Enable
#define MA860HEnable_ON    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
#define MA860HEnable_OFF   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
#define MA860HEnableToggle HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);

//ÒôÈ¦µç»ú--Î»ÖÃµ÷Õû
//MotorP
//Dir
#define VoiceCoilMotorP_ON   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
#define VoiceCoilMotorP_OFF   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);
#define VoiceCoilMotorPToggle HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_8);
//Enable
#define VoiceCoilMotorN_ON   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET);
#define VoiceCoilMotorN_OFF   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);
#define VoiceCoilMotorNToggle HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_7);

//ÒôÈ¦µç»ú--ÌÞ³ý
//MotorP
//Dir
#define EliminateVoiceCoilMotorP_ON   HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);
#define EliminateVoiceCoilMotorP_OFF   HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_RESET);
#define EliminateVoiceCoilMotorPToggle HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_5);
//Enable
#define EliminateVoiceCoilMotorN_ON   HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);
#define EliminateVoiceCoilMotorN_OFF   HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_RESET);
#define EliminateVoiceCoilMotorNToggle HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_4);
//ToDo ¹âµç¼ì²â

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */



#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

