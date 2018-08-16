/**
  ******************************************************************************
  * @file    main.c
  * @author  Uriel-WuJing
  * @brief   A Bearing Roller Surface Defect Detection System Based on Machine Vision.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "delay.h"
#include <string.h> 

/* Private typedef -----------------------------------------------------------*/
#define  ElectroMagnetTIM2PERIOD_VALUE       (uint32_t)(1000 - 1)  /* Period Value  */
#define  ElectroMagnetTIM2PULSE1_VALUE       (uint32_t)(ElectroMagnetTIM2PERIOD_VALUE/1.3)        /* Capture Compare 1 Value  */

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* TIM handle declaration */
TIM_HandleTypeDef    TimBaseHandle;
/* Prescaler declaration */
uint32_t TimBaseFrequency=10000000;//10MHz
uint32_t TimBasePeriodBaseNumber=700;
__IO uint32_t uwBasePrescalerValue = 0;
double SpeedUpFactor;
double SpeedUpFactor_final = 200;  //电机加速启动分频系数

UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */

/* Buffer used for transmission */
#define CommandLength 7
uint8_t aTxBuffer[CommandLength] = {0xEE,0x01,0x00,0xFF,0xFC,0xFF,0xFF};
uint8_t aRxBuffer[CommandLength];

char PCCompleteOutLog[256];

//Cylindrical surface
uint64_t PulseNumberPerAcquisition=0; //拍摄一次所需方波个数
uint64_t MotorStepCounterInAcquisition=0; //每次触发线阵相机采图步进电机已运转的步距数
uint64_t TotalAcquisitionNumber=0; //拍摄滚子一周所需方波个数
uint64_t AlreadyAcquisitionCounter=0; //滚子一周已采集次数

uint64_t RollerRadius = 5; //单位mm
uint8_t  Resolution = 15; //单位um

uint64_t LinearCameraStartupDelayTime = 60*522.3881;//0x32ff; //根据定时器周期与光电传感器至检测区域距离决定相机延时启动时间
uint64_t LinearCameraDelayStartCounter=0; //线阵相机延时启动计数器
FlagStatus LinearCameraDelayStartFlag   = RESET;    //线阵相机延时启动标志位
FlagStatus LinearCameraTriggerFlag = RESET;    //线阵相机外触发标志位

uint8_t RollerNumberOnLine=0; //需要检测的滚子数目
uint32_t ImageAcquisitionLineNumber=0; //滚子柱面已经采集的行数
uint32_t ImageAcquisitionLineActuallyNumber=0; //滚子柱面整幅图像实际采集行数

//FlagStatus PhotoelectricSensorStatus=SET;    //光电传感器电平

uint8_t OpticalFiberSensorToTransitionZoneMaxRollerNum=6;

FlagStatus RxCompleteFlag=RESET;    //接收完成标志位 Pc To MUC
FlagStatus TxCompleteFlag=RESET;    //发送完成标志位 MCU To Pc
FlagStatus PSCompleteFlag=RESET;    //光电传感器检测到滚子标志位   detected
FlagStatus CTCompleteFlag=RESET;    //相机触发标志位   Camera Triger
FlagStatus PCCompleteFlag=RESET;    //图像采集    picture collect
FlagStatus UMCompleteFlag=RESET;    //下料完成标志位 Unload Motor

//传感器

FlagStatus TransPositionAccuracyFlag=RESET;   //横移位置矫正传感器
FlagStatus TransTriggerFlag=RESET;            //横移触发传感器
FlagStatus BeltPositionAccuracyFlag=RESET;    //同步带位置矫正传感器
FlagStatus UnloadPositionAccuracyFlag=RESET;  //下料位置矫正传感器

uint64_t  OpticalFiberSensorToTransitionZoneDistance = 146269;//280*522.3881=nP*360*3.5./(0.072*33.5) 光纤传感器至过渡区域距离=》电机脉冲次数
FlagStatus TransitionalInstitutionsActivatedFlag = RESET; //过渡机构启动标志
TransitionalInstitutionsStatus TransInstStatus=CNotStarted; //过渡机构状态

uint64_t TransitionalInstitutionMovingDistance=50; //单位：mm
uint64_t SlidingPlatformVelocity =1000; //单位：mm/s
uint64_t ElectroMagnetStartWaitMaxTime=0; //100ms
uint64_t DM542MotorPositiveTurnExecutionMaxTime=0; //calc
uint64_t ElectroMagnetReleaseWaitMaxTime=0; //100ms
uint64_t DM542MotorReversalExecutionMaxTime=0; //calc
//新增下料电机DM541
uint64_t DM541MotorPositiveTurnExecutionMaxTime=0; //calc
uint64_t DM541MotorReversalExecutionMaxTime=0; //calc
 
 
uint64_t ElectroMagnetStartWaitTimer=0; 
uint64_t DM542MotorPositiveTurnExecutionTimer=0; 
uint64_t ElectroMagnetReleaseWaitTimer=0; 
uint64_t DM542MotorReversalExecutionTimer=0;
//新增下料电机DM541
uint64_t DM541MotorPositiveTurnExecutionTimer=0; 
uint64_t DM541MotorReversalExecutionTimer=0;

FlagStatus ElectroMagnetReleaseFlag = RESET; //过渡机构启动标志
sRoller *RollerListHead=NULL;
sRoller *RollerListTail=NULL;

uint64_t  CylindersDetectedRollerNumber=0;   //柱面已检测滚子数目
uint64_t  LeftEndFaceDetectedRollerNumber=0;  //左端面已检测滚子数目
uint64_t  RightEndFaceDetectedRollerNumber=0;  //右端面已检测滚子数目

FlagStatus BeltMotorStartFlag = RESET; //传送带电机启动标志
FlagStatus UnloadMotorStartFlag = RESET; //下料带电机启动标志
FlagStatus DM541MotorReversalFlag = RESET; //下料带电机启动标志
double MA860HMotorStepAngle=1.8;//Degree
double WheelIndexingCircleDiameter=95.49;//mm WheelIndexingCircleDiameter/(MA860HDriverSubdivision*2)
double BeltMotionStep=0.2387;//Unit:mm 1.8*95.49/(2*360)
uint32_t MA860HDriverSubdivision=25600;
uint8_t WorkingSlotLength=50;//Unit:mm
uint8_t TransitionPointToRightEndFaceWorkingSlotNumber=5; //个
uint8_t RightToLeftEndFaceWorkingSlotNumber=5; //个
uint8_t WorkingSlotsMovedNumber=0; //已运动工位槽数目

uint64_t TotalMoveAWorkingSlotMaxTime=0;  //运动一个工位槽电平转换次数
uint16_t TotalMoveAWorkingSlotTimer=0;
double MoveAWorkingSlotConfigTime=0.8; //Unit:s  运动一个工位槽设置时间
uint64_t MA860HTimeBaseForEachPulseInterval=0;  //每个脉冲间隔时间
uint16_t MA860HTimeBaseForEachPulseIntervalCounter=0;

FlagStatus RightFaceImageAcquisitionFlag = RESET; //右端面图像采集标志
FlagStatus LeftFaceImageAcquisitionFlag = RESET; //左端面图像采集标志
FlagStatus LeftFaceVoiceCoilMotorExecutiveFlag = RESET; //左端面滚子位置调整标志
FlagStatus LeftFaceVoiceCoilMotorStretchOutFlag = RESET; //左端面音圈电机伸出标志
FlagStatus LeftFaceVoiceCoilMotorTakeBackFlag = RESET; //左端面音圈电机收回标志
FlagStatus LeftFaceVoiceCoilMotorReleaseFlag = RESET; //左端面音圈电机释放标志

//新增剔除音圈电机
FlagStatus EliminateVoiceCoilMotorExecutiveFlag = RESET; //剔除滚子位置调整标志
FlagStatus EliminateVoiceCoilMotorStretchOutFlag = RESET; //剔除音圈电机伸出标志
FlagStatus EliminateVoiceCoilMotorTakeBackFlag = RESET; //剔除音圈电机收回标志
FlagStatus EliminateVoiceCoilMotorReleaseFlag = RESET; //剔除音圈电机释放标志

//新增下料电机驱动滑台

//FlagStatus CDM541MotorPositiveTurn=RESET;
//FlagStatus CDM541MotorReversalTurn=RESET;

double VoiceCoilMotorExecutiveTime=0.2; //Unit:s
uint64_t VoiceCoilMotorExecutiveMaxTimes=0;
uint64_t VoiceCoilMotorExecutiveTimecounter=0;
uint64_t EliminateVoiceCoilMotorExecutiveMaxTimes=0;
uint64_t EliminateVoiceCoilMotorExecutiveTimecounter=0;
/* Timer handler declaration */
TIM_HandleTypeDef    ElectroMagnetTIM2Handle;
/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sElectroMagnetTIM2Config;
/* Counter Prescaler value */
uint32_t uhElectroMagnetTIM2PrescalerValue = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
void TimeBasic_Init(void);
/* Private functions ---------------------------------------------------------*/
//static void LED_Init();
static void Usart_Init(void);
//static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void Restart_Key2_EXTI_Init(void); //KEY2 重启键
void PhotoelectricSensor_EXTI_Init(void);//1号传感器 
void TransTriggerPhotoelectricSensor_EXTI_Init(void);//3号传感器 横移触发
void TransPositionPhotoelectricSensor_EXTI_Init(void);//2号传感器 横移位置矫正
void BeltPositionPhotoelectricSensor_EXTI_Init(void);//4号传感器  带位置矫正
void UnloadPositionPhotoelectricSensor_EXTI_Init(void);//5号传感器 下料位置矫正

void Camera_Init(void);
void PK543AWT10Motor_Init(void);
void PK543AWT10Motor_SpeedUp_Config(void);
void PK543AWT10Motor_Shutdown_Config(void);
void LightSourceControl_Init(void);

void ElectroMagnetTIM2CH1PWM_Init(void);
void userElectroMagnetTIM2CH1PWMSetValue(uint16_t value);
void ElectroMagnetStart(void);
void ElectroMagnetRelease(void);
void ElectroMagnet_Init(void);
void DM542Motor_Init(void);
void DM541Motor_Init(void);//新增下料滑台电机
void MA860HMotor_Init(void);
void VoiceCoilMotor_Init(void);
void VoiceCoilMotorStretchOut(void);
void VoiceCoilMotorTakeBack(void);
void VoiceCoilMotorRelease(void);
/////新增剔除音圈电机
void EliminateVoiceCoilMotor_Init(void);
void EliminateVoiceCoilMotorStretchOut(void);
void EliminateVoiceCoilMotorTakeBack(void);
void EliminateVoiceCoilMotorRelease(void);
void AllGPIO_Mode_Init(void);


void CDM541MotorPositive(void);
void CDM541MotorReversal(void);
////////新增光控开关
void TransPositionPhotoelectricSensor_Init(void);
void TransTriggerPhotoelectricSensor_Init(void);

     
void Parameters_Init(void);
void RollerOnLine(void);

/////////TEST/////////
uint8_t code=0;
uint64_t testCounter=0;

uint8_t Elecode=0;
uint64_t testEleCounter=0;
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	/* This project template calls firstly two functions in order to configure MPU feature 
     and to enable the CPU Cache, respectively MPU_Config() and CPU_CACHE_Enable().
     These functions are provided as template implementation that User may integrate 
     in his application, to enhance the performance in case of use of AXI interface 
     with several masters. */ 
  
  /* Configure the MPU attributes as Write Through */
  MPU_Config();
	
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
  
  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
    HAL_Init();

  /* Configure the system clock to 216 MHz */
    SystemClock_Config();
	//systick
	delay_init(216);
	//Parameters
	Parameters_Init();
	
	RollerListHead=creatRollerList(OpticalFiberSensorToTransitionZoneMaxRollerNum);
	RollerListTail=RollerListHead;
	//LCD
	//BSP_LCD_Config();
	//LCD_UrielLog ((char *)"  State: Application Start ...\n");
	////LCD_UrielLog ((char *)"  State: ...\n");
	//LED
    //BSP_LED_Init(LED1);
	
	//LCD_UrielLog ((char *)"  State: Configure USART GPIO ...\n");
	
	//LED_Init(); 需要改下传感器连的引脚 PB0/1  =============================================================
	Usart_Init();
	
	//LCD_UrielLog ((char *)"  State: Boot triggered UASRT interrupt ...\n");
	if(HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)aRxBuffer, CommandLength) != HAL_OK)
  { 
	    Error_Handler();
	}
	
	//LCD_UrielLog ((char *)"  State: Configure PK543AWT10 Stepper Motor GPIO ...\n");
	PK543AWT10Motor_Init();
	
	//LCD_UrielLog ((char *)"  State: Configure User push-button in Interrupt mode =>Motor Deceleration stop ...\n");
	//KEY2 PC13
	Restart_Key2_EXTI_Init();   
	
	//LCD_UrielLog ((char *)"  State: Configure PhotoelectricSensor GPIO in Interrupt mode ...\n");
	PhotoelectricSensor_EXTI_Init();  
    TransTriggerPhotoelectricSensor_EXTI_Init();
	TransPositionPhotoelectricSensor_EXTI_Init();
	BeltPositionPhotoelectricSensor_EXTI_Init();
	//UnloadPositionPhotoelectricSensor_EXTI_Init();
	
	HAL_GPIO_EXTI_Callback(GPIO_PIN_0);
	HAL_GPIO_EXTI_Callback(GPIO_PIN_1);
	HAL_GPIO_EXTI_Callback(GPIO_PIN_2);
	HAL_GPIO_EXTI_Callback(GPIO_PIN_3);
	HAL_GPIO_EXTI_Callback(GPIO_PIN_4);
	
	//LCD_UrielLog ((char *)"  State: Configure Camera Trigger GPIO ...\n");
	Camera_Init();
	
	//LCD_UrielLog ((char *)"  State: Configure Linear Light Source Control GPIO ...\n");
	LightSourceControl_Init();
	
	//LCD_UrielLog ((char *)"  State: Configure Electro Magnet Control Board GPIO ...\n");
    ElectroMagnetTIM2CH1PWM_Init();
    userElectroMagnetTIM2CH1PWMSetValue(ElectroMagnetTIM2PERIOD_VALUE/1.5);
	ElectroMagnet_Init(); 
	
	//LCD_UrielLog ((char *)"  State: Configure DM542 Stepper Motor Controler GPIO ...\n");
    DM542Motor_Init();
	
	//LCD_UrielLog ((char *)"  State: Configure DM541 Stepper Motor Controler GPIO ...\n");
    DM541Motor_Init();
  
	//LCD_UrielLog ((char *)"  State: Configure MA860H Stepper Motor Controler GPIO ...\n");
	MA860HMotor_Init();
	
	//LCD_UrielLog ((char *)"  State: Configure VoiceCoilMotor Controler GPIO ...\n");
	VoiceCoilMotor_Init();
	
	//LCD_UrielLog ((char *)"  State: Configure EliminateVoiceCoilMotor Controler GPIO ...\n");
	EliminateVoiceCoilMotor_Init();
	
//	LCD_UrielLog ((char *)"  GPIO Settings:\n"
//	  "  PB4=>AWO PG6:CS PG7=>ACDOFF PI0=>MotorP PH6=>MotorN ...\n"
//	  "  PI3=>CameraTrigger PI2=>Photoelectric PA8=>LightSourceControl ...\n"
//	  "  PA15=>ElectroMagnetPWM PB15=>ElectroMagnetINA PB14=>ElectroMagnetINB  ...\n"
//	  "  PI1=>DM542Plus PB9=>DM542Dir PB8=>DM542Enable  ...\n"
//	  "  PE4=>DM541Plus PE5=>DM541Dir PE6=>DM541Enable  ...\n"
//	  "  PA0=>MA860HPlus PF10=>MA860HDir PF9=>MA860HEnable  ...\n"
//	  "  PA4=>EliminateVoiceCoilMotorP PA5=>EliminateVoiceCoilMotorN Todo ...\n"
//	  "  PC10=>TransInstOpticalControlSwitch PC11=>UnloadOpticalControlSwitch ...\n"
//	  "  PC12=>BeltOpticalControlSwitch...\n"
//	  "  PF8=>VoiceCoilMotorP PF7=>VoiceCoilMotorN Todo...\n");
	delay_ms(2000);
  AllGPIO_Mode_Init();
	PK543AWT10Motor_SpeedUp_Config();
	//Todo：基本步距角设定 CS
	while (1)
  {
      if(CTCompleteFlag)
		{
				CameraTriggerRise;
				delay_us(1);
				CameraTriggerFall;
				CTCompleteFlag=RESET;
				ImageAcquisitionLineActuallyNumber++;
		}
	 if(PSCompleteFlag)
		{
				//LCD_UrielLog ((char *)"  State: Photoelectric Sensor Detected! A Bearing Roller Coming ...\n");
			  PSCompleteFlag=RESET;
		}
	 if(TxCompleteFlag)
		{
				aTxBuffer[1]=0x01;
				if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer,7)!= HAL_OK) Error_Handler();
				while (UartReady != SET){}
				UartReady = RESET;
				LightSourceControl_ON;
				//LCD_UrielLog ((char *)"  State: MCU >>>>> PC !Liner Camera Trigger Command Send ...\n");
				TxCompleteFlag=RESET;
		}		   
	 if(RxCompleteFlag)
		{
			   //LCD_UrielLog ((char *)"  State: PC >>>>> MCU !Rx Transfer completed ...\n  Bearing Roller Raduis Changed!  \n ");
				 RxCompleteFlag=RESET;
		}
	 if(PCCompleteFlag)
		{
			 sprintf(PCCompleteOutLog,"%s \n  State: PlusCount:%d\n ",
			          (char *)"  State: A frame of image acquisition is completed ....",
									ImageAcquisitionLineActuallyNumber);
				 //LCD_UrielLog (PCCompleteOutLog);  	
				 PCCompleteFlag=RESET;
			     ImageAcquisitionLineActuallyNumber=0;
				 RollerNumberOnLine--;   //检测后减1
				 CylindersDetectedRollerNumber++;
		}
	 //if(TransitionalInstitutionsActivatedFlag)//以前的触发模式
	 if(TransTriggerFlag)
		{
				//LCD_UrielLog ((char *)"  State: Transitional Institutions Activated!  \n ");
				//TransitionalInstitutionsActivatedFlag=RESET;
				//Todo 执行过渡机构程序
				TransTriggerFlag=RESET;
				TransInstStatus=CElectroMagnetStart;
				ElectroMagnetStart();
		}
		 if(BeltPositionAccuracyFlag)
		{
				//TransitionalInstitutionsActivatedFlag=RESET;
				//Todo 执行过渡机构程序
				//BeltPositionAccuracyFlag=RESET;
				TransInstStatus=CElectroMagnetStart;
				ElectroMagnetStart();
		}
	 if(ElectroMagnetReleaseFlag)
		{
				ElectroMagnetReleaseFlag=RESET;
				ElectroMagnetRelease();
		}
	 if(RightFaceImageAcquisitionFlag)
		{
				aTxBuffer[1]=0x02;
				if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer,7)!= HAL_OK) Error_Handler();
				while (UartReady != SET){}
				UartReady = RESET;
				//LCD_UrielLog ((char *)"  State: Right End Face Camera Trigger Command Send ...\n");
				RightFaceImageAcquisitionFlag=RESET;
		    RightEndFaceDetectedRollerNumber++;
		}
	 if(LeftFaceVoiceCoilMotorExecutiveFlag)
		{
				//LCD_UrielLog ((char *)"  State: Left Face Voice Coil Motor Executive ...\n");
				LeftFaceVoiceCoilMotorExecutiveFlag=RESET;
				VoiceCoilMotorStretchOut();
				LeftFaceVoiceCoilMotorStretchOutFlag=SET;
		}
		if(UnloadMotorStartFlag)
		{
				//LCD_UrielLog ((char *)"  State: UnloadMotorStar ...\n");
			     CDM541MotorPositive();
			     delay_ms(50);
			     CDM541MotorReversal();
			     UnloadMotorStartFlag=RESET;
		
		}
	/*	if(LeftFaceVoiceCoilMotorExecutiveFlag)
		{
				//LCD_UrielLog ((char *)"  State: Left Face Voice Coil Motor Executive ...\n");
				LeftFaceVoiceCoilMotorExecutiveFlag=RESET;
				VoiceCoilMotorStretchOut();
				LeftFaceVoiceCoilMotorStretchOutFlag=SET;
		}*/
	 if(LeftFaceImageAcquisitionFlag)
		{
				aTxBuffer[1]=0x03;
				if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer,7)!= HAL_OK) Error_Handler();
				while (UartReady != SET){}
				UartReady = RESET;
			    //LCD_UrielLog ((char *)"  State: Left End Face Camera Trigger Command Send ...\n");
				LeftFaceImageAcquisitionFlag=RESET;
				LeftEndFaceDetectedRollerNumber++;
		}
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
  while (1)
  {
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
 // RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Activate the OverDrive to reach the 216 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; 
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }  
}

void TimeBasic_Init(void)
{
	/*##-1- Configure the TIM peripheral #######################################*/
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK)  is set to APB1 clock (PCLK1) x2,
    since APB1 prescaler is equal to 4.
      TIM3CLK = PCLK1*2
      PCLK1 = HCLK/4
      => TIM3CLK = HCLK/2 = SystemCoreClock/2
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as follows:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = ((SystemCoreClock/2) /10 KHz) - 1

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f7xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */
   
  /* Compute the prescaler value to have TIMx counter clock equal to 10MHz */
  uwBasePrescalerValue = (uint32_t)((SystemCoreClock / 2) / TimBaseFrequency) - 1;

  /* Set TIMx instance */
  TimBaseHandle.Instance = TIMx;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000000 - 1
       + Prescaler = ((SystemCoreClock / 2)/10000000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimBaseHandle.Init.Period            = (uint32_t)(TimBasePeriodBaseNumber*SpeedUpFactor);// max150; 500*SpeedUpFactor=>20KHz  
  TimBaseHandle.Init.Prescaler         = uwBasePrescalerValue;
  TimBaseHandle.Init.ClockDivision     = 0;
  TimBaseHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimBaseHandle.Init.RepetitionCounter = 0;

  if (HAL_TIM_Base_Init(&TimBaseHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimBaseHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None 
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIMx)
	{
		MotorPToggle;
		if(HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_0)==GPIO_PIN_SET)
		{
			  if((LinearCameraDelayStartFlag == SET) && 
					(++LinearCameraDelayStartCounter >= 10000/*LinearCameraStartupDelayTime*/))  //延时启动相机
				{
						LinearCameraDelayStartCounter = 0;
						LinearCameraTriggerFlag = SET;
						LinearCameraDelayStartFlag = RESET;
						TxCompleteFlag=SET;
				}
              if((RollerNumberOnLine>0)&&
					(LinearCameraTriggerFlag==SET)&&
				  (ImageAcquisitionLineNumber<=TotalAcquisitionNumber))
				{
					 if(++MotorStepCounterInAcquisition >= PulseNumberPerAcquisition)    //单次触发相机间隔判断
					 {
						  MotorStepCounterInAcquisition=0; //复位，等待下次采集计数
						  CTCompleteFlag=SET; //线阵相机采图
						  if(++AlreadyAcquisitionCounter >= TotalAcquisitionNumber)//滚子旋转一周判断
							{
								 LinearCameraTriggerFlag=RESET;
								 AlreadyAcquisitionCounter=0;
								 PCCompleteFlag=SET;
								 LightSourceControl_OFF;
							}
					 }
				}
              if(RollerListHead!=RollerListTail)
				{
					 sRoller * index=RollerListHead;
					 while(index!=RollerListTail)
					 {
						 index->MoveStepNum+=1;
						 index=index->link; 
					 }
					 if(RollerListHead->MoveStepNum>=100000/*OpticalFiberSensorToTransitionZoneDistance*/)
					 {
						 TransitionalInstitutionsActivatedFlag=SET;
						 RollerListHead->MoveStepNum=0;
						 RollerListHead=RollerListHead->link;
					 }
				}	
		}
		if(TransInstStatus!=CNotStarted)
		{
			 switch(TransInstStatus)
			 {
				 case CElectroMagnetStart:
				 {
						ElectroMagnetStartWaitTimer++;
						if(ElectroMagnetStartWaitTimer>=2000/*ElectroMagnetStartWaitMaxTime*/)
						{
							ElectroMagnetStartWaitTimer=0;
							TransInstStatus=CDM542MotorPositiveTurn;
						}
				 }
				 break;
				 case CDM542MotorPositiveTurn:
				 {
					   DM542Dir_ON;
					   DM542PlusToggle;
					   DM542MotorPositiveTurnExecutionTimer++;
					   if(DM542MotorPositiveTurnExecutionTimer>=8500/*DM542MotorPositiveTurnExecutionMaxTime*/)//以前是4500，后改为8500
						 {
							 DM542MotorPositiveTurnExecutionTimer=0;
							 TransInstStatus=CElectroMagnetRelease;
						 }
				 }
				 break;
				 case CElectroMagnetRelease:
				 {
						ElectroMagnetReleaseWaitTimer++;
						if(ElectroMagnetReleaseWaitTimer>=2000/*ElectroMagnetReleaseWaitMaxTime*/)
						{
							ElectroMagnetReleaseFlag=SET;
							ElectroMagnetReleaseWaitTimer=0; 
							TransInstStatus=CDM542MotorReversal;
						}
				 }
				 break;
				 case CDM542MotorReversal:
				 {
					   DM542Dir_OFF;
					   DM542PlusToggle;  
					   DM542MotorReversalExecutionTimer++;
					// if(DM542MotorReversalExecutionTimer>=8500/*DM542MotorReversalExecutionMaxTime*/)//以前是4500，后改为8500
					if(TransPositionAccuracyFlag)	
					 {
							//DM542MotorReversalExecutionTimer=0;
							 TransPositionAccuracyFlag=RESET;
							 TransInstStatus=CNotStarted;
							 BeltMotorStartFlag=SET;
					 }
				 }
				 break;
				 default:
				 break;
			 }
		 }
     if(BeltMotorStartFlag==SET)	
		 {

			  if(++MA860HTimeBaseForEachPulseIntervalCounter>=1 /*MA860HTimeBaseForEachPulseInterval*/)
				{
					 MA860HTimeBaseForEachPulseIntervalCounter=0;
					 MA860HPlusToggle;
					if(BeltPositionAccuracyFlag)
					{
						//BeltPositionAccuracyFlag=SET;
						
					    if(++TotalMoveAWorkingSlotTimer>=4100)	//跑完一个完整工位是8520/之前4100  TotalMoveAWorkingSlotMaxTime
					  {
						TotalMoveAWorkingSlotTimer=0;
						BeltMotorStartFlag=RESET;
						BeltPositionAccuracyFlag=RESET;
						RightFaceImageAcquisitionFlag=SET;
						LeftFaceVoiceCoilMotorExecutiveFlag=SET;
					   }
						//UnloadMotorStartFlag = SET;
//						if(++WorkingSlotsMovedNumber>=TransitionPointToRightEndFaceWorkingSlotNumber)
//						{
//							if(TotalMoveAWorkingSlotTimer<TransitionPointToRightEndFaceWorkingSlotNumber+RightToLeftEndFaceWorkingSlotNumber)
//							   RightFaceImageAcquisitionFlag=SET;
//							else
//							{
//								 RightFaceImageAcquisitionFlag=SET;
//								 LeftFaceVoiceCoilMotorExecutiveFlag=SET;
//							}
//						}
					}
					
				}
		 }
		 if(LeftFaceVoiceCoilMotorStretchOutFlag==SET)	
		 {
				VoiceCoilMotorExecutiveTimecounter=0;
				LeftFaceVoiceCoilMotorStretchOutFlag=RESET;
				LeftFaceVoiceCoilMotorTakeBackFlag=SET;
				LeftFaceImageAcquisitionFlag=SET;
			  /*  if (EliminateVoiceCoilMotorStretchOutFlag==SET)          //剔除音圈电机弹出
			     {
			        EliminateVoiceCoilMotorExecutiveTimecounter=0;
					EliminateVoiceCoilMotorStretchOutFlag=RESET;
					EliminateVoiceCoilMotorTakeBackFlag=SET;
			      }  */
			 
//			  if(++VoiceCoilMotorExecutiveTimecounter>=5520/*VoiceCoilMotorExecutiveMaxTimes*/)
//				{
//					VoiceCoilMotorExecutiveTimecounter=0;
//					LeftFaceVoiceCoilMotorStretchOutFlag=RESET;
//					LeftFaceVoiceCoilMotorTakeBackFlag=SET;
//					LeftFaceImageAcquisitionFlag=SET;
//				}
		 }
		 if(LeftFaceVoiceCoilMotorTakeBackFlag==SET)	 
		 {
			  if(++VoiceCoilMotorExecutiveTimecounter>=5000/*VoiceCoilMotorExecutiveMaxTimes*/)
				{
					VoiceCoilMotorExecutiveTimecounter=0;
					VoiceCoilMotorTakeBack();
					LeftFaceVoiceCoilMotorTakeBackFlag=RESET;
					LeftFaceVoiceCoilMotorReleaseFlag=SET;
					/*if(EliminateVoiceCoilMotorTakeBackFlag==SET)  //剔除音圈电机收回
					{
						EliminateVoiceCoilMotorExecutiveTimecounter=0;
					    EliminateVoiceCoilMotorTakeBack();
					    EliminateVoiceCoilMotorTakeBackFlag=RESET;
					    EliminateVoiceCoilMotorReleaseFlag=SET;
					}*/
				}
		 }
		 
		 if(LeftFaceVoiceCoilMotorReleaseFlag==SET)
		 {
			 if(++VoiceCoilMotorExecutiveTimecounter>=9020/*VoiceCoilMotorExecutiveMaxTimes*/)
				{
					VoiceCoilMotorExecutiveTimecounter=0;
					VoiceCoilMotorRelease();
					LeftFaceVoiceCoilMotorReleaseFlag=RESET;
				/*	if(EliminateVoiceCoilMotorReleaseFlag==SET)  //剔除音圈电机释放
					{
						EliminateVoiceCoilMotorExecutiveTimecounter=0;
					    EliminateVoiceCoilMotorRelease();
					    EliminateVoiceCoilMotorReleaseFlag=RESET;
					}*/
				}
		 }
	}
}

void Parameters_Init(void)
	{
    PulseNumberPerAcquisition = (uint64_t)(0.5223881*Resolution); //相机触发pwm波间隔数计算  360*3.5./(0.072*33.5)*0.001
	TotalAcquisitionNumber = (uint64_t)(6283.1854*RollerRadius/Resolution  );    //相机拍摄滚子一周PWM波总数计算  2*pi*r/e
	ElectroMagnetStartWaitMaxTime=TimBaseFrequency/TimBasePeriodBaseNumber*0.1; //100ms
    DM542MotorPositiveTurnExecutionMaxTime=2*(TransitionalInstitutionMovingDistance/SlidingPlatformVelocity)*(TimBaseFrequency/TimBasePeriodBaseNumber);
    ElectroMagnetReleaseWaitMaxTime=TimBaseFrequency/TimBasePeriodBaseNumber*0.1; //100ms
    DM542MotorReversalExecutionMaxTime=2*(TransitionalInstitutionMovingDistance/SlidingPlatformVelocity)*(TimBaseFrequency/TimBasePeriodBaseNumber);
	TotalMoveAWorkingSlotMaxTime=2*2*MA860HDriverSubdivision*WorkingSlotLength/WheelIndexingCircleDiameter;
	MA860HTimeBaseForEachPulseInterval=MoveAWorkingSlotConfigTime*TimBaseFrequency/TotalMoveAWorkingSlotMaxTime;
	VoiceCoilMotorExecutiveMaxTimes=TimBaseFrequency*VoiceCoilMotorExecutiveTime;
    }

//static void LED_Init() {
//	 GPIO_InitTypeDef GPIO_Initure;
//    __HAL_RCC_GPIOB_CLK_ENABLE();			//开启GPIOB时钟
//	
//    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1; //PB0,1
//    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
//    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
//    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
//    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
//	
//    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);	//PB1置0
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);	//PB1置1 
//}

static void Usart_Init(void)
{
	/*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;
  UartHandle.Init.BaudRate   = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *m_UartHandle)
{
	 /* Set transmission flag: trasfer complete*/
  UartReady = SET;
	if(m_UartHandle->pRxBuffPtr[0]!=0xEE||m_UartHandle->pRxBuffPtr[3]!=0xFF ||m_UartHandle->pRxBuffPtr[4]!=0xFC ||m_UartHandle->pRxBuffPtr[5]!=0xFF ||m_UartHandle->pRxBuffPtr[7]!=0xFF)
	  return;
	else
	{
		//add something to change some parameters
		RollerRadius=aRxBuffer[2];
		Parameters_Init();
	}
	if(HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)aRxBuffer, CommandLength) != HAL_OK){ Error_Handler();}   //引导触发下一次UART接收中断

	RxCompleteFlag=SET;
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
	  //LCD_ErrLog ((char *)"  UART transfer error ...\n");
	  
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
//static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
//{
//  while (BufferLength--)
//  {
//    if ((*pBuffer1) != *pBuffer2)
//    {
//      return BufferLength;
//    }
//    pBuffer1++;
//    pBuffer2++;
//  }
//  return 0;
//}

 void Restart_Key2_EXTI_Init(){
	 GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
	 
	  GPIO_InitStructure.Pin=GPIO_PIN_13;               //PC13
    GPIO_InitStructure.Mode=GPIO_MODE_IT_FALLING;     //下降沿触发
    GPIO_InitStructure.Pull=GPIO_PULLUP;				      //上拉
    HAL_GPIO_Init(GPIOC,&GPIO_InitStructure);
	 

	 /* Enable and set EXTI line 13 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn,2,3);   //抢占优先级为3，子优先级为3
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);         //使能中断线13  
}

 void PhotoelectricSensor_EXTI_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure PB.2 pin as input floating */
	GPIO_InitStructure.Pin = GPIO_PIN_2;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable and set EXTI line 2 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI2_IRQn, 0x02, 0x01);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

void BeltPositionPhotoelectricSensor_EXTI_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
  /* Enable GPIOC clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  /* Configure PC.1 pin as input floating */
	GPIO_InitStructure.Pin = GPIO_PIN_1;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable and set EXTI line 2 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0x02, 0x01);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}
void TransPositionPhotoelectricSensor_EXTI_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
 //  Enable GPIOB clock 
   __HAL_RCC_GPIOB_CLK_ENABLE();

 //  Configure PB.0 pin as input floating 
	GPIO_InitStructure.Pin = GPIO_PIN_0;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

 //  Enable and set EXTI line 2 Interrupt to the lowest priority 
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0x02, 0x01);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

 void TransTriggerPhotoelectricSensor_EXTI_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;

  // Enable GPIOI clock 
  __HAL_RCC_GPIOE_CLK_ENABLE();

  //Configure PC.12 pin as input floating 
	GPIO_InitStructure.Pin = GPIO_PIN_4;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;//原来的
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

  // Enable and set EXTI line 12 Interrupt to the lowest priority 
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0x02, 0x01);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
} 

void UnloadPositionPhotoelectricSensor_EXTI_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOI clock */
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* Configure PE.3 pin as input floating */
	GPIO_InitStructure.Pin = GPIO_PIN_3;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* Enable and set EXTI line 3 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI3_IRQn, 0x02, 0x01);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void RollerOnLine(void)
{
	 if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==GPIO_PIN_SET )  
		{
			RollerNumberOnLine++;
			PSCompleteFlag=SET;
		}
		else
		{				
			LinearCameraDelayStartFlag = SET;
		}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
  {
		case GPIO_PIN_2://PhotoelectricSensor_PIN 第一个触发光电  1号光电
		{
			delay_us(20);
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 1)
			{
				//RollerOnLine();
				RollerNumberOnLine++;
				LinearCameraDelayStartFlag = SET;
				PSCompleteFlag=SET;
				RollerListTail=RollerListTail->link;
			}
		}
		break;
		case GPIO_PIN_0://TransPositionPhotoelectricSensor_PIN  2号光电 横移位置矫正
		{
			delay_us(20);
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 1)
			{
					TransPositionAccuracyFlag=SET;
			}
		}
		break;
		case GPIO_PIN_4: //TransTriggerPhotoelectricSensor_PIN  3号光电 横移触发
		{
			delay_us(20);
			if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == 1)
			{
					TransTriggerFlag=SET;
			}
		}
		break;
		case GPIO_PIN_1://BeltPositionPhotoelectricSensor_PIN  4号光电 传送带位置矫正
		{
			delay_us(20);
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 1)
			{
				BeltPositionAccuracyFlag=SET;
			}
		}
		break;
		case GPIO_PIN_3://UnloadPositionPhotoelectricSensor_PIN  5号光电 下料位置矫正
		{
			delay_us(20);
			if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == 1)
			{
				 UnloadPositionAccuracyFlag=SET;
			}
		}
		break;
		case GPIO_PIN_13: //KEY2 重启
		{
			delay_us(50);
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==0) {
					PK543AWT10Motor_Shutdown_Config();
			}
			//LCD_UrielLog ((char *)"  State: Application Over ...\n");
		}
		break;
	}
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
}

// PK543AWT10 Stepper Motor IO Config
void PK543AWT10Motor_Init(void)
	{
		GPIO_InitTypeDef  GPIO_InitStruct;
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOF_CLK_ENABLE();
	 // __HAL_RCC_GPIOI_CLK_ENABLE();
	 // __HAL_RCC_GPIOH_CLK_ENABLE();
	
	  GPIO_InitStruct.Pin = GPIO_PIN_4;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP ;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	  GPIO_InitStruct.Pin = GPIO_PIN_6;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	  GPIO_InitStruct.Pin = GPIO_PIN_5;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	  
	  GPIO_InitStruct.Pin = GPIO_PIN_3;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	  
  	  GPIO_InitStruct.Pin = GPIO_PIN_6;
	  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    }

void PK543AWT10Motor_SpeedUp_Config(void)
  {
	SpeedUpFactor = SpeedUpFactor_final;
	for(uint8_t tmp = 1; tmp<=SpeedUpFactor_final; tmp++)
	 {
		SpeedUpFactor = SpeedUpFactor_final/tmp;
		delay_ms(25);
		if (HAL_TIM_Base_Stop_IT(&TimBaseHandle) != HAL_OK){/* Starting Error */Error_Handler();}
		if (HAL_TIM_Base_DeInit(&TimBaseHandle) != HAL_OK){/* Initialization Error */Error_Handler();}
		TimeBasic_Init();
	 }
   }

void PK543AWT10Motor_Shutdown_Config(void)
{
	for(uint8_t tmp_down = SpeedUpFactor_final; tmp_down > 1; tmp_down--)
			{
			  SpeedUpFactor= SpeedUpFactor_final/tmp_down;
			  delay_ms(25);
			  if (HAL_TIM_Base_Stop_IT(&TimBaseHandle) != HAL_OK){/* Starting Error */ Error_Handler();}
			  if (HAL_TIM_Base_DeInit(&TimBaseHandle) != HAL_OK){/* Initialization Error */Error_Handler();}
			  TimeBasic_Init();
			 }
}

void Camera_Init(void)
  {
	GPIO_InitTypeDef  GPIO_InitStruct;
	__HAL_RCC_GPIOG_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP ;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  }

void LightSourceControl_Init(void)
  {
	GPIO_InitTypeDef  GPIO_InitStruct;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP ;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
   }

void ElectroMagnetTIM2CH1PWM_Init(void){
	  /* Compute the prescaler value to have TIM2 counter clock equal to 21600000 Hz */
  uhElectroMagnetTIM2PrescalerValue = (uint32_t)((SystemCoreClock/2) / 18000000) - 1;


  /*##-1- Configure the TIM peripheral #######################################*/
  /* -----------------------------------------------------------------------
  TIM2 Configuration: generate 4 PWM signals with 4 different duty cycles.

    In this example TIM2 input clock (TIM2CLK) is set to APB1 clock (PCLK1) x2,
    since APB1 prescaler is equal to 4.
      TIM2CLK = PCLK1*2
      PCLK1 = HCLK/2
      => TIM2CLK = HCLK/2 = SystemCoreClock/2

    To get TIM2 counter clock at 21.6 MHz, the prescaler is computed as follows:
       Prescaler = (TIM2CLK / TIM2 counter clock) - 1
       Prescaler = ((SystemCoreClock/2) /21.6 MHz) - 1

    To get TIM2 output clock at 24 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM2 counter clock / TIM2 output clock) - 1
           = 899

    TIM2 Channel1 duty cycle = (TIM2_CCR1/ TIM2_ARR + 1)* 100 = 50%
    TIM2 Channel2 duty cycle = (TIM2_CCR2/ TIM2_ARR + 1)* 100 = 37.5%
    TIM2 Channel3 duty cycle = (TIM2_CCR3/ TIM2_ARR + 1)* 100 = 25%
    TIM2 Channel4 duty cycle = (TIM2_CCR4/ TIM2_ARR + 1)* 100 = 12.5%

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f7xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */

  /* Initialize TIMx peripheral as follows:
       + Prescaler = ((SystemCoreClock/2) / 21600000) - 1
       + Period = (900 - 1)
       + ClockDivision = 0
       + Counter direction = Up
  */
  ElectroMagnetTIM2Handle.Instance = ElectroMagnetTIM2CH1PWM;
  ElectroMagnetTIM2Handle.Init.Prescaler         = uhElectroMagnetTIM2PrescalerValue;
  ElectroMagnetTIM2Handle.Init.Period            = ElectroMagnetTIM2PERIOD_VALUE;
  ElectroMagnetTIM2Handle.Init.ClockDivision     = 0;
  ElectroMagnetTIM2Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  ElectroMagnetTIM2Handle.Init.RepetitionCounter = 0;
  ElectroMagnetTIM2Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&ElectroMagnetTIM2Handle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the PWM channels #########################################*/
  /* Common configuration for all channels */
  sElectroMagnetTIM2Config.OCMode       = TIM_OCMODE_PWM1;
  sElectroMagnetTIM2Config.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sElectroMagnetTIM2Config.OCFastMode   = TIM_OCFAST_DISABLE;
  sElectroMagnetTIM2Config.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sElectroMagnetTIM2Config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sElectroMagnetTIM2Config.OCIdleState  = TIM_OCIDLESTATE_RESET;

  /* Set the pulse value for channel 1 */
  sElectroMagnetTIM2Config.Pulse = ElectroMagnetTIM2PULSE1_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&ElectroMagnetTIM2Handle, &sElectroMagnetTIM2Config, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /*##-3- Start PWM signals generation #######################################*/
  /* Start channel 1 */
  if (HAL_TIM_PWM_Start(&ElectroMagnetTIM2Handle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
}

void userElectroMagnetTIM2CH1PWMSetValue(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&ElectroMagnetTIM2Handle, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&ElectroMagnetTIM2Handle, TIM_CHANNEL_1);  
}

// Electro Magnet Control Board IO Config
void ElectroMagnet_Init(void)
{
  GPIO_InitTypeDef  gpio_init_structure;
  GPIO_TypeDef*     gpio_ElectroMagnet;
  gpio_ElectroMagnet = GPIOB;
  /* Enable the GPIO_LED clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure the GPIO_ pin */
  gpio_init_structure.Pin = GPIO_PIN_15;//INA
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(gpio_ElectroMagnet, &gpio_init_structure);
  //HAL_GPIO_WritePin(gpio_ElectroMagnet, GPIO_PIN_15, GPIO_PIN_SET);
	
	gpio_init_structure.Pin = GPIO_PIN_14;//INB
	HAL_GPIO_Init(gpio_ElectroMagnet, &gpio_init_structure);
	//HAL_GPIO_WritePin(gpio_ElectroMagnet, GPIO_PIN_14, GPIO_PIN_SET);
}
// DM542 Stepper Motor IO Config
void DM542Motor_Init(void)
{
	GPIO_InitTypeDef  gpio_init_structure;
  GPIO_TypeDef*     gpio_DM542Motor;
  gpio_DM542Motor = GPIOB;
  /* Enable the GPIO_DM542Motor clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
   gpio_init_structure.Pin = GPIO_PIN_9;//DM542Dir
   gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  //gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Pull = GPIO_NOPULL;
	gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  
    HAL_GPIO_Init(gpio_DM542Motor, &gpio_init_structure);
  //HAL_GPIO_WritePin(gpio_DM542Motor, GPIO_PIN_9, GPIO_PIN_SET);
	
	gpio_init_structure.Pin = GPIO_PIN_8;//DM542Enable
	HAL_GPIO_Init(gpio_DM542Motor, &gpio_init_structure);
	//HAL_GPIO_WritePin(gpio_DM542Motor, GPIO_PIN_8, GPIO_PIN_SET);
	
	GPIO_TypeDef*     gpio_DM542MotorPlus;
    gpio_DM542MotorPlus = GPIOB;
  /* Enable the gpio_DM542MotorPlus clock */
    __HAL_RCC_GPIOI_CLK_ENABLE();
	
	gpio_init_structure.Pin = GPIO_PIN_7;//DM542Plus
	HAL_GPIO_Init(gpio_DM542MotorPlus, &gpio_init_structure);
	//HAL_GPIO_WritePin(gpio_DM542MotorPlus, GPIO_PIN_7, GPIO_PIN_RESET);
}

void DM541Motor_Init(void)
{
  GPIO_InitTypeDef  gpio_init_structure;
  GPIO_TypeDef*     gpio_DM541Motor;
  gpio_DM541Motor = GPIOD;
  /* Enable the GPIO_DM541Motor clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
   gpio_init_structure.Pin = GPIO_PIN_7;//DM541Dir
   gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  //gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Pull = GPIO_NOPULL;
	gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  
    HAL_GPIO_Init(gpio_DM541Motor, &gpio_init_structure);
	
	gpio_init_structure.Pin = GPIO_PIN_8;//DM542Enable
	HAL_GPIO_Init(gpio_DM541Motor, &gpio_init_structure);
	//HAL_GPIO_WritePin(gpio_DM541Motor, GPIO_PIN_8, GPIO_PIN_SET);
	
	GPIO_TypeDef*     gpio_DM541MotorPlus;
    gpio_DM541MotorPlus = GPIOC;
  /* Enable the gpio_DM542MotorPlus clock */
    __HAL_RCC_GPIOC_CLK_ENABLE();
	
	gpio_init_structure.Pin = GPIO_PIN_9;//DM541Plus
	HAL_GPIO_Init(gpio_DM541MotorPlus, &gpio_init_structure);
	//HAL_GPIO_WritePin(gpio_DM541MotorPlus, GPIO_PIN_9, GPIO_PIN_RESET);
}

//				void DM541Motor_Init(void)
//				{
//				  GPIO_InitTypeDef  gpio_init_structure;
//				  GPIO_TypeDef*     gpio_DM541Motor;
//				  gpio_DM541Motor = GPIOD;
//				  /* Enable the GPIO_DM541Motor clock */
//				  __HAL_RCC_GPIOE_CLK_ENABLE();
//				  /* Configure the GPIO_LED pin */
//				  gpio_init_structure.Pin = GPIO_PIN_6;//DM541Dir
//				  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
//				  gpio_init_structure.Pull = GPIO_NOPULL;
//				  gpio_init_structure.Speed = GPIO_SPEED_HIGH;			  
//				  HAL_GPIO_Init(gpio_DM541Motor, &gpio_init_structure);
//				  //HAL_GPIO_WritePin(gpio_DM541Motor, GPIO_PIN_5, GPIO_PIN_SET);
//					// GPIO_InitTypeDef  gpio_init_structure;
//				 // GPIO_TypeDef*     gpio_DM541Motor;
//				   gpio_DM541Motor = GPIOE;
//				  /* Enable the GPIO_DM541Motor clock */
//				  __HAL_RCC_GPIOE_CLK_ENABLE();
//					gpio_init_structure.Pin = GPIO_PIN_6;//DM541Enable
//					HAL_GPIO_Init(gpio_DM541Motor, &gpio_init_structure);
//					//HAL_GPIO_WritePin(gpio_DM541Motor, GPIO_PIN_6, GPIO_PIN_SET);					
//					GPIO_TypeDef*     gpio_DM541MotorPlus;
//				  gpio_DM541MotorPlus = GPIOE;
//				  /* Enable the gpio_DM541MotorPlus clock */
//				  __HAL_RCC_GPIOE_CLK_ENABLE();				
//					gpio_init_structure.Pin = GPIO_PIN_4;//DM541Plus
//					HAL_GPIO_Init(gpio_DM541MotorPlus, &gpio_init_structure);
//					//HAL_GPIO_WritePin(gpio_DM541MotorPlus, GPIO_PIN_4, GPIO_PIN_RESET);
//				} 
void MA860HMotor_Init(void)
{
	GPIO_InitTypeDef  gpio_init_structure;
  GPIO_TypeDef*     gpio_MA860HMotor;
  gpio_MA860HMotor = GPIOA;
  /* Enable the GPIO_MA860HMotor clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure the GPIO_MA860HDir pin */
  gpio_init_structure.Pin = GPIO_PIN_6;//
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  
  HAL_GPIO_Init(gpio_MA860HMotor, &gpio_init_structure);
  HAL_GPIO_WritePin(gpio_MA860HMotor, GPIO_PIN_6, GPIO_PIN_SET);
	
	/* Configure the GPIO_MA860HEnable pin */
	//GPIO_TypeDef*     gpio_MA860HMotorEnable;
	gpio_MA860HMotor = GPIOF;
	  __HAL_RCC_GPIOF_CLK_ENABLE();
	gpio_init_structure.Pin = GPIO_PIN_9;
	HAL_GPIO_Init(gpio_MA860HMotor, &gpio_init_structure);
	HAL_GPIO_WritePin(gpio_MA860HMotor, GPIO_PIN_9, GPIO_PIN_SET);
	
	GPIO_TypeDef*     gpio_MA860HMotorPlus;
    gpio_MA860HMotorPlus = GPIOH;
  /* Enable the gpio_MA860HMotorPlus clock */
    __HAL_RCC_GPIOH_CLK_ENABLE();
	
	gpio_init_structure.Pin = GPIO_PIN_3;//DM542Plus
	HAL_GPIO_Init(gpio_MA860HMotorPlus, &gpio_init_structure);
	HAL_GPIO_WritePin(gpio_MA860HMotorPlus, GPIO_PIN_0, GPIO_PIN_RESET);
}
//void MA860HMotor_Init(void)
//{
//	GPIO_InitTypeDef  gpio_init_structure;
//  GPIO_TypeDef*     gpio_MA860HMotor;
//  gpio_MA860HMotor = GPIOA;
//  /* Enable the GPIO_MA860HMotor clock */
//  __HAL_RCC_GPIOF_CLK_ENABLE();

//  /* Configure the GPIO_MA860HDir pin */
//  gpio_init_structure.Pin = GPIO_PIN_6;//
//  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
//  gpio_init_structure.Pull = GPIO_PULLUP;
//	gpio_init_structure.Speed = GPIO_SPEED_HIGH;
//  
//  HAL_GPIO_Init(gpio_MA860HMotor, &gpio_init_structure);
//  HAL_GPIO_WritePin(gpio_MA860HMotor, GPIO_PIN_6, GPIO_PIN_SET);
//	
//	/* Configure the GPIO_MA860HEnable pin */
//	gpio_MA860HMotor = GPIOF;
//	gpio_init_structure.Pin = GPIO_PIN_9;
//	HAL_GPIO_Init(gpio_MA860HMotor, &gpio_init_structure);
//	HAL_GPIO_WritePin(gpio_MA860HMotor, GPIO_PIN_9, GPIO_PIN_SET);
//	
//	GPIO_TypeDef*     gpio_MA860HMotorPlus;
//    gpio_MA860HMotorPlus = GPIOA;
//  /* Enable the gpio_MA860HMotorPlus clock */
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//	
//	gpio_init_structure.Pin = GPIO_PIN_0;//DM542Plus
//	HAL_GPIO_Init(gpio_MA860HMotorPlus, &gpio_init_structure);
//	HAL_GPIO_WritePin(gpio_MA860HMotorPlus, GPIO_PIN_0, GPIO_PIN_RESET);
//}
void VoiceCoilMotor_Init(void)
{
  GPIO_InitTypeDef  gpio_init_structure;
  GPIO_TypeDef*     gpio_VoiceCoilMotor;
  gpio_VoiceCoilMotor = GPIOF;
  /* Enable the GPIO_VoiceCoilMotor clock */
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* Configure the GPIO_VoiceCoilMotorP pin */
  gpio_init_structure.Pin = GPIO_PIN_8;//
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  
  HAL_GPIO_Init(gpio_VoiceCoilMotor, &gpio_init_structure);
  HAL_GPIO_WritePin(gpio_VoiceCoilMotor, GPIO_PIN_8, GPIO_PIN_SET);
	
	/* Configure the GPIO_VoiceCoilMotorN pin */
	gpio_init_structure.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(gpio_VoiceCoilMotor, &gpio_init_structure);
	HAL_GPIO_WritePin(gpio_VoiceCoilMotor, GPIO_PIN_7, GPIO_PIN_RESET);
}

void EliminateVoiceCoilMotor_Init(void)
{
	GPIO_InitTypeDef  gpio_init_structure;
  GPIO_TypeDef*     gpio_EliminateVoiceCoilMotor;
  gpio_EliminateVoiceCoilMotor = GPIOH;
  /* Enable the GPIO_EliminateVoiceCoilMotor clock */
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Configure the GPIO_EliminateVoiceCoilMotorP pin */
  gpio_init_structure.Pin = GPIO_PIN_5;//
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  
  HAL_GPIO_Init(gpio_EliminateVoiceCoilMotor, &gpio_init_structure);
  HAL_GPIO_WritePin(gpio_EliminateVoiceCoilMotor, GPIO_PIN_5, GPIO_PIN_SET);
	
	/* Configure the GPIO_EliminateVoiceCoilMotorN pin */
	gpio_init_structure.Pin = GPIO_PIN_4;
	HAL_GPIO_Init(gpio_EliminateVoiceCoilMotor, &gpio_init_structure);
	HAL_GPIO_WritePin(gpio_EliminateVoiceCoilMotor, GPIO_PIN_4, GPIO_PIN_RESET);
}

void AllGPIO_Mode_Init(void)
	{
	AWO_ON; 
    CS_ON;
    ACDOFF_ON;
	MotorP_LOW;
	MotorN_LOW;
	
	CameraTriggerFall;
	LightSourceControl_OFF;
	
	ElectroMagnetINA_ON;
	ElectroMagnetINB_ON;
	
	DM542Plus_OFF;	
	DM542Dir_ON;
	DM542Enable_ON;

	DM541Plus_OFF;
	DM541Dir_ON;
	DM541Enable_ON;
	
	MA860HPlus_OFF;
	MA860HDir_OFF;  
	MA860HEnable_OFF;

	
//	VoiceCoilMotorStretchOut();
//	delay_ms(200);
	VoiceCoilMotorTakeBack();
	delay_ms(200);
	VoiceCoilMotorRelease();

    EliminateVoiceCoilMotorTakeBack();
	delay_ms(200);
	EliminateVoiceCoilMotorRelease();
}

sRoller * creatRollerList(uint8_t n) 
{  
    sRoller *p,*h,*s;       
    int i;  
    if((h=(sRoller *)malloc(sizeof(sRoller)))==NULL)
    {  
        Error_Handler();
	      //LCD_ErrLog ((char *)"  sRoller Malloc Error ...\n"); 
    }  
    h->MoveStepNum=0; 
    h->link=NULL;  
    p=h;     
    for(i=0;i<n;i++)  
    {  
        if((s= (sRoller *) malloc(sizeof(sRoller)))==NULL)
        {  
            Error_Handler();
	          //LCD_ErrLog ((char *)"  sRoller Malloc Error ...\n"); 
        }  
        p->link=s; 
				s->MoveStepNum=0;
        s->link=NULL;  
        p=s;  
    }  
    p->link=h;   
    return(h);  
} 

void ElectroMagnetStart(void)
{
	ElectroMagnetINA_OFF;
	ElectroMagnetINB_ON;
}
void ElectroMagnetRelease(void)
{
	ElectroMagnetINA_ON;
	ElectroMagnetINB_ON;
}

void CDM541MotorPositive(void)
{
	do{
		    DM541Dir_ON;
			delay_ms(1);
	        DM541PlusToggle;
		    DM541MotorPositiveTurnExecutionTimer++;
	   }
	   while(DM541MotorPositiveTurnExecutionTimer<=1600/*DM541MotorPositiveTurnExecutionMaxTime*/);
	   
	     DM541MotorPositiveTurnExecutionTimer=0;	
         
}

void CDM541MotorReversal(void)
{
	UnloadPositionAccuracyFlag=RESET; 
    do{
		  DM541Dir_OFF;
		  delay_ms(1);
	      DM541PlusToggle;
		  DM541MotorReversalExecutionTimer++;
	  } 
     while(UnloadPositionAccuracyFlag==RESET);	
	 //while(DM541MotorReversalExecutionTimer<=1600/*DM541MotorPositiveTurnExecutionMaxTime*/); 
	  DM541MotorReversalExecutionTimer=0;
	  UnloadPositionAccuracyFlag=RESET;  
 }

void VoiceCoilMotorStretchOut(void)
{
    VoiceCoilMotorP_OFF;
	VoiceCoilMotorN_ON;
}
void VoiceCoilMotorTakeBack(void)
{
	
	VoiceCoilMotorP_ON;
	VoiceCoilMotorN_OFF;
}

void VoiceCoilMotorRelease(void)
{
	VoiceCoilMotorP_ON;
	VoiceCoilMotorN_ON;
}

void EliminateVoiceCoilMotorStretchOut(void)
{
    EliminateVoiceCoilMotorP_OFF;
	EliminateVoiceCoilMotorN_ON;
}
void EliminateVoiceCoilMotorTakeBack(void)
{
	EliminateVoiceCoilMotorP_ON;
	EliminateVoiceCoilMotorN_OFF;
}

void EliminateVoiceCoilMotorRelease(void)
{
	EliminateVoiceCoilMotorP_ON;
	EliminateVoiceCoilMotorN_ON;
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

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @brief  Configure the MPU attributes as Write Through for SRAM1/2.
  * @note   The Base Address is 0x20010000 since this memory interface is the AXI.
  *         The Region Size is 256KB, it is related to SRAM1 and SRAM2  memory size.
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}


/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
