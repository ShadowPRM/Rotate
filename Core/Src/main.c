/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MB_veribl_my.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MB_D_MAS 150         // Размер массива модбас данных
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//********************************* МО�? ПЕРЕМЕННЫЕ *********************************
union {
    struct{
      uint16_t   oborMin1;
      uint16_t   oborMin2;
    };
    float oborMin;
  } Uobor;
uint16_t adc1=0;
uint16_t zashelka1=0;
////////.................... МОДбус ....................////////
//uint16_t mb_data[MB_D_MAS]={0,}; // МодБас регистры
volatile _Bool	t35end=0,			//флаг время МБ Т35 прошло
				wattTXtimC=0;		//флаг "пора опросить ваттметр"
//uint16_t txPac=7;
//uint8_t MB_BUFF[200]={0,};          //буфер УАРТ 1байт, но регистры 2байта!
//uint8_t MB_preBUFF[1]={0},
//		MB_buffCount=0;
//uint8_t MB_Prot_kolbyt=0,
//		MB_MyPac_OK=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
unsigned int Crc16Table[256] = {
/*
  Name  : CRC-16
  Poly  : 0x8005    x^16 + x^15 + x^2 + 1
  Init  : 0xFFFF
  Revert: true
  XorOut: 0x0000
  Check : 0x4B37 ("123456789")
  MaxLen: 4095 байт (32767 бит) - обнаружение одинарных, двойных, тройных и всех нечетных ошибок
*/
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

unsigned int Crc16(unsigned char *pcBlock, unsigned int len){
    unsigned int crc = 0xFFFF;
    while (len--) crc = (crc >> 8) ^ Crc16Table[(crc & 0xFF) ^ *pcBlock++];
    return crc;
    }

void SetupMBdata (void){
	//mb_data[UART_SPD] = huart2.Instance->BRR;
	mb_data[MY_ID] = 1;
	mb_data[UART_DAT] = 0;		// Нахуя это? (это вроде 7 или 8 бит настройка)
	mb_data[UART_STP] = 0;		// Стоп-битов
	mb_data[UART_PAR] = 0;		// Проверка паритеета
	mb_data[UART_MB_T15] = 0;	// Тайминг 1,5Т для модбуси
	mb_data[UART_MB_T35] = 0;	// Тайминг 3,5Т для модбуси
	mb_data[MAST_LI] =0;		// Бит Мастер Ли?!  1-да
	mb_data[PARAM_SL] =0;		// Разные параметры платы (собрать бы всё сюда 8,56,...
	mb_data[MB_DATA0] =0;
	mb_data[MB_DATA1] =0;
	mb_data[MB_DATA2] =0;
	mb_data[MB_DATA3] =0;
                        // 0-7                                       8-15                                Coil:
	mb_data[MAS_P3] = 0;  // резервный регистр для мастер платы (коммутации и измерений)                  //0-15
	mb_data[MAS_P2] = 0;  // _/_/_/_/_/_/_/_/                          12V/24V/110V/Охлажд/_/_/_/_        //16-31
	mb_data[MAS_P1] = 0;  // I02/I1/I2/I5/U50/U150/Uместа/Uист/        Uбп/Uвнеш/ШунтPA/ЗарC/_/_/U++/U--  //32-47
	mb_data[REL_FUN] = 0;  // capR/cap3/izmU/cap2/Pusk/cap1/Rout/izmI/  obk/_/_/_/_/_/_/_                  //48-63
	mb_data[AVARII] = 0;  // Флаги ошибок и аварий                                                        //64-79

	mb_data[COUN_KZ]=0;  //25 Счётчик КЗ

	//mb_data[IZM_FAZ] = kolvoIZMobsh/4;   // Кол-во измер по фазам // Значение расчётное
	//mb_data[ZERO_A] = 1022;   // Знач средн точки фазы А
	//mb_data[ZERO_B] = 1022;   // Знач средн точки фазы B
	//mb_data[ZERO_C] = 1022;   // Знач средн точки фазы C

	mb_data[IZM_OI] = 50;   // Кол-во измер до Превыш тока
	mb_data[IZM_OF] = 25;   // Кол-во измер до Обрыва фазы
	mb_data[IZM_KZ] = 50;   // Кол-во измер до Коротк зам (или количество захватов по таймеру)

	mb_data[OVR_I] = 5;   // Знач превыш тока
	mb_data[OBR_F] = 10;   // Знач обрыва фазы
	mb_data[OVR_IKZ] = 1023;   // Знач тока КЗ (ну или если с предохранителем, то л0 с GPIO)

	mb_data[SCALADC] = 0;   // Код текущей подобраной шкалы
	mb_data[LED_BR] = 50;   // Яркость индикатора
	mb_data[TIMREL_OFF] = 0;   // отключение реле по таймеру (например при потере связи)
	mb_data[TIM_CAPR] = 0;   // отключение Шунтирующего конденсаторы резистора (когда кандёры зарядились)
	mb_data[TIM_RAZC] = 0;   // Задержка подачи напряжения на место для безопастной разрядки конденаторов
	mb_data[OVR_I5] = 0;   // Знач превыш тока ЭВ5
	mb_data[OVR_I6] = 0;   // Знач превыш тока ЭВ6
	mb_data[OVR_I7] = 0;   // Знач превыш тока ЭВ7
	mb_data[OVR_I8] = 0;   // Знач превыш тока ЭВ8

	mb_data[OBR_FA] = 0;    // Знач обрыва фазы АВТО
	mb_data[OBR_F05] = 0;    // Знач обрыва "половины" фазы
	mb_data[OBR_F03] = 0;    // Знач обрыва "трети" фазы
	mb_data[OBR_F02] = 0;    // Знач обрыва "четверти" фазы
	mb_data[OBR_F1] = 0;    // Знач обрыва фазы 1
	mb_data[OBR_F2] = 0;    // Знач обрыва фазы 2

	mb_data[LED_CMB] = 0;    // Комбинация светодиодов
	mb_data[VRM_RL1] = 40;    // Время задержки переключения реле 1 (с обкатки на замер)
	mb_data[VRM_RL2] = 40;    // Время задержки переключения реле 2 (с замера на обкатку)
	mb_data[CRC_EEP] = 0;    // Значение контрольной суммы EEPROM массива

	mb_data[PROG_VER] = 1512;   // Версия прошивки
////////////////////////// �?змерения источнтка на нижней плате ///////////////////////////////
// тут этого нет, это МП
	mb_data[IN_IST] = 0;  // Состояние входов и сигналов платы источника (тут не используется)
	mb_data[OUT_IST] = 0;  // Состояние выходов и сигналов платы источника (тут не используется)
////////////////////////// �?змерения источнтка китайским ваттметром ///////////////////////////////
	// тут этого нет, это МП
////////////////////////// Переменные управления шаговым ЭД ///////////////////////////////
// тут этого нет, это МП
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM2){		// каждые 3,4635мс(T35 для 9600)
		t35end=1;
	} //end  1,75мс(T35)
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart==&huart1){;}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart==&huart1){
		TIM2->CNT = 0; // сброс Т�?М1, начало отсчёта Т35
		t35end=0;      // сбрасываем Т35
		HAL_UART_Receive_IT(&huart1, (uint8_t*)MB_preBUFF, rxPac); //запуск следующего приёма
		MB_BUFF[MB_buffCount++]=MB_preBUFF[0];   //сохраняем полученый байт в буфер
		MB_MyPac_OK=0;
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1) //check if the interrupt comes from ACD1
    {
        adc1 = HAL_ADC_GetValue(hadc);
    }
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
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_ADC_Start_IT(&hadc);
  //HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1); // генерирует прерывание
  //HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
  //HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
  //////////////////////////////////// Запускаем ожидание приёма по УАРТ
  HAL_UART_Receive_IT(&huart1, (uint8_t*)MB_preBUFF, rxPac);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  mb_data[MY_ID] = 1;
  //TIM1->ARR = 3000; //начальное значение оборотов (1 об/мин)
  mb_data[2] = TIM1->ARR;

  while (1){
	  uint16_t arr1=0;
	  uint16_t arrADC=0;

	  //  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  //HAL_GPIO_WritePin(GPIOA, DIR_Mot_Pin|ENA_Mot_Pin, GPIO_PIN_RESET);
	//out6_GPIO_Port->ODR&=~out6_Pin;
	  //DIR_Mot_GPIO_Port->ODR |= DIR_Mot_Pin;
	  //ENA_Mot_GPIO_Port->ODR |= ENA_Mot_Pin;
	  Uobor.oborMin = (float)((48000000/(((TIM1->PSC)+1)*((TIM1->ARR)+1)*2.0*3200.0))*60); //подсчёт реальных оборотов/мин
	  //Uobor.oborMin = 123.456;
	  mb_data[3] = Uobor.oborMin1;
	  mb_data[4] = Uobor.oborMin2;
	  mb_data[7]=adc1;



	    //********************** >Т35 **********************//
	    	if ( !MB_MyPac_OK && t35end ) {
	    			TIM2->CNT = 0; t35end=0; // сброс счётчика и флага
	    			MB_buffCount=0;
	    			//if (CtrlBT_GPIO_Port->IDR & CtrlBT_Pin) {
	    			if (InBT_GPIO_Port->IDR & InBT_Pin) {					//Блютус ОТКЛ
	    				DIR_Mot_GPIO_Port->ODR |= DIR_Mot_Pin;				//Это напрявление вращения
	    				ENA_Mot_GPIO_Port->ODR &=~ ENA_Mot_Pin;				//Это отпустить/схватить ЭД
	    				arrADC = (48000000/(2*((TIM1->PSC) + 1)*(mb_data[7]/67)*3200/60))-1;
	    				TIM1->ARR = arrADC + mb_data[5] - mb_data[6];
	    			}
	    			MB_BUFF[0]=0xff; MB_BUFF[1]=0xff;   //это стираем ID и COM навсякий, но это ОПАСНО!
	    		}

	    //********************** (Tx) ОТВЕЧАЕМ КОГДА ПАКЕТ МОЙ и Т35 **********************//
	    		if ( MB_MyPac_OK && t35end ) {
	    			TIM2->CNT = 0; t35end=0; // сброс счётчика и флага
	    			MB_buffCount=0;
	    			MB_MyPac_OK=0;
	    			//кнопки
	    			if ( !(InBT_GPIO_Port->IDR & InBT_Pin) && (InR_GPIO_Port->IDR & InR_Pin) ) {	//Работа от Блютус
	    				if (mb_data[9]) {DIR_Mot_GPIO_Port->ODR |= DIR_Mot_Pin;} //реверс
	    				else            {DIR_Mot_GPIO_Port->ODR &=~ DIR_Mot_Pin;}

	    				if (mb_data[8]) {ENA_Mot_GPIO_Port->ODR |= ENA_Mot_Pin;} //отпустить ЭД
	    				else            {ENA_Mot_GPIO_Port->ODR &=~ ENA_Mot_Pin;}

	    				if    (mb_data[10]) {HAL_TIM_OC_Stop_IT (&htim1, TIM_CHANNEL_1);  zashelka1=0;} //стоп генерации
	    				else if (!(zashelka1)) {HAL_TIM_OC_Start_IT (&htim1, TIM_CHANNEL_1); zashelka1=1;} //zashelka1
	    			}
	    			//рачёт ARR, если мы задаём в об/мин
	    			//if ( !(CtrlBT_GPIO_Port->IDR & CtrlBT_Pin) && (CtrlR_GPIO_Port->IDR & CtrlR_Pin) &&
	    			if ( !(InBT_GPIO_Port->IDR & InBT_Pin) && (InR_GPIO_Port->IDR & InR_Pin) &&		//Работа от Блютус
	    					(mb_data[2] >= 0)&& (mb_data[2] <= 60)  ) {
	    				arr1 = (48000000/(2*((TIM1->PSC) + 1)*mb_data[2]*3200/60))-1;
	    				TIM1->ARR = arr1 + mb_data[5] - mb_data[6];
	    			}
	    			//if ( (CtrlBT_GPIO_Port->IDR & CtrlBT_Pin) && !(CtrlR_GPIO_Port->IDR & CtrlR_Pin) ) {
	    			if ( (InBT_GPIO_Port->IDR & InBT_Pin) && !(InR_GPIO_Port->IDR & InR_Pin) ) {		//Работа от Резистора
	    				arrADC = (48000000/(2*((TIM1->PSC) + 1)*(mb_data[7]/67)*3200/60))-1;
	    				TIM1->ARR = arrADC + mb_data[5] - mb_data[6];
	    			}
	    			mb_data[5]=0; mb_data[6]=0;
	    			HAL_UART_Transmit_IT(&huart1,(uint8_t*)MB_BUFF, txPac);
	    			MB_BUFF[0]=0xff; // чё зачем??????
	    		}

	    //***********************************************************************************
	    //************************ ОБРАБОТКА и СОХРАНЕН�?Е ДАННЫХ ПАКЕТА ********************
	    //***********************************************************************************
	    		if (!MB_MyPac_OK) {
	    			if ( (MB_BUFF[0]==0) || (MB_BUFF[0]==mb_data[MY_ID]) ) {
	    				//LedDbg_GPIO_Port->ODR ^= LedDbg_Pin;
	    //********************** Команда 16(0x10) ЗАП�?СЬ нескольких регистров ******************
	    				if ( (MB_BUFF[1]==0x10) && (MB_buffCount==7) ){  //проверка команды и вычисление длины пакета
	    				   MB_Prot_adr1reg = (MB_BUFF[2]<<8)|MB_BUFF[3]; //адресс первого регистра
	    				   MB_Prot_kolreg =  (MB_BUFF[4]<<8)|MB_BUFF[5]; //кол регистров
	    				   MB_Prot_kolbyt = MB_BUFF[6];				     //кол байт далее
	    				}
	    				if (MB_buffCount==MB_Prot_kolbyt+7+2) {
	    					if (!Crc16(MB_BUFF, MB_Prot_kolbyt+7+2)) {
	    						while(MB_Prot_kolreg)   //сохраняем положение регистров (реле)
	    						{
	    							mb_data[--MB_Prot_kolreg + MB_Prot_adr1reg] = (MB_BUFF[6 + MB_Prot_kolbyt--]) |
	    							(MB_BUFF[6 + MB_Prot_kolbyt--]<<8);
	    						}
	    					crc16rez = Crc16(MB_BUFF, 6);                  //расчёт crc16 для ответа
	    					MB_BUFF[6] = crc16rez;                         //в ответ: Lo crc16
	    					MB_BUFF[7] = crc16rez>>8;                      //в ответ: Hi crc16
	    					txPac = 8; //должен быть кол-во отправляемых байт -1
	    					MB_MyPac_OK=1;
	    					mb_data[AVARII]&=~(1<<13);

	    					//SetRelayZash(); SetLED();
	    					}
	    					else {mb_data[AVARII]|=(1<<13);} // Ошибка crc-16
	    				}//end Команда 0x10


	    //********************** Команда 06(0x06) ЗАП�?СЬ 1 регистра ******************
	    				if ( (MB_BUFF[1]==0x06) && (MB_buffCount==8) ) {
	    					if (!Crc16(MB_BUFF, 8)) {
	    						MB_Prot_adr1reg = (MB_BUFF[2]<<8)|MB_BUFF[3];          //адресс первого регистра
	    						mb_data[MB_Prot_adr1reg] = (MB_BUFF[4]<<8)|MB_BUFF[5]; //сохраняем положение регистров (реле)
	    						txPac = 8; //должен быть кол-во отправляемых байт -1
	    						MB_MyPac_OK=1;
	    						mb_data[AVARII]&=~(1<<13);
	    						//SetRelayZash(); SetLED();
	    					}
	    					else {mb_data[AVARII]|=(1<<13);} // Ошибка crc-16
	    				}//end Команда 0x06

	    //********************** Команда 03 (0x03) ЧТЕН�?Е нескольких регистров ******************
	    				if ( (MB_BUFF[1]==0x03) && (MB_buffCount==8) ) {
	    					if (!Crc16(MB_BUFF, 8)) {   //проверка команды и вычисление длины пакета
	    						MB_Prot_adr1reg = (MB_BUFF[2]<<8)|MB_BUFF[3]; //адресс первого регистра
	    						MB_Prot_kolreg =  ( (MB_BUFF[4]<<8)|MB_BUFF[5] ); //кол регистров
	    						//далее формирование отвера (запрошеных регистров)
	    						MB_Prot_kolbyt = MB_Prot_kolreg*2;  //количество байт далее
	    						MB_BUFF[2] = MB_Prot_kolbyt;
	    						while(MB_Prot_kolbyt)   //сохраняем положение регистров (реле)
	    							{
	    							MB_BUFF[--MB_Prot_kolbyt+3] = mb_data[MB_Prot_kolreg + MB_Prot_adr1reg-1];
	    							MB_BUFF[--MB_Prot_kolbyt+3] = (mb_data[MB_Prot_kolreg-- + MB_Prot_adr1reg-1])>>8;
	    							}
	    						crc16rez = Crc16(MB_BUFF, 3+MB_BUFF[2]);                  //расчёт crc16 для ответа
	    						MB_BUFF[3+MB_BUFF[2]] = crc16rez;                         //в ответ: Lo crc16
	    						MB_BUFF[4+MB_BUFF[2]] = crc16rez>>8;                      //в ответ: Hi crc16
	    						txPac = 5+MB_BUFF[2]; //должен быть кол-во отправляемых байт -1
	    						MB_MyPac_OK=1;
	    						mb_data[AVARII]&=~(1<<13);
	    					}
	    					else {mb_data[AVARII]|=(1<<13);} // Ошибка crc-16
	    				}//end Команда 0x03

	    //********************** Команда 04 (0x04) ЧТЕН�?Е нескольких регистров ******************
	    				if ( (MB_BUFF[1]==0x04) && (MB_buffCount==8) ) {
	    					if (!Crc16(MB_BUFF, 8)) {   //проверка команды и вычисление длины пакета
	    						MB_Prot_adr1reg = (MB_BUFF[2]<<8)|MB_BUFF[3]; //адресс первого регистра
	    						MB_Prot_kolreg =  ( (MB_BUFF[4]<<8)|MB_BUFF[5] ); //кол регистров
	    						//далее формирование отвера (запрошеных регистров)
	    						MB_Prot_kolbyt = MB_Prot_kolreg*2;  //количество байт далее
	    						MB_BUFF[2] = MB_Prot_kolbyt;
	    						while(MB_Prot_kolbyt)   //сохраняем положение регистров (реле)
	    							{
	    							MB_BUFF[--MB_Prot_kolbyt+3] = mb_data[MB_Prot_kolreg + MB_Prot_adr1reg-1];
	    							MB_BUFF[--MB_Prot_kolbyt+3] = (mb_data[MB_Prot_kolreg-- + MB_Prot_adr1reg-1])>>8;
	    							}
	    						crc16rez = Crc16(MB_BUFF, 3+MB_BUFF[2]);                  //расчёт crc16 для ответа
	    						MB_BUFF[3+MB_BUFF[2]] = crc16rez;                         //в ответ: Lo crc16
	    						MB_BUFF[4+MB_BUFF[2]] = crc16rez>>8;                      //в ответ: Hi crc16
	    						txPac = 5+MB_BUFF[2]; //должен быть кол-во отправляемых байт -1
	    						mb_data[AVARII]&=~(1<<13);
	    						MB_MyPac_OK=1;
	    					}
	    					else {mb_data[AVARII]|=(1<<13);} // Ошибка crc-16
	    				}//end Команда 0x04
	    			}//end MB_BUFF[0] (My ID)
	    		} //end MB_MyPac_OK



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 149;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 166249;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_Plus_GPIO_Port, ADC_Plus_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADC_Minus_Pin|DIR_Mot_Pin|ENA_Mot_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC_Plus_Pin DIR_Mot_Pin ENA_Mot_Pin */
  GPIO_InitStruct.Pin = ADC_Plus_Pin|DIR_Mot_Pin|ENA_Mot_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_Minus_Pin */
  GPIO_InitStruct.Pin = ADC_Minus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADC_Minus_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : InBT_Pin InR_Pin CtrlR_Pin CtrlBT_Pin */
  GPIO_InitStruct.Pin = InBT_Pin|InR_Pin|CtrlR_Pin|CtrlBT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

