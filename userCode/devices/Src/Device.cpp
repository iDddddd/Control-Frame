//
// Created by LEGION on 2021/10/4.
//
#include "Device.h"
#include "RemoteControl.h"
#include "Motor.h"
#include "IMU.h"
#include "ARMMotor.h"
#include "Buzzer.h"
#include "LED.h"

volatile float vccMoni = 0;
volatile float vccBat = 0;

void bsp_ADC_vccMoni() {
    static uint8_t calibrationCplt = 0;
    static uint32_t calibrationCnt = 0;
    static uint32_t vrefSum = 0, vrefValue = 0;
    if (calibrationCplt == 0) {
        if (calibrationCnt == 0) {
            calibrationCnt++;
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 2);
            vrefValue = HAL_ADC_GetValue(&hadc1);
            vrefSum += vrefValue;
        } else if (calibrationCnt < 200) {
            calibrationCnt++;
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 2);
            vrefValue = HAL_ADC_GetValue(&hadc1);
            vrefSum += vrefValue;
        } else {
            calibrationCplt = 1;
            vrefValue = vrefSum / 200.0f;
            HAL_ADC_Stop(&hadc1);

            HAL_ADC_Start(&hadc3);
            HAL_ADC_PollForConversion(&hadc3, 2);
            vccMoni = HAL_ADC_GetValue(&hadc3) * 1.25f / (float) vrefValue;

        }

    } else {
        HAL_ADC_Start(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, 2);
        vccMoni = HAL_ADC_GetValue(&hadc3) * 1.25f / (float) vrefValue;

    }
    vccBat = vccMoni / 22.0f * 222.0f;
}

/**
 * @brief 利用串口重定向函数，可选usb虚拟串口或硬件串口，使用需安装相应驱动
 * @param fmt
 * @param ...
 */
void usart_printf(const char *fmt, ...) {

    static uint8_t tx_buf[256] = {0};//TODO 爆栈？
    static va_list ap;
    static uint16_t len;

    va_start(ap, fmt);

    len = vsprintf((char *) tx_buf, fmt, ap);

    va_end(ap);

    //HAL_UART_Transmit_DMA(&huart1,tx_buf,len);
    CDC_Transmit_FS(tx_buf, len);

}

#define FLASH_SECTOR_9_ADDRESS 0x080A0000
flash_data_t flashData;

void bsp_flash_write(flash_data_t *_flashData) {
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInit;
    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.Sector = FLASH_SECTOR_9;
    eraseInit.NbSectors = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&eraseInit, nullptr);
    uint32_t flash_ptr = FLASH_SECTOR_9_ADDRESS;

    for (auto *ptr = (uint32_t *) _flashData; ptr < (uint32_t *) (sizeof(flash_data_t) / 4 + _flashData); ptr++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_ptr, *ptr);
        flash_ptr += 4;
    }
    HAL_FLASH_Lock();
}

void bsp_flash_read(flash_data_t *_flashData) {
    memcpy(_flashData, (uint32_t *) FLASH_SECTOR_9_ADDRESS, sizeof(flash_data_t) / 4);
}


uint32_t init_Flag = 0;
uint32_t period = 5000;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (init_Flag == 0)return;
    if (htim == &htim10) {//1ms
        //HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
        aRGB_led_change(period);

        bsp_ADC_vccMoni();

        static uint32_t cnt = 0;
        cnt++;

        /**只需关注该部分代码**/
        ChassisHandle();//底盘数据处理
        ARMHandle();
        Motor::MotorsHandle();//电机数据处理
        CAN::CANPackageSend();//CAN发送

        /**只需关注该部分代码**/

        if (cnt > 20) {
            if (vccBat < 10)HAL_IWDG_Refresh(&hiwdg);
            cnt = 0;
        }

    }
    if (htim == &htim6) {//4ms
        RS485::RS485PackageSend();

    }
    if (htim == &htim7) {
        IMU::imu.Handle();
        CtrlHandle();
    }
}

volatile uint8_t key_raw_state = 1;//
uint32_t key_last_stamp;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (init_Flag == 0)return;
    if (GPIO_Pin == GPIO_PIN_0) {
        key_raw_state = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
        uint32_t time_interval = HAL_GetTick() - key_last_stamp;
        key_last_stamp = HAL_GetTick();
        if (time_interval >= 50) {
            if (key_raw_state == 1) {
                period = 5000;
            } else {
                period = 500;
            }
        }
    }
    IMU::imu.ITHandle(GPIO_Pin);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main() {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */


    /* MCU Configuration--、、、、、、------------------------------------------------------*/

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
    MX_DMA_Init();
    MX_TIM5_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM10_Init();
    MX_ADC1_Init();
    MX_ADC3_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_USART3_UART_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_I2C3_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_IWDG_Init();//看门狗,若不使用遥控器需注释改行，否则程序不运行
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start_IT(&htim5);//该定时器作PWM输出

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

    HAL_TIM_Base_Start_IT(&htim1);//舵机
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                          1750);//初始化爪子
    //TODO adc校准？
    RemoteControl::init();//遥控器通讯初始化，使用UART3串口
    ManiControl::Init();//上位机通讯初始化，使用UART6串口
    RS485::RS485Init();//RS485通讯初始化，使用UART1串口
    bsp_flash_read(&flashData);
    HAL_TIM_Base_Start_IT(&htim10);//1ms
    HAL_TIM_Base_Start_IT(&htim6);//4ms
    HAL_TIM_Base_Start_IT(&htim7);//1ms
    CAN::CANInit();//CAN初始化
    Motor_4310::Init();//电机初始化(大部分电机不需初始化）
    IMU::imu.Init();//IMU初始化
  //  bsp_BuzzerOn(1000);
    init_Flag = 1;

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */
        //循环中不做处理，所有处理在中断中完成
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}