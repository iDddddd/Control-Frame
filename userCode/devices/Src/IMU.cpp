//
// Created by LEGION on 2021/10/17.
//

#include "IMU.h"

#define IMU_USE_MAG
IMU IMU::imu;

void IMU::Init() {
    if (BMI088_init() != BMI088_NO_ERROR) Error_Handler();
#ifdef IMU_USE_MAG
    if(ist8310_init() != IST8310_NO_ERROR) Error_Handler();
#endif
    BMI088_read(rawData.gyro, rawData.accel, &rawData.temp);
    offset();
    PID_Regulator_t _tempPID = {
            .kp = 340,
            .ki = 0.04,
            .kd = 0.0,
            .componentKpMax = 2000,
            .componentKiMax = 900,
            .outputMax = 999
    };
    tempPid.PIDInfo = _tempPID;
    quat[0] = 1.0f, quat[1] = 0, quat[2] = 0, quat[3] = 0;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
    SPI1_DMA_init((uint32_t) buf.gyro_dma_tx_buf, (uint32_t) buf.gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    state.imu_start_dma_flag = 1;

}

void IMU::Handle() {

    record_accel(position._accel, proData.accel);
    record_velocity(position._velocity, position.velocity);
    if (state.gyro_update_flag & (1u << IMU_NOTIFY_SHFITS)) {
        state.gyro_update_flag &= ~(1u << IMU_NOTIFY_SHFITS);
        BMI088_gyro_read_over(buf.gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, rawData.gyro);
    }

    if (state.accel_update_flag & (1u << IMU_UPDATE_SHFITS)) {
        state.accel_update_flag &= ~(1u << IMU_UPDATE_SHFITS);
        BMI088_accel_read_over(buf.accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, rawData.accel, &rawData.time);

    }

    if (state.accel_temp_update_flag & (1u << IMU_UPDATE_SHFITS)) {
        state.accel_temp_update_flag &= ~(1u << IMU_UPDATE_SHFITS);
        BMI088_temperature_read_over(buf.accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &rawData.temp);
        imu_temp_control(rawData.temp);
    }
    data_adjust(proData.accel, rawData.accel);

    filter(&proData.accel[0], axFilter);
    filter(&proData.accel[1], ayFilter);
    velocityVerify();
    get_velocity(position.velocity, position._accel, proData.accel);
    get_displace(position.displace, position._velocity, position.velocity);
    AHRS_update(quat, 0.001f, rawData.gyro, proData.accel, rawData.mag);
    get_angle(quat, &attitude.yaw, &attitude.pitch, &attitude.rol);
    attitude.pitch_v = rawData.gyro[0];
    attitude.rol_v = rawData.gyro[1];
    attitude.yaw_v = rawData.gyro[2];

    attitude.neg_yaw_v = -attitude.yaw_v;
    attitude.neg_pitch_v = -attitude.pitch_v;
    attitude.neg_rol_v = -attitude.rol_v;
		/*static int imu_cut = 0;
		if(imu_cut > 5){
			IMU_Send();
			imu_cut = 0;
		}
		imu_cut++;*/
}

void IMU::ITHandle(uint16_t GPIO_Pin) {
    if (GPIO_Pin == INT1_ACCEL_Pin) {
        state.accel_update_flag |= 1u << IMU_DR_SHFITS;
        state.accel_temp_update_flag |= 1u << IMU_DR_SHFITS;
        if (state.imu_start_dma_flag) {
            imu_cmd_spi_dma();
        }
    } else if (GPIO_Pin == INT1_GRYO_Pin) {
        state.gyro_update_flag |= 1u << IMU_DR_SHFITS;
        if (state.imu_start_dma_flag) {
            imu_cmd_spi_dma();
        }
    } else if (GPIO_Pin == IST8310_DRDY_Pin) {
        state.mag_update_flag |= 1u << IMU_DR_SHFITS;
        if (state.mag_update_flag &= 1u << IMU_DR_SHFITS) {
            state.mag_update_flag &= ~(1u << IMU_DR_SHFITS);
            state.mag_update_flag |= (1u << IMU_SPI_SHFITS);

            ist8310_read_mag(rawData.mag);
        }
    }
}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
void IMU::imu_cmd_spi_dma(void) {

    //开启陀螺仪的DMA传输
    if ((state.gyro_update_flag & (1u << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(state.accel_update_flag & (1u << IMU_SPI_SHFITS)) &&
        !(state.accel_temp_update_flag & (1u << IMU_SPI_SHFITS))) {
        state.gyro_update_flag &= ~(1u << IMU_DR_SHFITS);
        state.gyro_update_flag |= (1u << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t) buf.gyro_dma_tx_buf, (uint32_t) buf.gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    //开启加速度计的DMA传输
    if ((state.accel_update_flag & (1u << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(state.gyro_update_flag & (1u << IMU_SPI_SHFITS)) &&
        !(state.accel_temp_update_flag & (1u << IMU_SPI_SHFITS))) {
        state.accel_update_flag &= ~(1u << IMU_DR_SHFITS);
        state.accel_update_flag |= (1u << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t) buf.accel_dma_tx_buf, (uint32_t) buf.accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }

    if ((state.accel_temp_update_flag & (1u << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(state.gyro_update_flag & (1u << IMU_SPI_SHFITS)) && !(state.accel_update_flag & (1u << IMU_SPI_SHFITS))) {
        state.accel_temp_update_flag &= ~(1u << IMU_DR_SHFITS);
        state.accel_temp_update_flag |= (1u << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t) buf.accel_temp_dma_tx_buf, (uint32_t) buf.accel_temp_dma_rx_buf,
                        SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }

}

void DMA2_Stream0_IRQHandler(void) {

    IMU::imu.ITHandle();

}


void IMU::ITHandle() {

    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET) {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //陀螺仪读取完毕
        if (state.gyro_update_flag & (1u << IMU_SPI_SHFITS)) {
            // SystemEstimator::systemEstimator.ITEstimatorTrigger(MPU_GYRO_IT);
            state.gyro_update_flag &= ~(1u << IMU_SPI_SHFITS);
            state.gyro_update_flag |= (1u << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

        }

        //accel read over
        //加速度计读取完毕
        if (state.accel_update_flag & (1u << IMU_SPI_SHFITS)) {
            //  SystemEstimator::systemEstimator.ITEstimatorTrigger(MPU_ACCEL_IT);
            state.accel_update_flag &= ~(1u << IMU_SPI_SHFITS);
            state.accel_update_flag |= (1u << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //温度读取完毕
        if (state.accel_temp_update_flag & (1u << IMU_SPI_SHFITS)) {
            // SystemEstimator::systemEstimator.ITEstimatorTrigger(MPU_TEMP_IT);
            state.accel_temp_update_flag &= ~(1u << IMU_SPI_SHFITS);
            state.accel_temp_update_flag |= (1u << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if (state.gyro_update_flag & (1u << IMU_UPDATE_SHFITS)) {
            state.gyro_update_flag &= ~(1u << IMU_UPDATE_SHFITS);
            state.gyro_update_flag |= (1u << IMU_NOTIFY_SHFITS);
        }
    }

}

void IMU::ErrorHandle() {

}

/**
 *
 * @brief 姿态解算
 * @param quat[4]:四元数
 */
void IMU::AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3]) {
#ifdef IMU_USE_MAG
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2],
            accel[0], accel[1], accel[2],
            mag[0],mag[1], mag[2]);
#else
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2],
                     accel[0], accel[1], accel[2],
                     0, 0, 0);
#endif
}

void IMU::get_angle(float q[4], float *yaw, float *pitch, float *roll) {
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}


/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
void IMU::imu_temp_control(float temp) {
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate) {
        tempPid.PIDCalc(45.0f, temp);
        if (tempPid.PIDInfo.output < 0.0f) {
            tempPid.PIDInfo.output = 0.0f;
        }
        tempPWM = (uint16_t) tempPid.PIDInfo.output;

        IMU_temp_PWM(tempPWM);

    } else {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > 45.0f) {
            temp_constant_time++;
            if (temp_constant_time > 200) {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                tempPid.PIDInfo.componentKiMax = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

void IMU::IMU_temp_PWM(float pwm) {
            __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}

/**
 * @brief 位置获取
 * @param _accel 上一次加速度值
 * @param accel  本次加速度值
 */
void IMU::record_accel(float _accel[3], float accel[3]) {
    _accel[0] = accel[0];
    _accel[1] = accel[1];
    _accel[2] = accel[2];
}

void IMU::get_velocity(float velocity[3], float _accel[3], float accel[3]) {
    velocity[0] += ((_accel[0] + accel[0]) / 2 * 0.001f);
    velocity[1] += ((_accel[1] + accel[1]) / 2 * 0.001f);
    velocity[2] += ((_accel[2] + accel[2]) / 2 * 0.001f);
}

void IMU::record_velocity(float _velocity[3], float velocity[3]) {
    _velocity[0] = velocity[0];
    _velocity[1] = velocity[1];
    _velocity[2] = velocity[2];
}

void IMU::get_displace(float displace[3], float _velocity[3], float velocity[3]) {
    displace[0] += ((_velocity[0] + velocity[0]) / 2 * 0.001f);
    displace[1] += ((_velocity[1] + velocity[1]) / 2 * 0.001f);
    displace[2] += ((_velocity[2] + velocity[2]) / 2 * 0.001f);
}

/**
 * @brief 数据处理
 * @param accel
 * @param _accel
 */
void IMU::offset() {
    float accel[3], gyro[3], temp;
    for (int i = 0; i < 1000; i++) {
        BMI088_read(gyro, accel, &temp);
        rawData.accel_offset[0] += accel[0];
        rawData.accel_offset[1] += accel[1];
        rawData.accel_offset[2] += accel[2];
    }
    rawData.accel_offset[0] /= 1000;
    rawData.accel_offset[1] /= 1000;
    rawData.accel_offset[2] /= 1000;
}

void IMU::data_adjust(float accel[3], float _accel[3]) {
    //  accel[0] = C1 * _accel[0] + C2 * _accel[1] + C3 * _accel[2] + Cx - rawData.accel_offset[0];
    //  accel[1] = C4 * _accel[0] + C5 * _accel[1] + C6 * _accel[2] + Cy - rawData.accel_offset[1];
    accel[0] = _accel[0] - rawData.accel_offset[0];
    accel[1] = _accel[1] - rawData.accel_offset[1];
    accel[2] = _accel[2];
}

void IMU::velocityVerify() {
    static int xcount = 0;
    static int ycount = 0;
    if (abs(proData.accel[0]) < 0.05) {
        xcount++;
    } else {
        xcount = 0;
    }
    if (xcount >= 10) {
        position.velocity[0] = 0;
    }
    if (abs(proData.accel[1]) < 0.05) {
        ycount++;
    } else {
        ycount = 0;
    }
    if (ycount >= 10) {
        position.velocity[1] = 0;
    }

}

void IMU::filter(float *current, IMU_Filter_t Filter) {

    int i, j;
    float sum = 0;
    Filter.current = *current;
    if (Filter.buff_init == 0) {
        Filter.history[Filter.index] = Filter.current;
        Filter.index++;
        if (Filter.index >= (SUM_WIN_SIZE - 1)) {
            Filter.buff_init = 1;//index有效范围是0-5，前面放到5，下一个就可以输出
        }
        return;//当前无法输出，做个特殊标记区分
    } else {
        Filter.history[Filter.index] = Filter.current;
        Filter.index++;
        if (Filter.index >= SUM_WIN_SIZE) {
            Filter.index = 0;//index有效最大5,下次再从0开始循环覆盖
        }

        j = Filter.index;
        for (i = 0; i < SUM_WIN_SIZE; i++) {
            //注意i=0的值并不是最早的值
            sum += Filter.history[j] * Filter.factor[i];//注意防止数据溢出
            j++;
            if (j == SUM_WIN_SIZE) {
                j = 0;
            }
        }
        *current = sum / Filter.K;
    }
}
uint8_t LRCcalc(uint8_t* data, int data_len){
    uint8_t LRC = 0;
    for(int i = 0; i < data_len; i++){
        LRC += data[i];
    }
    return LRC;
}

void IMU::IMU_Send() {
    imu_data.accel[0].f = rawData.accel[0];
    imu_data.accel[1].f = rawData.accel[1];
    imu_data.accel[2].f = rawData.accel[2];
    imu_data.gyro[0].f = rawData.gyro[0];
    imu_data.gyro[1].f = rawData.gyro[1];
    imu_data.gyro[2].f = rawData.gyro[2];
    imu_data.mag[0].f = rawData.mag[0];
    imu_data.mag[1].f = rawData.mag[1];
    imu_data.mag[2].f = rawData.mag[2];

    imu_send_data[0] = 0x7A;
    imu_send_data[1] = 0x02;
    imu_send_data[2] = 0x03;
    imu_send_data[3] = 0x24;
    imu_send_data[4] = imu_data.accel[0].u8[0];
    imu_send_data[5] = imu_data.accel[0].u8[1];
    imu_send_data[6] = imu_data.accel[0].u8[2];
    imu_send_data[7] = imu_data.accel[0].u8[3];
    imu_send_data[8] = imu_data.accel[1].u8[0];
    imu_send_data[9] = imu_data.accel[1].u8[1];
    imu_send_data[10] = imu_data.accel[1].u8[2];
    imu_send_data[11] = imu_data.accel[1].u8[3];
    imu_send_data[12] = imu_data.accel[2].u8[0];
    imu_send_data[13] = imu_data.accel[2].u8[1];
    imu_send_data[14] = imu_data.accel[2].u8[2];
    imu_send_data[15] = imu_data.accel[2].u8[3];
    imu_send_data[16] = imu_data.gyro[0].u8[0];
    imu_send_data[17] = imu_data.gyro[0].u8[1];
    imu_send_data[18] = imu_data.gyro[0].u8[2];
    imu_send_data[19] = imu_data.gyro[0].u8[3];
    imu_send_data[20] = imu_data.gyro[1].u8[0];
    imu_send_data[21] = imu_data.gyro[1].u8[1];
    imu_send_data[22] = imu_data.gyro[1].u8[2];
    imu_send_data[23] = imu_data.gyro[1].u8[3];
    imu_send_data[24] = imu_data.gyro[2].u8[0];
    imu_send_data[25] = imu_data.gyro[2].u8[1];
    imu_send_data[26] = imu_data.gyro[2].u8[2];
    imu_send_data[27] = imu_data.gyro[2].u8[3];
    imu_send_data[28] = imu_data.mag[0].u8[0];
    imu_send_data[29] = imu_data.mag[0].u8[1];
    imu_send_data[30] = imu_data.mag[0].u8[2];
    imu_send_data[31] = imu_data.mag[0].u8[3];
    imu_send_data[32] = imu_data.mag[1].u8[0];
    imu_send_data[33] = imu_data.mag[1].u8[1];
    imu_send_data[34] = imu_data.mag[1].u8[2];
    imu_send_data[35] = imu_data.mag[1].u8[3];
    imu_send_data[36] = imu_data.mag[2].u8[0];
    imu_send_data[37] = imu_data.mag[2].u8[1];
    imu_send_data[38] = imu_data.mag[2].u8[2];
    imu_send_data[39] = imu_data.mag[2].u8[3];
    imu_send_data[40] = LRCcalc(imu_send_data, 40);
    HAL_UART_Transmit_DMA(&huart6, imu_send_data, sizeof(imu_send_data));

}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART6){
      
    }
}
