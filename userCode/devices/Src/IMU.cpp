//
// Created by LEGION on 2021/10/17.
//

#include "IMU.h"

IMU IMU::imu;
void IMU::Init() {
    if (BMI088_init() != BMI088_NO_ERROR) Error_Handler();
#ifdef IMU_USE_MAG
    if(ist8310_init() != IST8310_NO_ERROR) Error_Handler();
#endif
    BMI088_read(rawData.gyro,rawData.accel,&rawData.temp);
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
    quat[0] = 1.0f,quat[1] = 0,quat[2] = 0,quat[3] = 0;

    if(HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
    SPI1_DMA_init((uint32_t)buf.gyro_dma_tx_buf, (uint32_t)buf.gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    state.imu_start_dma_flag = 1;

}
void IMU::Handle() {

    record_accel(position._accel, proData.accel);
    record_velocity(position._velocity, position.velocity);
    if(state.gyro_update_flag & (1u << IMU_NOTIFY_SHFITS))
    {
        state.gyro_update_flag &= ~(1u << IMU_NOTIFY_SHFITS);
        BMI088_gyro_read_over(buf.gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, rawData.gyro);
    }

    if(state.accel_update_flag & (1u << IMU_UPDATE_SHFITS))
    {
        state.accel_update_flag &= ~(1u << IMU_UPDATE_SHFITS);
        BMI088_accel_read_over(buf.accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, rawData.accel, &rawData.time);
        rawData.ax = rawData.accel[0];
        rawData.ay = rawData.accel[1];
        rawData.az = rawData.accel[2];
    }

    if(state.accel_temp_update_flag & (1u << IMU_UPDATE_SHFITS))
    {
        state.accel_temp_update_flag &= ~(1u << IMU_UPDATE_SHFITS);
        BMI088_temperature_read_over(buf.accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &rawData.temp);
        imu_temp_control(rawData.temp);
    }
    data_adjust(proData.accel, rawData.accel);

    filter(&proData.accel[0], axFilter);
    filter(&proData.accel[1], ayFilter);

    proData.ay = proData.accel[1];
    velocityVerify();
    get_velocity(position.velocity, position._accel, proData.accel);
    get_displace(position.displace, position._velocity, position.velocity);
    position.vy = position.velocity[1];
    position.xy = position.displace[1];
    AHRS_update(quat, 0.001f, rawData.gyro, proData.accel, rawData.mag);
    get_angle(quat,&attitude.yaw, &attitude.pitch, &attitude.rol);
    attitude.yaw_v = rawData.gyro[2];
    attitude.pitch_v = rawData.gyro[0];
    attitude.rol_v = rawData.gyro[1];

    attitude.neg_yaw_v = -attitude.yaw_v;
    attitude.neg_pitch_v = -attitude.pitch_v;
    attitude.neg_rol_v = -attitude.rol_v;

}
void IMU::ITHandle(uint16_t GPIO_Pin) {
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        state.accel_update_flag |= 1u << IMU_DR_SHFITS;
        state.accel_temp_update_flag |= 1u << IMU_DR_SHFITS;
        if(state.imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GRYO_Pin)
    {
        state.gyro_update_flag |= 1u << IMU_DR_SHFITS;
        if(state.imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == IST8310_DRDY_Pin)
    {
        state.mag_update_flag |= 1u << IMU_DR_SHFITS;
        if(state.mag_update_flag &= 1u << IMU_DR_SHFITS)
        {
            state.mag_update_flag &= ~(1u<< IMU_DR_SHFITS);
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
void IMU::imu_cmd_spi_dma(void)
{

    //开启陀螺仪的DMA传输
    if( (state.gyro_update_flag & (1u << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(state.accel_update_flag & (1u << IMU_SPI_SHFITS)) && !(state.accel_temp_update_flag & (1u << IMU_SPI_SHFITS)))
    {
        state.gyro_update_flag &= ~(1u << IMU_DR_SHFITS);
        state.gyro_update_flag |= (1u << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.gyro_dma_tx_buf, (uint32_t)buf.gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    //开启加速度计的DMA传输
    if((state.accel_update_flag & (1u << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
       && !(state.gyro_update_flag & (1u << IMU_SPI_SHFITS)) && !(state.accel_temp_update_flag & (1u << IMU_SPI_SHFITS)))
    {
        state.accel_update_flag &= ~(1u << IMU_DR_SHFITS);
        state.accel_update_flag |= (1u << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.accel_dma_tx_buf, (uint32_t)buf.accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }

    if((state.accel_temp_update_flag & (1u << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
       && !(state.gyro_update_flag & (1u << IMU_SPI_SHFITS)) && !(state.accel_update_flag & (1u<< IMU_SPI_SHFITS)))
    {
        state.accel_temp_update_flag &= ~(1u << IMU_DR_SHFITS);
        state.accel_temp_update_flag |= (1u << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.accel_temp_dma_tx_buf, (uint32_t)buf.accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }

}

void DMA2_Stream0_IRQHandler(void){

    IMU::imu.ITHandle();

}


void IMU::ITHandle() {

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //陀螺仪读取完毕
        if(state.gyro_update_flag & (1u << IMU_SPI_SHFITS))
        {
           // SystemEstimator::systemEstimator.ITEstimatorTrigger(MPU_GYRO_IT);
            state.gyro_update_flag &= ~(1u << IMU_SPI_SHFITS);
            state.gyro_update_flag |= (1u << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

        }

        //accel read over
        //加速度计读取完毕
        if(state.accel_update_flag & (1u << IMU_SPI_SHFITS))
        {
          //  SystemEstimator::systemEstimator.ITEstimatorTrigger(MPU_ACCEL_IT);
            state.accel_update_flag &= ~(1u << IMU_SPI_SHFITS);
            state.accel_update_flag |= (1u << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //温度读取完毕
        if(state.accel_temp_update_flag & (1u << IMU_SPI_SHFITS))
        {
           // SystemEstimator::systemEstimator.ITEstimatorTrigger(MPU_TEMP_IT);
            state.accel_temp_update_flag &= ~(1u << IMU_SPI_SHFITS);
            state.accel_temp_update_flag |= (1u << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if(state.gyro_update_flag & (1u << IMU_UPDATE_SHFITS))
        {
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
void IMU::AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3])
{
#ifdef IMU_USE_MAG
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2],
            accel[0], accel[1], accel[2],
            mag[0],mag[1], mag[2]);
#else
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2],
            accel[0], accel[1], accel[2],
            0,0,0);
#endif
}

void IMU::get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}


/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
void IMU::imu_temp_control(float temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        tempPid.PIDCalc(45.0f,temp);
        if (tempPid.PIDInfo.output < 0.0f)
        {
            tempPid.PIDInfo.output = 0.0f;
        }
        tempPWM = (uint16_t)tempPid.PIDInfo.output;

        IMU_temp_PWM(tempPWM);

    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > 45.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
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
void IMU::record_accel(float _accel[3], float accel[3]){
    _accel[0] = accel[0];
    _accel[1] = accel[1];
    _accel[2] = accel[2];
}
void IMU::get_velocity(float velocity[3],float _accel[3], float accel[3]){
    velocity[0] += ((_accel[0] + accel[0]) / 2 *0.001f);
    velocity[1] += ((_accel[1] + accel[1]) / 2 *0.001f);
    velocity[2] += ((_accel[2] + accel[2]) / 2 *0.001f);
}
void IMU::record_velocity(float _velocity[3], float velocity[3]){
    _velocity[0] = velocity[0];
    _velocity[1] = velocity[1];
    _velocity[2] = velocity[2];
}
void IMU::get_displace(float displace[3], float _velocity[3], float velocity[3]){
    displace[0] += ((_velocity[0] + velocity[0]) / 2 * 0.001f);
    displace[1] += ((_velocity[1] + velocity[1]) / 2 * 0.001f);
    displace[2] += ((_velocity[2] + velocity[2]) / 2 * 0.001f);
}
/**
 * @brief 数据处理
 * @param accel
 * @param _accel
 */
void IMU::offset(){
    float accel[3], gyro[3], temp;
    for (int i = 0; i<1000; i++){
        BMI088_read(gyro, accel, &temp);
        rawData.accel_offset[0] +=accel [0];
        rawData.accel_offset[1] +=accel [1];
        rawData.accel_offset[2] +=accel [2];
    }
    rawData.accel_offset[0] /= 1000;
    rawData.accel_offset[1] /= 1000;
    rawData.accel_offset[2] /= 1000;
}
void IMU::data_adjust(float accel[3], float _accel[3]){
  //  accel[0] = C1 * _accel[0] + C2 * _accel[1] + C3 * _accel[2] + Cx - rawData.accel_offset[0];
  //  accel[1] = C4 * _accel[0] + C5 * _accel[1] + C6 * _accel[2] + Cy - rawData.accel_offset[1];
    accel[0] = _accel[0] - rawData.accel_offset[0];
    accel[1] = _accel[1] - rawData.accel_offset[1];
    accel[2] = _accel[2];
}
void IMU::velocityVerify(){
    static int xcount = 0;
    static int ycount = 0;
    if (abs(proData.accel[0]) < 0.2) {
        xcount++;
    }
    else {xcount = 0;
    }
    if (xcount >= 50){
        position.velocity[0] = 0;
    }
    if (abs(proData.accel[1]) < 0.2) {
        ycount++;
    }
    else {ycount = 0;
    }
    if (ycount >= 50){
        position.velocity[1] = 0;
    }

}
void IMU::filter(float *current, IMU_Filter_t Filter){

    int i,j;
    float sum = 0;
    Filter.current = *current;
    if(Filter.buff_init == 0)
    {
        Filter.history[Filter.index] = Filter.current;
        Filter.index++;
        if(Filter.index >= (SUM_WIN_SIZE-1))
        {
            Filter.buff_init = 1;//index有效范围是0-5，前面放到5，下一个就可以输出
        }
        return ;//当前无法输出，做个特殊标记区分
    }
    else
    {
        Filter.history[Filter.index] = Filter.current;
        Filter.index++;
        if(Filter.index >= SUM_WIN_SIZE)
        {
            Filter.index = 0;//index有效最大5,下次再从0开始循环覆盖
        }

        j = Filter.index;
        for(i = 0;i < SUM_WIN_SIZE;i++)
        {
            //注意i=0的值并不是最早的值
            sum += Filter.history[j] * Filter.factor[i];//注意防止数据溢出
            j++;
            if(j == SUM_WIN_SIZE)
            {
                j = 0;
            }
        }
        *current = sum / Filter.K;
    }
}

