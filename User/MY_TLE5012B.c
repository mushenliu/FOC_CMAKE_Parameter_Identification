#include "MY_TLE5012B.h"
#include "spi.h"

uint16_t TLE5012B_SPI_Read(uint16_t Order_Word) {
    uint16_t ReceiveData = 0;
    SPI_CS_ENABLE;
    HAL_SPI_Transmit(&hspi1, (uint8_t *) &Order_Word, 1,HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, (uint8_t *) &ReceiveData, 1,HAL_MAX_DELAY);
    ReceiveData = ReceiveData & 0x7FFF; //舍弃最高位状态位
    SPI_CS_DISABLE;
    return ReceiveData;
}

uint16_t TLE5012B_SPI_Write(uint16_t Order_Word, uint16_t WriteData) {
    SPI_CS_ENABLE;
    uint16_t Data[2] = {Order_Word, WriteData};
    HAL_SPI_Transmit(&hspi1, (uint8_t *) Data, 2, HAL_MAX_DELAY);
    HAL_Delay(1);
    SPI_CS_DISABLE;
    HAL_Delay(10);
}

double TLE5012B_Angle(void) {
    uint16_t Data = TLE5012B_SPI_Read(CMD_READ_ANGLE_VALUE);
    double Angle = Data * 360.0 / 32768;
    return Angle;
}

double TLE5012B_Speed(void) {
    uint16_t raw_data = TLE5012B_SPI_Read(CMD_READ_SPEED_VALUE);
    int16_t speed_counts = 0;
    if (raw_data & 0x4000) {
        //符号位为1，负数
        speed_counts = (int16_t) (raw_data | 0x8000);
    } else {
        //符号位为0，正数
        speed_counts = (int16_t) raw_data;
    }

    float delta_angle = (speed_counts / 32768.0) * 360.0;
    float omega_rad_s = delta_angle / 42.7e-6;
    return (double)omega_rad_s;
}
