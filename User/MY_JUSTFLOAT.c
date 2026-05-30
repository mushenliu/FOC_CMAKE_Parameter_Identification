#include "MY_JUSTFLOAT.h"

void JUSTFLOAT_Init(void) {
    jf_var_cnt = 0;
    for (int i=0;i<JUSTFLOAT_MAX_VAR;i++) {
        jf_var_list[i] = NULL;
    }
}

void JUSTFLOAT_BindUart(UART_HandleTypeDef *huart) {
    jf_huart = huart;
}

void JUSTFLOAT_AddData(float *Data) {
    if (jf_var_cnt < JUSTFLOAT_MAX_VAR){
        jf_var_list[jf_var_cnt] = Data;
    }
    jf_var_cnt++;
}

void JUSTFLOAT_SendData(void) {
    uint8_t i;
    for (i = 0; i < jf_var_cnt; i++)
    {
        jf_tx_buf[i] = *(uint32_t *)jf_var_list[i];
    }
    jf_tx_buf[jf_var_cnt] = JUSTFLOAT_FRAME_TAIL;
    HAL_UART_Transmit_DMA(jf_huart,(uint8_t *)jf_tx_buf,(jf_var_cnt + 1) * sizeof(uint32_t));
}