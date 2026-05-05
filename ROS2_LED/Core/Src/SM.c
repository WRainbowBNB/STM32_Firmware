#include "SM.h"

uint8_t rx_buffer[64];
RXBUFFER rx_buf ={
    .buf = rx_buffer,
    .buf_size = sizeof(rx_buffer),
    .old_pos = 0,
    .line_pos = 0,
    .line_temp = {0},
};
FLOAT_UNION float_union;
uint16_t brightness = 500;
uint8_t led_state = 0;
uint16_t sum[32] = {0};
uint8_t tx_buf[128] = {0};
uint8_t key_last_state = 0;
uint8_t key_current_state = 0;
void SM_UART(RXBUFFER *rx_buf)
{
  //不让搁default后面直接定义就搁这儿定义
  uint8_t check_pos = 0;
  //获取DMA剩余空间
  uint16_t remaining_space = __HAL_DMA_GET_COUNTER(huart3.hdmarx);
  //计算当前数据下标
  uint8_t dma_write_ptr = (rx_buf->buf_size - remaining_space) % rx_buf->buf_size;
  //写指针变化，说明有新数据到达
  if(dma_write_ptr != rx_buf->old_pos){
    //处理新数据
    while(dma_write_ptr != rx_buf->old_pos){
        //获取当前数据字节
        uint8_t byte = rx_buf->buf[rx_buf->old_pos];
        rx_buf->line_temp[rx_buf->line_pos] = byte;
        //根据当前行数据位置进行状态机处理
        switch (rx_buf->line_pos){  
        //帧头1
        case 0:
            if(rx_buf->line_temp[rx_buf->line_pos] == HEADER1){
                sum[rx_buf->line_pos] = byte;
                rx_buf->line_pos = 1;
            }else{
                rx_buf->line_pos = 0;
                
            }
            break;
        //帧头2
        case 1:
            if(rx_buf->line_temp[rx_buf->line_pos] == HEADER2){
                sum[rx_buf->line_pos] = sum[0] + byte;
                rx_buf->line_pos = 2;
            }else if(rx_buf->line_temp[rx_buf->line_pos] == HEADER1){
                sum[0] = byte;
                rx_buf->line_pos = 1;
            }else{
                rx_buf->line_pos = 0;
                sum[rx_buf->line_pos] = 0;
            }
            break;
        //Command
        case 2:
            if(rx_buf->line_temp[rx_buf->line_pos] == 0x01 || rx_buf->line_temp[rx_buf->line_pos] == 0x02){
                sum[rx_buf->line_pos] = sum[1] + byte;
                rx_buf->line_pos = 3;
            }else{
                rx_buf->line_pos = 0;
                sum[rx_buf->line_pos] = 0;
            }
            break;
        //后续处理
        default:
            //根据Command判断校验位长度
            if(rx_buf->line_temp[2] == 0x01){
                check_pos = 4;
            }else if(rx_buf->line_temp[2] == 0x02){
                check_pos = 7;
            }
            //没到校验位位置，继续接收和累加数据
            if(rx_buf->line_pos < check_pos){
                sum[rx_buf->line_pos] = sum[rx_buf->line_pos - 1] + byte;
                rx_buf->line_pos++;
                break;
            }else{
                //校验位位置，校验数据（试一下空间换时间）
                if(rx_buf->line_temp[rx_buf->line_pos] == (sum[check_pos - 1] & 0xFF)){
                    //数据校验成功，处理数据
                    if(rx_buf->line_temp[2] == 0x01){
                        switch(rx_buf->line_temp[3]){
                            case 0x00:
                                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                                led_state = 0;
                                break;
                            case 0x01:
                                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, brightness);
                                led_state = 1;
                                break;
                            case 0x02:
                                if(led_state == 0){
                                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, brightness);
                                }else{
                                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                                }
                                break;
                            default:
                                break;
                        }
                    }else if(rx_buf->line_temp[2] == 0x02){
                        //将4字节数据转换为float类型
                        float_union.u4[0] = rx_buf->line_temp[3];
                        float_union.u4[1] = rx_buf->line_temp[4];
                        float_union.u4[2] = rx_buf->line_temp[5];
                        float_union.u4[3] = rx_buf->line_temp[6];
                        brightness = (uint16_t)(float_union.f * 500);
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, brightness);
                    }
                }
                rx_buf->line_pos = 0;
                break;
            }
        }
        rx_buf->old_pos++;
        //让读指针指向下一个数据位置
        if(rx_buf->old_pos >= rx_buf->buf_size){
            rx_buf->old_pos = 0;
        }
    }
  }
}

void stm32_to_ros2(){
    //发送数据到ROS2
    tx_buf[0] = HEADER1;
    tx_buf[1] = HEADER2;
    if(Read_Key() == 1){
        tx_buf[2] = 0x01; // 引脚高电平
    }else{
        tx_buf[2] = 0x00; // 引脚低电平
    }
    tx_buf[3] = tx_buf[0] + tx_buf[1] + tx_buf[2]; // Checksum
    HAL_UART_Transmit_DMA(&huart3, tx_buf, 4);
}

uint8_t Read_Key(void){
    uint8_t key_state = HAL_GPIO_ReadPin(Toggle_GPIO_Port, Toggle_Pin);
    return key_state;
}