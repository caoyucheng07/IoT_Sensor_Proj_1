#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#define IR_RX_PIN      GPIO_Pin_0      
#define BTN_L1_PIN     GPIO_Pin_1      // learn slot 0
#define BTN_L2_PIN     GPIO_Pin_3      // learn slot 1

#define MAX_PULSES     1200
#define NUM_SLOTS      2

volatile uint32_t ir_buffer[NUM_SLOTS][MAX_PULSES];
volatile uint16_t ir_len[NUM_SLOTS];
volatile uint8_t  ir_start_level[NUM_SLOTS];


volatile uint32_t tim2_ovf = 0;

void TIM2_IRQHandler(void){
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        tim2_ovf++;
    }
}

static inline uint32_t micros32(void){
    uint32_t o1 = tim2_ovf;
    uint32_t c  = (uint32_t)TIM_GetCounter(TIM2);
    uint32_t o2 = tim2_ovf;
    if (o2 != o1) {
        c = (uint32_t)TIM_GetCounter(TIM2);
        o1 = o2;
    }
    return (o1 << 16) | c;
}

static inline void micros32_reset(void){
    tim2_ovf = 0;
    TIM_SetCounter(TIM2, 0);
}

void TIM2_Init(void){
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    tim.TIM_Prescaler     = 72 - 1;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    tim.TIM_Period        = 0xFFFF;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM2, ENABLE);
}

void delay_us(uint32_t us){
    uint32_t start = micros32();
    while ((uint32_t)(micros32() - start) < us);
}

#define IR_ON()   TIM_Cmd(TIM1, ENABLE)
#define IR_OFF()  TIM_Cmd(TIM1, DISABLE)

void IR_PWM_Init_38k(void){
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_8;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    tim.TIM_Prescaler     = 0;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    tim.TIM_Period        = 1999;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tim);

    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 1999 / 3;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &oc);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    IR_OFF();
}


void GPIO_Init_All(void){
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    gpio.GPIO_Pin   = IR_RX_PIN | BTN_L1_PIN | BTN_L2_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
}


static volatile char rxLine[32];
static volatile uint8_t rxIdx = 0;
static volatile uint8_t cmdReady = 0;

void USART1_Init_115200(void){
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef us;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    us.USART_BaudRate = 115200;
    us.USART_WordLength = USART_WordLength_8b;
    us.USART_StopBits = USART_StopBits_1;
    us.USART_Parity = USART_Parity_No;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &us);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART1, ENABLE);
}

void USART1_SendStr(const char* s){
    while(*s){
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *s++);
    }
}

void USART1_IRQHandler(void){
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
        char c = (char)USART_ReceiveData(USART1);
        if(c == '\r') return;

        if(c == '\n'){
            rxLine[rxIdx] = 0;
            rxIdx = 0;
            cmdReady = 1;
        } else {
            if(rxIdx < (sizeof(rxLine)-1)) rxLine[rxIdx++] = c;
            else rxIdx = 0;
        }
    }
}


static void ClearBuffers(void){
    uint16_t s;
    uint16_t i;

    for (s = 0; s < NUM_SLOTS; s++){
        ir_len[s] = 0;
        ir_start_level[s] = 1;
        for (i = 0; i < MAX_PULSES; i++){
            ir_buffer[s][i] = 0;
        }
    }
}


#define QUIET_END_US   150000u

void IR_Learn(uint8_t slot){
    uint8_t last, now;
    uint32_t t_last_change, t_now;

    ir_len[slot] = 0;
    ir_start_level[slot] = GPIO_ReadInputDataBit(GPIOA, IR_RX_PIN);
    last = ir_start_level[slot];

    micros32_reset();
    t_last_change = micros32();

    while (ir_len[slot] < MAX_PULSES){
        now = GPIO_ReadInputDataBit(GPIOA, IR_RX_PIN);

        if (now != last){
            t_now = micros32();
            ir_buffer[slot][ir_len[slot]++] = (uint32_t)(t_now - t_last_change);
            t_last_change = t_now;
            last = now;
        }

        if ((uint32_t)(micros32() - t_last_change) > QUIET_END_US && ir_len[slot] > 10){
            break;
        }
    }

    if (ir_len[slot] > 0) USART1_SendStr("LEARN OK\n");
    else                  USART1_SendStr("LEARN FAIL\n");
}

void IR_Send(uint8_t slot){
    uint16_t i;
    uint8_t level = ir_start_level[slot];

    for (i = 0; i < ir_len[slot]; i++){
        if (level == 0){
            IR_ON();
            delay_us(ir_buffer[slot][i]);
            IR_OFF();
        } else {
            IR_OFF();
            delay_us(ir_buffer[slot][i]);
        }
        level ^= 1;
    }
    IR_OFF();
}

void IR_Send_Repeat(uint8_t slot, int times){
    int r;
    for(r = 0; r < times; r++){
        IR_Send(slot);
        delay_us(40000);
    }
}


int main(void){
    TIM2_Init();
    IR_PWM_Init_38k();
    GPIO_Init_All();
    USART1_Init_115200();

    ClearBuffers();

    delay_us(50000);
    USART1_SendStr("STM32 online\n");

    while (1){
        if (!GPIO_ReadInputDataBit(GPIOA, BTN_L1_PIN)){
            delay_us(20000);
            USART1_SendStr("LEARN0...\n");
            IR_Learn(0);
            while (!GPIO_ReadInputDataBit(GPIOA, BTN_L1_PIN));
        }

        if (!GPIO_ReadInputDataBit(GPIOA, BTN_L2_PIN)){
            delay_us(20000);
            USART1_SendStr("LEARN1...\n");
            IR_Learn(1);
            while (!GPIO_ReadInputDataBit(GPIOA, BTN_L2_PIN));
        }

        if(cmdReady){
            cmdReady = 0;

            if(rxLine[0]=='S' && rxLine[1]=='E' && rxLine[2]=='N' && rxLine[3]=='D'){
                if(rxLine[4]=='0'){
                    USART1_SendStr("SEND0\n");
                    IR_Send_Repeat(0, 4);
                } else if(rxLine[4]=='1'){
                    USART1_SendStr("SEND1\n");
                    IR_Send_Repeat(1, 4);
                } else {
                    USART1_SendStr("BAD SEND\n");
                }
            } else {
                USART1_SendStr("UNKNOWN CMD\n");
            }
        }
    }
}
