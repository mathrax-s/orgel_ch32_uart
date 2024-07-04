
#include "debug.h"
#include "pitches.h"
#include "wavetable1.h"
#include "wavetable2.h"

// PROTOTYPE
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USARTx_CFG(void);

/* CH1CVR register Definition */
#define TIM1_CH1CVR_ADDRESS 0x40012C34

/* Private variables */
#define OSCILLATOR_COUNT 32
#define CLIP 1016

uint32_t increments_pot1[OSCILLATOR_COUNT];
uint32_t phase_accu_pot1[OSCILLATOR_COUNT];
uint32_t envelope_positions_envpot1[OSCILLATOR_COUNT];
uint8_t next_osc1 = 0;

uint32_t increments_pot2[OSCILLATOR_COUNT];
uint32_t phase_accu_pot2[OSCILLATOR_COUNT];
uint32_t envelope_positions_envpot2[OSCILLATOR_COUNT];
uint8_t next_osc2 = 0;

const uint32_t sizeof_wt_sustain_pot1 = ((uint32_t)sizeof(wt_sustain1) << POT1);
const uint32_t sizeof_wt_attack_pot1 = ((uint32_t)sizeof(wt_attak1) << POT1);
const uint32_t sizeof_wt_pot1 = ((uint32_t)sizeof(wt_attak1) << POT1) + ((uint32_t)sizeof(wt_sustain1) << POT1);
const uint32_t sizeof_envelope_table_envpot1 = ((uint32_t)sizeof(envelope_table1) << ENVPOT1);

const uint32_t sizeof_wt_sustain_pot2 = ((uint32_t)sizeof(wt_sustain2) << POT2);
const uint32_t sizeof_wt_attack_pot2 = ((uint32_t)sizeof(wt_attak2) << POT2);
const uint32_t sizeof_wt_pot2 = ((uint32_t)sizeof(wt_attak2) << POT2) + ((uint32_t)sizeof(wt_sustain2) << POT2);
const uint32_t sizeof_envelope_table_envpot2 = ((uint32_t)sizeof(envelope_table2) << ENVPOT2);

uint8_t rcvData;
uint8_t rxCounter;
uint8_t lastBuffer[2];
uint8_t receiveBuffer[300] = {0};
uint8_t sensorData;

uint32_t wave[1] = {0};
float average;

#define TICKS_LIMIT 125
uint16_t ticks = 0;
uint16_t time = 0;
uint16_t event_index = 0;
uint32_t timeout =0;
uint8_t soundoff = 0;

const uint8_t startup_scale[] = {
    NOTE_Gb6, NOTE_Ab6, NOTE_Bb6, NOTE_Db7};

// for MATHRAX
const uint8_t scale[] = {
    // NOTE_Db3,
    // NOTE_Eb3,
    // NOTE_Gb3,
    // NOTE_Ab3,
    // NOTE_Bb3,
    NOTE_Db4,
    NOTE_Eb4,
    NOTE_Gb4,
    NOTE_Ab4,
    NOTE_Bb4,
    NOTE_Db5,
    NOTE_Eb5,
    NOTE_Gb5,
    NOTE_Ab5,
    NOTE_Bb5,
    NOTE_Db6,
    NOTE_Eb6,
    NOTE_Gb6,
    NOTE_Ab6,
    NOTE_Bb6,
    NOTE_Db7,
    NOTE_Eb7,
    NOTE_Gb7,
    NOTE_Ab7,
    NOTE_Bb7,
    NOTE_Db8,
    NOTE_Eb8,
    NOTE_Gb8,
    NOTE_Ab8,
    NOTE_Bb8,
    // NOTE_Db9,
    // NOTE_Eb9,
    // NOTE_Gb9,
    // NOTE_Ab9,
    // NOTE_Bb9,
    // NOTE_Db10,
    // NOTE_Eb10,
    // NOTE_Gb10,
    // NOTE_Ab10,
    // NOTE_Bb10,

};

/*********************************************************************
 * @fn      mapRange
 */
float mapRange(float value, float a, float b, float c, float d)
{
    value = (value - a) / (b - a);
    return c + value * (d - c);
}
/*********************************************************************
 * @fn      TIM1_PWMOut_Init
 */
void TIM1_PWMOut_Init(u16 arr, u16 psc, u16 ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
}

/*********************************************************************
 * @fn      TIM1_DMA_Init
 */
void TIM1_DMA_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);

    DMA_Cmd(DMA_CHx, ENABLE);
}
/********************************************************************
 * @fn      TIM2_Init
 */
void TIM2_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // TIM2クロックを有効化する

    TIM_TimeBaseInitStructure.TIM_Period = arr;                       // カウント上限回数を設定する
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;                    // クロックの分周に使用するプリスケーラの値を指定
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;       // クロック分周係数
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Down; // TIMカウントモード, ダウンカウントモード
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);               // 設定したパラメータにしたがってTIM2を初期化するコード

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // TIM2割り込みを有効化し, 割り込み更新を許可する
    TIM_ARRPreloadConfig(TIM2, ENABLE);        // TIM2のプリロード機能を有効化
    TIM_Cmd(TIM2, ENABLE);                     // TIM2を有効にする
}

/********************************************************************
 * @fn      Interrupt_Init
 */
void Interrupt_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/*********************************************************************
 * @fn      TIM2_IRQHandler
 */
void TIM2_IRQHandler(void)
{
    TIM_ClearFlag(TIM2, TIM_FLAG_Update); // 割り込みフラグリセット
    /* update counter */
    int32_t value = 0;
    int32_t value1 = 0;
    int32_t value2 = 0;

    for (uint8_t osc = 0; osc < OSCILLATOR_COUNT; osc++)
    {
        phase_accu_pot1[osc] += increments_pot1[osc];

        while (phase_accu_pot1[osc] >= (sizeof_wt_attack_pot1 + sizeof_wt_sustain_pot1))
        {
            phase_accu_pot1[osc] -= sizeof_wt_sustain_pot1;
        }
        uint16_t phase_accu = 0;
        int8_t accu = 0;
        if (phase_accu_pot1[osc] < sizeof_wt_attack_pot1)
        {
            phase_accu = (phase_accu_pot1[osc] >> POT1);
            accu = (wt_attak1[phase_accu]);
        }
        else
        {
            phase_accu = (phase_accu_pot1[osc] - sizeof_wt_attack_pot1) >> POT1;
            accu = (wt_sustain1[phase_accu]);
        }
        value1 += envelope_table1[envelope_positions_envpot1[osc] >> ENVPOT1] * accu;

        if (phase_accu_pot1[osc] >= sizeof_wt_attack_pot1 &&
            envelope_positions_envpot1[osc] < sizeof_envelope_table_envpot1 - 1)
        {
            ++envelope_positions_envpot1[osc];
        }
    }
    value1 >>= 8; // envelope_table resolution

    for (uint8_t osc = 0; osc < OSCILLATOR_COUNT; osc++)
    {
        phase_accu_pot2[osc] += increments_pot2[osc];

        while (phase_accu_pot2[osc] >= (sizeof_wt_attack_pot2 + sizeof_wt_sustain_pot2))
        {
            phase_accu_pot2[osc] -= sizeof_wt_sustain_pot2;
        }
        uint16_t phase_accu = 0;
        int8_t accu = 0;
        if (phase_accu_pot2[osc] < sizeof_wt_attack_pot2)
        {
            phase_accu = (phase_accu_pot2[osc] >> POT2);
            accu = (wt_attak2[phase_accu]);
        }
        else
        {
            phase_accu = (phase_accu_pot2[osc] - sizeof_wt_attack_pot2) >> POT2;
            accu = (wt_sustain2[phase_accu]);
        }
        value2 += envelope_table2[envelope_positions_envpot2[osc] >> ENVPOT2] * accu;

        if (phase_accu_pot2[osc] >= sizeof_wt_attack_pot2 &&
            envelope_positions_envpot2[osc] < sizeof_envelope_table_envpot2 - 1)
        {
            ++envelope_positions_envpot2[osc];
        }
    }
    value2 >>= 8; // envelope_table resolution

    value = (value1 + value2) >> 1;

    if (value > CLIP)
    {
        value = CLIP;
    }
    else if (value < -CLIP)
    {
        value = -CLIP;
    }
    wave[0] = (1024 + value) & 2047;

    ++ticks;
    if (ticks >= TICKS_LIMIT)
    {
        ticks = 0;
        time += 1;
    }
    if(timeout > 300000){
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
        __WFI();
    }else{
        timeout++;
    }
}

/*********************************************************************
 * @fn      playSound
 */
uint8_t my_sound_num = 0;
uint8_t last_sound_num = 0;

void playSound(uint8_t note)
{
    // while (time > 15 * 2)
    // {
    time = 0;
    int d = 0;

    increments_pot1[next_osc1] = scale_table[note] + (6.586368) * 0;
    phase_accu_pot1[next_osc1] = 0;
    envelope_positions_envpot1[next_osc1] = 0;
    next_osc1++;
    if (next_osc1 >= OSCILLATOR_COUNT)
    {
        next_osc1 = 0;
    }
    d = 1; // rand()%4;
    increments_pot1[next_osc1] = scale_table[note] + (6.586368) * d;
    phase_accu_pot1[next_osc1] = 0;
    envelope_positions_envpot1[next_osc1] = 0;
    next_osc1++;
    if (next_osc1 >= OSCILLATOR_COUNT)
    {
        next_osc1 = 0;
    }

    increments_pot2[next_osc2] = scale_table[note] + (6.586368) * 0;
    phase_accu_pot2[next_osc2] = 0;
    envelope_positions_envpot2[next_osc2] = 0;
    next_osc2++;
    if (next_osc2 >= OSCILLATOR_COUNT)
    {
        next_osc2 = 0;
    }
    d = 2; // rand()%4;
    increments_pot2[next_osc2] = scale_table[note] + (6.586368) * d;
    phase_accu_pot2[next_osc2] = 0;
    envelope_positions_envpot2[next_osc2] = 0;
    next_osc2++;
    if (next_osc2 >= OSCILLATOR_COUNT)
    {
        next_osc2 = 0;
    }
    // }
}

void GPIO_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*********************************************************************
 * @fn      main
 */
int main(void)
{
    SystemCoreClockUpdate();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    TIM1_PWMOut_Init(2047, 1 - 1, 1024);
    TIM1_DMA_Init(DMA1_Channel5, (u32)TIM1_CH1CVR_ADDRESS, (u32)wave, 1);

    TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    for (uint8_t osc = 0; osc < OSCILLATOR_COUNT; ++osc)
    {
        increments_pot1[osc] = 0;
        phase_accu_pot1[osc] = 0;
        envelope_positions_envpot1[osc] = 0;
        increments_pot2[osc] = 0;
        phase_accu_pot2[osc] = 0;
        envelope_positions_envpot2[osc] = 0;
    }
    USARTx_CFG();

    TIM2_Init(4608 - 1, 1 - 1); // 31250
    Interrupt_Init();           // 割り込み初期化

    Delay_Init();

    GPIO_INIT();
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);

    for (int i = 0; i < 4; i++)
    {
        GPIO_WriteBit(GPIOA, GPIO_Pin_3, 0);
        Delay_Ms(125);
        GPIO_WriteBit(GPIOA, GPIO_Pin_3, 1);
        Delay_Ms(125);
    }

    for (int i = 0; i < 4; i++)
    {
        playSound(startup_scale[i]);
        Delay_Ms(125);
    }

    while (1)
    {
        my_sound_num = (uint8_t)mapRange(sensorData, 0, 100, 0, sizeof(scale) - 1);
        if (last_sound_num != my_sound_num)
        {
            playSound(scale[my_sound_num]);
        }
        last_sound_num = my_sound_num;
    }
}

/*********************************************************************
 * @fn      USARTx_CFG
 */
void USARTx_CFG(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    /* CH32V203K8T6 .. USART1 TX-->A.9  RX-->A.10 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure); // A_PORT

    USART_InitStructure.USART_BaudRate = 57600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART1, ENABLE);
}

/*********************************************************************
 * @fn      USART1_IRQHandler
 *
 * @brief   This function handles USART1 global interrupt request.
 *
 * @return  none
 */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        rcvData = USART_ReceiveData(USART1);
        if (rcvData == 0x0A && lastBuffer[0] == 0x0D)
        {
            rxCounter = 0;
            GPIO_WriteBit(GPIOA, GPIO_Pin_3, 0);
        }
        else if (rcvData == 0xFF && lastBuffer[0] == 0xFF && lastBuffer[1] == 0x00)
        {
            sensorData = (receiveBuffer[0]) | (receiveBuffer[1] << 8);

            GPIO_WriteBit(GPIOA, GPIO_Pin_3, 1);
            timeout = 0;
            GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
        }
        else
        {
            receiveBuffer[rxCounter] = rcvData;
            rxCounter++;
        }
        lastBuffer[1] = lastBuffer[0];
        lastBuffer[0] = rcvData;
    }
}