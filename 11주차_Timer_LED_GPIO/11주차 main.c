#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"

int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};
uint16_t value;

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void TIM_Configure(void);
void NVIC_Configure(void);
void TIM2_IRQHandler(void);
void delay(void);

//---------------------------------------------------------------------------------------------------

void RCC_Configure(void) // stm32f10x_rcc.h ????
{
	// TODO: Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'
        
	/* LCD_CS, LCD_RS, LCD_WR, LCD_RD  */
        /* PC6,    PD13    PD14    PD15*/
        /* lcd.c do RCC Configuration in LCD_Configuration()
        
        /* LED 1, 2 */
        /* PD2, 3     */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

        
        /* TIM2_CH2 */
        /* PB0 */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
        
	/* Alternate Function IO clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configure(void) // stm32f10x_gpio.h ????
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // TODO: Initialize the GPIO pins using the structure 'GPIO_InitTypeDef' and the function 'GPIO_Init'
    
    /* LED 1, 2 */
    /* PD2, 3     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    /* TIM3 */
    /* PB0 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
   
}

void TIM_Configure(void) {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  
  //int prescale = (uint16_t)(SystemCoreClock / 10000);
  
  TIM_TimeBaseStructure.TIM_Period = 10000;
  TIM_TimeBaseStructure.TIM_Prescaler = 7200;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
  
  
  TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1500; // us
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
   
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}


void NVIC_Configure(void) { // misc.h
  
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // TODO: fill the arg you want
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
    // TIM2_IRQn
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
   
}

void delay() {
  int i;
  for(i = 0; i < 5000000; ++i) {}
}

void change_pulse(uint16_t pulse){
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = pulse; // us
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
}

int led2 = 0;
int cnt = 0;
uint16_t sub[2] = {1000, 2000};
int isOnOff = 0;

void TIM2_IRQHandler() {
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET ) {
    if(isOnOff) {
      led2++;
      GPIO_SetBits(GPIOD, GPIO_Pin_2);
      if(led2 == 5) {
        led2 = 0;
        GPIO_SetBits(GPIOD, GPIO_Pin_3);
      }
      delay();
      GPIO_ResetBits(GPIOD, GPIO_Pin_2);
      if(led2 == 0) GPIO_ResetBits(GPIOD, GPIO_Pin_3);
    }
    
    change_pulse(sub[cnt++]);
    if (cnt == 2)
      cnt = 0;
    
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}

int main(void)
{

    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    TIM_Configure();
    NVIC_Configure();

    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
    LCD_Clear(WHITE);

    char msg1[] = "THU_Team03";
    LCD_ShowString(0, 0, msg1, color[2], color[0] );
    
    char msgOn[] = "ON ";
    char msgOff[] = "OFF";
    
    char msgBtn[] = "BTN";
    LCD_DrawRectangle(16, 64, 64, 112);
    LCD_ShowString(32, 76, msgBtn, color[3], color[0] );
    
    uint16_t x, y;
    
    while (1) {
      if(isOnOff) {
        LCD_ShowString(0, 32, msgOn, color[3], color[0] );
      }
      else {
        LCD_ShowString(0, 32, msgOff, color[3], color[0] );
      }
      
      Touch_GetXY(&x, &y, 1);
      Convert_Pos(x, y, &x, &y);
      if(x >= 16 && x <= 64 && y >= 64 && y <= 112) {
        isOnOff = !isOnOff;
        led2 = 0;
      }
    }
        
    return 0;
}
