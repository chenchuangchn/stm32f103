//#include "core_cm3.h"
#include "stm32f10x.h"
#include "sys.h"

/*GPIO Number*/
#define PA(x)	x
#define PB(x)	(16 + x)
#define PC(x)	(16 * 2 + x)
#define PD(x)	(16 * 3 + x)
#define PE(x)	(16 * 4 + x)
#define PF(x)	(16 * 5 + x)
#define PG(x)	(16 * 6 + x)

/*RCC APB2 Enable*/
#define AFIOEN		1
#define IOPAEN		(1 << 2)
#define IOPBEN		(1 << 3)
#define IOPCEN		(1 << 4)
#define IOPDEN		(1 << 5)
#define IOPEEN		(1 << 6)
#define IOPFEN		(1 << 7)
#define IOPGEN		(1 << 8)
#define ADC1EN		(1 << 9)
#define ADC2EN		(1 << 10)
#define TIM1EN		(1 << 11)
#define SPI1EN		(1 << 12)
#define TIM8EN		(1 << 13)
#define USART1EN	(1 << 14)
#define ADC3EN		(1 << 15)


/*GPIO mode*/
#define INPUT_MODE				0x00
#define UP10MHZ_MODE			0x01
#define UP2MHZ_MODE				0x02
#define UP50MHZ_MODE			0x03


/*GPIO config*/
#define ANALOG_INPUT			0x00
#define FLOAT_INPUT 			0x01
//high 4 bits for pull up/down low 4 bits for config
#define PULLDOWN_INPUT			0x02
#define PULLUP_INPUT			0x12 

#define PUSHPULL_OUTPUT			0x00
#define OPEN_DRAIN_OUTPUT		0x01
#define AF_PUSHPULL_OUTPUT		0x02
#define AF_OPEN_DRAIN_OUTPUT	0x03

#define NON_EDGE				0x00
#define RISING_EDGE				0x01
#define FALLING_EDGE			0x02
#define BOTH_EDGE				0x03


#define LED0	PA(8)
#define LED1	PD(2)
//#define LED0 PAout(8)	// PA8
//#define LED1 PDout(2)	// PD2	

#define KEY0 	PC(5)
#define KEY1	PA(15)

#define ON 		0
#define OFF		1


void rcc_apb2_enable(unsigned int enbits)
{
	RCC->APB2ENR |= enbits;
}

void rcc_apb2_disable(unsigned int debits)
{
	RCC->APB2ENR &= ~debits;
}


void gpio_config(unsigned int gpionum, unsigned char mode, unsigned char config)

{
	GPIO_TypeDef *gpio_offset = (GPIO_TypeDef *)((unsigned int *)GPIOA + ((gpionum / 16) << 8)); // gpio_off = GPIOA + ((gpionum / 16 *4) << 8)
	unsigned int pinnum = gpionum % 16;
	if(pinnum < 8) {
		gpio_offset->CRL &= ~(0x0f << (pinnum << 2));
		gpio_offset->CRL |= ((config & 0x0f) << 2 | mode) << (pinnum << 2);
	}
	else {
		gpio_offset->CRH &= ~(0x0f << ((pinnum - 8) << 2));
		gpio_offset->CRH |= ((config & 0x0f) << 2 | mode) << ((pinnum - 8) << 2);
	}

	if(0 == mode) {
		gpio_offset->ODR &= ~(1 << pinnum);
		gpio_offset->ODR |= ((config & 0xf0) >> 4) << pinnum;
	}

}

void set_gpio_value(unsigned int gpionum, int value) 
{
	GPIO_TypeDef *gpio_offset = (GPIO_TypeDef *)((unsigned int *)GPIOA + ((gpionum / 16) << 8)); // gpio_off = GPIOA + ((gpionum / 16 *4) << 8)
	unsigned int pinnum = gpionum % 16;
	if(value > 0)	
		gpio_offset->ODR |= 1 << pinnum;
	else 
		gpio_offset->ODR &= ~(1 << pinnum);
}


int get_gpio_value(unsigned int gpionum)
{	
	GPIO_TypeDef *gpio_offset = (GPIO_TypeDef *)((unsigned int *)GPIOA + ((gpionum / 16) << 8)); // gpio_off = GPIOA + ((gpionum / 16 *4) << 8)
	unsigned int pinnum = gpionum % 16;
	
	return ((gpio_offset->IDR >> pinnum) & 0x01);
}

unsigned int get_exti_pendings(void)
{
	unsigned int ret = EXTI->PR;

	return ret;
}

void clear_exti_pendings(int exti_line)
{
	EXTI->PR |= 1 << exti_line;
}

void afio_extint_config(unsigned int gpionum, unsigned int open, unsigned int edge)
{	
	unsigned int port = gpionum / 16;
	unsigned int pinnum = gpionum % 16;
	unsigned int index = pinnum / 4;
	unsigned int shift = pinnum % 4 * 4;

	/*choice input source*/
	AFIO->EXTICR[index] &= ~(0x0f << shift);
	AFIO->EXTICR[index] |= (port << shift);

	/*open extern interrut mask on line x*/
	EXTI->IMR &= ~(1 << pinnum);
	EXTI->IMR |= open << pinnum;

	/*choice the trigger mode*/
	EXTI->RTSR &= ~((edge & 0x01) << pinnum);
	EXTI->RTSR |= ((edge & 0x01) << pinnum);
	EXTI->FTSR &= ~(((edge >> 1) & 0x01) << pinnum);
	EXTI->FTSR |= ((edge >> 1) & 0x01) << pinnum;
}

#if 0
void nvic_init(unsigned char intnum, unsigned char prigroup, 
		unsigned char preempt_priority, unsigned char sub_priority)
{
	if(prigroup < 7) {
		SCB->AIRCR = 0x05fa << 16 | prigroup << 8;
		NVIC->IP[39-16] = (preempt_priority << prigroup) | (sub_priority && ~(0xff << prigroup));
	}
	else {
		/*invalid argument*/
	}
}

void nvic_config(unsigned char intnum, unsigned char enable)
{
	if(enable > 0) {
		NVIC->ISER[intnum / 32] |= 1 << (intnum % 32);
	}
	else {
		//NVIC->ICER[intnum / 32] |= 1 << (intnum % 32);
	}
}
#endif

/********************************************************************/
void led_init(void)
{
	rcc_apb2_enable(IOPDEN);
	gpio_config(LED0, UP50MHZ_MODE, PUSHPULL_OUTPUT);
	gpio_config(LED1, UP50MHZ_MODE, PUSHPULL_OUTPUT);
}

void led_set(int gpionum, int on)
{
	set_gpio_value(gpionum, on);
}

void key_scan_init(void)
{
	rcc_apb2_enable(IOPCEN | IOPAEN);
	gpio_config(KEY0, INPUT_MODE, PULLUP_INPUT);
	gpio_config(KEY1, INPUT_MODE, PULLUP_INPUT);
}

void key_interrupt_init(void)
{
	key_scan_init();
	rcc_apb2_enable(AFIOEN);
	afio_extint_config(KEY0, 1, FALLING_EDGE);
}

#if 1
void EXTI9_5_IRQHandler(void)
{
	static unsigned char onoff = ON;
	led_set(LED1, onoff);
	onoff = !onoff;
	EXTI->PR = 1 << 5;
}
#else
void EXTI9_5_IRQHandler(void)
{			
	//delay_ms(10);   //消抖			 
	//if(KEY0==0)	{
		PAout(8)=!PAout(8);
	//}
 	 EXTI_ClearITPendingBit(EXTI_Line5);    //清除LINE5上的中断标志位  
}
#endif


void mco_init(void)
{
	RCC->CFGR |= (7 << 24);
	rcc_apb2_enable(IOPAEN);
	gpio_config(PA(8), UP50MHZ_MODE, AF_PUSHPULL_OUTPUT);
	rcc_apb2_enable(AFIOEN);
}

//按键初始化函数 
//PA0.15和PC5 设置成输入
void KEY_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC,ENABLE);//使能PORTA,PORTC时钟

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//关闭jtag，使能SWD，可以用SWD模式调试
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;//PA15
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA15
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;//PC5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC5
 
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;//PA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0设置成输入，默认下拉	  
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.0
	
} 


//外部中断初始化函数
void EXTIX_Init(void)
{
 
 	  EXTI_InitTypeDef EXTI_InitStructure;
 	  NVIC_InitTypeDef NVIC_InitStructure;

  	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟
	rcc_apb2_enable(AFIOEN);
	//KEY_Init();//初始化按键对应io模式
	key_scan_init();

    //GPIOC.5 中断线以及中断初始化配置
  	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource5);
  	AFIO->EXTICR[1] |= 2 << 4;
#if 0
  	EXTI_InitStructure.EXTI_Line=EXTI_Line5;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
#endif
	/*Enable EXTI*/
	EXTI->IMR |= 1 << 5;
	/*Set falling trigger edge*/
	EXTI->FTSR |= 1 << 5;
#if 0	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure); 
#endif
	/*Set interrupt priority*/
	NVIC->IP[23] = 0x20;

	/*Enable interrput*/
	NVIC->ISER[0] |= 1 << 23;

}

#if 1
void interrupt_init(void)
{
	/*Set priority group*/
	SCB->AIRCR = 0x05fa0500;
	/*Set interrupt priority*/
	NVIC->IP[23] = 0x20;
	/*Enable AFIO*/
	rcc_apb2_enable(AFIOEN);
	/*Config PC5 to EXTI5*/
	AFIO->EXTICR[1] |= 2 << 4;
	/*Set falling trigger edge*/
	EXTI->FTSR |= 1 << 5;
	/*Enable EXTI*/
	EXTI->IMR |= 1 << 5;
	/*Enable interrput*/
	NVIC->ISER[0] |= 1 << 23;
}
#else

void interrupt_init(void)
{
	/*Set priority group*/
	SCB->AIRCR = 0x05fa0500;
	EXTIX_Init();
}
#endif

void TIM3_IRQHandler(void)
{
	static unsigned char onoff = ON;
	if(TIM3->SR & 1) {
		led_set(LED1, onoff);
		onoff = !onoff;
		TIM3->SR &= ~1;
	}
}

void timer_init(void)
{
	/*Set timer3 interrupt priority*/
	NVIC->IP[29] = 0x40;
	RCC->APB1ENR |= 1 << 1;
	/*set AREP*/
	TIM3->CR1 |= 1 << 7;
	/*Set psc*/
	TIM3->PSC = 7199;
	/*set arr*/
	TIM3->ARR = 4999;
	/*enable irq*/
	TIM3->DIER = 1;
	/*enable cnt*/
	TIM3->CR1 = 1;
	/*Enable interrput*/
	NVIC->ISER[0] |= 1 << 29;
}

int main(void)
{
	//unsigned char sw = 0;
	//clock_reset();
	//clock_init_to_72M();
	//unsigned char extinum = (KEY0 % 16);
	led_init();
	//mco_init();
	//nvic_init(extinum, 4, 3, 0);
	//key_scan_init();
	key_scan_init();
	interrupt_init();
	timer_init();
	//key_scan_init();
	//key_interrupt_init();
	//nvic_config(extinum, 1);
	led_set(LED0, ON);
	led_set(LED1, OFF);
	while(1);
	#if 0
	while(1) {
		if(0 == get_gpio_value(KEY0)) {
			led_set(LED0, OFF);
			led_set(LED1, ON);
		}
		else {
			led_set(LED0, ON);
			led_set(LED1, OFF);
		}
	}
	#endif
	//while(1);
	return 0;
}
