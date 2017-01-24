//#include "core_cm3.h"
#include "stm32f10x.h"

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

/********************************************************************/
void led_init(void)
{
	rcc_apb2_enable(IOPDEN);
	//gpio_config(LED0, UP50MHZ_MODE, PUSHPULL_OUTPUT);
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

void EXTI9_5_IRQHandler(void)
{
	if(0 == get_gpio_value(KEY0)) {
		led_set(LED0, ON);
		led_set(LED1, OFF);
	}
	else {
		led_set(LED0, ON);
		led_set(LED1, OFF);
	}
}

void MYRCC_DeInit(void)
{	
 	RCC->APB1RSTR = 0x00000000;//复位结束			 
	RCC->APB2RSTR = 0x00000000; 
	  
  	RCC->AHBENR = 0x00000014;  //睡眠模式闪存和SRAM时钟使能.其他关闭.	  
  	RCC->APB2ENR = 0x00000000; //外设时钟关闭.			   
  	RCC->APB1ENR = 0x00000000;   
	RCC->CR |= 0x00000001;     //使能内部高速时钟HSION	 															 
	RCC->CFGR &= 0xF8FF0000;   //复位SW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]					 
	RCC->CR &= 0xFEF6FFFF;     //复位HSEON,CSSON,PLLON
	RCC->CR &= 0xFFFBFFFF;     //复位HSEBYP	   	  
	RCC->CFGR &= 0xFF80FFFF;   //复位PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
	RCC->CIR = 0x00000000;     //关闭所有中断		 
	//配置向量表				  
#ifdef  VECT_TAB_RAM
	//MY_NVIC_SetVectorTable(0x20000000, 0x0);
#else   
	//MY_NVIC_SetVectorTable(0x08000000,0x0);
#endif
}

void Stm32_Clock_Init(u8 PLL)
{
	unsigned char temp=0;   
	MYRCC_DeInit();		  //复位并配置向量表
 	RCC->CR|=0x00010000;  //外部高速时钟使能HSEON
	while(!(RCC->CR>>17));//等待外部时钟就绪
	RCC->CFGR=0X00000400; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
	PLL-=2;//抵消2个单位
	RCC->CFGR|=PLL<<18;   //设置PLL值 2~16
	RCC->CFGR|=1<<16;	  //PLLSRC ON 
	FLASH->ACR|=0x32;	  //FLASH 2个延时周期

	RCC->CR|=0x01000000;  //PLLON
	while(!(RCC->CR>>25));//等待PLL锁定
	RCC->CFGR|=0x00000002;//PLL作为系统时钟	 
	while(temp!=0x02)     //等待PLL作为系统时钟设置成功
	{   
		temp=RCC->CFGR>>2;
		temp&=0x03;
	}    
}	

void mco_init(void)
{
	RCC->CFGR |= (7 << 24);
	rcc_apb2_enable(IOPAEN);
	gpio_config(PA(8), UP50MHZ_MODE, AF_PUSHPULL_OUTPUT);
	rcc_apb2_enable(AFIOEN);
}

int main(void)
{
	//unsigned char sw = 0;
	//clock_reset();
	//clock_init_to_72M();
	//unsigned char extinum = (KEY0 % 16);
	//Stm32_Clock_Init(9);
	led_init();
	mco_init();
	//nvic_init(extinum, 4, 3, 0);
	key_scan_init();
	//key_interrupt_init();
	//nvic_config(extinum, 1);
	//led_set(LED0, OFF);
	led_set(LED1, OFF);
	//while(1);
	//#if 0
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
	//#endif
	//while(1);
	return 0;
}
