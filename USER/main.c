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



#define LED0	PA(8)
#define LED1	PD(2)

#define KEY0 	PC(5)
#define KEY1	PA(15)

#define ON 		0
#define OFF		1


void rcc_apb2_enable(int enbits)
{
	RCC->APB2ENR |= enbits;
}

void rcc_apb2_disable(int debits)
{
	RCC->APB2ENR &= ~debits;
}


void gpio_config(int gpionum, unsigned char mode, unsigned char config)

{
	GPIO_TypeDef *gpio_offset = (GPIO_TypeDef *)((unsigned int *)GPIOA + ((gpionum / 16) << 8)); // gpio_off = GPIOA + ((gpionum / 16 *4) << 8)
	int pinnum = gpionum % 16;
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

	/*if(0 == mode) {
		if((config & 0xf0) >> 4)
			gpio_offset->ODR |= (1 << pinnum);
		else 
			gpio_offset->ODR &= ~(1 << pinnum);
	}*/
}

void set_gpio_value(int gpionum, int value) 
{
	GPIO_TypeDef *gpio_offset = (GPIO_TypeDef *)((unsigned int *)GPIOA + ((gpionum / 16) << 8)); // gpio_off = GPIOA + ((gpionum / 16 *4) << 8)
	int pinnum = gpionum % 16;
	if(value > 0)	
		gpio_offset->ODR |= 1 << pinnum;
	else 
		gpio_offset->ODR &= ~(1 << pinnum);
}


int get_gpio_value(int gpionum)
{	
	GPIO_TypeDef *gpio_offset = (GPIO_TypeDef *)((unsigned int *)GPIOA + ((gpionum / 16) << 8)); // gpio_off = GPIOA + ((gpionum / 16 *4) << 8)
	int pinnum = gpionum % 16;
	
	return ((gpio_offset->IDR >> pinnum) & 0x01);
}

void led_init(void)
{
	rcc_apb2_enable(IOPAEN | IOPDEN);
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
	rcc_apb2_enable(IOPCEN | IOPAEN);
	gpio_config(KEY0, INPUT_MODE, FLOAT_INPUT);
	gpio_config(KEY1, INPUT_MODE, FLOAT_INPUT);
}


int main(void)
{
	unsigned char sw = 0;
	led_init();
	key_scan_init();
	//key_interrupt_init();

	
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
