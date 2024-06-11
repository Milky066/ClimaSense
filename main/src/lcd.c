#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"

typedef struct _lcd_info
{
	uint16_t lcd_id;
	int16_t lcd_wid;
	int16_t lcd_heg;
}lcd_info;

volatile int8_t *spicsPort, *spicdPort, *spimisoPort , *spimosiPort, *spiclkPort;
int8_t  spicsPinSet, spicdPinSet  ,spimisoPinSet , spimosiPinSet , spiclkPinSet,
        spicsPinUnset, spicdPinUnset, spimisoPinUnset,  spimosiPinUnset,spiclkPinUnset,
        _reset,_led;
bool _hw_spi;
uint16_t WIDTH,HEIGHT,width, height, rotation,lcd_driver,lcd_model;
lcd_info current_lcd_info[] = 
						 { 
							 0x9325,240,320,			
							 0x9328,240,320,
							 0x9341,240,320,
							 0x9090,320,480,
							 0x7575,240,320,
							 0x9595,240,320,
							 0x9486,320,480,
							 0x7735,128,160,
							 0x1283,130,130,
						 };

bool tft_spi_init(int16_t model,int8_t cs, int8_t cd, int8_t reset, int8_t led, bool hw_spi = true){
    _reset = reset;
    _led = led;
    _hw_spi = hw_spi;
    return 0;
}

LCDWIKI_SPI::LCDWIKI_SPI(int16_t wid,int16_t heg,int8_t cs, int8_t cd, int8_t reset,int8_t led)
{
	spicsPort = portOutputRegister(digitalPinToPort(cs));
	spicsPinSet = digitalPinToBitMask(cs);
	spicsPinUnset = ~spicsPinSet;
	if(cd < 0)
	{
		spicdPort = 0;
		spicdPinSet = 0;
		spicdPinUnset = 0;
	}
	else
	{
		spicdPort = portOutputRegister(digitalPinToPort(cd));
		spicdPinSet = digitalPinToBitMask(cd);
		spicdPinUnset = ~spicdPinSet;	
	}
	spimisoPort = 0;
	spimisoPinSet = 0;
	spimisoPinUnset = 0;
	spimosiPort = 0;
	spimosiPinSet = 0;
	spimosiPinUnset = 0;
	spiclkPort = 0;
	spiclkPinSet = 0;
	spiclkPinUnset = 0;

	*spicsPort     |=  spicsPinSet; // Set all control bits to HIGH (idle)
	*spicdPort     |=  spicdPinSet; // Signals are ACTIVE LOW

	pinMode(cs, OUTPUT);	  // Enable outputs
	pinMode(cd, OUTPUT);	

	if(reset >= 0) 
	{
		digitalWrite(reset, HIGH);
		pinMode(reset, OUTPUT);
	}
	if(led >= 0)
	{
		//digitalWrite(led, HIGH);
		pinMode(led, OUTPUT);
	}
	SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz (half speed)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
	
	rotation = 0;
 	lcd_model = 0xFFFF;
    setWriteDir();
	WIDTH = wid;
	HEIGHT = heg;
 	width = WIDTH;
	height = HEIGHT;
}

