#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "rc522.c"

#define GPIO_LED 12

#define VSPI_MOSI_PIN 23
#define VSPI_CLK 18
#define VSPI_CS0 5

#define CS_PIN    5                 // Chip Select pin
#define RST_PIN   33                // Reset pin
#define DC_PIN    27                // A0 Pin
#define SDA_PIN   23                // SPI MOSI pin
#define SCK_PIN   18                // SPI Clock
#define LED_PIN   32

#define LCD_START_TRANSMIT gpio_set_level(CS_PIN, 0)
#define LCD_STOP_TRANSMIT gpio_set_level(CS_PIN, 1)
#define LCD_CMD_MODE gpio_set_level(DC_PIN, 0)
#define LCD_DATA_MODE gpio_set_level(DC_PIN, 1)

#define CS_ACTIVE gpio_set_level(CS_PIN, 0)
#define CS_IDLE gpio_set_level(CS_PIN, 1)
#define CMD_MODE gpio_set_level(DC_PIN, 0)
#define DATA_MODE gpio_set_level(DC_PIN, 1)

#define write8(data) spi_device_transmit(spi, &(spi_transaction_t) { \
    .length = 8, \
    .tx_data = { data }, \
    .flags = SPI_TRANS_USE_TXDATA \
})
#define write16(data) spi_device_transmit(spi, &(spi_transaction_t) { \
    .length = 16, \
    .tx_data = { (uint8_t )(data >> 8), (uint8_t )data }, \
    .flags = SPI_TRANS_USE_TXDATA \
})
#define read16(dst) { uint8_t hi; read8(hi); read8(dst); dst |= (hi << 8); }
//#define writeCmd8(x) { CD_COMMAND; write8(x); CD_DATA;}
#define writeCmd8(x) CMD_MODE; write8(x)
//#define writeData8(x) {  write8(x) }
#define writeData8(x)  DATA_MODE; write8(x) 

#define writeCmd16(x)  CMD_MODE; write16(x)
#define writeData16(x)  DATA_MODE; write16(x)
#define writeCmdData8(a, d) CMD_MODE; write8(a); DATA_MODE; write8(d)
#define writeCmdData16(a, d)  CMD_MODE; write8(a>>8); write8(a); DATA_MODE; write8(d>>8); write8(d)


void set_addr_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

// static const char *TAG = "LCD Module";
static spi_device_handle_t spi;
static spi_device_handle_t spi_handle;

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

#define TFTLCD_DELAY16 0xFFFF

static const uint16_t SSD1283A_regValues[] = 
			{
				0x10, 0x2F8E,   // Power control         
            	0x11, 0x000C,  
            	0x07, 0x0021,           
            	0x28, 0x0006,      
            	0x28, 0x0005,     
            	0x27, 0x057F,        
            	0x29, 0x89A1,     
            	0x00, 0x0001,     
            	TFTLCD_DELAY16, 100,       
            	0x29, 0x80B0,   
            	TFTLCD_DELAY16, 30, 
            	0x29, 0xFFFE,
            	0x07, 0x0223,
            	TFTLCD_DELAY16, 30, 
            	0x07, 0x0233,
            	0x01, 0x2183,
            	0x03, 0x6830,
            	0x2F, 0xFFFF,
            	0x2C, 0x8000,
            	0x27, 0x0570,
            	0x02, 0x0300,
            	0x0B, 0x580C,
            	0x12, 0x0609,
            	0x13, 0x3100, 
			};
void Write_Cmd(uint16_t cmd)
{
  writeCmd16(cmd);
}

void Write_Data(uint16_t data)
{
  writeData16(data);
}

void Write_Cmd_Data(uint16_t cmd, uint16_t data)
{
  writeCmdData16(cmd,data);
}


static uint8_t XC,YC,CC,RC,SC1,SC2,MD,VL,R24BIT;

uint8_t reverse_bits(uint8_t byte) {
    byte = (byte & 0xF0) >> 4 | (byte & 0x0F) << 4;
    byte = (byte & 0xCC) >> 2 | (byte & 0x33) << 2;
    byte = (byte & 0xAA) >> 1 | (byte & 0x55) << 1;
    return byte;
}


void lcd_reset(void){

    // Trigger hardware reset
    gpio_set_level(RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(3000));
    // Start clock
    writeCmd8(0x00);
}

static void write_cmd_data_pair(uint16_t cmd, uint16_t data){
    spi_transaction_t cmd_trans = {
        .length = 8,
        // .tx_buffer = &cmd, 
        .tx_data = {(uint8_t) cmd},
        .flags = SPI_TRANS_USE_TXDATA
    };

    spi_transaction_t data_trans = {
        .length = 16,
        // .tx_buffer = &data,
        .tx_data = {(uint8_t)(data >> 8), (uint8_t) data},
        .flags = SPI_TRANS_USE_TXDATA
    };
    ESP_LOGI("TFT_INIT","CMD: %x, DATA: %x %x", cmd_trans.tx_data[0], data_trans.tx_data[0], data_trans.tx_data[1]);
    ESP_LOGI("TFT_INIT","CMD: %d, DATA: %d %d", cmd_trans.tx_data[0], data_trans.tx_data[0], data_trans.tx_data[1]);
    LCD_START_TRANSMIT;
    LCD_CMD_MODE;
    esp_err_t ret = spi_device_transmit(spi, &cmd_trans);
    if (ret != ESP_OK) {
        ESP_LOGE("TFT", "Failed to send command: %s", esp_err_to_name(ret));
    }
    LCD_DATA_MODE;
    ret = spi_device_transmit(spi, &data_trans);
    if (ret != ESP_OK) {
        ESP_LOGE("TFT", "Failed to send data: %s", esp_err_to_name(ret));
    }
    LCD_STOP_TRANSMIT;
}


void Fill_Rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	int16_t end;
	if (w < 0) 
	{
        w = -w;
        x -= w;
    }                           //+ve w
    end = x + w;
    if (x < 0)
    {
        x = 0;
    }
    if (end > 130)
    {
        end = 130;
    }
    w = end - x;
    if (h < 0) 
	{
        h = -h;
        y -= h;
    }                           //+ve h
    end = y + h;
    if (y < 0)
    {
        y = 0;
    }
    if (end > 130)
    {
        end = 130;
    }
    h = end - y;
    set_addr_window(x, y, x + w - 1, y + h - 1);
	LCD_START_TRANSMIT;
    
    LCD_CMD_MODE;
    spi_transaction_t cmd= {
    .length = 8,
    .tx_data = {CC},
    .flags = SPI_TRANS_USE_TXDATA
    };
    esp_err_t ret = spi_device_transmit(spi, &cmd);
	if (h > w) 
	{
        end = h;
        h = w;
        w = end;
    }
    LCD_DATA_MODE;
    spi_transaction_t data= {
        .length = 16,
        .tx_data = {(uint8_t)(color >> 8), (uint8_t) color},
        .flags = SPI_TRANS_USE_TXDATA
    };
	while (h-- > 0) 
	{
		end = w;
		do 
		{
   			spi_device_transmit(spi, &data);
        } while (--end != 0);
	}
	LCD_STOP_TRANSMIT;
}

void cs_active(spi_transaction_t *t) {
    // printf("Pre-transaction callback: CS_ACTIVE\n");
    CS_ACTIVE;
    // gpio_set_level(5, 0);
}

void cs_idle(spi_transaction_t *t) {
    // printf("Post-transaction callback: CS_IDLE\n");
    CS_IDLE;
    // gpio_set_level(5, 1);
}


void spi_init() {

    spi_bus_config_t buscfg = {
        .miso_io_num = NULL,
        .mosi_io_num = SDA_PIN,
        .sclk_io_num = SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    gpio_set_direction(DC_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(CS_PIN, GPIO_MODE_OUTPUT);

    esp_err_t ret = spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus: %s", esp_err_to_name(ret));
    }

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .clock_speed_hz = 4000000,           // Clock out at 1 MHz
        .mode = 0,                           // SPI mode 0
        .spics_io_num = CS_PIN,              // CS pin
        .queue_size = 7,                     // We want to be able to queue 7 transactions at a time
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .pre_cb = cs_active,
        .post_cb = cs_idle,
    };

    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
    }

    gpio_set_level(LED_PIN, 1);
    gpio_set_level(RST_PIN, 1);

    gpio_dump_io_configuration(stdout, (1ULL << DC_PIN)|(1ULL << RST_PIN)|(1ULL << LED_PIN)|(1ULL << CS_PIN));
    ESP_LOGI("SPI", "Initialisation completed!");
}
void Set_Addr_Window(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    XC=0x45,YC=0x44,CC=0x22,RC=0x2E,SC1=0x41,SC2=0x42,MD=0x03,VL=1,R24BIT=0;
//   CS_ACTIVE;

    int16_t t1,t2;
    writeCmd8(XC);
    writeData8(x2);
    writeData8(x1);
    writeCmd8(YC);
    writeData8(y2);
    writeData8(y1);
    writeCmd8(0x21);
    writeData8(x1);
    writeData8(y1);
    writeCmd8(CC);
  
//   CS_IDLE;    
}


void tft_init() {

    lcd_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
    // TODO: Figure out what these variables below do
    XC=0x45,YC=0x44,CC=0x22,RC=0x2E,SC1=0x41,SC2=0x42,MD=0x03,VL=1,R24BIT=0;


    uint16_t size = sizeof(SSD1283A_regValues) / sizeof(uint16_t);
    uint16_t *p = SSD1283A_regValues;
    while (size > 0) 
	{
        uint16_t cmd = *(const uint16_t *)p++;
        uint16_t data = *(const uint16_t *)p++;
        
        if (cmd == 0xFFFF)
        {
            vTaskDelay(pdMS_TO_TICKS(data));
        }
        else 
		{
			write_cmd_data_pair(cmd, data); 
		}
        size -= 2;
    }
    ESP_LOGI("TFT_INIT", "TFT Init Completed.");
}

static rc522_handle_t scanner;

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))

void init_table16(const void *table, int16_t size)
{
    uint16_t *p = (uint16_t *) table;
    while (size > 0) 
    {
        //TODO: This shit is causing bit misalignment for some reason
    // uint16_t cmd = pgm_read_word(p++);
    // uint16_t d = pgm_read_word(p++);
    uint16_t cmd = *p++;
    uint16_t d = *p++;
        if (cmd == TFTLCD_DELAY16)
        {
            vTaskDelay(pdMS_TO_TICKS(d));
        }
        else 
        {
            Write_Cmd_Data(cmd, d);                      //static function
        }
        size -= 2;
    }
}

void app_main(void) {
    uint32_t colors[] = {0x0000, 0xFFFF, 0x1234, 0xABCD};
    XC=0x45,YC=0x44,CC=0x22,RC=0x2E,SC1=0x41,SC2=0x42,MD=0x03,VL=1,R24BIT=0;
    int count = 0;
    // spi_init();
    // tft_init();
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << DC_PIN) | (1ULL << RST_PIN) | (1ULL << LED_PIN) | (1ULL << CS_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = SDA_PIN,
        .sclk_io_num = SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    esp_err_t ret = spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus: %s", esp_err_to_name(ret));
    }

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .clock_speed_hz = 4000000,           // Clock out at 1 MHz
        .mode = 0,                           // SPI mode 0
        .spics_io_num = CS_PIN,              // CS pin
        .queue_size = 7,                     // We want to be able to queue 7 transactions at a time
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .pre_cb = cs_active,
        .post_cb = cs_idle,
    };

    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
    }
    gpio_set_level(LED_PIN, 1);
    gpio_set_level(RST_PIN, 1);

    lcd_reset();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // static const uint16_t _SSD1283A_regValues[] = 
    //     {
    //         0x10, 0x2F8E,   // Power control         
    //         0x11, 0x000C,  
    //         0x07, 0x0021,           
    //         0x28, 0x0006,      
    //         0x28, 0x0005,     
    //         0x27, 0x057F,        
    //         0x29, 0x89A1,     
    //         0x00, 0x0001,     
    //         TFTLCD_DELAY16, 100,       
    //         0x29, 0x80B0,   
    //         TFTLCD_DELAY16, 30, 
    //         0x29, 0xFFFE,
    //         0x07, 0x0223,
    //         TFTLCD_DELAY16, 30, 
    //         0x07, 0x0233,
    //         0x01, 0x2183,
    //         0x03, 0x6830,
    //         0x2F, 0xFFFF,
    //         0x2C, 0x8000,
    //         0x27, 0x0570,
    //         0x02, 0x0300,
    //         0x0B, 0x580C,
    //         0x12, 0x0609,
    //         0x13, 0x3100, 
    //     };
    static const uint16_t _SSD1283A_regValues[] = 
        {
            0x10, 0x2FCE, // Power Control 1: Basic power settings
            0x11, 0x000C, // Power Control 2: Additional power settings
            0x07, 0x0021, // Display Control 1: Initial display settings
            0x28, 0x0006, // Display Control 3: Fine-tuning display timings (first step)
            0x28, 0x0005, // Display Control 3: Fine-tuning display timings (second step)
            0x27, 0x057F, // Display Control 2: Advanced display settings
            0x29, 0x89A1, // Frame Cycle Control: Set frame timing parameters
            0x00, 0x0001, // Start Oscillation
            0xFFFF, 100,  // Delay for 100 ms
            0x29, 0x80B0, // Adjust frame cycle control settings
            0xFFFF, 30,   // Delay for 30 ms
            0x29, 0xFFFE, // Finalize frame cycle control settings
            0x07, 0x0223, // Update display control settings
            0xFFFF, 30,   // Delay for 30 ms
            0x07, 0x0233, // Update display control settings (final step)
            0x01, 0x2183, // Driver Output Control: Set driver output parameters Original 0x2183 130 pixel lines
            0x03, 0x6830, // Entry Mode: Set entry mode for RAM operations
            0x2F, 0xFFFF, // Color Set: Initialize color settings
            0x2C, 0x8000, // Memory Write: Begin writing to display memory
            0x27, 0x0570, // Finalize display control 2 settings
            0x02, 0x0300, // LCD Driving Waveform Control: Set driving waveform
            0x0B, 0x580C, // Frame Cycle Control: Adjust frame cycle parameters
            0x12, 0x0609, // Power Control 3: Additional power settings
            0x13, 0x3100,  // Power Control 4: Final power settings
            // 0x16, 0x8102, // 130 pixels per line
        };
    init_table16(_SSD1283A_regValues, sizeof(_SSD1283A_regValues) / sizeof(uint16_t)); 

    // lcd_reset();
    int start_x, start_y, end_x, end_y;
    int offset = 2; // Somehow we have to add these offsets
    start_x = 0 + offset;
    start_y = 0 + offset;
    end_x = 130 + offset;
    end_y = 130 + offset;
    Set_Addr_Window(0, 0, 132 - 1, 132 - 1);
    writeCmd8(0x22); //CC 
    for(int _h = 0; _h < 132; _h++){
        for(int _w = 0; _w < 132; _w++){
            writeData16(0xFFFF);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    Set_Addr_Window(start_x, start_y, end_x - 1, end_y - 1);
    int end = (end_x > end_y) ? end_x : end_y;
    int h = (end_x > end_y) ? end_x : end_y;
    int start_pos = (start_x < start_y) ? start_x : start_y;
    // CS_ACTIVE;
    ESP_LOGI("Main","Start drawing");
    
    writeCmd8(0x22); //CC 

    //Dummy writes
    // TODO

    for(int _h = start_pos; _h < h; _h++){
        for(int _w = start_pos; _w < end; _w++){
            writeData16(0xABCD);
        }
    }
    writeCmd8(CC);
    vTaskDelay(pdMS_TO_TICKS(1000));
    for(int _h = start_pos; _h < h; _h++){
        for(int _w = start_pos; _w < end; _w++){
            if(_w > 50){
                writeData16(0xAAAA);

            } else {
                writeData16(0x1111);
            } 
        }
    }
    writeCmd8(CC);   //Read data from GRAM
    ESP_LOGI("Main", "Finished drawing");

}

