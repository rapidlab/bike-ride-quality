#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <drivers/i2c.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>

#include <string.h>
#include <kernel.h>
#include <stdio.h>
#include <stdlib.h>

#include <disk/disk_access.h>
#include <logging/log.h>
#include <fs/fs.h>
#include <ff.h>
#include <kernel.h> 

#define I2C_DEV_0 DT_LABEL(DT_ALIAS(i2c_0))
#define I2C_ADDR 0x18
#define BUF_MAXSIZE	256*8

#define OUTX_L_XL                                            0x28  //X-axis LSB output register
#define OUTX_H_XL                                            0x29  //X-axis MSB output register 
#define OUTY_L_XL                                            0x2A  //Y-axis LSB output register
#define OUTY_H_XL                                            0x2B  //Y-axis MSB output register
#define OUTZ_L_XL                                            0x2C  //Z-axis LSB output regitser
#define OUTZ_H_XL                                            0x2D  //Z-axis MSB output register
#define CTRL1                                                0x20  //Control register 1
#define CTRL2                                                0x21  //Control register 2
#define CTRL3                                                0x22  //Control register 3
#define CTRL4                                                0x23  //Control register 4
#define CTRL6                                                0x25  //Control register 6
#define CTRL7                                                0x3F  //Control register 7

#define STACKSIZE                                            1024  //Size of stack area used by each thread
#define PRIORITY                                             -2    //Thread's priority (negative priority value - a cooperative thread; non-negative priority value - a preemptible thread) 

/*
 * Devicetree helper macro which gets the 'flags' cell from a 'gpios'
 * property, or returns 0 if the property has no 'flags' cell.
 */

#define FLAGS_OR_ZERO(node)						\
	COND_CODE_1(DT_PHA_HAS_CELL(node, gpios, flags),		\
		    (DT_GPIO_FLAGS(node, gpios)),			\
		    (0))

#define SW0_NODE	DT_ALIAS(sw0)
#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
#define SW0_GPIO_LABEL	DT_GPIO_LABEL(SW0_NODE, gpios)
#define SW0_GPIO_PIN	DT_GPIO_PIN(SW0_NODE, gpios)
#define SW0_GPIO_FLAGS	(GPIO_INPUT | FLAGS_OR_ZERO(SW0_NODE))
#else
#error "Unsupported board: sw0 devicetree alias is not defined"
#define SW0_GPIO_LABEL	""
#define SW0_GPIO_PIN	0
#define SW0_GPIO_FLAGS	0
#endif

#define SW1_NODE	DT_ALIAS(sw1)
#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
#define SW1_GPIO_LABEL	DT_GPIO_LABEL(SW1_NODE, gpios)
#define SW1_GPIO_PIN	DT_GPIO_PIN(SW1_NODE, gpios)
#define SW1_GPIO_FLAGS	(GPIO_INPUT | FLAGS_OR_ZERO(SW1_NODE))
#else
#error "Unsupported board: sw1 devicetree alias is not defined"
#define SW1_GPIO_LABEL	""
#define SW1_GPIO_PIN	0
#define SW1_GPIO_FLAGS	0
#endif

#define LED0_NODE	DT_ALIAS(led0)
#if DT_NODE_HAS_STATUS(LED0_NODE, okay) && DT_NODE_HAS_PROP(LED0_NODE, gpios)
#define LED0_GPIO_LABEL	DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED0_GPIO_PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define LED0_GPIO_FLAGS	(GPIO_OUTPUT | FLAGS_OR_ZERO(LED0_NODE))
#endif


char __aligned(1) rx_queue_msgq_buffer[1024];
struct k_msgq rx_queue;

typedef unsigned char       u8_t;
typedef unsigned int        u32_t;
static volatile uint8_t rx_buf[BUF_MAXSIZE+1] = {[BUF_MAXSIZE] = 0,};
static volatile int16_t os_x_buff[100] = {};
static volatile int16_t os_y_buff[100] = {};
static volatile int16_t os_z_buff[100] = {};
static volatile int os_buff_len = 0;
uint8_t data[7];


struct device *uart1_dev;
struct fs_file_t zfp; 

static FATFS fat_fs;
LOG_MODULE_REGISTER(main);
/* mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
    //.mnt_point = FATFS_MNTP,
	.fs_data = &fat_fs,
};

int fs_register(enum fs_type type, struct fs_file_system_t *fs);

static const char *disk_mount_pt = "/SD:";

//GPIO callback structures for button 'start', button 'stop' and accelerometer
static struct gpio_callback button_start_cb_data;
static struct gpio_callback button_stop_cb_data;
static struct gpio_callback accelerometer_int1_data;

struct device *i2c_dev_0;
struct device *button_start, *button_stop;
struct device *led;
struct device *acc_interrupt;
struct device *dev_gpio;

typedef struct GPS_data
{
    char time[7];
    char date[14];
    char width[11];
    char n_s[2];
    char length[12];
    char w_e[2];
    
} GPS_reading;

GPS_reading GPS_data;

//Name of the file saved on the SD card
char file_name[30]="/SD:/";

volatile int rx_len=0;
//Position in the file
int position = 0;

bool pressed;
bool there_is_file_name = false;
bool is_there_mount_point = false;

//Function called when pressing the start button
void start_pressed(struct device *dev, struct gpio_callback *cb, unsigned int pins)
{
    //Cleaning rx queue
    k_msgq_purge(&rx_queue);
    printk("Button Start \n");
    //Configuring led 0 to glow when the start button is pressed
    gpio_pin_configure(led, LED0_GPIO_PIN, GPIO_OUTPUT_INACTIVE);
    //Cleaning rx buffer
    for(int i=0; i<BUF_MAXSIZE;i++)
    {
         rx_buf[i]= 0;
    }
    file_name[0] = '/';
    file_name[1] = 'S';
    file_name[2] = 'D';
    file_name[3] = ':';
    file_name[4] = '/';
    file_name[5] = '\0';

    position=0;
    pressed = true;
    
}

//Function called when pressing the stop button
void stop_pressed(struct device *dev, struct gpio_callback *cb, uint32_t pins)
{   
    printk("Button Stop \n");
    //Configuring led 0 to stop glow when the stop button is pressed
    gpio_pin_configure(led, LED0_GPIO_PIN, GPIO_OUTPUT_ACTIVE);
    pressed = false;
}

//Interrupt function for uart 1
static void uart1_isr(struct device *x)
{   
    //Start processing interrupts in ISR
    uart_irq_update(x);
    int read = 1;
    uint8_t  character;
    //Disabling rx interrupt
    uart_irq_rx_disable(uart1_dev);
    //Checking if UART RX buffer has at least one pending character
    if(uart_irq_rx_ready(uart1_dev))
    {
        while(read)
        {
            //Reading data from FIFO
            read = uart_fifo_read(x, &character,1);
            if(read)
            {
                //Sending data to a message queue
                k_msgq_put(&rx_queue, &character, K_NO_WAIT);
            }
        }
    }
    //Enabling RX interrupt
    uart_irq_rx_enable(uart1_dev);	
}

//Uart 1 initialization
static void uart1_init(void)
{
    struct uart_config uart_settings;
    //Setting baudrate in bps
    uart_settings.baudrate = 9600;
    //Setting parity bit
    uart_settings.parity = UART_CFG_PARITY_NONE;
    //Setting stop bits
    uart_settings.stop_bits = UART_CFG_STOP_BITS_1;
    //Setting data bits
    uart_settings.data_bits = UART_CFG_DATA_BITS_8;
    //Setting the flow control
    uart_settings.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

    //Retrieving the device structure for Uart 1
    uart1_dev = device_get_binding("UART_1");
    if(!uart1_dev)
    {
        printk("ERROR - device get binding");
    }
    //Checking whether an error was detected
    int err = uart_err_check(uart1_dev);
    if(err)
    {
        printk("ERROR - uart err check %x \n", err);
    }
    //Initializing a rx queue with GPS data
    k_msgq_init(&rx_queue, rx_queue_msgq_buffer, sizeof(u8_t),1024);

    uart_irq_rx_disable(uart1_dev);
    //Setting up the callback for IRQ
    uart_irq_callback_set(uart1_dev, uart1_isr);
    uart_irq_rx_enable(uart1_dev);

    printk("%s() done\n", __func__);
}

//Reading one data frame from GPS
static int uart_wifi_readln(volatile u8_t *buf, size_t max_len)
{
    int total = 0;
    int i = 0;
    int status; 

    for( ; i < max_len; ++i )
    {
        status = k_msgq_get(&rx_queue,(void*) &buf[i], K_SECONDS(10));
        if( buf[i] == '\n')
        {
            break;
        }
        if(buf[i] == 0 || buf[i] == EOF || buf[i] == '\r' || status != 0)
        {
            i--;
        } 
    }           
    total = i; 
    buf[total] = '\0';
    return total;
}

//Checking if the buffer size has not been exceeded
bool validate_rxbuf_ptr(volatile char*ptr)
{
    if(ptr > (char*) rx_buf+BUF_MAXSIZE)
    {
        rx_len =0;
        return false;  
    }
    return true;
}

//Reading GPS data
int reading_GPS_data(GPS_reading* GPS_data_read)
{
    char* stop=NULL;
    //Reading one data frame from GPS	
    uart_wifi_readln(rx_buf,sizeof(rx_buf));
    //Printing data frames from GPS               
    printk("%s \n",rx_buf);
    //Searching for GPRMC frames        
    char* start = strstr((const char*) rx_buf,"GPRMC");
    if(start!=NULL)
    {
        if(validate_rxbuf_ptr(start)==false) return 0;
             
        start = strchr(start,',');
        if(validate_rxbuf_ptr(start)==false) return 0; 
            
        stop = strchr(start+1,',');
        if(validate_rxbuf_ptr(stop)==false) return 0;
            
        //Writing the actual time from the GPRMC frame to GPS_data_read->time
        int i = stop - start;
        strncpy(GPS_data_read->time, start+1, stop-start);
                
        char zmienna[1];
        start = stop +1;
        strncpy(zmienna,start,1);

        //Checking if GPS is active
        if(zmienna[0]=='A')
        {        
            start = strchr(stop+1,',');
            if(validate_rxbuf_ptr(start)==false) return 0;

            stop = strchr(start+1,',');
            if(validate_rxbuf_ptr(stop)==false) return 0;

            //Writing latitude from the GPRMC frame to GPS_data_read->time
            i = stop-start;
            strncpy(GPS_data_read->width,start+1,stop-start);
            GPS_data_read->width[10]=';';            
            
            //Writing geographic direction N or S from the GPRMC frame to GPS_data_read->n_s
            GPS_data_read->n_s[0] = *(stop+1);
            GPS_data_read->n_s[1] = ';';
            
            start = stop+2;
            stop = strchr(start+1,',');
            
            //Writing longitude from the GPRMC frame to GPS_data_read->length
            i = stop-start;
            strncpy(GPS_data_read->length,start+1,stop-start);
            GPS_data_read->length[11]=';';
                
            //Writing geographic direction W or E from the GPRMC frame to GPS_data_read->w_e
            GPS_data_read->w_e[0] = *(stop+1);
            GPS_data_read->w_e[1] = ';';
            
            start = stop+2;
            start = strchr(start+1, ',');
            if(validate_rxbuf_ptr(start)==false) return 0;
       
            start = strchr(start+1, ',');
            if(validate_rxbuf_ptr(start)==false) return 0;
             
            stop = strchr(start+1, ',');
            if(validate_rxbuf_ptr(stop)==false) return 0;
        
            //Writing the actual date from the GPRMC frame to GPS_data_read->date
            i = stop-start;
            strncpy(GPS_data_read->date, start+1,stop-start);
                
        }
        //Checking if GPS is inactive
        else if (zmienna[0]=='V')
        {
            start = start + 8;
            stop = strchr(start,',');
            //Writing the actual date from the GPRMC frame to GPS_data_read->date
            strncpy(GPS_data_read->date, start,stop-start);

            //Writing NULL to GPS_data_read->width
            GPS_data_read ->width[0]='\0';
            GPS_data_read->width[1]=';';

            //Writing NULL to GPS_data_read->n_s
            GPS_data_read->n_s[0]='\0';
            GPS_data_read->n_s[1]=';';

            //Writing NULL to GPS_data_read->length
            GPS_data_read->length[0] = '\0';
            GPS_data_read->length[1]=';';

            //Writing NULL to GPS_data_read->w_e
            GPS_data_read->w_e[0]='\0';
            GPS_data_read->w_e[1]=';';
               
        }
        
        //Concatenating date and time
        GPS_data_read->date[4] = '\0';
        GPS_data_read->time[6] = '\0';
        strcat(GPS_data_read->date,GPS_data_read->time);
        GPS_data_read->date[13]='\0';
            	
        return 1;
 		
 	} else{ return 0;}
    
}

//Saving GPS data
int save_gps_data(struct fs_file_t *zfp, GPS_reading* GPS_data_read, char delimiter)
{
    //Moving the file position to a new location in the file (offset is calculated from beginning of file)
    fs_seek(zfp, position, FS_SEEK_SET);

    if(GPS_data_read!=NULL)
    {   
        int number_of_bytes_written;
        //Saving the latitude
        if(GPS_data_read->width[0]=='\0')
        {
            number_of_bytes_written = fs_write(zfp, ";",1);
        }else
        {
            number_of_bytes_written = fs_write(zfp, GPS_data_read->width, sizeof(GPS_data_read->width));
        }
        //Shifting the file position by the number of bytes written
        position = position + number_of_bytes_written;

        //Saving geographic direction (N or S)
        if(GPS_data_read->n_s[0]=='\0')
        {
            number_of_bytes_written = fs_write(zfp, ";",1);
        }else
        {
            number_of_bytes_written = fs_write(zfp, GPS_data_read->n_s, sizeof(GPS_data_read->n_s));
        }
        //Shifting the file position by the number of bytes written
        position = position + number_of_bytes_written;

        //Saving the longitude
        if(GPS_data_read->length[0]=='\0')
        {
            number_of_bytes_written = fs_write(zfp, ";",1);
        }else
        {
            number_of_bytes_written = fs_write(zfp, GPS_data_read->length, sizeof(GPS_data_read->length));
        }
        //Shifting the file position by the number of bytes written
        position = position + number_of_bytes_written;

        //Saving geographic direction (W or E)
        if(GPS_data_read->w_e[0]=='\0')
        {
            number_of_bytes_written = fs_write(zfp, ";",1);
        }else
        {
            number_of_bytes_written = fs_write(zfp, GPS_data_read->w_e, sizeof(GPS_data_read->w_e));
        }
       
        //Returning number of bytes written
        return number_of_bytes_written;
    }
    return 0;

}

//Saving accelerometer data (axis X, axis Y, axis Z)
int save_accelerometer_data(int16_t register_16_b,struct fs_file_t *zfp,char delimiter )
{
    //Moving the file position to a new location in the file (offset is calculated from beginning of file)
    fs_seek(zfp, position, FS_SEEK_SET);

    char tab[32];
    char tab1[32];
    int i=0;
    int j=0;

    if(register_16_b>0)
    {
        while(register_16_b >0)
        {
            //Converting int to ASCII code
            int digit = register_16_b % 10;   
            digit |= 0x30;
            register_16_b= register_16_b/10;
            tab[i]=digit;
            i++;
        }   
    }
    else if (register_16_b <0)
    {   
        //Changing the register contents to a positive value
        register_16_b = - register_16_b;

        while(register_16_b >0)
        {
            //Converting int to ASCII code
            int digit = register_16_b % 10;   
            digit |= 0x30;
            register_16_b= register_16_b/10;
            tab[i]=digit;
            i++;
        }   
        tab[i] = '-';
        i++;
    }
    else if (register_16_b ==0)
    {
        tab[i] = 0x30;
        i++;
    }
     
    //Reversing the array
    for(j=0; j<i; j++)   
    {
        tab1[j] = tab[i-1-j];
    }
	
    tab1[j]= delimiter; 
    j++;
    //Saving accelerometer data
    int number_of_bytes_written = fs_write(zfp,tab1, j);

    //Returning number of bytes written
    return number_of_bytes_written;
}

//Reading accelerometer data - axis X, axis Y or axis Z
int16_t reading_accelerometer_axis(uint8_t out_h, uint8_t out_l, uint8_t datah, uint8_t datal )
{
    //Reading register with axis data
    i2c_reg_read_byte(i2c_dev_0, I2C_ADDR , out_h, &datah);
    int osX_H = datah;

    i2c_reg_read_byte(i2c_dev_0, I2C_ADDR , out_l, &datal);
    int osX_L = datal;
    
    //Concatenation of osX_H and osX_L 
    int16_t register_16_b;
    register_16_b = osX_H;
    register_16_b = (register_16_b << 8);
    register_16_b += osX_L;
    register_16_b = register_16_b >> 2; 
    
    return register_16_b;
}

//Reading accelerometer data    
void accelerometer_data_read(void)
{
    //Retrieving the device structure for I2C
    i2c_dev_0 = device_get_binding(I2C_DEV_0);

    //Writing values to an 8-bit internal register of an I2C device
    i2c_reg_write_byte(i2c_dev_0, I2C_ADDR, CTRL1 ,0x44);
    k_msleep(100);
   
    i2c_reg_write_byte(i2c_dev_0, I2C_ADDR, CTRL2, 0x04); 
    k_msleep(100);

    i2c_reg_write_byte(i2c_dev_0, I2C_ADDR, CTRL3, 0x28);
    k_msleep(100);

    i2c_reg_write_byte(i2c_dev_0, I2C_ADDR, CTRL4, 0x01);
    k_msleep(100);

    i2c_reg_write_byte(i2c_dev_0, I2C_ADDR, CTRL6, 0x1C);
    k_msleep(100);

    i2c_reg_write_byte(i2c_dev_0, I2C_ADDR, CTRL7, 0x80);

    while(1)
    {
        k_msleep(10);

        if(os_buff_len <100)
        {
            //X-axis value reading
            int16_t register_16_b = reading_accelerometer_axis(OUTX_H_XL, OUTX_L_XL, data[0], data[1]);
            //Y-axis value reading
            int16_t rejestr_16_b_2 = reading_accelerometer_axis(OUTY_H_XL, OUTY_L_XL, data[2], data[3]);
            //Z-axis value reading
            int16_t rejestr_16_b_3 = reading_accelerometer_axis(OUTZ_H_XL, OUTZ_L_XL, data[4], data[5]);

            //Writing the read data to the buffer
            os_x_buff[os_buff_len] = register_16_b;
            os_y_buff[os_buff_len] = rejestr_16_b_2;
            os_z_buff[os_buff_len] = rejestr_16_b_3;

            //Increasing the buffer length by the number of bytes read
            os_buff_len++;
        }
    }
}

//Creating a separate thread for reading accelerometer data
K_THREAD_DEFINE(accelerometer_read_id,STACKSIZE,accelerometer_data_read,NULL, NULL, NULL, PRIORITY,0,0);

void main(void)
{
    //Uart 1 initialization
    uart1_init();
    k_msleep(100);

    int ret;
    int ret1;
    int interr;

    //Retrieving the device structure for start and stop buttons
    button_start = device_get_binding(SW0_GPIO_LABEL);
    button_stop = device_get_binding(SW1_GPIO_LABEL);
    if (button_start == NULL || button_stop == NULL)
    {
        printk("Error: didn't find %s device\n", SW0_GPIO_LABEL);
        return;
    }
    //Retrieving the device structure for GPIO 1
    dev_gpio = device_get_binding("GPIO_1");
    if (dev_gpio == NULL)
    {
        printk("Error: didn't find GPIO device \n");
        return;
    }
    //Pin configuration for start and stop buttons
    ret = gpio_pin_configure(button_start, SW0_GPIO_PIN, SW0_GPIO_FLAGS);
    ret1 = gpio_pin_configure(button_stop, SW1_GPIO_PIN, SW1_GPIO_FLAGS);
    if (ret != 0 || ret1 != 0)
    {
        printk("Error %d: failed to configure %s pin %d\n", ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
        return;
    }
    //Configuring interrupts on pins for start and stop buttons
    ret = gpio_pin_interrupt_configure(button_start, SW0_GPIO_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    ret1 = gpio_pin_interrupt_configure(button_stop, SW1_GPIO_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0 || ret1 !=0)
    {
        printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
        return;
    }
    //Configuring interrupts
    interr = gpio_pin_interrupt_configure(dev_gpio, 10, GPIO_INT_EDGE_TO_ACTIVE);
    if (interr != 0)
    {
        printk("Error %d: failed to configure interrupt on pin 10 \n",interr);
        return;
    }
    //Initializing a struct gpio_callback for start button
    gpio_init_callback(&button_start_cb_data, start_pressed, BIT(SW0_GPIO_PIN));
    gpio_add_callback(button_start, &button_start_cb_data);
    printk("Set up button at %s pin %d\n", SW0_GPIO_LABEL, SW0_GPIO_PIN);

    //Initializing a struct gpio_callback for stop button
    gpio_init_callback(&button_stop_cb_data, stop_pressed, BIT(SW1_GPIO_PIN));
    gpio_add_callback(button_stop, &button_stop_cb_data);
    printk("Set up button at %s pin %d\n", SW1_GPIO_LABEL, SW1_GPIO_PIN);

    //Retrieving the device structure for led 0
    led = device_get_binding(LED0_GPIO_LABEL);
    if (led == NULL)
    {
        printk("Didn't find LED device %s\n", LED0_GPIO_LABEL);
        return;
    }

    printk("Press the button\n");

    //Starting an inactive thread
    k_thread_start(accelerometer_data_read);
   
    while(1)
    {
        //Checking if start button was pressed         
        if(pressed == true)
        {   
            //Checking if there is already a mount point
            if(!is_there_mount_point)
            {
                //Mounting filesystem
                mp.mnt_point = disk_mount_pt;
                int sd_card = fs_mount(&mp);    
                k_msleep(1000);

                //Checking if there is a SD card in the device
                if (sd_card !=0)
                {
                    //Switching on and off the led 0
                    gpio_pin_configure(led, LED0_GPIO_PIN, GPIO_OUTPUT_INACTIVE);
                    k_msleep(500);
                    gpio_pin_configure(led, LED0_GPIO_PIN, GPIO_OUTPUT_ACTIVE);
                    printk("There is no SD card in the device \n");

                    is_there_mount_point = false;
                }else
                {
                    is_there_mount_point = true;
                }
            }
        
            //Checking if there is already a mount point
            if (is_there_mount_point)
            {
                int successful_reading = reading_GPS_data(&GPS_data);
                //Checking if there is already a file name
                if(!there_is_file_name)
                {
                    if(successful_reading== 1)
                    {   
                        strcat(file_name, GPS_data.date);
                        file_name[13] = '\0';
                        //Creating a file on SD card with the name consisting of actual data and time
                        int ok = fs_open(&zfp, file_name);
                        if(ok!=0)
                        {
                            printk("Error opening file %s [%d] \n", file_name,ok);
                            while(1);
                        }
                        there_is_file_name = true;
                    }
                }

                if(there_is_file_name)
                {
                    for(int i=0;i<os_buff_len;i++)
                    {
                        //Saving GPS data on SD card
                        int number_of_bytes_written_gps = save_gps_data(&zfp, &GPS_data ,';');
                        //Shifting the file position by the number of bytes written
                        position = position + number_of_bytes_written_gps;
                        
                        //Saving accelerometer data (axis x) on SD card
                        int number_of_bytes_written = save_accelerometer_data(os_x_buff[i],&zfp,';');
                        //Shifting the file position by the number of bytes written
                        position = position + number_of_bytes_written;
            
                        //Saving accelerometer data (axis y) on SD card
                        number_of_bytes_written = save_accelerometer_data(os_y_buff[i],&zfp,';');
                        //Shifting the file position by the number of bytes written
                        position = position + number_of_bytes_written;
                
                        //Saving accelerometer data (axis z) on SD card
                        number_of_bytes_written = save_accelerometer_data(os_z_buff[i],&zfp,'\n');
                        //Shifting the file position by the number of bytes written
                        position = position + number_of_bytes_written;
                    }

                    //Setting the buffer length to 0 
                    os_buff_len = 0;
                
                    //Checking if stop button was pressed
                    if(pressed == false)
                    {
                        //Closing the file
                        fs_close(&zfp);
                        there_is_file_name = false;
                    }
                }
            }

        //Checking if stop button was pressed
        }else if(pressed == false)
        {
            there_is_file_name = false;
            //Checking if there is already a mount point
            if(is_there_mount_point)
            {
                //Unmounting filesystem.
                fs_unmount(&mp);
                is_there_mount_point = false;
            }
            k_msleep(1000);
        }

    }

}



