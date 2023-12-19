/*=================================================================

						EDIT HISTORY FOR MODULE

This section contains comments describing changes made to the module.
Notice that changes are listed in reverse chronological order.

WHEN			  WHO		  WHAT, WHERE, WHY
------------	 -------	 -------------------------------------------------------------------------------

=================================================================*/


/*===========================================================================
 * include files
 ===========================================================================*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ql_api_osi.h"
#include "ql_log.h"
#include "ql_pin_cfg.h"

#include "my_lib.h"
#include "ql_uart.h"


#define QL_UART_DEMO_LOG_LEVEL			QL_LOG_LEVEL_DEBUG
#define QL_UART_DEMO_LOG(msg, ...)		QL_LOG(QL_UART_DEMO_LOG_LEVEL, "ql_uart_demo", msg, ##__VA_ARGS__)


#define QL_UART_TASK_STACK_SIZE     		4096
#define QL_UART_TASK_PRIO          	 	    APP_PRIORITY_NORMAL
#define QL_UART_TASK_EVENT_CNT      		5


#define QL_UART_RX_BUFF_SIZE                2048
#define QL_UART_TX_BUFF_SIZE                2048

#define MIN(a,b) ((a) < (b) ? (a) : (b))

/*===========================================================================
 * Macro Definition
 ===========================================================================*/
#define QL_GPIODEMO_LOG_LEVEL             QL_LOG_LEVEL_INFO
#define QL_GPIODEMO_LOG(msg, ...)         QL_LOG(QL_GPIODEMO_LOG_LEVEL, "ql_GPIODEMO\n", msg, ##__VA_ARGS__)
#define QL_GPIODEMO_LOG_PUSH(msg, ...)    QL_LOG_PUSH("ql_GPIODEMO\n", msg, ##__VA_ARGS__)

/*===========================================================================
 * Variate
 ===========================================================================*/
//static ql_gpio_cfg _ql_gpio_cfg[] =
//{   /* gpio_num   gpio_dir     gpio_pull    gpio_lvl    */
//    {  GPIO_0,   GPIO_INPUT,   PULL_DOWN,   0xff     },   // set input pull-down
//    {  GPIO_1,   GPIO_OUTPUT,  0xff,        LVL_HIGH }    // set output high-level
//};

/*===========================================================================
 * Functions
 ===========================================================================*/
volatile char str[1000] = "";
volatile uint8_t flaguart = 0;
volatile uint8_t strln = 0 ; 
void ql_uart_notify_cb(unsigned int ind_type, ql_uart_port_number_e port, unsigned int size)
{
    unsigned char *recv_buff = calloc(1, QL_UART_RX_BUFF_SIZE+1);
    //unsigned int real_size = 0;
    int read_len = 0;
    
    QL_UART_DEMO_LOG("UART port %d receive ind type:0x%x, receive data size:%d", port, ind_type, size);
    switch(ind_type)
    {
        case QUEC_UART_RX_OVERFLOW_IND:  //rx buffer overflow
        case QUEC_UART_RX_RECV_DATA_IND:
        {
            while(size > 0)
            {
                
                memset(recv_buff, 0, QL_UART_RX_BUFF_SIZE+1);
                //real_size= MIN(size, QL_UART_RX_BUFF_SIZE);
                
                read_len = ql_uart_read(port, (unsigned char*)str, 1000);
                strln = read_len;
                flaguart = 1;
                //ql_uart_read(QL_UART_PORT_2,recv_buff,strlen(recv_buff));
               // ql_uart_write(QL_UART_PORT_2, (unsigned char*)"TEST IS", 8);
                //ql_rtos_task_sleep_s(1);
                
                QL_UART_DEMO_LOG("read_len=%d, recv_data=%s", read_len, recv_buff);
                if((read_len > 0) && (size >= read_len))
                {
                    size -= read_len;
                }
                else
                {
                    break;
                }
            }
            break;
        }
        case QUEC_UART_TX_FIFO_COMPLETE_IND: 
        {
            QL_UART_DEMO_LOG("tx fifo complete");
            break;
        }
    }
    free(recv_buff);
    recv_buff = NULL;
}

/*void my_ql_gpio_demo_init(  void)
{
    uint16_t num;
    for( num = 0; num < sizeof(_ql_gpio_cfg)/sizeof(_ql_gpio_cfg[0]); num++ )
    {
        ql_gpio_deinit(_ql_gpio_cfg[num].gpio_num);
        ql_gpio_init(_ql_gpio_cfg[num].gpio_num, _ql_gpio_cfg[num].gpio_dir, _ql_gpio_cfg[num].gpio_pull, _ql_gpio_cfg[num].gpio_lvl);
    }
}*/

void my_lib_demo_thread(void *param)
{
    //int num_ = 100;
    char param_[200] = "";
    //char bafuart[1000] = "";
    //sprintf(param_,"UART = %d",num_);

    //QL_GPIODEMO_LOG("gpio demo thread enter, param 0x%x\n", param);

    ql_event_t event;

    //uint8_t change_flg = 0;
    //uint16_t num;

    //ql_GpioDir  gpio_dir;
    //ql_PullMode gpio_pull;
    //ql_LvlMode  gpio_lvl;

    //uart demo 

    int ret = 0;
	//QlOSStatus err = 0;
    ql_uart_config_s uart_cfg = {0};
    //int write_len = 0;
    //ql_uart_tx_status_e tx_status;
    //unsigned char data[] = "saba ino neveshte! :) \r\n";

    /***********************************************************
	Note start:
        1.If the BAUD rate is QL UART BAUD_AUTO,a string of 'at'should be sent at least once to identify the baud rate.
        2.Once the baud rate is identified, it cannot be changed unless restarted.
    ************************************************************/
    uart_cfg.baudrate = QL_UART_BAUD_115200;
    uart_cfg.flow_ctrl = QL_FC_NONE;
    uart_cfg.data_bit = QL_UART_DATABIT_8;
    uart_cfg.stop_bit = QL_UART_STOP_1;
    uart_cfg.parity_bit = QL_UART_PARITY_NONE;

    ret = ql_uart_set_dcbconfig(QL_UART_PORT_2, &uart_cfg);
    QL_UART_DEMO_LOG("ret: 0x%x", ret);
	if(QL_UART_SUCCESS != ret)
	{
		//goto exit;
	}
    ret = ql_pin_set_func(QL_UART2_TX_PIN, QL_UART2_TX_FUNC);
	if(QL_GPIO_SUCCESS != ret)
	{
		//goto exit;
	}
	ret = ql_pin_set_func(QL_UART2_RX_PIN, QL_UART2_RX_FUNC);
	if(QL_GPIO_SUCCESS != ret)
	{
		//goto exit;
	}
	/*Note end*/
	
    ret = ql_uart_open(QL_UART_PORT_2);
    QL_UART_DEMO_LOG("ret: 0x%x", ret);

	if(QL_UART_SUCCESS == ret)
	{
	    ret = ql_uart_register_cb(QL_UART_PORT_2, ql_uart_notify_cb);
	    QL_UART_DEMO_LOG("ret: 0x%x", ret);

	    memset(&uart_cfg, 0, sizeof(ql_uart_config_s));
	    ret = ql_uart_get_dcbconfig(QL_UART_PORT_2, &uart_cfg);
	    QL_UART_DEMO_LOG("ret: 0x%x, baudrate=%d, flow_ctrl=%d, data_bit=%d, stop_bit=%d, parity_bit=%d", 
	                        ret, uart_cfg.baudrate, uart_cfg.flow_ctrl, uart_cfg.data_bit, uart_cfg.stop_bit, uart_cfg.parity_bit);
        //write_len = ql_uart_write(QL_UART_PORT_2, (unsigned char*)param_, strlen(param_));
    }
	
    //end uart demo

    /* init demo gpio array */
   // ql_pin_set_func(QL_TEST1_PIN_GPIO0, QL_TEST1_PIN_GPIO0_FUNC_GPIO);      // TEST1_PIN set GPIO0
   // ql_pin_set_func(QL_TEST1_PIN_GPIO1, QL_TEST1_PIN_GPIO1_FUNC_GPIO);      // TEST1_PIN set GPIO1

   // ql_pin_set_func(QL_TEST2_PIN_GPIO0, QL_TEST2_PIN_GPIO0_FUNC_LCD_SIO);   // TEST2_PIN set spi_lcd_sio
   // ql_pin_set_func(QL_TEST2_PIN_GPIO1, QL_TEST2_PIN_GPIO1_FUNC_LCD_SDC);   // TEST2_PIN set spi_lcd_sdc

   // my_lib_app_init();

    /* get init info */
  /*  for( num = 0; num < sizeof(_ql_gpio_cfg)/sizeof(_ql_gpio_cfg[0]); num++ )
    {
        ql_gpio_get_direction(_ql_gpio_cfg[num].gpio_num, &gpio_dir);
        ql_gpio_get_pull(_ql_gpio_cfg[num].gpio_num, &gpio_pull);
        ql_gpio_get_level(_ql_gpio_cfg[num].gpio_num, &gpio_lvl);

        //QL_GPIODEMO_LOG("gpio[%d] init num = %d \n", _ql_gpio_cfg[num].gpio_num, num);
        //QL_GPIODEMO_LOG("gpio[%d] get dir:[%d], pull:[%d], lvl:[%d]\n", _ql_gpio_cfg[num].gpio_num, gpio_dir, gpio_pull, gpio_lvl);
    }*/
    //ql_rtos_task_sleep_s(3);
    sprintf(param_,"TEST UART IS OK");
    ql_uart_write(QL_UART_PORT_2, (unsigned char*)param_, strlen(param_));


    while(1)
    {

        if(flaguart)
        {
            ql_uart_write(QL_UART_PORT_2, (unsigned char*)str,strln);
            flaguart = 0;
        }
        ql_event_wait(&event, 1);

        /* output low test */
       // gpio_dir  = GPIO_OUTPUT;
       // gpio_lvl  = LVL_LOW;
       // for( num = 0; num < sizeof(_ql_gpio_cfg)/sizeof(_ql_gpio_cfg[0]); num++ )
       // {
            /* set output low */
          //  ql_gpio_set_direction(_ql_gpio_cfg[num].gpio_num, gpio_dir);
           // ql_gpio_set_level(_ql_gpio_cfg[num].gpio_num, gpio_lvl);
            //num_ = 100;
            //sprintf(param_,"gpionum = %d ",num_);
            //ql_uart_write(QL_UART_PORT_2, (unsigned char*)param_, strlen(param_));
            
            //ql_uart_read(QL_UART_PORT_2,(unsigned char*)param_,strlen(param_));
           //  if(*param_ != '\0')
            // {
             //ql_rtos_task_sleep_s(1);
            // ql_uart_write(QL_UART_PORT_2, (unsigned char*)"Whileloop1", 11);
             //ql_rtos_task_sleep_s(1);
             //ql_uart_write(QL_UART_PORT_2, (unsigned char*)"Whileloop2", 11);    
             //ql_rtos_task_sleep_s(1);
             //ql_uart_write(QL_UART_PORT_2, (unsigned char*)"Whileloop3", 11); 
            // }
            //    QL_GPIODEMO_LOG("\n\n\n\n\n it is working!!! \n\n\n\n\n\n");
            
            //QL_GPIODEMO_LOG("gpio[%d] output low-level\n\n\n NUM = %d \n\n\n", _ql_gpio_cfg[num].gpio_num,8);
            //QL_GPIODEMO_LOG("gpio[%d] set dir:[%d], lvl:[%d]\n", _ql_gpio_cfg[num].gpio_num, gpio_dir, gpio_lvl);

            /* get output low */
            //ql_gpio_get_direction(_ql_gpio_cfg[num].gpio_num, &gpio_dir);
           // ql_gpio_get_level(_ql_gpio_cfg[num].gpio_num, &gpio_lvl);

            //QL_GPIODEMO_LOG("gpio[%d] output low-level\n", _ql_gpio_cfg[num].gpio_num);
            //QL_GPIODEMO_LOG("gpio[%d] get dir:[%d], lvl:[%d]\n", _ql_gpio_cfg[num].gpio_num, gpio_dir, gpio_lvl);
       // }
     //  ql_rtos_task_sleep_s(0.5);
        //QL_GPIODEMO_LOG("\n\n\n\n\n it is working!!! \n\n\n\n\n\n");

        /* output high test */
       // gpio_dir  = GPIO_OUTPUT;
        //gpio_lvl  = LVL_HIGH;
       // ql_rtos_task_sleep_s(3);
      /*  for( num = 0; num < sizeof(_ql_gpio_cfg)/sizeof(_ql_gpio_cfg[0]); num++ )
        {
             set output high 
           ql_gpio_set_direction(_ql_gpio_cfg[num].gpio_num, gpio_dir);
            ql_gpio_set_level(_ql_gpio_cfg[num].gpio_num, gpio_lvl);

            //QL_GPIODEMO_LOG("gpio[%d] output high-level\n", _ql_gpio_cfg[num].gpio_num);
            //QL_GPIODEMO_LOG("gpio[%d] set dir:[%d], lvl:[%d]\n", _ql_gpio_cfg[num].gpio_num, gpio_dir, gpio_lvl);

             get output high 
            ql_gpio_get_direction(_ql_gpio_cfg[num].gpio_num, &gpio_dir);
            ql_gpio_get_level(_ql_gpio_cfg[num].gpio_num, &gpio_lvl);

            //QL_GPIODEMO_LOG("gpio[%d] output high-level\n", _ql_gpio_cfg[num].gpio_num);
            //QL_GPIODEMO_LOG("gpio[%d] get dir:[%d], lvl:[%d]\n", _ql_gpio_cfg[num].gpio_num, gpio_dir, gpio_lvl);
        }
        ql_rtos_task_sleep_s(3);

         input pull-down test 
        gpio_dir  = GPIO_INPUT;
        gpio_pull = PULL_DOWN;
        for( num = 0; num < sizeof(_ql_gpio_cfg)/sizeof(_ql_gpio_cfg[0]); num++ )
        {
             set input pull-down 
            ql_gpio_set_direction(_ql_gpio_cfg[num].gpio_num, gpio_dir);
            ql_gpio_set_pull(_ql_gpio_cfg[num].gpio_num, gpio_pull);

            //QL_GPIODEMO_LOG("gpio[%d] input pull-down\n", _ql_gpio_cfg[num].gpio_num);
            //QL_GPIODEMO_LOG("gpio[%d] set dir:[%d], pull:[%d]\n", _ql_gpio_cfg[num].gpio_num, gpio_dir, gpio_pull);

             get input pull-down 
            ql_gpio_get_direction(_ql_gpio_cfg[num].gpio_num, &gpio_dir);
            ql_gpio_get_pull(_ql_gpio_cfg[num].gpio_num, &gpio_pull);
            ql_gpio_get_level(_ql_gpio_cfg[num].gpio_num, &gpio_lvl);

            //QL_GPIODEMO_LOG("gpio[%d] input pull-down\n", _ql_gpio_cfg[num].gpio_num);
            //QL_GPIODEMO_LOG("gpio[%d] get dir:[%d], pull:[%d], lvl:[%d]\n", _ql_gpio_cfg[num].gpio_num, gpio_dir, gpio_pull, gpio_lvl);
        }
        ql_rtos_task_sleep_s(3);

         input pull-up test 
        gpio_dir  = GPIO_INPUT;
        gpio_pull = PULL_UP;
        for( num = 0; num < sizeof(_ql_gpio_cfg)/sizeof(_ql_gpio_cfg[0]); num++ )
        {
             set input pull-up 
            ql_gpio_set_direction(_ql_gpio_cfg[num].gpio_num, gpio_dir);
            ql_gpio_set_pull(_ql_gpio_cfg[num].gpio_num, gpio_pull);

            //QL_GPIODEMO_LOG("gpio[%d] input pull-up\n", _ql_gpio_cfg[num].gpio_num);
            //QL_GPIODEMO_LOG("gpio[%d] set dir:[%d], pull:[%d]\n", _ql_gpio_cfg[num].gpio_num, gpio_dir, gpio_pull);

             get input pull-up 
            ql_gpio_get_direction(_ql_gpio_cfg[num].gpio_num, &gpio_dir);
            ql_gpio_get_pull(_ql_gpio_cfg[num].gpio_num, &gpio_pull);
            ql_gpio_get_level(_ql_gpio_cfg[num].gpio_num, &gpio_lvl);

            //QL_GPIODEMO_LOG("gpio[%d] input pull-up\n", _ql_gpio_cfg[num].gpio_num);
            //QL_GPIODEMO_LOG("gpio[%d] get dir:[%d], pull:[%d], lvl:[%d]\n", _ql_gpio_cfg[num].gpio_num, gpio_dir, gpio_pull, gpio_lvl);
        }
        ql_rtos_task_sleep_s(3);

         change GPIO's Pin 
        if( change_flg == 0 )
        {
            ql_pin_set_func(QL_TEST1_PIN_GPIO0, QL_TEST1_PIN_GPIO0_FUNC_FLASH1_CLK);    // TEST1_PIN set spi_flash1_clk
            ql_pin_set_func(QL_TEST1_PIN_GPIO1, QL_TEST1_PIN_GPIO1_FUNC_FLASH1_CS);     // TEST1_PIN set spi_flash1_cs

            ql_pin_set_func(QL_TEST2_PIN_GPIO0, QL_TEST2_PIN_GPIO0_FUNC_GPIO);          // TEST2_PIN set GPIO0
            ql_pin_set_func(QL_TEST2_PIN_GPIO1, QL_TEST2_PIN_GPIO1_FUNC_GPIO);          // TEST2_PIN set GPIO1

            //QL_GPIODEMO_LOG("GPIO0/1 pin is TEST1 -> TEST2");
            change_flg = 1;
        }
        else
        {
            ql_pin_set_func(QL_TEST1_PIN_GPIO0, QL_TEST1_PIN_GPIO0_FUNC_GPIO);          // TEST1_PIN set GPIO0
            ql_pin_set_func(QL_TEST1_PIN_GPIO1, QL_TEST1_PIN_GPIO1_FUNC_GPIO);          // TEST1_PIN set GPIO1

            ql_pin_set_func(QL_TEST2_PIN_GPIO0, QL_TEST2_PIN_GPIO0_FUNC_LCD_SIO);       // TEST2_PIN set spi_lcd_sio
            ql_pin_set_func(QL_TEST2_PIN_GPIO1, QL_TEST2_PIN_GPIO1_FUNC_LCD_SDC);       // TEST2_PIN set spi_lcd_sdc

            //QL_GPIODEMO_LOG("GPIO0/1 pin is TEST2 -> TEST1");
            change_flg = 0;
        }
        */
    //}

    /*ql_rtos_task_delete(NULL);

    exit:
    err = ql_rtos_task_delete(NULL);
	if(err != QL_OSI_SUCCESS)
	{
		QL_UART_DEMO_LOG("task deleted failed");
	}*/
  }
}

void my_lib_app_init(void)
{
    QlOSStatus err = QL_OSI_SUCCESS;
    ql_task_t my_lib_task = NULL;

    err = ql_rtos_task_create(&my_lib_task, 1024, APP_PRIORITY_NORMAL, "my_lib\n", my_lib_demo_thread, NULL, 1);
    if( err != QL_OSI_SUCCESS )
    {
        QL_GPIODEMO_LOG("my_lib task created failed");
    }
}


