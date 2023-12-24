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


void my_lib_demo_thread(void *param)
{
    while(1)
    {

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


