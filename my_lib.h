/*=================================================================

						EDIT HISTORY FOR MODULE

This section contains comments describing changes made to the module.
Notice that changes are listed in reverse chronological order.

WHEN			  WHO		  WHAT, WHERE, WHY
------------	 -------	 -------------------------------------------------------------------------------

=================================================================*/


#ifndef _MY_LIB_H
#define _MY_LIB_H
#include "gpio_demo.h"
#include "ql_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * Macro Definition
 ===========================================================================*/

/*===========================================================================
 * Struct
 ===========================================================================*/


/*===========================================================================
 * Functions declaration
 ===========================================================================*/
void my_lib_app_init(void);
void my_lib_demo_thread(void *param);
void my_ql_gpio_demo_init(  void);

#ifdef __cplusplus
} /*"C" */
#endif

#endif /* _MY_LIB_H */


