#ifndef __VMT_COMMAND_H__
#define __VMT_COMMAND_H__

// #pragma anon_unions
#include <stdint.h>

#include "vmt_uart.h"
#include "vmt_string.h"

/* ==========================================================================
   CONSTANTS & PARAMETERS
   ========================================================================== */
#define CMD_VER_MAIN (1)
#define CMD_VER_SUB  (7)

/* ==========================================================================
   CORE LIFECYCLE API
   ========================================================================== */
void cmd_init(void);
void cmd_process(void);

/* ==========================================================================
   STATE REPORTING API
   ========================================================================== */
void cmd_print_bottom(uart_select_t channel);
void cmd_print_water(uart_select_t channel);
void cmd_print_wake(uart_select_t channel);
void cmd_print_init_g_log(uart_select_t channel);

/* ==========================================================================
   LOGGING & DATA EXPORT API
   ========================================================================== */
void cmd_print_log_save(uart_select_t channel);
void cmd_print_log_report_busy(uart_select_t channel);
void cmd_print_log_report_data(uart_select_t channel);
void cmd_print_log_erase(uart_select_t channel);

/* ==========================================================================
   ASYNCHRONOUS FLASH/FIFO CALLBACKS
   ========================================================================== */
void cmd_print_flash_fifo_write_finish(uart_select_t channel);
void cmd_print_flash_fifo_delete_finish(uart_select_t channel);
void cmd_print_flash_fifo_mark_finish(uart_select_t channel);

#endif /* __VMT_COMMAND_H__ */
