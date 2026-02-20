/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VMT_UART_H__
#define __VMT_UART_H__

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>


#include "usart.h"

/* parameter */
#define UART_VER_MAIN (1)
#define UART_VER_SUB  (8)

#define UART_RX_BUFF_LEN (512)
#define UART_TX_BUFF_LEN (256)

/* type define */
typedef enum uart_select_
{
    UART_FIRST,
    UART_1      = UART_FIRST,
    UART_UPHOLE = UART_1,
    UART_3,
    UART_VOID = UART_3,
    UART_6,
    UART_DEBUG = UART_6,
    UART_NUMBER,
    UART_ALL,
} uart_select_t;

typedef struct h_uart_tx_
{
    uint8_t *p_buff;
    size_t   buff_size, front, back;
} h_uart_tx_t;

typedef struct h_uart_rx_
{
    uint8_t *p_buff;
    size_t   buff_size, back;
} h_uart_rx_t;

typedef struct
{
    UART_HandleTypeDef *p_h_hal;
    USART_TypeDef      *p_reg;
    DMA_HandleTypeDef  *p_dma_rx;
    h_uart_tx_t         h_tx;
    h_uart_rx_t         h_rx;
} h_uart_t;

/* function declaration */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* Check if it is actually ARM Compiler 6 (which defines __GNUC__ but needs fputc) */
  #if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6000000)
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  #else
    /* True GCC/Raisonance */
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  #endif
#else
  /* Legacy ARM Compiler 5 or other */
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

bool uart_rx_update(uart_select_t uart_channel);
bool uart_tx_update(uart_select_t uart_channel);

void uart_init(void);
void uart_init_it(UART_HandleTypeDef *huart);
void uart_deinit(void);

void          uart_tx_channel_set(uart_select_t ch);
uart_select_t uart_tx_channel_get(void);
uart_select_t uart_tx_channel_undo(void);
bool          uart_tx_enqueue(uart_select_t uart_channel, uint8_t data);

bool   uart_tx_front_add(uart_select_t ch, size_t *p_front, size_t gain);
size_t uart_tx_data_len_get(uart_select_t uart_channel);
void   uart_tx_wait_sent(uart_select_t ch, uint32_t timeout);

void uart_tx_en_set(uart_select_t ch, bool b_enalbe);

/**
 * @param  ch:    uart channel
 * @param  front: get data from front to rear
 * @param  ptr:   Pointer to the data to be read
 * @param  len:   Number of bytes to read
 * @param  index: Pointer to number of bytes read
 */
bool uart_rx_dequeue_it(uart_select_t ch, size_t front, uint8_t *ptr, size_t len, size_t *p_index);
/**
 * @param  ch:    uart channel
 * @param  front: get data from front to rear
 * @param  ptr:   Pointer to the data to be read
 * @param  len:   Number of bytes to read
 * @param  index: Pointer to number of bytes read
 */
bool   uart_rx_dequeue_dma(uart_select_t ch, size_t front, uint8_t *ptr, size_t len, size_t *p_index);
bool   uart_rx_front_add(uart_select_t ch, size_t *p_front, size_t gain);
size_t uart_rx_back_get(uart_select_t ch);
void   uart_rx_en_set(uart_select_t ch, bool b_enalbe);

/* Inline functions at the end */
static inline void debug_send(const char *fmt, ...)
{
    va_list args;
    uart_tx_channel_set(UART_DEBUG);
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
    uart_tx_channel_undo();
}

static inline void uphole_send(const char *fmt, ...)
{
    va_list args;
    uart_tx_channel_set(UART_UPHOLE);
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
    uart_tx_channel_undo();
}

#endif /* __VMT_UART_H__ */
