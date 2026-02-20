#include "vmt_uart.h"

/* variable */
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

static uart_select_t uart_channel     = UART_DEBUG;
static uart_select_t uart_channel_log = UART_DEBUG;

static uint8_t uart_rx_buff[UART_NUMBER][UART_RX_BUFF_LEN] = { 0 };
static uint8_t uart_tx_buff[UART_NUMBER][UART_TX_BUFF_LEN] = { 0 };

static h_uart_t h_uart[UART_NUMBER] = {
    [UART_1] = {
        .p_h_hal = &huart1,
        .p_reg = USART1,
        .p_dma_rx = &hdma_usart1_rx,
        .h_tx = {
            .p_buff = uart_tx_buff[UART_1],
            .buff_size = UART_TX_BUFF_LEN, },
        .h_rx = {
            .p_buff = uart_rx_buff[UART_1],
            .buff_size = UART_RX_BUFF_LEN, }, },
    [UART_3] = {
        .p_h_hal = &huart3,
        .p_reg = USART3,
        .p_dma_rx = &hdma_usart3_rx,
        .h_tx = {
            .p_buff = uart_tx_buff[UART_3],
            .buff_size = UART_TX_BUFF_LEN, },
        .h_rx = {
            .p_buff = uart_rx_buff[UART_3],
            .buff_size = UART_RX_BUFF_LEN, }, },
    [UART_6] = {
        .p_h_hal = &huart6,
        .p_reg = USART6,
        .p_dma_rx = &hdma_usart6_rx,
        .h_tx = {
            .p_buff = uart_tx_buff[UART_6],
            .buff_size = UART_TX_BUFF_LEN, },
        .h_rx = {
            .p_buff = uart_rx_buff[UART_6],
            .buff_size = UART_RX_BUFF_LEN, }, }, };

// static const uint32_t uart_sr_rx_it = UART_FLAG_RXNE | UART_FLAG_ORE
//    | UART_FLAG_NE;
static const uint32_t uart_sr_rx_it = UART_FLAG_RXNE | UART_FLAG_ORE;

/* function */
bool uart_rx_update(uart_select_t uart_channel)
{
    static USART_TypeDef *p_reg;
    static uint32_t       uart_sr;
    static h_uart_rx_t   *p_h_uart;
    static uint8_t        data;

    p_reg   = h_uart[uart_channel].p_reg;
    uart_sr = p_reg->ISR;
    if ((uart_sr & uart_sr_rx_it) == 0x0)
        return false;

    data = p_reg->RDR;
    if ((uart_sr & UART_FLAG_RXNE) == 0x0)
        return false;

    p_h_uart = &h_uart[uart_channel].h_rx;

    p_h_uart->back %= p_h_uart->buff_size;
    p_h_uart->p_buff[p_h_uart->back] = data;
    p_h_uart->back                   = (p_h_uart->back + 1) % p_h_uart->buff_size;
    return true;
}

bool uart_tx_update(uart_select_t uart_channel)
{
    USART_TypeDef *p_reg = h_uart[uart_channel].p_reg;
    if ((p_reg->ISR & UART_FLAG_TC) == 0x0)
        return false;

    h_uart_tx_t *p_h_uart = &h_uart[uart_channel].h_tx;

    if (p_h_uart->front != p_h_uart->back)
    {
        p_h_uart->front %= p_h_uart->buff_size;
        p_reg->TDR      = p_h_uart->p_buff[p_h_uart->front];
        p_h_uart->front = (p_h_uart->front + 1) % p_h_uart->buff_size;
        p_reg->CR1 |= USART_CR1_TCIE;
    }
    else
    {
        p_reg->CR1 &= ~USART_CR1_TCIE;
    }
    return true;
}

void uart_init(void)
{
    printf("uart init,");
    printf("ver:%d.%d\r\n", UART_VER_MAIN, UART_VER_SUB);
    for (uart_select_t uart_n = UART_FIRST; uart_n < UART_NUMBER; uart_n++)
    {
        printf("uart,%d,%d\r\n", uart_n, h_uart[uart_n].p_h_hal->Init.BaudRate);
        //        uart_init_it(h_uart[uart_n].p_h_hal);
        h_uart_t *p_h_uart = &h_uart[uart_n];
        HAL_UART_Receive_DMA(p_h_uart->p_h_hal, p_h_uart->h_rx.p_buff, p_h_uart->h_rx.buff_size);
        CLEAR_BIT(p_h_uart->p_reg->CR3, USART_CR3_EIE);
    }
}

void uart_init_it(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
        return;
    __HAL_UART_FLUSH_DRREGISTER(huart);
    huart->Instance->CR1 |= USART_CR1_RXNEIE;
}

void uart_deinit(void)
{
    for (uart_select_t i = UART_FIRST; i < UART_NUMBER; i++)
        HAL_UART_DeInit(h_uart[i].p_h_hal);
}

void uart_tx_channel_set(uart_select_t ch)
{
    uart_channel_log = uart_channel;
    uart_channel     = ch;
}

uart_select_t uart_tx_channel_get(void)
{
    return uart_channel;
}

uart_select_t uart_tx_channel_undo(void)
{
    uart_channel = uart_channel_log;
    return uart_channel;
}

bool uart_tx_enqueue(uart_select_t uart_channel, uint8_t data)
{
    h_uart_tx_t *p_h_uart = &h_uart[uart_channel].h_tx;

    size_t back_next = (p_h_uart->back + 1) % p_h_uart->buff_size;
    if (back_next == p_h_uart->front)
        return false;

    p_h_uart->back %= p_h_uart->buff_size;
    p_h_uart->p_buff[p_h_uart->back] = data;
    p_h_uart->back                   = back_next;

    h_uart[uart_channel].p_reg->CR1 |= USART_CR1_TCIE;
    return true;
}

bool uart_tx_front_add(uart_select_t ch, size_t *p_front, size_t gain)
{
    if (ch >= UART_NUMBER)
        return false;

    *p_front = (*p_front + gain) % h_uart[ch].h_tx.buff_size;
    return true;
}

size_t uart_tx_data_len_get(uart_select_t uart_channel)
{
    h_uart_tx_t *p_h_uart = &h_uart[uart_channel].h_tx;
    return (p_h_uart->back + p_h_uart->buff_size - p_h_uart->front) % p_h_uart->buff_size;
}

void uart_tx_wait_sent(uart_select_t ch, uint32_t timeout)
{
    uint32_t time = HAL_GetTick();
    while (uart_tx_data_len_get(ch))
    {
        if (HAL_GetTick() - time > timeout)
            break;
    }
}

void uart_tx_en_set(uart_select_t ch, bool b_enalbe)
{
    if (ch >= UART_NUMBER)
        return;

    if (b_enalbe)
    {
        h_uart[ch].p_reg->CR1 |= USART_CR1_TE;
    }
    else
    {
        h_uart[ch].p_reg->CR1 &= ~USART_CR1_TE;
    }
}

/**
 * @param  ch:    uart channel
 * @param  front: get data from front to rear
 * @param  ptr:   Pointer to the data to be read
 * @param  len:   Number of bytes to read
 * @param  index: Pointer to number of bytes read
 */
bool uart_rx_dequeue_it(uart_select_t ch, size_t front, uint8_t *ptr, size_t len, size_t *p_index)
{
    if (ch >= UART_NUMBER)
        return false;
    h_uart_rx_t *p_h_uart = &h_uart[ch].h_rx;

    if (front >= p_h_uart->buff_size)
        front = 0;
    for (*p_index = 0; *p_index < len; (*p_index)++)
    {
        if (p_h_uart->back == front)
            break;
        ptr[*p_index] = p_h_uart->p_buff[front];
        front         = (front + 1) % p_h_uart->buff_size;
    }
    return true;
}

/**
 * @param  ch:    uart channel
 * @param  front: get data from front to rear
 * @param  ptr:   Pointer to the data to be read
 * @param  len:   Number of bytes to read
 * @param  index: Pointer to number of bytes read
 */
bool uart_rx_dequeue_dma(uart_select_t ch, size_t front, uint8_t *ptr, size_t len, size_t *p_index)
{
    if (ch >= UART_NUMBER)
        return false;
    h_uart_t    *p_h_uart  = &h_uart[ch];
    h_uart_rx_t *p_h_rx    = &p_h_uart->h_rx;
    size_t       buff_size = p_h_rx->buff_size;

    size_t q_back = buff_size - __HAL_DMA_GET_COUNTER(p_h_uart->p_dma_rx);
    if (q_back >= buff_size)
        q_back = 0;
    uint8_t *p_buff = p_h_rx->p_buff;
    size_t   index  = *p_index;
    for (index = 0; index < len; index++)
    {
        if (front >= buff_size)
            front = 0;
        if (q_back == front)
            break;
        ptr[index] = p_buff[front];
        front++;
    }
    *p_index     = index;
    p_h_rx->back = q_back;
    return true;
}

bool uart_rx_front_add(uart_select_t ch, size_t *p_front, size_t gain)
{
    if (ch >= UART_NUMBER)
        return false;

    *p_front = (*p_front + gain) % h_uart[ch].h_rx.buff_size;
    return true;
}

size_t uart_rx_back_get(uart_select_t ch)
{
    if (ch >= UART_NUMBER)
        return 0;
    return h_uart[ch].h_rx.back;
}

void uart_rx_en_set(uart_select_t ch, bool b_enalbe)
{
    if (ch >= UART_NUMBER)
        return;

    if (b_enalbe)
    {
        h_uart[ch].p_reg->CR1 |= USART_CR1_RE;
    }
    else
    {
        h_uart[ch].p_reg->CR1 &= ~USART_CR1_RE;
    }
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    uint32_t time = HAL_GetTick();

    if (uart_channel == UART_ALL)
    {
        for (uart_select_t select = UART_FIRST; select < UART_NUMBER; select++)
        {
            //  HAL_UART_Transmit(h_uart[select].p_h_hal, (uint8_t*) &ch, 1, 2);
            while (!uart_tx_enqueue(select, ch))
                ;
        }
    }
    else
    {
        //  HAL_UART_Transmit(h_uart[uart_channel].p_h_hal, (uint8_t*) &ch, 1, 2);
        while (!uart_tx_enqueue(uart_channel, ch))
            ;
    }

    return ch;
}
