#ifndef __VMT_FLASH_FIFO_H__
#define __VMT_FLASH_FIFO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "vmt_flash.h"

/* define */
#define FLASH_FIFO_VER_MAIN (0)
#define FLASH_FIFO_VER_SUB  (18)

/* Naming instructions
 * memory : used flash memory.
 * EU : erase unit, flash erase unit size.
 * pack : data pack.
 * PU : program unit, flash program unit size.
 *
 * addr_begin : used flash memory address begin.
 * memory_size : used flash memory size.
 *   for example : used flash address 0x08000000 ~ 0x08001000.
 *                 addr_begin = 0x08000000.
 *                 memory_size = 0x1000.
 *
 * FIFO_seq = EU_seq * pack_per_eu + pack_seq
 * */

/* type define */
typedef struct h_flash_fifo_cfg_
{
    size_t addr_begin; // used memory address begin.
    size_t pu_size;    // program unit size.
    size_t pack_size;
    size_t eu_size;     // erase unit size.
    size_t memory_size; // used memory size.
    size_t eu_in_memory;
    size_t pack_per_eu;
    size_t pu_per_pack;
    size_t pack_in_memory;
    size_t data_size_max;
} h_flash_fifo_cfg_t;

typedef enum flash_fifo_res_
{
    FLASH_FIFO_RES_ERROR, /* internal process error. */
    FLASH_FIFO_RES_OK,
    FLASH_FIFO_RES_BUSY,
    FLASH_FIFO_RES_ERROR_PARA, /* input parameter error. */
    FLASH_FIFO_RES_ERROR_HAL,  /* HAL layer error. */
    FLASH_FIFO_RES_ERROR_CRC,  /* CRC check error. */
    FLASH_FIFO_RES_ERASED,     /* data pack is erased */
    FLASH_FIFO_RES_NO_DATA,
    FLASH_FIFO_RES_NO_SPACE,
    FLASH_FIFO_RES_NOT_FOUND,
    FLASH_FIFO_RES_NOT_ERASED, /* data pack is not erased */
} flash_fifo_res_t;

typedef struct h_flash_fifo_init_
{
    uint8_t  step;
    size_t   seq_check;
    size_t   seq_first;
    size_t   seq_log;
    uint16_t serial_now;
} h_flash_fifo_init_t;

typedef struct __attribute__((packed)) h_flash_fifo_header_
{
    uint8_t  data_size;
    uint8_t  mark;
    uint16_t serial_seq;
    uint16_t crc;
} h_flash_fifo_header_t;
#define FLASH_FIFO_HEADER_DATA_SIZE_ERASED (0xFF)
#define FLASH_FIFO_HEADER_DATA_SIZE_DELETE (0x00)

typedef struct h_flash_fifo_order_
{
    void                  *p_data;
    size_t                 data_size;
    h_flash_fifo_header_t *p_h_header; // header buffer pointer to read.
} h_flash_fifo_order_t;

typedef struct h_flash_fifo_write_
{
    uint8_t                step;
    bool                   b_busy;
    flash_fifo_res_t       res;
    void                  *p_data;
    size_t                 data_size;
    h_flash_fifo_header_t *p_h_header_order;
    h_flash_fifo_header_t  h_header;
    size_t                 pack_addr_header;
    size_t                 pack_addr_data;
} h_flash_fifo_write_t;

typedef struct h_flash_fifo_mark_
{
    size_t  seq;
    uint8_t mark;
} h_flash_fifo_mark_t;

typedef struct h_flash_fifo_erase_
{
    size_t eu_seq_begin;
    size_t eu_n;
    size_t eu_count;
    size_t eu_erase;
} h_flash_fifo_erase_t;

typedef struct h_flash_fifo_debug_
{
    bool b_init      : 1;
    bool b_write     : 1;
    bool b_read      : 1;
    bool b_delete    : 1;
    bool b_mark      : 1;
    bool b_erase     : 1;
    bool b_available : 1;
} h_flash_fifo_debug_t;

typedef struct h_flash_fifo_
{
    uint8_t                  id;
    const h_flash_fifo_cfg_t h_cfg;
    h_flash_fifo_init_t      h_init;
    h_flash_fifo_debug_t     h_debug;

    /* used parameter, can read but don't modify */
    size_t fifo_back;
    size_t fifo_front;
    bool   b_init_finish : 1, b_mark_busy : 1, b_delete_busy : 1, b_erase_busy : 1;
    bool   b_wait_to_erase : 1;

    h_flash_fifo_write_t h_write;
    h_flash_fifo_mark_t  h_mark;
    h_flash_fifo_erase_t h_erase;
    size_t               delete_seq;
} h_flash_fifo_t;

/* marco */
#define FLASH_FIFO_MEMORY_SIZE(__EU_SIZE, __EU_IN_MEMORY)        (__EU_SIZE * __EU_IN_MEMORY)
#define FLASH_FIFO_PACK_SIZE(__EU_SIZE, __PACK_PER_EU)           (__EU_SIZE / __PACK_PER_EU)
#define FLASH_FIFO_PACK_PER_EU(__EU_SIZE, __PACK_SIZE)           (__EU_SIZE / __PACK_SIZE)
#define FLASH_FIFO_EU_IN_MEMORY(__MEMORY_SIZE, __EU_SIZE)        (__MEMORY_SIZE / __EU_SIZE)
#define FLASH_FIFO_PU_PER_PACK(__PU_SIZE, __PACK_SIZE)           (__PACK_SIZE / __PU_SIZE)
#define FLASH_FIFO_PACK_IN_MEMORY(__PACK_PER_EU, __EU_IN_MEMORY) (__PACK_PER_EU * __EU_IN_MEMORY)

#define FLASH_FIFO_DATA_SIZE_MAX(__PACK_SIZE) (__PACK_SIZE - sizeof(h_flash_fifo_header_t))

#define FLASH_FIFO_EU_SEQ_GET(__H_CFG, __FIFO_SEQ)   (__FIFO_SEQ / (__H_CFG)->pack_per_eu)
#define FLASH_FIFO_PACK_SEQ_GET(__H_CFG, __FIFO_SEQ) (__FIFO_SEQ % (__H_CFG)->pack_per_eu)
#define FLASH_FIFO_EU_ADDR_GET(__H_CFG, __EU_SEQ)    ((__H_CFG)->addr_begin + __EU_SEQ * (__H_CFG)->eu_size)
#define FLASH_FIFO_EU_POINTER_GET(__H_CFG, __EU_SEQ) ((void *)FLASH_FIFO_EU_ADDR_GET(__H_CFG, __EU_SEQ))
#define FLASH_FIFO_PACK_ADDR_GET(__H_CFG, __FIFO_SEQ) \
    (FLASH_FIFO_EU_ADDR_GET(__H_CFG, FLASH_FIFO_EU_SEQ_GET(__H_CFG, __FIFO_SEQ)) + FLASH_FIFO_PACK_SEQ_GET(__H_CFG, __FIFO_SEQ) * (__H_CFG)->pack_size)
#define FLASH_FIFO_PACK_POINTER_GET(__H_CFG, __FIFO_SEQ)            ((void *)FLASH_FIFO_PACK_ADDR_GET(__H_CFG, __FIFO_SEQ))
#define FLASH_FIFO_PACK_DATA_ADDR_GET(__H_CFG, __HEADER_ADDR)       (__HEADER_ADDR + sizeof(h_flash_fifo_header_t))
#define FLASH_FIFO_PACK_DATA_POINTER_GET(__H_CFG, __HEADER_POINTER) ((void *)(((size_t)__HEADER_POINTER) + sizeof(h_flash_fifo_header_t)))
#define FLASH_FIFO_FIFO_SEQ_ADD(__H_CFG, __FIFO_SEQ, __ADD)         ((__FIFO_SEQ + __ADD) % __H_CFG->pack_in_memory)
#define FLASH_FIFO_FIFO_SEQ_NEXT(__H_CFG, __FIFO_SEQ)               ((__FIFO_SEQ + 1) % __H_CFG->pack_in_memory)

/* parameter */
#define FLASH_FIFO_1_ADDR_BEGIN   (0x08020000)
#define FLASH_FIFO_1_PU_SIZE      (1)
#define FLASH_FIFO_1_EU_SIZE      (0x20000)
#define FLASH_FIFO_1_EU_SEQ_BEGIN (FLASH_SECTOR_5)
#define FLASH_FIFO_1_EU_IN_MEMORY (3)
#define FLASH_FIFO_1_PACK_SIZE    (32)
// #define FLASH_FIFO_1_PACK_SIZE (256)

#define FLASH_FIFO_1_MEMORY_SIZE    (FLASH_FIFO_MEMORY_SIZE(FLASH_FIFO_1_EU_SIZE, FLASH_FIFO_1_EU_IN_MEMORY))
#define FLASH_FIFO_1_PACK_PER_EU    (FLASH_FIFO_PACK_PER_EU(FLASH_FIFO_1_EU_SIZE, FLASH_FIFO_1_PACK_SIZE))
#define FLASH_FIFO_1_PU_PER_PACK    (FLASH_FIFO_PU_PER_PACK(FLASH_FIFO_1_PU_SIZE, FLASH_FIFO_1_PACK_SIZE))
#define FLASH_FIFO_1_PACK_IN_MEMORY (FLASH_FIFO_PACK_IN_MEMORY(FLASH_FIFO_1_PACK_PER_EU, FLASH_FIFO_1_EU_IN_MEMORY))

#define FLASH_FIFO_1_DATA_SIZE_MAX (FLASH_FIFO_DATA_SIZE_MAX(FLASH_FIFO_1_PACK_SIZE))

/* variable */
extern h_flash_fifo_t h_flash_fifo_1;

/* function */
void flash_fifo_write_finish_cb(h_flash_fifo_t *p_h_flash, flash_fifo_res_t res);
void flash_fifo_delete_finish_cb(h_flash_fifo_t *p_h_flash, size_t seq);
void flash_fifo_mark_set_finish_cb(h_flash_fifo_t *p_h_flash, h_flash_fifo_mark_t *p_h_mark);
void flash_fifo_erase_finish_cb(h_flash_fifo_t *p_h_flash);

/* return value: true == busy, false == idle,
 * */
bool flash_fifo_process(h_flash_fifo_t *p_h_flash);

/* return value:
 *   FLASH_FIFO_RES_BUSY
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_NO_SPACE
 *   FLASH_FIFO_RES_OK
 *   FLASH_FIFO_RES_NOT_ERASED
 * */
flash_fifo_res_t flash_fifo_write(h_flash_fifo_t *p_h_flash, h_flash_fifo_order_t *p_h_order);

size_t flash_fifo_pack_n_get_saved(h_flash_fifo_t *p_h_flash);
size_t flash_fifo_pack_n_get_free(h_flash_fifo_t *p_h_flash);

/* return value:
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_NO_DATA
 *   FLASH_FIFO_RES_ERASED
 *   FLASH_FIFO_RES_ERROR_CRC
 *   FLASH_FIFO_RES_OK
 * */
flash_fifo_res_t flash_fifo_available_get(h_flash_fifo_t *p_h_flash, size_t seq);

/* return value:
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_NO_DATA
 *   FLASH_FIFO_RES_BUSY
 *   FLASH_FIFO_RES_ERASED
 *   FLASH_FIFO_RES_ERROR_CRC
 *   FLASH_FIFO_RES_OK
 * */
flash_fifo_res_t flash_fifo_read_front(h_flash_fifo_t *p_h_flash, h_flash_fifo_order_t *p_h_order);

/* return value:
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_NO_DATA
 *   FLASH_FIFO_RES_BUSY
 *   FLASH_FIFO_RES_ERASED
 *   FLASH_FIFO_RES_ERROR_CRC
 *   FLASH_FIFO_RES_OK
 * */
flash_fifo_res_t flash_fifo_read_seq(h_flash_fifo_t *p_h_flash, size_t seq, h_flash_fifo_order_t *p_h_order);

/* return value:
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_NO_DATA
 *   FLASH_FIFO_RES_ERASED
 *   FLASH_FIFO_RES_ERROR_CRC
 *   FLASH_FIFO_RES_OK
 * */
flash_fifo_res_t flash_fifo_header_get(h_flash_fifo_t *p_h_flash, size_t seq, h_flash_fifo_header_t *p_h_header);

/* return value:
 *   FLASH_FIFO_RES_OK
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_BUSY
 *   FLASH_FIFO_RES_NO_SPACE
 * */
flash_fifo_res_t flash_fifo_mark_set(h_flash_fifo_t *p_h_flash, h_flash_fifo_mark_t *p_h_mark);

/* return value:
 *   FLASH_FIFO_RES_OK
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_BUSY
 *   FLASH_FIFO_RES_NO_DATA
 * */
flash_fifo_res_t flash_fifo_delete_front(h_flash_fifo_t *p_h_flash);

/* return value:
 *   FLASH_FIFO_RES_OK
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_BUSY
 * */
flash_fifo_res_t flash_fifo_delete_seq(h_flash_fifo_t *p_h_flash, size_t seq);

/* return value:
 *   FLASH_FIFO_RES_OK
 *   FLASH_FIFO_RES_BUSY
 * */
flash_fifo_res_t flash_fifo_erase_all(h_flash_fifo_t *p_h_flash);

/* return value:
 *   FLASH_FIFO_RES_OK
 *   FLASH_FIFO_RES_BUSY
 *   FLASH_FIFO_RES_NOT_FOUND
 * */
flash_fifo_res_t flash_fifo_erase_dirty(h_flash_fifo_t *p_h_flash);

bool             flash_fifo_init_finish_busy_get(h_flash_fifo_t *p_h_flash);
bool             flash_fifo_write_busy_get(h_flash_fifo_t *p_h_flash);
flash_fifo_res_t flash_fifo_write_res_get(h_flash_fifo_t *p_h_flash);
void             flash_fifo_write_error_clear(h_flash_fifo_t *p_h_flash);
bool             flash_fifo_delete_busy_get(h_flash_fifo_t *p_h_flash);
bool             flash_fifo_mark_busy_get(h_flash_fifo_t *p_h_flash);
bool             flash_fifo_erase_busy_get(h_flash_fifo_t *p_h_flash);
bool             flash_fifo_wait_to_erase_get(h_flash_fifo_t *p_h_flash);

#ifdef __cplusplus
}
#endif
#endif //__VMT_FLASH_FIFO_H__
