#include "vmt_flash_fifo.h"
#include "vmt_device.h"
#include <string.h>
#include <stdio.h>

/* variable */
#define FLASH_CRC16_INITIAL_VALUE (0xFFFF)
// @formatter:off
static const uint16_t flash_CRC16_table[0x100] = {
    0x0000, 0xC1C0, 0x81C1, 0x4001, 0x01C3, 0xC003, 0x8002, 0x41C2, 0x01C6, 0xC006, 0x8007, 0x41C7, 0x0005, 0xC1C5, 0x81C4, 0x4004, 0x01CC, 0xC00C, 0x800D, 0x41CD,
    0x000F, 0xC1CF, 0x81CE, 0x400E, 0x000A, 0xC1CA, 0x81CB, 0x400B, 0x01C9, 0xC009, 0x8008, 0x41C8, 0x01D8, 0xC018, 0x8019, 0x41D9, 0x001B, 0xC1DB, 0x81DA, 0x401A,
    0x001E, 0xC1DE, 0x81DF, 0x401F, 0x01DD, 0xC01D, 0x801C, 0x41DC, 0x0014, 0xC1D4, 0x81D5, 0x4015, 0x01D7, 0xC017, 0x8016, 0x41D6, 0x01D2, 0xC012, 0x8013, 0x41D3,
    0x0011, 0xC1D1, 0x81D0, 0x4010, 0x01F0, 0xC030, 0x8031, 0x41F1, 0x0033, 0xC1F3, 0x81F2, 0x4032, 0x0036, 0xC1F6, 0x81F7, 0x4037, 0x01F5, 0xC035, 0x8034, 0x41F4,
    0x003C, 0xC1FC, 0x81FD, 0x403D, 0x01FF, 0xC03F, 0x803E, 0x41FE, 0x01FA, 0xC03A, 0x803B, 0x41FB, 0x0039, 0xC1F9, 0x81F8, 0x4038, 0x0028, 0xC1E8, 0x81E9, 0x4029,
    0x01EB, 0xC02B, 0x802A, 0x41EA, 0x01EE, 0xC02E, 0x802F, 0x41EF, 0x002D, 0xC1ED, 0x81EC, 0x402C, 0x01E4, 0xC024, 0x8025, 0x41E5, 0x0027, 0xC1E7, 0x81E6, 0x4026,
    0x0022, 0xC1E2, 0x81E3, 0x4023, 0x01E1, 0xC021, 0x8020, 0x41E0, 0x01A0, 0xC060, 0x8061, 0x41A1, 0x0063, 0xC1A3, 0x81A2, 0x4062, 0x0066, 0xC1A6, 0x81A7, 0x4067,
    0x01A5, 0xC065, 0x8064, 0x41A4, 0x006C, 0xC1AC, 0x81AD, 0x406D, 0x01AF, 0xC06F, 0x806E, 0x41AE, 0x01AA, 0xC06A, 0x806B, 0x41AB, 0x0069, 0xC1A9, 0x81A8, 0x4068,
    0x0078, 0xC1B8, 0x81B9, 0x4079, 0x01BB, 0xC07B, 0x807A, 0x41BA, 0x01BE, 0xC07E, 0x807F, 0x41BF, 0x007D, 0xC1BD, 0x81BC, 0x407C, 0x01B4, 0xC074, 0x8075, 0x41B5,
    0x0077, 0xC1B7, 0x81B6, 0x4076, 0x0072, 0xC1B2, 0x81B3, 0x4073, 0x01B1, 0xC071, 0x8070, 0x41B0, 0x0050, 0xC190, 0x8191, 0x4051, 0x0193, 0xC053, 0x8052, 0x4192,
    0x0196, 0xC056, 0x8057, 0x4197, 0x0055, 0xC195, 0x8194, 0x4054, 0x019C, 0xC05C, 0x805D, 0x419D, 0x005F, 0xC19F, 0x819E, 0x405E, 0x005A, 0xC19A, 0x819B, 0x405B,
    0x0199, 0xC059, 0x8058, 0x4198, 0x0188, 0xC048, 0x8049, 0x4189, 0x004B, 0xC18B, 0x818A, 0x404A, 0x004E, 0xC18E, 0x818F, 0x404F, 0x018D, 0xC04D, 0x804C, 0x418C,
    0x0044, 0xC184, 0x8185, 0x4045, 0x0187, 0xC047, 0x8046, 0x4186, 0x0182, 0xC042, 0x8043, 0x4183, 0x0041, 0xC181, 0x8180, 0x4040,
};
// @formatter:on

/* function declaration */
static bool flash_fifo_init_process(h_flash_fifo_t *p_h_flash);
static bool flash_fifo_write_process(h_flash_fifo_t *p_h_flash);
static bool flash_fifo_erase_process(h_flash_fifo_t *p_h_flash);

// void flash_fifo_config_update(h_flash_fifo_cfg_t *p_h_cfg);
static void flash_fifo_crc16(uint16_t *p_crc_value, void *ptr, size_t size);

/* variable */
h_flash_fifo_t h_flash_fifo_1 = {
    .h_cfg = {
        .addr_begin = FLASH_FIFO_1_ADDR_BEGIN,
        .pu_size = FLASH_FIFO_1_PU_SIZE,
        .pack_size = FLASH_FIFO_1_PACK_SIZE,
        .eu_size = FLASH_FIFO_1_EU_SIZE,
        .memory_size = FLASH_FIFO_1_MEMORY_SIZE,
        .eu_in_memory = FLASH_FIFO_1_EU_IN_MEMORY,
        .pack_per_eu = FLASH_FIFO_1_PACK_PER_EU,
        .pu_per_pack = FLASH_FIFO_1_PU_PER_PACK,
        .pack_in_memory = FLASH_FIFO_1_PACK_IN_MEMORY,
        .data_size_max = FLASH_FIFO_1_DATA_SIZE_MAX, },
    .h_debug = {
        .b_init = true,
        .b_write = true,
        .b_read = true,
        .b_delete = true,
        .b_erase = true,
        // .b_available = true,
            }, };

/* function */
__weak void flash_fifo_write_finish_cb(h_flash_fifo_t *p_h_flash, flash_fifo_res_t res)
{
}

__weak void flash_fifo_delete_finish_cb(h_flash_fifo_t *p_h_flash, size_t seq)
{
}

__weak void flash_fifo_mark_set_finish_cb(h_flash_fifo_t *p_h_flash, h_flash_fifo_mark_t *p_h_mark)
{
}
__weak void flash_fifo_erase_finish_cb(h_flash_fifo_t *p_h_flash)
{
}

bool flash_fifo_process(h_flash_fifo_t *p_h_flash)
{
    if (p_h_flash->b_init_finish == false)
    {
        return flash_fifo_init_process(p_h_flash);
    }

    bool b_busy = flash_fifo_write_process(p_h_flash);
    b_busy      = flash_fifo_erase_process(p_h_flash);

    if (p_h_flash->b_mark_busy)
    {
        if (flash_program_busy_get() == false)
        {
            p_h_flash->b_mark_busy = false;
            flash_fifo_mark_set_finish_cb(p_h_flash, &p_h_flash->h_mark);
        }
    }
    if (p_h_flash->b_delete_busy)
    {
        if (flash_program_busy_get() == false)
        {
            p_h_flash->b_delete_busy = false;
            flash_fifo_delete_finish_cb(p_h_flash, p_h_flash->delete_seq);
        }
    }
    return b_busy;
}

static bool flash_fifo_init_process(h_flash_fifo_t *p_h_flash)
{
    typedef enum step_
    {
        STEP_CHECK_START,
        STEP_IDLE,
        STEP_FIND_FIRST,
        STEP_FIND_LAST,
        STEP_CHECK_ERASE,
        STEP_FINISH,
    } step_t;

    const h_flash_fifo_cfg_t *p_h_cfg     = &p_h_flash->h_cfg;
    h_flash_fifo_init_t      *p_h_process = &p_h_flash->h_init;
    h_flash_fifo_debug_t     *p_h_debug   = &p_h_flash->h_debug;

    switch (p_h_process->step)
    {
    case STEP_CHECK_START:
        p_h_process->step = STEP_IDLE;
    case STEP_IDLE:
    {
        if (p_h_flash->b_init_finish)
            break;

        p_h_process->seq_check = 0;
        p_h_process->step      = STEP_FIND_FIRST;
    }
    case STEP_FIND_FIRST:
    {
        size_t seq_check = p_h_process->seq_check;
        if (seq_check < p_h_cfg->pack_in_memory)
        {
            h_flash_fifo_header_t *p_h_header;
            p_h_header = FLASH_FIFO_PACK_POINTER_GET(p_h_cfg, seq_check);
            if (p_h_header->data_size == FLASH_FIFO_HEADER_DATA_SIZE_ERASED)
            {
                p_h_process->seq_check++;
                break;
            }
            flash_fifo_res_t res;
            res = flash_fifo_available_get(p_h_flash, seq_check);
            if (res == FLASH_FIFO_RES_ERASED)
            {
                size_t pack_per_eu     = p_h_cfg->pack_per_eu;
                size_t next_eu         = seq_check / pack_per_eu + 1;
                p_h_process->seq_check = next_eu * pack_per_eu;
                break;
            }
            if (res != FLASH_FIFO_RES_OK)
            {
                p_h_process->seq_check++;
                break;
            }
        }
        else
        {
            /* not find available data,
             * use default value. */
            p_h_flash->fifo_back                   = 0;
            p_h_flash->fifo_front                  = 0;
            p_h_flash->h_write.h_header.serial_seq = 0;
            p_h_process->step                      = STEP_CHECK_ERASE;
            break;
        }

        /* find available data */
        p_h_process->seq_check = seq_check;
        p_h_process->seq_first = seq_check;
        p_h_process->seq_log   = seq_check;

        h_flash_fifo_header_t *p_h_header;
        p_h_header              = FLASH_FIFO_PACK_POINTER_GET(p_h_cfg, seq_check);
        p_h_process->serial_now = p_h_header->serial_seq;
        p_h_process->seq_check  = ++seq_check;

        p_h_process->step = STEP_FIND_LAST;
    }
    case STEP_FIND_LAST:
    {
        size_t                 seq_check = p_h_process->seq_check;
        flash_fifo_res_t       res;
        h_flash_fifo_header_t *p_h_header;
        uint16_t               serial_log;
        size_t                 pack_in_memory = p_h_cfg->pack_in_memory;
        size_t                 pack_per_eu    = p_h_cfg->pack_per_eu;
        if (seq_check < pack_in_memory)
        {
            if (seq_check % pack_per_eu == 0)
            {
                /* if erase_unit is erased,
                 * jump to next erase_unit. */
                p_h_header = FLASH_FIFO_PACK_POINTER_GET(p_h_cfg, seq_check);
                if (p_h_header->data_size == FLASH_FIFO_HEADER_DATA_SIZE_ERASED)
                {
                    p_h_process->seq_check += pack_per_eu;
                    break;
                }
            }
            res = flash_fifo_available_get(p_h_flash, seq_check);
            if (res == FLASH_FIFO_RES_ERASED)
            {
                size_t next_eu         = seq_check / pack_per_eu + 1;
                p_h_process->seq_check = next_eu * pack_per_eu;
                break;
            }
            if (res != FLASH_FIFO_RES_OK)
            {
                p_h_process->seq_check++;
                break;
            }
            p_h_header              = FLASH_FIFO_PACK_POINTER_GET(p_h_cfg, seq_check);
            serial_log              = p_h_process->serial_now;
            p_h_process->serial_now = p_h_header->serial_seq;
            int16_t serial_diff     = p_h_process->serial_now - serial_log;
            if (serial_diff >= 0)
            {
                p_h_process->seq_log = seq_check;
                p_h_process->seq_check++;
                break;
            }
        }
        size_t seq_last = p_h_process->seq_log;
        if (seq_check < pack_in_memory)
            p_h_process->seq_first = seq_check;

        p_h_flash->fifo_front               = p_h_process->seq_first;
        p_h_flash->fifo_back                = (seq_last + 1) % pack_in_memory;
        p_h_header                          = FLASH_FIFO_PACK_POINTER_GET(p_h_cfg, seq_last);
        h_flash_fifo_header_t *p_h_header_w = &p_h_flash->h_write.h_header;
        p_h_header_w->serial_seq            = p_h_header->serial_seq + 1;

        if (p_h_debug->b_init)
        {
            printf("> flash_fifo:i,found");
            printf(",%d", serial_log);
            printf(",%d", p_h_process->serial_now);
            printf(",%d", p_h_process->seq_check);
            printf(",%d", p_h_process->seq_first);
            printf(",%d", seq_last);
            printf(",%d", p_h_header_w->serial_seq);
            printf("\r\n");
        }
        p_h_process->step = STEP_CHECK_ERASE;
    }
    break;
    case STEP_CHECK_ERASE:
    {
        size_t eu_in_memory = p_h_cfg->eu_in_memory;

        size_t fifo_back  = p_h_flash->fifo_back;
        size_t fifo_front = p_h_flash->fifo_front;

        size_t eu_n         = 0;
        size_t begin_eu_seq = 0;

        do
        {
            size_t back_eu_seq = FLASH_FIFO_EU_SEQ_GET(p_h_cfg, fifo_back);
            if (fifo_back == fifo_front)
            {
                begin_eu_seq = back_eu_seq;
                eu_n         = eu_in_memory;
                break;
            }

            size_t front_eu_seq = FLASH_FIFO_EU_SEQ_GET(p_h_cfg, fifo_front);

            if ((back_eu_seq == front_eu_seq) && (fifo_back < fifo_front))
                break;

            size_t back_next = (fifo_back + 1) % p_h_cfg->pack_in_memory;
            if (back_next == fifo_front)
                break;

            if (FLASH_FIFO_PACK_SEQ_GET(p_h_cfg, fifo_back) == 0)
            {
                begin_eu_seq = back_eu_seq;
            }
            else
            {
                begin_eu_seq = (back_eu_seq + 1) % eu_in_memory;
            }
            eu_n = (eu_in_memory + front_eu_seq - begin_eu_seq) % eu_in_memory;
        }
        while (0);

        uint8_t                eu_seq = begin_eu_seq;
        h_flash_fifo_header_t *p_h_header;
        h_flash_fifo_erase_t  *p_h_erase = &p_h_flash->h_erase;
        for (size_t eu_count = 0; eu_count < eu_n; eu_count++)
        {
            p_h_header = FLASH_FIFO_EU_POINTER_GET(p_h_cfg, eu_seq);
            if (p_h_header->data_size != FLASH_FIFO_HEADER_DATA_SIZE_ERASED)
            {
                p_h_erase->eu_seq_begin    = eu_seq;
                p_h_erase->eu_erase        = eu_seq;
                p_h_erase->eu_n            = eu_n - eu_count;
                p_h_erase->eu_count        = 0;
                p_h_flash->b_wait_to_erase = true;
                break;
            }
            if (++eu_seq >= eu_in_memory)
                eu_seq = 0;
        }

        if (p_h_debug->b_init)
        {
            printf("> flash_fifo:i,check_erase");
            printf(",%d", p_h_flash->fifo_front);
            printf(",%d", p_h_flash->fifo_back);
            printf(",%d", begin_eu_seq);
            printf(",%d", eu_n);
            printf(",%d", eu_seq);
            printf(",%d", p_h_flash->b_wait_to_erase);
            printf(",%d", p_h_erase->eu_seq_begin);
            printf(",%d", p_h_erase->eu_n);
            printf("\r\n");
        }
        p_h_process->step = STEP_FINISH;
    }
    case STEP_FINISH:
        if (p_h_debug->b_init)
        {
            printf("> flash_fifo:i,finish");
            printf(",%d", flash_fifo_pack_n_get_saved(p_h_flash));
            printf(",%d", flash_fifo_pack_n_get_free(p_h_flash));
            printf("\r\n");
            printf("pack_in_memory:%d\r\n", p_h_cfg->pack_in_memory);
            printf("data_size_max:%d\r\n", p_h_cfg->data_size_max);
        }
        p_h_flash->b_init_finish = true;
        p_h_process->step        = STEP_IDLE;
    }
    return p_h_process->step != STEP_IDLE;
}

static bool flash_fifo_write_process(h_flash_fifo_t *p_h_flash)
{
    typedef enum step_
    {
        STEP_CHEAK_ORDER,
        STEP_IDLE,
        STEP_PROGRAM_HEADER_START,
        STEP_PROGRAM_HEADER_WAIT,
        STEP_PROGRAM_DATA_START,
        STEP_PROGRAM_DATA_WAIT,
        STEP_CHECK_DATA,
        STEP_NEXT_SEQ,
        STEP_FINISH,
    } step_t;

    const h_flash_fifo_cfg_t *p_h_cfg     = &p_h_flash->h_cfg;
    h_flash_fifo_write_t     *p_h_process = &p_h_flash->h_write;
    h_flash_fifo_debug_t     *p_h_debug   = &p_h_flash->h_debug;

    switch (p_h_process->step)
    {
    case STEP_CHEAK_ORDER:
        p_h_process->step = STEP_IDLE;
    case STEP_IDLE:
    {
        static uint32_t write_interval;

        if (HAL_GetTick() < write_interval + 10)
            break;
        write_interval = HAL_GetTick();
        flash_data_reset();
        if (p_h_process->b_busy == false)
            break;

        h_flash_fifo_header_t *p_h_header     = &p_h_process->h_header;
        size_t                 pack_in_memory = p_h_cfg->pack_in_memory;
        if (p_h_flash->fifo_back >= pack_in_memory)
            p_h_flash->fifo_back = 0;
        if (p_h_flash->fifo_front >= pack_in_memory)
            p_h_flash->fifo_front = 0;
        size_t fifo_back = p_h_flash->fifo_back;
        size_t data_size = p_h_process->data_size;

        p_h_header->data_size = data_size;
        p_h_header->mark      = ~0x0;
        uint16_t crc_16       = FLASH_CRC16_INITIAL_VALUE;
        flash_fifo_crc16(&crc_16, p_h_process->p_data, data_size);
        p_h_header->crc = crc_16;

        size_t addr                   = FLASH_FIFO_PACK_ADDR_GET(p_h_cfg, fifo_back);
        p_h_process->pack_addr_header = addr;
        p_h_process->pack_addr_data   = addr + sizeof(h_flash_fifo_header_t);

        if (fifo_back >= pack_in_memory)
        {
            p_h_process->res  = FLASH_FIFO_RES_ERROR;
            p_h_process->step = STEP_FINISH;
        }
        else
        {
            p_h_process->step = STEP_PROGRAM_HEADER_START;
        }

        if (p_h_debug->b_write)
        {
            // printf("> flash_fifo:w,start");
            // printf(",%d", fifo_back);
            // printf(",%d", p_h_flash->fifo_front);
            // printf(",%d", p_h_header->data_size);
            // printf(",%02X", p_h_header->mark);
            // printf(",%d", p_h_header->serial_seq);
            // printf(",%04X", p_h_header->crc);
            // printf(",%X", p_h_process->pack_addr_header);
            // printf(",%X", p_h_process->pack_addr_data);
            // printf(",%d", p_h_process->step);
            // printf("\r\n");
        }
    }
    break;
    case STEP_PROGRAM_HEADER_START:
    {
        flash_res_t res = flash_program_data(p_h_process->pack_addr_header, &p_h_process->h_header, sizeof(h_flash_fifo_header_t));
        if (res == FLASH_RES_BUSY)
            break;

        if (res == FLASH_RES_OK)
        {
            p_h_process->step = STEP_PROGRAM_HEADER_WAIT;
        }
        else
        {
            p_h_process->res  = FLASH_FIFO_RES_ERROR_HAL;
            p_h_process->step = STEP_NEXT_SEQ;
        }
        if (p_h_debug->b_write)
        {
            // printf("> flash_fifo:w,header_w");
            // printf(",%d", res);
            // printf(",%d", p_h_process->res);
            // printf(",%d", p_h_process->step);
            // printf("\r\n");
        }
    }
    break;
    case STEP_PROGRAM_HEADER_WAIT:
        if (flash_program_busy_get())
            break;
        p_h_process->step = STEP_PROGRAM_DATA_START;
    case STEP_PROGRAM_DATA_START:
    {
        flash_res_t res = flash_program_data(p_h_process->pack_addr_data, p_h_process->p_data, p_h_process->data_size);
        if (res == FLASH_RES_BUSY)
            break;

        if (res == FLASH_RES_OK)
        {
            p_h_process->step = STEP_PROGRAM_DATA_WAIT;
        }
        else
        {
            p_h_process->res  = FLASH_FIFO_RES_ERROR_HAL;
            p_h_process->step = STEP_NEXT_SEQ;
        }
        if (p_h_debug->b_write)
        {
            // printf("> flash_fifo:w,data_w");
            // printf(",%d", res);
            // printf(",%d", p_h_process->res);
            // printf(",%d", p_h_process->step);
            // printf("\r\n");
        }
    }
    break;
    case STEP_PROGRAM_DATA_WAIT:
        if (flash_program_busy_get())
            break;
        p_h_process->step = STEP_CHECK_DATA;
    case STEP_CHECK_DATA:
    {
        p_h_process->res = FLASH_FIFO_RES_ERROR_CRC;
        do
        {
            void  *p_check = (void *)p_h_process->pack_addr_header;
            size_t size    = sizeof(h_flash_fifo_header_t);
            if (memcmp(p_check, &p_h_process->h_header, size) != 0)
                break;
            p_check = (void *)p_h_process->pack_addr_data;
            size    = p_h_process->data_size;
            if (memcmp(p_check, p_h_process->p_data, size) != 0)
                break;
            p_h_process->res = FLASH_FIFO_RES_BUSY;
        }
        while (0);

        void *p_target = p_h_process->p_h_header_order;
        if (p_target != NULL)
        {
            void *p_source = (void *)p_h_process->pack_addr_header;
            memcpy(p_target, p_source, sizeof(h_flash_fifo_header_t));
        }

        if (p_h_debug->b_write)
        {
            // printf("> flash_fifo:w,check");
            // printf(",%d", p_h_process->res);
            // printf("\r\n");
        }
        p_h_process->step = STEP_NEXT_SEQ;
    }
    break;
    case STEP_NEXT_SEQ:
    {
        size_t pack_in_memory = p_h_cfg->pack_in_memory;
        p_h_process->h_header.serial_seq++;
        size_t seq_back_next = p_h_flash->fifo_back + 1;
        seq_back_next %= pack_in_memory;
        do
        {
            if (p_h_flash->fifo_front == seq_back_next)
            {
                p_h_process->res = FLASH_FIFO_RES_NO_SPACE;
                break;
            }
            p_h_flash->fifo_back = seq_back_next;
            p_h_process->res     = FLASH_FIFO_RES_OK;
        }
        while (0);
        if (p_h_debug->b_write)
        {
            // printf("> flash_fifo:w,next");
            // printf(",%d", p_h_flash->fifo_back);
            // printf(",%d", p_h_flash->fifo_front);
            // printf(",%d", p_h_process->res);
            // printf("\r\n");
        }
        p_h_process->step = STEP_FINISH;
    }
        // break;
    case STEP_FINISH:
    {
        p_h_process->b_busy = false;
        if (p_h_debug->b_write)
        {
            // printf("> flash_fifo:w,finish");
            // printf(",%d", p_h_process->res);
            // printf("\r\n");
        }
        flash_fifo_write_finish_cb(p_h_flash, p_h_process->res);
        p_h_process->step = STEP_CHEAK_ORDER;
    }
    }
    return p_h_process->step != STEP_IDLE;
}

static bool flash_fifo_erase_process(h_flash_fifo_t *p_h_flash)
{
    if (p_h_flash->b_erase_busy == false)
        return false;

    const h_flash_fifo_cfg_t *p_h_cfg   = &p_h_flash->h_cfg;
    h_flash_fifo_erase_t     *p_h_erase = &p_h_flash->h_erase;

    if (p_h_erase->eu_count >= p_h_erase->eu_n)
    {
        p_h_flash->b_erase_busy    = false;
        p_h_flash->b_wait_to_erase = false;
        if (p_h_flash->h_debug.b_erase)
        {
            printf("> flash_fifo:e,finish");
            printf(",%d", p_h_erase->eu_seq_begin);
            printf(",%d", p_h_erase->eu_n);
            printf(",%d", p_h_erase->eu_count);
            printf(",%d", p_h_erase->eu_erase);
            printf("\r\n");
        }
        flash_fifo_erase_finish_cb(p_h_flash);
        return false;
    }

    do
    {
        uint8_t eu_seq = p_h_erase->eu_seq_begin + p_h_erase->eu_count;
        eu_seq %= p_h_flash->h_cfg.eu_in_memory;
        h_flash_fifo_header_t *p_h_header;
        p_h_header = FLASH_FIFO_EU_POINTER_GET(p_h_cfg, eu_seq);
        if (p_h_header->data_size != FLASH_FIFO_HEADER_DATA_SIZE_ERASED)
        {
            eu_seq += FLASH_FIFO_1_EU_SEQ_BEGIN;
            if (flash_erase_sector(eu_seq) == false)
                break;
        }
        p_h_erase->eu_count++;
        p_h_erase->eu_erase = eu_seq;
    }
    while (0);
    return true;
}

/* return value:
 *   FLASH_FIFO_RES_BUSY
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_NO_SPACE
 *   FLASH_FIFO_RES_OK
 *   FLASH_FIFO_RES_NOT_ERASED
 * */
flash_fifo_res_t flash_fifo_write(h_flash_fifo_t *p_h_flash, h_flash_fifo_order_t *p_h_order)
{
    h_flash_fifo_write_t *p_h_write = &p_h_flash->h_write;
    if (p_h_write->b_busy)
        return FLASH_FIFO_RES_BUSY;
    void *p_data = p_h_order->p_data;
    if (p_data == NULL)
        return FLASH_FIFO_RES_ERROR_PARA;
    size_t data_size = p_h_order->data_size;
    if (data_size == 0)
        return FLASH_FIFO_RES_ERROR_PARA;
    const h_flash_fifo_cfg_t *p_h_cfg       = &p_h_flash->h_cfg;
    size_t                    data_size_max = p_h_cfg->data_size_max;
    if (data_size > data_size_max)
        return FLASH_FIFO_RES_ERROR_PARA;
    size_t pack_n_free = flash_fifo_pack_n_get_free(p_h_flash);
    if (pack_n_free <= 0)
        return FLASH_FIFO_RES_NO_SPACE;
    size_t                 fifo_back = p_h_flash->fifo_back;
    h_flash_fifo_header_t *p_h_header_flash;
    p_h_header_flash = FLASH_FIFO_PACK_POINTER_GET(p_h_cfg, fifo_back);
    if (p_h_header_flash->data_size != FLASH_FIFO_HEADER_DATA_SIZE_ERASED)
    {
        p_h_flash->fifo_back = FLASH_FIFO_FIFO_SEQ_NEXT(p_h_cfg, fifo_back);
        return FLASH_FIFO_RES_NOT_ERASED;
    }

    if (p_h_flash->b_erase_busy)
    {
        size_t back_eu = FLASH_FIFO_EU_SEQ_GET(p_h_cfg, fifo_back);
        if (back_eu == p_h_flash->h_erase.eu_erase)
            return FLASH_FIFO_RES_BUSY;
    }

    p_h_write->b_busy                 = true;
    p_h_write->res                    = FLASH_FIFO_RES_BUSY;
    h_flash_fifo_header_t *p_h_header = p_h_order->p_h_header;
    p_h_write->p_h_header_order       = p_h_header;
    p_h_write->p_data                 = p_data;
    p_h_write->data_size              = data_size;
    if (p_h_header != NULL)
        memset(p_h_header, 0x0, sizeof(h_flash_fifo_header_t));
    if (p_h_flash->h_debug.b_write)
    {
        // printf("> flash_fifo:w,set");
        // printf(",%p", p_h_write->p_data);
        // printf(",%d", p_h_write->data_size);
        // printf(",%d", pack_n_free);
        // printf("\r\n");
    }
    return FLASH_FIFO_RES_OK;
}

size_t flash_fifo_pack_n_get_saved(h_flash_fifo_t *p_h_flash)
{
    size_t pack_in_memory = p_h_flash->h_cfg.pack_in_memory;
    size_t data_n         = pack_in_memory + p_h_flash->fifo_back;
    data_n                = (data_n - p_h_flash->fifo_front) % pack_in_memory;
    return data_n;
}

size_t flash_fifo_pack_n_get_free(h_flash_fifo_t *p_h_flash)
{
    const h_flash_fifo_cfg_t *p_h_cfg     = &p_h_flash->h_cfg;
    size_t                    pack_per_eu = p_h_cfg->pack_per_eu;

    size_t seq_back = p_h_flash->fifo_back;
    size_t seq_log;

    if (p_h_flash->b_wait_to_erase)
    {
        h_flash_fifo_erase_t *p_h_erase = &p_h_flash->h_erase;
        seq_log                         = p_h_erase->eu_erase * p_h_cfg->pack_per_eu;
    }
    else
    {
        seq_log = p_h_flash->fifo_front;
    }
    size_t eu_back = FLASH_FIFO_EU_SEQ_GET(p_h_cfg, seq_back);
    size_t eu_log  = FLASH_FIFO_EU_SEQ_GET(p_h_cfg, seq_log);

    if ((eu_back == eu_log) && (seq_back < seq_log))
        return 0;

    size_t back_next = (seq_back + 1) % p_h_cfg->pack_in_memory;
    if (back_next == seq_log)
        return 0;

    size_t eu_in_memory = p_h_cfg->eu_in_memory;
    size_t back_eu_next = (eu_back + 1) % eu_in_memory;
    size_t eu_n         = (eu_in_memory + eu_log - back_eu_next) % eu_in_memory;

    size_t back_pack_seq = seq_back % pack_per_eu;
    size_t pack_n_free   = (pack_per_eu - back_pack_seq) + (eu_n * pack_per_eu);
    if (FLASH_FIFO_PACK_SEQ_GET(p_h_cfg, seq_log) == 0)
        pack_n_free--;
    return pack_n_free;
}

/* return value:
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_NO_DATA
 *   FLASH_FIFO_RES_ERASED
 *   FLASH_FIFO_RES_ERROR_CRC
 *   FLASH_FIFO_RES_OK
 * */
flash_fifo_res_t flash_fifo_available_get(h_flash_fifo_t *p_h_flash, size_t seq)
{
    flash_fifo_res_t          res        = FLASH_FIFO_RES_ERROR;
    const h_flash_fifo_cfg_t *p_h_cfg    = &p_h_flash->h_cfg;
    size_t                    pack_addr  = FLASH_FIFO_PACK_ADDR_GET(p_h_cfg, seq);
    h_flash_fifo_header_t    *p_h_header = (void *)pack_addr;
    size_t                    data_size  = p_h_header->data_size;
    uint16_t                  crc_16     = FLASH_CRC16_INITIAL_VALUE;

    do
    {
        if (seq >= p_h_cfg->pack_in_memory)
        {
            res = FLASH_FIFO_RES_ERROR_PARA;
            break;
        }
        if (data_size == 0)
        {
            res = FLASH_FIFO_RES_NO_DATA;
            break;
        }
        if (data_size > p_h_cfg->data_size_max)
        {
            res = FLASH_FIFO_RES_ERASED;
            break;
        }
        size_t    header_size  = sizeof(h_flash_fifo_header_t);
        uint16_t *p_flash_data = (void *)(pack_addr + header_size);
        flash_fifo_crc16(&crc_16, p_flash_data, data_size);
        if (p_h_header->crc != crc_16)
        {
            res = FLASH_FIFO_RES_ERROR_CRC;
        }
        else
        {
            res = FLASH_FIFO_RES_OK;
        }
    }
    while (0);

    if (p_h_flash->h_debug.b_available)
    {
        printf("> flash_fifo:a");
        printf(",%d", seq);
        printf(",%d", data_size);
        // printf(",%d", p_h_cfg->data_size_max);
        printf(",%d", crc_16);
        printf(",%d", p_h_header->crc);
        printf(",%d", res);
        printf("\r\n");
    }
    return res;
}

/* return value:
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_NO_DATA
 *   FLASH_FIFO_RES_BUSY
 *   FLASH_FIFO_RES_ERASED
 *   FLASH_FIFO_RES_ERROR_CRC
 *   FLASH_FIFO_RES_OK
 * */
flash_fifo_res_t flash_fifo_read_front(h_flash_fifo_t *p_h_flash, h_flash_fifo_order_t *p_h_order)
{
    return flash_fifo_read_seq(p_h_flash, p_h_flash->fifo_front, p_h_order);
}

/* return value:
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_NO_DATA
 *   FLASH_FIFO_RES_BUSY
 *   FLASH_FIFO_RES_ERASED
 *   FLASH_FIFO_RES_ERROR_CRC
 *   FLASH_FIFO_RES_OK
 * */
flash_fifo_res_t flash_fifo_read_seq(h_flash_fifo_t *p_h_flash, size_t seq, h_flash_fifo_order_t *p_h_order)
{
    const h_flash_fifo_cfg_t *p_h_cfg = &p_h_flash->h_cfg;
    if (seq >= p_h_cfg->pack_in_memory)
        return FLASH_FIFO_RES_ERROR_PARA;
    void *p_buff = p_h_order->p_data;
    if (p_buff == NULL)
        return FLASH_FIFO_RES_ERROR_PARA;
    size_t buff_size = p_h_order->data_size;
    if (p_h_order->data_size == 0)
        return FLASH_FIFO_RES_ERROR_PARA;
    size_t fifo_front = p_h_flash->fifo_front;
    size_t fifo_back  = p_h_flash->fifo_back;
    if (fifo_front == fifo_back)
        return FLASH_FIFO_RES_NO_DATA;
    if (p_h_flash->b_erase_busy)
    {
        size_t eu_seq = FLASH_FIFO_EU_SEQ_GET(p_h_cfg, seq);
        if (eu_seq == p_h_flash->h_erase.eu_erase)
            return FLASH_FIFO_RES_BUSY;
    }

    flash_fifo_res_t res = flash_fifo_available_get(p_h_flash, fifo_front);

    if (p_h_flash->h_debug.b_read)
    {
        // printf("> flash_fifo:r,data");
        // printf(",%p", p_buff);
        // printf(",%d", buff_size);
        // printf(",%d", seq);
        // printf(",%d", fifo_front);
        // printf(",%d", fifo_back);
        // printf(",%d", res);
        // printf("\r\n");
    }
    if (res != FLASH_FIFO_RES_OK)
        return res;

    size_t                 pack_addr  = FLASH_FIFO_PACK_ADDR_GET(p_h_cfg, seq);
    h_flash_fifo_header_t *p_source   = (void *)pack_addr;
    h_flash_fifo_header_t *p_h_header = p_h_order->p_h_header;
    if (p_h_header != NULL)
        memcpy(p_h_header, p_source, sizeof(h_flash_fifo_header_t));
    size_t data_size    = p_source->data_size;
    void  *p_flash_data = (void *)(pack_addr + sizeof(h_flash_fifo_header_t));

    size_t cpy_size = buff_size;
    if (buff_size > data_size)
    {
        uint8_t *p_set_buff = p_buff;
        memset(&p_set_buff[data_size], 0x0, buff_size - data_size);
        cpy_size = data_size;
    }
    memcpy(p_buff, p_flash_data, cpy_size);
    return FLASH_FIFO_RES_OK;
}

/* return value:
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_NO_DATA
 *   FLASH_FIFO_RES_ERASED
 *   FLASH_FIFO_RES_ERROR_CRC
 *   FLASH_FIFO_RES_OK
 * */
flash_fifo_res_t flash_fifo_header_get(h_flash_fifo_t *p_h_flash, size_t seq, h_flash_fifo_header_t *p_h_header)
{
    flash_fifo_res_t res = flash_fifo_available_get(p_h_flash, seq);

    if (p_h_flash->h_debug.b_read)
    {
        printf("> flash_fifo:r,header");
        printf(",%d", seq);
        printf(",%d", res);
        printf("\r\n");
    }
    if (res != FLASH_FIFO_RES_OK)
        return res;

    void *p_source = FLASH_FIFO_PACK_POINTER_GET(&p_h_flash->h_cfg, seq);
    memcpy(p_h_header, p_source, sizeof(h_flash_fifo_header_t));
    return FLASH_FIFO_RES_OK;
}

/* return value:
 *   FLASH_FIFO_RES_OK
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_BUSY
 *   FLASH_FIFO_RES_NO_SPACE
 * */
flash_fifo_res_t flash_fifo_mark_set(h_flash_fifo_t *p_h_flash, h_flash_fifo_mark_t *p_h_mark)
{
    const h_flash_fifo_cfg_t *p_h_cfg = &p_h_flash->h_cfg;
    size_t                    seq     = p_h_mark->seq;
    if (seq >= p_h_cfg->pack_in_memory)
        return FLASH_FIFO_RES_ERROR_PARA;

    h_flash_fifo_header_t *p_h_header;
    p_h_header                = FLASH_FIFO_PACK_POINTER_GET(p_h_cfg, seq);
    flash_fifo_res_t res_fifo = FLASH_FIFO_RES_ERROR;

    if (p_h_flash->b_mark_busy)
        return FLASH_FIFO_RES_BUSY;

    if (p_h_header->mark == p_h_mark->mark)
    {
        flash_fifo_mark_set_finish_cb(p_h_flash, p_h_mark);
        return FLASH_FIFO_RES_OK;
    }

    if (flash_program_busy_get())
        return FLASH_FIFO_RES_BUSY;
    if (p_h_flash->b_erase_busy)
    {
        size_t eu_seq = FLASH_FIFO_EU_SEQ_GET(p_h_cfg, seq);
        if (eu_seq == p_h_flash->h_erase.eu_erase)
            return FLASH_FIFO_RES_BUSY;
    }

    static uint8_t mark;
    mark                   = p_h_mark->mark;
    p_h_flash->b_mark_busy = true;
    size_t      addr       = (size_t)&p_h_header->mark;
    flash_res_t res_program;
    res_program = flash_program_data(addr, &mark, sizeof(mark));

    switch (res_program)
    {
    case FLASH_RES_OK:
        memcpy(&p_h_flash->h_mark, p_h_mark, sizeof(h_flash_fifo_mark_t));
        res_fifo = FLASH_FIFO_RES_OK;
        break;
    case FLASH_RES_BUSY:
        res_fifo = FLASH_FIFO_RES_BUSY;
        break;
    case FLASH_RES_ERROR:
        res_fifo = FLASH_FIFO_RES_ERROR_PARA;
    }
    if (res_fifo != FLASH_FIFO_RES_OK)
        p_h_flash->b_mark_busy = false;

    if (p_h_flash->h_debug.b_mark)
    {
        printf("> flash_fifo:e");
        printf(",%d", seq);
        printf(",%d", mark);
        printf(",%d", p_h_flash->b_mark_busy);
        printf(",%d", res_fifo);
        printf("\r\n");
    }
    return res_fifo;
}

/* return value:
 *   FLASH_FIFO_RES_OK
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_BUSY
 *   FLASH_FIFO_RES_NO_DATA
 * */
flash_fifo_res_t flash_fifo_delete_front(h_flash_fifo_t *p_h_flash)
{
    size_t fifo_front = p_h_flash->fifo_front;
    if (fifo_front == p_h_flash->fifo_back)
        return FLASH_FIFO_RES_NO_DATA;

    flash_fifo_res_t res = flash_fifo_delete_seq(p_h_flash, fifo_front);
    if (res != FLASH_FIFO_RES_OK)
        return res;

    const h_flash_fifo_cfg_t *p_h_cfg        = &p_h_flash->h_cfg;
    size_t                    pack_in_memory = p_h_cfg->pack_in_memory;
    size_t                    front_log      = fifo_front;
    p_h_flash->fifo_front                    = (p_h_flash->fifo_front + 1) % pack_in_memory;

    if (FLASH_FIFO_PACK_SEQ_GET(p_h_cfg, fifo_front) == 0)
    {
        /* fifo_front cross to next erase_unit,
         * log in erase order. */
        h_flash_fifo_erase_t *p_h_erase   = &p_h_flash->h_erase;
        size_t                eu_to_erase = FLASH_FIFO_EU_SEQ_GET(p_h_cfg, front_log);
        if (p_h_flash->b_wait_to_erase == false)
        {
            /* set erase order. */
            p_h_erase->eu_count        = 0;
            p_h_erase->eu_seq_begin    = eu_to_erase;
            p_h_erase->eu_erase        = eu_to_erase;
            p_h_erase->eu_n            = 1;
            p_h_flash->b_wait_to_erase = true;
            if (p_h_flash->h_debug.b_delete)
            {
                printf("> flash_fifo:d,erase_set");
                printf(",%d", p_h_flash->b_wait_to_erase);
                printf(",%d", p_h_erase->eu_seq_begin);
                printf(",%d", p_h_erase->eu_n);
                printf("\r\n");
            }
        }
        else
        {
            /* update erase erase_unit number. */
            size_t eu_in_memory = p_h_cfg->eu_in_memory;
            size_t erase_n      = eu_in_memory + eu_to_erase;
            erase_n             = (erase_n - p_h_erase->eu_seq_begin) % eu_in_memory;
            if (p_h_erase->eu_n < erase_n)
            {
                p_h_erase->eu_n = erase_n;
                if (p_h_flash->h_debug.b_delete)
                {
                    printf("> flash_fifo:d,erase_set");
                    printf(",%d", p_h_flash->b_wait_to_erase);
                    printf(",%d", p_h_erase->eu_seq_begin);
                    printf(",%d", p_h_erase->eu_n);
                    printf("\r\n");
                }
            }
        }
    }
    return FLASH_FIFO_RES_OK;
}

/* return value:
 *   FLASH_FIFO_RES_OK
 *   FLASH_FIFO_RES_ERROR_PARA
 *   FLASH_FIFO_RES_BUSY
 * */
flash_fifo_res_t flash_fifo_delete_seq(h_flash_fifo_t *p_h_flash, size_t seq)
{
    const h_flash_fifo_cfg_t *p_h_cfg = &p_h_flash->h_cfg;
    if (seq >= p_h_cfg->pack_in_memory)
        return FLASH_FIFO_RES_ERROR_PARA;

    if (p_h_flash->b_delete_busy)
        return FLASH_FIFO_RES_BUSY;

    h_flash_fifo_header_t *p_h_header;
    p_h_header = FLASH_FIFO_PACK_POINTER_GET(p_h_cfg, seq);

    if (p_h_header->data_size == FLASH_FIFO_HEADER_DATA_SIZE_DELETE)
    {
        flash_fifo_delete_finish_cb(p_h_flash, seq);
        return FLASH_FIFO_RES_OK;
    }

    if (flash_program_busy_get())
        return FLASH_FIFO_RES_BUSY;
    if (p_h_flash->b_erase_busy)
    {
        size_t eu_seq = FLASH_FIFO_EU_SEQ_GET(p_h_cfg, seq);
        if (eu_seq == p_h_flash->h_erase.eu_erase)
            return FLASH_FIFO_RES_BUSY;
    }

    p_h_flash->b_delete_busy       = true;
    size_t               addr      = (size_t)&p_h_header->data_size;
    static const uint8_t data_size = FLASH_FIFO_HEADER_DATA_SIZE_DELETE;
    flash_res_t          res_program;
    res_program = flash_program_data(addr, (void *)&data_size, sizeof(data_size));

    flash_fifo_res_t res_fifo = FLASH_FIFO_RES_ERROR;
    switch (res_program)
    {
    case FLASH_RES_OK:
        p_h_flash->delete_seq = seq;
        res_fifo              = FLASH_FIFO_RES_OK;
        break;
    case FLASH_RES_BUSY:
        res_fifo = FLASH_FIFO_RES_BUSY;
        break;
    case FLASH_RES_ERROR:
        res_fifo = FLASH_FIFO_RES_ERROR_PARA;
    }
    if (res_fifo != FLASH_FIFO_RES_OK)
        p_h_flash->b_delete_busy = false;

    if (p_h_flash->h_debug.b_delete)
    {
        printf("> flash_fifo:d");
        printf(",%d", seq);
        printf(",%d", p_h_flash->b_delete_busy);
        printf(",%d", p_h_flash->delete_seq);
        printf(",%d", res_fifo);
        printf("\r\n");
    }
    return res_fifo;
}

/* return value:
 *   FLASH_FIFO_RES_OK
 *   FLASH_FIFO_RES_BUSY
 * */
flash_fifo_res_t flash_fifo_erase_all(h_flash_fifo_t *p_h_flash)
{
    if (p_h_flash->b_erase_busy)
        return FLASH_FIFO_RES_BUSY;
    p_h_flash->b_erase_busy                = true;
    h_flash_fifo_erase_t *p_h_erase        = &p_h_flash->h_erase;
    p_h_erase->eu_seq_begin                = 0;
    p_h_erase->eu_erase                    = 0;
    p_h_erase->eu_n                        = p_h_flash->h_cfg.eu_in_memory;
    p_h_erase->eu_count                    = 0;
    p_h_flash->fifo_back                   = 0;
    p_h_flash->fifo_front                  = 0;
    p_h_flash->h_write.h_header.serial_seq = 0;
    if (p_h_flash->h_debug.b_erase)
    {
        printf("> flash_fifo:e,start_a");
        printf(",%d", p_h_erase->eu_seq_begin);
        printf(",%d", p_h_erase->eu_erase);
        printf(",%d", p_h_erase->eu_n);
        printf(",%d", p_h_erase->eu_count);
        printf("\r\n");
    }
    return FLASH_FIFO_RES_OK;
}

/* return value:
 *   FLASH_FIFO_RES_OK
 *   FLASH_FIFO_RES_BUSY
 *   FLASH_FIFO_RES_NOT_FOUND
 * */
flash_fifo_res_t flash_fifo_erase_dirty(h_flash_fifo_t *p_h_flash)
{
    if (p_h_flash->b_wait_to_erase == false)
        return FLASH_FIFO_RES_NOT_FOUND;
    if (p_h_flash->b_erase_busy)
        return FLASH_FIFO_RES_BUSY;

    p_h_flash->b_erase_busy         = true;
    h_flash_fifo_erase_t *p_h_erase = &p_h_flash->h_erase;
    p_h_erase->eu_erase             = p_h_erase->eu_seq_begin;
    p_h_erase->eu_count             = 0;

    if (p_h_flash->h_debug.b_erase)
    {
        printf("> flash_fifo:e,start_d");
        printf(",%d", p_h_erase->eu_seq_begin);
        printf(",%d", p_h_erase->eu_erase);
        printf(",%d", p_h_erase->eu_n);
        printf(",%d", p_h_erase->eu_count);
        printf("\r\n");
    }
    return FLASH_FIFO_RES_OK;
}

bool flash_fifo_init_finish_busy_get(h_flash_fifo_t *p_h_flash)
{
    return p_h_flash->b_init_finish;
}

bool flash_fifo_write_busy_get(h_flash_fifo_t *p_h_flash)
{
    return p_h_flash->h_write.b_busy;
}

flash_fifo_res_t flash_fifo_write_res_get(h_flash_fifo_t *p_h_flash)
{
    return p_h_flash->h_write.res;
}

bool flash_fifo_delete_busy_get(h_flash_fifo_t *p_h_flash)
{
    return p_h_flash->b_delete_busy;
}

bool flash_fifo_mark_busy_get(h_flash_fifo_t *p_h_flash)
{
    return p_h_flash->b_mark_busy;
}

bool flash_fifo_erase_busy_get(h_flash_fifo_t *p_h_flash)
{
    return p_h_flash->b_erase_busy;
}

bool flash_fifo_wait_to_erase_get(h_flash_fifo_t *p_h_flash)
{
    return p_h_flash->b_wait_to_erase;
}

static void flash_fifo_crc16(uint16_t *p_crc_value, void *ptr, size_t size)
{
    uint8_t *p_u8 = ptr;
    uint8_t  index; /* will index index into CRC lookup */
    uint16_t crc_value = *p_crc_value;
    while (size--) /* pass through message buffer */
    {
        index     = (crc_value >> 8) ^ *(p_u8++); /* calculate the CRC */
        crc_value = (crc_value << 8) ^ flash_CRC16_table[index];
    }
    *p_crc_value = crc_value;
}
