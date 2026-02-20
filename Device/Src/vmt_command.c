#include "vmt_command.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "mti_imu.h"
#include "mti_system.h"
#include "mti_void.h"
#include "mti_water.h"
#include "mti_radar.h"
#include "spi.h"
#include "vmt_adc.h"
#include "vmt_device.h"
#include "vmt_flash.h"
#include "vmt_flash_fifo.h"
#include "vmt_uart.h"
#include "xm125_profiles.h"

/* ==========================================================================
   PRIVATE DEFINITIONS & TYPES
   ========================================================================== */
#define CMD_STR_LEN_MAX   (255)
#define CMD_STR_BUFF_SIZE (CMD_STR_LEN_MAX + 1)

typedef struct h_cmd_str_buff_
{
    char   buff[CMD_STR_BUFF_SIZE];
    size_t front;
} h_cmd_str_buff_t;

/* ==========================================================================
   STATIC VARIABLES
   ========================================================================== */

/* --- General Control --- */
bool                 auto_log;
static uart_select_t cmd_uart_ch  = UART_FIRST;
static const char    delimiters[] = ",";
static const char    end_char[]   = "\n";

/* --- UART Buffers --- */
static h_cmd_str_buff_t h_cmd_str_buff[UART_NUMBER] = { 0x0 };

/* --- SPI Buffers --- */
#define CMD_SPI_BUFF_SIZE (32)
static uint8_t spi_tx_buff[CMD_SPI_BUFF_SIZE];
static uint8_t spi_rx_buff[CMD_SPI_BUFF_SIZE];

/* --- Flash Buffers --- */
#define CMD_FLASH_BUFF_LEN (256)
static uint8_t flash_buff[CMD_FLASH_BUFF_LEN];

/* --- FIFO / Log Buffers & State --- */
#define CMD_LOG_BUFF_SIZE (FLASH_FIFO_1_DATA_SIZE_MAX)
static uint8_t               fifo_buff_w[CMD_LOG_BUFF_SIZE];
static h_flash_fifo_header_t h_log_header_w;
static h_flash_fifo_order_t  h_flash_fifo_w = {
     .p_data = fifo_buff_w,
};
static uint8_t             fifo_buff_r[CMD_LOG_BUFF_SIZE];
static uint8_t             ff_delete_seq = 0;
static h_flash_fifo_mark_t h_ff_mark;

static bool b_ff_write_busy    = false;
static bool b_ff_delete_busy   = false;
static bool b_ff_mark_set_busy = false;
static bool b_ff_check         = false;

static size_t ff_write_count  = 0;
static size_t ff_delete_count = 0;

/* ==========================================================================
   FORWARD DECLARATIONS (INTERNAL)
   ========================================================================== */
void cmd_find_cb(uint8_t id);
void cmd_spi_finish(h_dev_spi_send_t *p_h_spi);

static void cmd_uart_process(uart_select_t ch);
static void cmd_fifo_multi_w_process(void);
static void cmd_fifo_check_process(void);
static void cmd_fifo_multi_d_process(void);

/* Command Callback Declarations */
// static void cmd_example(str_pointers_t *str_p);
static void cmd_detect(h_str_pointers_t *str_p);
static void cmd_bottom(h_str_pointers_t *str_p);
static void cmd_water(h_str_pointers_t *str_p);
static void cmd_sensor(h_str_pointers_t *str_p);
static void cmd_start(h_str_pointers_t *str_p);
static void cmd_finish(h_str_pointers_t *str_p);
static void cmd_initial(h_str_pointers_t *str_p);
static void cmd_connect(h_str_pointers_t *str_p);
static void cmd_sleep(h_str_pointers_t *p_h_str_p);
static void cmd_echo(h_str_pointers_t *str_p);
static void cmd_spi(h_str_pointers_t *str_p);
static void cmd_log(h_str_pointers_t *str_p);
static void cmd_flash_fifo(h_str_pointers_t *str_p);
static void cmd_flash(h_str_pointers_t *str_p);
static void cmd_debug(h_str_pointers_t *str_p);
static void cmd_imu(h_str_pointers_t *str_p);
static void cmd_void(h_str_pointers_t *str_p);

/* ==========================================================================
   COMMAND STRINGS & TABLES
   ========================================================================== */

/* --- Strings: Operational --- */
static const char cmd_str_detect[]  = "@dt";
static const char cmd_str_bottom[]  = "@bt";
static const char cmd_str_water[]   = "@wt";
static const char cmd_str_start[]   = "@st";
static const char cmd_str_finish[]  = "@fn";
static const char cmd_str_connect[] = "@connect";
static const char cmd_str_init[]    = "@init";
static const char cmd_str_sleep[]   = "@sleep";
static const char cmd_str_wake[]    = "@wake";
static const char cmd_str_void[]    = "@vd";

/* --- Strings: Debug & Test --- */
// static const char cmd_str_example[] = "@example";
static const char cmd_str_echo[]       = "@echo";
static const char cmd_str_spi[]        = "@spi";
static const char cmd_str_log[]        = "@log";
static const char cmd_str_flash_fifo[] = "@ff";
static const char cmd_str_flash[]      = "@flash";
static const char cmd_str_imu[]        = "@imu";
static const char cmd_str_debug[]      = "@cmd";
static const char cmd_str_sensor[]     = "@sensor";

/* --- Table: Debug UART --- */
static h_str_cmd_t h_str_cmd_debug[] = {
    {
        .ptr      = (char *)cmd_str_echo,
        .callback = cmd_echo,
    },
    {
        .ptr      = (char *)cmd_str_spi,
        .callback = cmd_spi,
    },
    {
        .ptr      = (char *)cmd_str_log,
        .callback = cmd_log,
    },
    {
        .ptr      = (char *)cmd_str_debug,
        .callback = cmd_debug,
    },
    {
        .ptr      = (char *)cmd_str_sleep,
        .callback = cmd_sleep,
    },
    {
        .ptr      = (char *)cmd_str_flash_fifo,
        .callback = cmd_flash_fifo,
    },
    {
        .ptr      = (char *)cmd_str_flash,
        .callback = cmd_flash,
    },
    {
        .ptr      = (char *)cmd_str_sensor,
        .callback = cmd_sensor,
    },
    {
        .ptr      = (char *)cmd_str_init,
        .callback = cmd_initial,
    },
    {
        .ptr      = (char *)cmd_str_connect,
        .callback = cmd_connect,
    },
    {
        .ptr      = (char *)cmd_str_start,
        .callback = cmd_start,
    },
    {
        .ptr      = (char *)cmd_str_finish,
        .callback = cmd_finish,
    },
    {
        .ptr      = (char *)cmd_str_imu,
        .callback = cmd_imu,
    },
    {
        .ptr      = (char *)cmd_str_void,
        .callback = cmd_void,
    },
};

/* --- Table: Uphole UART --- */
static h_str_cmd_t h_str_cmd_uphole[] = {
    {
        .ptr      = (char *)cmd_str_detect,
        .callback = cmd_detect,
    },
    {
        .ptr      = (char *)cmd_str_bottom,
        .callback = cmd_bottom,
    },
    {
        .ptr      = (char *)cmd_str_sensor,
        .callback = cmd_sensor,
    },
    {
        .ptr      = (char *)cmd_str_water,
        .callback = cmd_water,
    },
    {
        .ptr      = (char *)cmd_str_start,
        .callback = cmd_start,
    },
    {
        .ptr      = (char *)cmd_str_finish,
        .callback = cmd_finish,
    },
    {
        .ptr      = (char *)cmd_str_init,
        .callback = cmd_initial,
    },
    {
        .ptr      = (char *)cmd_str_connect,
        .callback = cmd_connect,
    },
    {
        .ptr      = (char *)cmd_str_echo,
        .callback = cmd_echo,
    },
    {
        .ptr      = (char *)cmd_str_debug,
        .callback = cmd_debug,
    },
    {
        .ptr      = (char *)cmd_str_sleep,
        .callback = cmd_sleep,
    },
    {
        .ptr      = (char *)cmd_str_log,
        .callback = cmd_log,
    },
    {
        .ptr      = (char *)cmd_str_imu,
        .callback = cmd_imu,
    },
    {
        .ptr      = (char *)cmd_str_void,
        .callback = cmd_void,
    },
};

static char str_head_uphole[STR_H_STR_CMD_HEAD_LEN_GET(h_str_cmd_uphole)] = { 0x0 };
static char str_head_debug[STR_H_STR_CMD_HEAD_LEN_GET(h_str_cmd_debug)]   = { 0x0 };

static h_string_t h_str[UART_NUMBER] = {
    [UART_UPHOLE] = {
        .id          = UART_UPHOLE,
        .p_h_cmd     = h_str_cmd_uphole,
        .cmd_n       = STR_H_STR_CMD_LEN_GET(h_str_cmd_uphole),
        .p_cmd_head  = (char*)str_head_uphole,
        .p_delimiters= (char*)delimiters,
        .p_end_char  = (char*)end_char,
        .cmd_find_cb = cmd_find_cb,
    },
    [UART_DEBUG] = {
        .id          = UART_DEBUG,
        .p_h_cmd     = h_str_cmd_debug,
        .cmd_n       = STR_H_STR_CMD_LEN_GET(h_str_cmd_debug),
        .p_cmd_head  = (char*)str_head_debug,
        .p_delimiters= (char*)delimiters,
        .p_end_char  = (char*)end_char,
        .cmd_find_cb = cmd_find_cb,
    },
    // Safe dummy for UART_VOID to avoid NULL deref when iterating [UART_NUMBER]
    [UART_VOID] = {
        .id          = UART_VOID,
        .p_h_cmd     = NULL,
        .cmd_n       = 0,
        .p_cmd_head  = (char*)"",
        .p_delimiters= (char*)delimiters,
        .p_end_char  = (char*)end_char,
        .cmd_find_cb = NULL,
    },
};

/* ==========================================================================
   HELPER FUNCTIONS (INTERNAL)
   ========================================================================== */

void cmd_find_cb(uint8_t id)
{
    uart_select_t     ch_n     = (uart_select_t)id;
    h_cmd_str_buff_t *p_h_buff = &h_cmd_str_buff[ch_n];
    uart_rx_front_add(ch_n, &p_h_buff->front, h_str[ch_n].check_n);
}

void cmd_spi_finish(h_dev_spi_send_t *p_h_spi)
{
    if (p_h_spi->p_tx_buff != spi_tx_buff)
        return;

    uart_tx_channel_set(UART_DEBUG);
    size_t   data_size = p_h_spi->data_size;
    uint8_t *p_rx_buff = p_h_spi->p_rx_buff;

    printf("%s", cmd_str_spi);
    printf(",%d", p_h_spi->channel);
    printf(",%d", data_size);
    printf(",r");
    printf(",");
    for (size_t data_n = 0; data_n < data_size; data_n++)
        printf("%02X ", p_rx_buff[data_n]);
    printf("\n");
    uart_tx_channel_undo();
}

static void cmd_uart_process(uart_select_t ch)
{
    cmd_uart_ch = ch;

    h_cmd_str_buff_t *p_h_buff = &h_cmd_str_buff[ch];
    char             *p_buff   = p_h_buff->buff;
    size_t            str_len  = 0;
    uart_rx_dequeue_dma(ch, p_h_buff->front, (uint8_t *)p_buff, CMD_STR_LEN_MAX, &str_len);
    p_buff[str_len] = '\0';

    string_decoder_by_end(&h_str[ch], p_buff, str_len);
}

// static void cmd_example(str_pointers_t *str_p)
// {
//     //    uart_tx_channel_set(UART_DEBUG);
//     uart_tx_channel_set(cmd_uart_ch);
//     printf("cmd_example,");
//     printf("len:%d\r\n", str_p->end - str_p->head + 1);
//     printf("CMD:%p|%s\r\n", str_p->head, str_p->head);
//     printf("sub:%p|%s\r\n", str_p->sub, str_p->sub);
//     for (int i = 0; i < STR_SECTION_MAX; i++)
//     {
//         if (str_p->part[i] == NULL)
//             break;
//         printf("part:%d|%p|%c|%s\r\n", i, str_p->part[i], str_p->delimiters[i], str_p->part[i]);
//     }
//     printf("other:%p|%s\r\n", str_p->end + 1, str_p->end + 1);
//     uart_tx_channel_undo();
// }

/* ==========================================================================
   CALLBACKS: SYSTEM & LIFECYCLE
   ========================================================================== */

static void cmd_connect(h_str_pointers_t *str_p)
{ //@connect
    if (str_p->part[1] == NULL)
    {
        uart_tx_channel_set(cmd_uart_ch);
        // uart_tx_channel_set(UART_UPHOLE);
        printf("@status,down,%d,ver,%d,%d,%d\n", module_status_get(), FW_VER_MAJOR, FW_VER_MINOR, FW_VER_SUB);
        state_set(stopped_state);
        if (device_init_finish_get())
            imu_test_reset();
        uart_tx_channel_undo();
        return;
    }
    else if (strstr(str_p->part[1], "void") != NULL)
    {
        uart_tx_channel_set(UART_DEBUG);
        printf("Void module initialised\n");
    }
}

static void cmd_initial(h_str_pointers_t *str_p)
{
    uart_tx_channel_set(cmd_uart_ch);
    if (str_p->part[1][0] == 'g')
    {
        do
        {
            if (str_p->part[1][1] == '?')
                break;
            if (str_p->part[2] == NULL)
                return;
            bool b_active = atoi(str_p->part[2]) != 0;
            imu_g_log_en_set(0, b_active);
            imu_g_log_en_set(1, b_active);
        }
        while (0);

        cmd_print_init_g_log(cmd_uart_ch);
    }
    else if (strcmp(str_p->part[1], "ver") == 0)
    {
        if (strcmp(str_p->part[2], "ack") == 0)
            version_ack(true);
        else
            version_ack(false);
    }
    else if (strstr("all", str_p->part[1]) != NULL)
    {
        bool b_finish = device_init_finish_get();
        b_finish |= imu_g_log_busy_get(0);
        b_finish |= imu_g_log_busy_get(1);
        uart_tx_channel_set(cmd_uart_ch);
        printf("%s,all", cmd_str_init);
        printf(",%d", b_finish);
        printf("\n");
    }
    else if (strcmp(str_p->part[1], "void") == 0)
    {
        printf("ACK,init\n");
        uart_tx_channel_set(UART_DEBUG);
        printf("Void module initialised");
    }
    else
    {
        uart_tx_channel_set(cmd_uart_ch);
        // uart_tx_channel_set(UART_UPHOLE);
        // printf("@init,down,ok\n");
        printf("@status,%d\n", module_status_get());
    }
    uart_tx_channel_undo();
}

static void cmd_start(h_str_pointers_t *str_p)
{
    state_set(measure_state);
    uart_tx_channel_set(cmd_uart_ch);
    if (dev_log_save_enable_set(auto_log))
        printf("@db,Log starting\n");
    dev_dump_printf_set(true);
    dev_water_det_printf_set(true);

    // printf("\n");
    device_restart();
    state_set(measure_state);
    keepalive_reset();
    imu_test_reset();
    printf("@st\n");

    /* auto log enable */

    uart_tx_channel_undo();
}

static void cmd_finish(h_str_pointers_t *str_p)
{
    state_set(stopped_state);
    uart_tx_channel_set(cmd_uart_ch);
    dev_dump_printf_set(false);
    dev_water_det_printf_set(false);
    keepalive_reset();
    printf("%s", cmd_str_finish);
    printf("\n");
    if (auto_log)
    {
        dev_log_save_enable_set(false);
        printf("@db,Logging complete\n");
    }
    imu_test_reset();
    uart_tx_channel_undo();
}

static void cmd_sleep(h_str_pointers_t *p_h_str_p)
{
    printf("%s\n", cmd_str_sleep);
    dev_sleep();
}

static void cmd_echo(h_str_pointers_t *str_p)
{
    uart_tx_channel_set(cmd_uart_ch);
    // printf("cmd_echo\r\n");
    printf("%s", cmd_str_echo);
    printf("%s", str_p->part[0]);
    for (uint8_t part_n = 1; part_n < STR_SECTION_MAX; part_n++)
    {
        if (str_p->part[part_n] == NULL)
            break;
        printf(",%s", str_p->part[part_n]);
    }
    printf("\n");
    uart_tx_channel_undo();
}

/* ==========================================================================
   CALLBACKS: SENSOR & APPLICATION
   ========================================================================== */

static void cmd_detect(h_str_pointers_t *str_p)
{
    // uart_tx_channel_set(cmd_uart_ch);
    // printf("%s", cmd_str_bottom);
    // printf(",%d", dev_dump_get());
    // printf(",%d", dev_water_det_get());
    // printf("\n");
    // uart_tx_channel_undo();
}

static void cmd_bottom(h_str_pointers_t *str_p)
{
    cmd_print_bottom(cmd_uart_ch);
}

static void cmd_water(h_str_pointers_t *str_p)
{
    cmd_print_water(cmd_uart_ch);
}

static void cmd_sensor(h_str_pointers_t *str_p)
{
    uart_tx_channel_set(cmd_uart_ch);
    if (str_p->part[1] == NULL)
        return;
    // printf("@db,sensor: %c\n",str_p->part[1][0]);
    switch (str_p->part[1][0])
    {
    case 'w':
        if (str_p->part[2] == NULL)
        { // calibrate
            uint16_t water_1 = adc_value_get(ADC_SEQ_WATER_BEGIN);
            uint16_t water_2 = adc_value_get(ADC_SEQ_WATER_BEGIN + 1);
            printf("%s,w,%u,%u\n", cmd_str_sensor, water_1, water_2);
            reserve_set(water_1 - 500);
        }
        else
        {
            reserve_set(atoi(str_p->part[2])); // set from config EEPROM
            printf("@db,Water threshold set: %s\n", str_p->part[2]);
        }
        break;
    case 'g':
        imu_profile_set(atoi(str_p->part[2])); // set from config EEPROM
        printf("@db,IMU set: %s\n", str_p->part[2]);
        break;
    }
    uart_tx_channel_undo();
}

static void cmd_void(h_str_pointers_t *str_p)
{
    // Handles Void module config commands:
    //  - @vd,prf,<0–5>
    //  - @vd,thr,<0–3000>   (mm threshold)
    //  - @vd,str,<0–100>    (minimum peak strength)
    //  - @vd,hys,<0–100>    (hysteresis percent)
    //  - @vd,deb,<1-10>,<1-10>  (debounce enter/exit frames)
    //  - @vd,?              -> respond current config
    //  - @vd,state,?        -> respond void state (global + per-satellite)
    //  - @vd,<p>,<mm>,<s>,<h>   (compact form; all required)

    uart_tx_channel_set(cmd_uart_ch);

    // Query current config
    if (str_p->part[1] && strcmp(str_p->part[1], "?") == 0)
    {
        printf("%s,prf,%d,thr,%lu,dia,%lu,str,%u,hys,%u,deb,%u,%u\n",
               cmd_str_void,
               xm125_profile_get(),
               (unsigned long)mti_void_get_threshold_mm(),
               (unsigned long)mti_void_get_hole_diameter_mm(),
               (unsigned)mti_radar_get_min_strength(),
               (unsigned)mti_void_get_hysteresis_pct(),
               (unsigned)mti_void_get_debounce_enter_frames(),
               (unsigned)mti_void_get_debounce_exit_frames());
        uart_tx_channel_undo();
        return;
    }

    // Query void state (global + per-satellite)
    if (str_p->part[1] && strcmp(str_p->part[1], "state") == 0)
    {
        if (str_p->part[2] && strcmp(str_p->part[2], "?") == 0)
        {
            printf("%s,state,%d", cmd_str_void, mti_void_is_detected() ? 1 : 0);
            for (uint8_t sat_id = 1; sat_id <= 6; sat_id++)
            {
                printf(",s%u,%d", (unsigned)sat_id, mti_void_get_satellite_state(sat_id) ? 1 : 0);
            }
            printf("\n");
        }
        else
        {
            printf("@db,Use @vd,state,? to query void state\n");
        }
        uart_tx_channel_undo();
        return;
    }

    bool updated = false;

    // Named argument forms
    if (str_p->part[1] && str_p->part[2])
    {
        if (strcmp(str_p->part[1], "prf") == 0)
        {
            // Set profile (1-5 only; 0 is reserved)
            char *end = NULL;
            long  p   = strtol(str_p->part[2], &end, 10);
            if (end != str_p->part[2] && p >= 1 && p <= 5)
            {
                xm125_profile_set((xm125_profile_id_t)p);
                updated = true;
            }
            else
            {
                printf("@db,Invalid prf: %ld (range 1-5)\n", p);
            }
        }
        else if (strcmp(str_p->part[1], "thr") == 0)
        {
            // Set threshold (mm)
            char *end = NULL;
            long  mm  = strtol(str_p->part[2], &end, 10);
            if (end != str_p->part[2] && mm >= 0 && mm <= 3000)
            {
                mti_void_set_threshold_mm((uint32_t)mm);
                updated = true;
            }
            else
            {
                printf("@db,Invalid thr(mm): %ld\n", mm);
            }
        }
        else if (strcmp(str_p->part[1], "str") == 0)
        {
            // Set minimum peak strength
            char *end = NULL;
            long  s   = strtol(str_p->part[2], &end, 10);
            if (end != str_p->part[2] && s >= 0 && s <= 100)
            {
                mti_radar_set_min_strength((uint16_t)s);
                updated = true;
            }
            else
            {
                printf("@db,Invalid str: %ld\n", s);
            }
        }
        else if (strcmp(str_p->part[1], "hys") == 0)
        {
            // Set hysteresis percent
            char *end = NULL;
            long  h   = strtol(str_p->part[2], &end, 10);
            if (end != str_p->part[2] && h >= 0 && h <= 100)
            {
                mti_void_set_hysteresis_pct((uint8_t)h);
                updated = true;
            }
            else
            {
                printf("@db,Invalid hys: %ld\n", h);
            }
        }
        else if (strcmp(str_p->part[1], "dia") == 0)
        {
            // Set hole diameter (mm)
            char *end = NULL;
            long  dia = strtol(str_p->part[2], &end, 10);
            if (end != str_p->part[2] && dia >= 50 && dia <= 1000)
            {
                mti_void_set_hole_diameter_mm((uint32_t)dia);
                updated = true;
            }
            else
            {
                printf("@db,Invalid dia(mm): %ld (range 50-1000)\n", dia);
            }
        }
        else if (strcmp(str_p->part[1], "deb") == 0)
        {
            // Set debounce frames: @vd,deb,<enter>,<exit>
            if (str_p->part[3])
            {
                char *end1 = NULL, *end2 = NULL;
                long  enter = strtol(str_p->part[2], &end1, 10);
                long  exit  = strtol(str_p->part[3], &end2, 10);
                if (end1 != str_p->part[2] && end2 != str_p->part[3] && enter >= 1 && enter <= 10 && exit >= 1 && exit <= 10)
                {
                    mti_void_set_debounce_frames((uint8_t)enter, (uint8_t)exit);
                    updated = true;
                }
                else
                {
                    printf("@db,Invalid deb: enter=%ld exit=%ld (range 1-10)\n", enter, exit);
                }
            }
            else
            {
                printf("@db,Missing deb exit arg\n");
            }
        }
        else
        {
            // Compact form: @vd,<p>,<mm>,<s>,<h> (profile 1-5 only)
            char *end1 = NULL, *end2 = NULL;
            long  p  = strtol(str_p->part[1], &end1, 10);
            long  mm = strtol(str_p->part[2], &end2, 10);

            if (end1 != str_p->part[1] && end2 != str_p->part[2] && p >= 1 && p <= 5 && mm >= 0 && mm <= 3000)
            {
                if (str_p->part[3] && str_p->part[4])
                {
                    char *end3 = NULL, *end4 = NULL;
                    long  s = strtol(str_p->part[3], &end3, 10);
                    long  h = strtol(str_p->part[4], &end4, 10);
                    if (end3 == str_p->part[3] || end4 == str_p->part[4] || s < 0 || s > 100 || h < 0 || h > 100)
                    {
                        printf("@db,Invalid compact str/hys\n");
                    }
                    else
                    {
                        xm125_profile_set((xm125_profile_id_t)p);
                        mti_void_set_threshold_mm((uint32_t)mm);
                        mti_radar_set_min_strength((uint16_t)s);
                        mti_void_set_hysteresis_pct((uint8_t)h);
                        updated = true;
                    }
                }
                else
                {
                    printf("@db,Missing compact args s,h\n");
                }
            }
            else
            {
                printf("@db,Invalid compact p/mm (prf 1-5, thr 0-3000)\n");
            }
        }
    }
    else
    {
        printf("@db,Invalid @vd args\n");
    }

    // Print updated config if changed
    if (updated)
    {
        printf("%s,prf,%d,thr,%lu,dia,%lu,str,%u,hys,%u,deb,%u,%u\n",
               cmd_str_void,
               xm125_profile_get(),
               (unsigned long)mti_void_get_threshold_mm(),
               (unsigned long)mti_void_get_hole_diameter_mm(),
               (unsigned)mti_radar_get_min_strength(),
               (unsigned)mti_void_get_hysteresis_pct(),
               (unsigned)mti_void_get_debounce_enter_frames(),
               (unsigned)mti_void_get_debounce_exit_frames());
    }

    uart_tx_channel_undo();
}

/* ==========================================================================
   CALLBACKS: HARDWARE TEST & DEBUG
   ========================================================================== */

static void cmd_spi(h_str_pointers_t *str_p)
{
    uart_tx_channel_set(cmd_uart_ch);
    // printf("cmd_spi\r\n");

    if (str_p->part[1] == NULL)
        return;
    if (str_p->part[2] == NULL)
        return;
    if (str_p->part[3] == NULL)
        return;

    uint8_t channel       = atoi(str_p->part[1]);
    size_t  spi_data_size = atoi(str_p->part[2]);
    if (spi_data_size > CMD_SPI_BUFF_SIZE)
        spi_data_size = CMD_SPI_BUFF_SIZE;
    atox_n(str_p->part[3], spi_tx_buff, spi_data_size);

    printf("%s", cmd_str_spi);
    printf(",%d", channel);
    printf(",%d", spi_data_size);
    printf(",t");
    // if (b_res)
    // {
    //     printf(",ok");
    // }
    // else
    // {
    //     printf(",%d", res);
    // }
    printf(",");
    // printf("\r\n");
    for (size_t data_n = 0; data_n < spi_data_size; data_n++)
        printf("%02X", spi_tx_buff[data_n]);
    // printf(",%d", res);
    printf("\r\n");

    dev_res_t res = DEV_RES_ERROR;
    // bool b_res = false;
    do
    {
        h_dev_spi_send_t h_spi = {
            .channel   = channel,
            .p_tx_buff = spi_tx_buff,
            .p_rx_buff = spi_rx_buff,
            .data_size = spi_data_size,
            .finish_cb = cmd_spi_finish,
        };
        res = dev_spi_send(&h_spi);
        if (res != DEV_RES_SUCCESS)
            break;
        // b_res = true;
    }
    while (0);

    uart_tx_channel_undo();
}

static void cmd_log(h_str_pointers_t *str_p)
{
    uart_tx_channel_set(cmd_uart_ch);
    // printf("cmd_log\r\n");

    if (str_p->part[1] == NULL)
        return;

    if (strstr(str_p->part[1], "save") != NULL)
    {
        if (str_p->part[2] != NULL)
        {
            /* data log save enable set */
            if (dev_log_save_enable_set(atoi(str_p->part[2])))
                printf("@db,Log enabled\n");
            else
                printf("@db,Log disabled\n");
        }
        cmd_print_log_save(cmd_uart_ch);
    }
    else if (strstr(str_p->part[1], "report") != NULL)
    {
        if (str_p->part[2] != NULL)
        {
            dev_log_report_enable_set(atoi(str_p->part[2]));
            dev_log_report_uart_ch_set(cmd_uart_ch);
        }
        cmd_print_log_report_busy(cmd_uart_ch);
        /* data log report */
    }
    else if (strstr(str_p->part[1], "erase") != NULL)
    {
        /* data log erase all */
        dev_log_erase_all_set();
        cmd_print_log_erase(cmd_uart_ch);
        uart_tx_wait_sent(cmd_uart_ch, 100);
    }
    else if (strstr(str_p->part[1], "auto") != NULL)
    {
        auto_log = atoi(str_p->part[2]);
        printf("@db,Auto log ");
        if (auto_log)
            printf("enabled\n");
        else
            printf("disabled\n");
    }
    uart_tx_channel_undo();
}

static void cmd_flash_fifo(h_str_pointers_t *str_p)
{
    uart_tx_channel_set(cmd_uart_ch);
    // printf("cmd_flash_fifo\r\n");

    if (str_p->part[1] == NULL)
        return;

    switch (str_p->part[1][0])
    {
    case 't':
    {
        switch (str_p->part[1][1])
        {
        case 'w':
            if (str_p->part[2] != NULL)
            {
                ff_write_count = atoi(str_p->part[2]);
            }
            else
            {
                ff_write_count = FLASH_FIFO_1_PACK_IN_MEMORY;
            }
            h_flash_fifo_1.h_debug.b_write = false;
            printf("%s,tw,start,%d\n", cmd_str_flash_fifo, ff_write_count);
            break;
        case 'c':
            b_ff_check = true;
            printf("%s,tc,start\n", cmd_str_flash_fifo);
            break;
        case 'd':
            if (str_p->part[2] != NULL)
            {
                ff_delete_count = atoi(str_p->part[2]);
            }
            else
            {
                ff_delete_count = FLASH_FIFO_1_PACK_IN_MEMORY;
            }
            h_flash_fifo_1.h_debug.b_delete = false;
            printf("%s,td,start,%d\n", cmd_str_flash_fifo, ff_delete_count);
        }
    }
    break;
    case 'w':
    {
        if (str_p->part[2] == NULL)
        {
            // printf("program");
            printf("%s,ws", cmd_str_flash_fifo);
            printf(",%d", h_flash_fifo_1.h_write.b_busy);
            printf(",%d", h_flash_fifo_1.h_write.res);
            printf("\n");
            return;
        }
        size_t data_len = atoi(str_p->part[2]);
        if (data_len > CMD_LOG_BUFF_SIZE)
            data_len = CMD_LOG_BUFF_SIZE;

        if (str_p->part[3] != NULL)
        {
            char *p_str = str_p->part[3];
            char *p_str_log;
            for (size_t data_n = 0; data_n < data_len; data_n++)
            {
                p_str_log           = p_str;
                fifo_buff_w[data_n] = strtol(p_str, &p_str, 16);
                if (p_str_log == p_str)
                {
                    data_len = data_n;
                    break;
                }
            }
        }
        else
        {
            srand(HAL_GetTick());
            for (size_t data_n = 0; data_n < data_len; data_n++)
                fifo_buff_w[data_n] = rand();
        }

        flash_fifo_res_t res;
        if (b_ff_write_busy || h_flash_fifo_1.h_write.b_busy)
        {
            res = FLASH_FIFO_RES_BUSY;
        }
        else
        {
            h_flash_fifo_w.data_size  = data_len;
            h_flash_fifo_w.p_h_header = &h_log_header_w;
            b_ff_write_busy           = true;
            res                       = flash_fifo_write(&h_flash_fifo_1, &h_flash_fifo_w);
            if (res != FLASH_FIFO_RES_OK)
                b_ff_write_busy = false;
        }
        // printf("fifo_write");
        printf("%s,w", cmd_str_flash_fifo);
        printf(",%d", data_len);
        printf(",%d", res);
        // printf("\r\n");
        printf(",");
        for (size_t data_n = 0; data_n < data_len; data_n++)
            printf("%02X ", fifo_buff_w[data_n]);
        printf("\n");
    }
    break;
    case 'n':
        printf("%s,n", cmd_str_flash_fifo);
        printf(",%d", h_flash_fifo_1.fifo_front);
        printf(",%d", h_flash_fifo_1.fifo_back);
        printf(",%d", flash_fifo_pack_n_get_saved(&h_flash_fifo_1));
        printf(",%d", flash_fifo_pack_n_get_free(&h_flash_fifo_1));
        printf("\n");
        break;
    case 'a':
    {
        size_t seq;
        if (str_p->part[2] == NULL)
        {
            seq = h_flash_fifo_1.fifo_front;
        }
        else
        {
            seq = atoi(str_p->part[2]);
        }
        flash_fifo_res_t res = flash_fifo_available_get(&h_flash_fifo_1, seq);
        printf("%s,a", cmd_str_flash_fifo);
        printf(",%d", seq);
        printf(",%d", res);
        printf("\n");
    }
    break;
    case 'r':
    {
        h_flash_fifo_header_t h_header;
        size_t                data_len = CMD_LOG_BUFF_SIZE;
        h_flash_fifo_order_t  h_order  = {
              .p_data     = fifo_buff_r,
              .data_size  = data_len,
              .p_h_header = &h_header,
        };

        flash_fifo_res_t res;
        res = flash_fifo_read_front(&h_flash_fifo_1, &h_order);
        // printf("fifo_read");
        printf("%s,r", cmd_str_flash_fifo);
        printf(",%d", data_len);
        printf(",%d", res);
        // printf("\r\n");
        if (res == FLASH_FIFO_RES_OK)
        {
            printf(",%d", h_header.data_size);
            printf(",%d", h_header.mark);
            printf(",%d", h_header.serial_seq);
            printf(",%04X", h_header.crc);
            printf(",");
            data_len = h_header.data_size;
            for (size_t data_n = 0; data_n < data_len; data_n++)
                printf("%02X ", fifo_buff_r[data_n]);
        }
        printf("\n");
    }
    break;
    case 'h':
    {
        size_t seq;
        if (str_p->part[2] == NULL)
        {
            seq = h_flash_fifo_1.fifo_front;
        }
        else
        {
            seq = atoi(str_p->part[2]);
        }
        h_flash_fifo_header_t h_header;
        flash_fifo_res_t      res;
        res = flash_fifo_header_get(&h_flash_fifo_1, seq, &h_header);
        printf("%s,h", cmd_str_flash_fifo);
        printf(",%d", res);
        if (res == FLASH_FIFO_RES_OK)
        {
            printf(",%d", h_header.data_size);
            printf(",%d", h_header.mark);
            printf(",%d", h_header.serial_seq);
            printf(",%04X", h_header.crc);
        }
        printf("\n");
    }
    break;
    case 'm':
    {
        if (str_p->part[2] == NULL)
            break;
        if (str_p->part[3] == NULL)
            break;
        size_t           seq  = atoi(str_p->part[2]);
        uint8_t          mark = atoi(str_p->part[3]);
        flash_fifo_res_t res;
        if (b_ff_mark_set_busy)
        {
            res = FLASH_FIFO_RES_BUSY;
        }
        else
        {
            h_ff_mark.seq      = seq;
            h_ff_mark.mark     = mark;
            b_ff_mark_set_busy = true;
            res                = flash_fifo_mark_set(&h_flash_fifo_1, &h_ff_mark);
            if (res != FLASH_FIFO_RES_OK)
                b_ff_mark_set_busy = false;
        }
        printf("%s,m", cmd_str_flash_fifo);
        printf(",%d", seq);
        printf(",%d", mark);
        printf(",%d", res);
        printf("\n");
    }
    break;
    case 'd':
    {
        uint8_t          seq;
        flash_fifo_res_t res;
        if (str_p->part[2] == NULL)
        {
            seq = h_flash_fifo_1.fifo_front;
        }
        else
        {
            seq = atoi(str_p->part[2]);
        }
        if (b_ff_delete_busy)
        {
            res = FLASH_FIFO_RES_BUSY;
        }
        else
        {
            ff_delete_seq    = seq;
            b_ff_delete_busy = true;
            if (str_p->part[2] == NULL)
            {
                res = flash_fifo_delete_front(&h_flash_fifo_1);
            }
            else
            {
                res = flash_fifo_delete_seq(&h_flash_fifo_1, ff_delete_seq);
            }
            if (res != FLASH_FIFO_RES_OK)
                b_ff_delete_busy = false;
        }
        printf("%s,d", cmd_str_flash_fifo);
        printf(",%d", seq);
        printf(",%d", res);
        printf("\n");
    }
    break;
    case 'e':
    {
        switch (str_p->part[1][1])
        {
        case 's':
        {
            h_flash_fifo_erase_t *p_h_erase = &h_flash_fifo_1.h_erase;
            printf("%s,es", cmd_str_flash_fifo);
            printf(",%d", h_flash_fifo_1.b_wait_to_erase);
            printf(",%d", p_h_erase->eu_seq_begin);
            printf(",%d", p_h_erase->eu_n);
            printf(",%d", p_h_erase->eu_count);
            printf(",%d", p_h_erase->eu_erase);
            printf("\n");
        }
        break;
        case 'a':
        {
            flash_fifo_res_t res = flash_fifo_erase_all(&h_flash_fifo_1);
            printf("%s,ea", cmd_str_flash_fifo);
            printf(",%d", res);
            printf("\n");
            uart_tx_wait_sent(cmd_uart_ch, 100);
        }
        break;
        case 'd':
        {
            flash_fifo_res_t res = flash_fifo_erase_dirty(&h_flash_fifo_1);
            printf("%s,ed", cmd_str_flash_fifo);
            printf(",%d", res);
            printf("\n");
            uart_tx_wait_sent(cmd_uart_ch, 100);
        }
        }
    }
    }
    uart_tx_channel_undo();
}

static void cmd_flash(h_str_pointers_t *str_p)
{
    uart_tx_channel_set(cmd_uart_ch);
    // printf("cmd_flash\r\n");

    if (str_p->part[1] == NULL)
        return;

    switch (str_p->part[1][0])
    {
    case 'e':
    {
        if (str_p->part[2] == NULL)
            break;
        uint8_t sector = atoi(str_p->part[2]);
        if (sector > FLASH_SECTOR_7)
            break;
        flash_res_t res = flash_erase_sector(sector);
        // printf("erase");
        printf("%s", cmd_str_flash);
        printf(",e");
        printf(",%d", sector);
        printf(",%d", res);
        printf("\n");
    }
    break;
    case 'p':
    {
        if (str_p->part[1][1] == 's')
        {
            // printf("program");
            printf("%s", cmd_str_flash);
            printf(",ps");
            printf(",%d", flash_program_busy_get());
            printf("\n");
            return;
        }
        if (str_p->part[2] == NULL)
            return;
        if (str_p->part[3] == NULL)
            return;

        char  *p_str = str_p->part[2];
        size_t addr  = FLASH_BASE + strtol(p_str, &p_str, 16);

        size_t len = atoi(str_p->part[3]);
        if (len > CMD_FLASH_BUFF_LEN)
            len = CMD_FLASH_BUFF_LEN;

        if (str_p->part[4] != NULL)
        {
            char *p_str_log;
            p_str = str_p->part[4];
            for (size_t data_n = 0; data_n < len; data_n++)
            {
                p_str_log          = p_str;
                flash_buff[data_n] = strtol(p_str, &p_str, 16);
                if (p_str_log == p_str)
                {
                    len = data_n;
                    break;
                }
            }
        }
        else
        {
            srand(HAL_GetTick());
            for (size_t data_n = 0; data_n < len; data_n++)
                flash_buff[data_n] = rand();
        }

        flash_res_t res = flash_program_data(addr, flash_buff, len);

        // printf("program");
        printf("%s", cmd_str_flash);
        printf(",p");
        printf(",%X", addr);
        printf(",%d", len);
        printf(",%d", res);
        // printf("\r\n");
        printf(",");
        for (size_t data_n = 0; data_n < len; data_n++)
        {
            printf("%02X ", flash_buff[data_n]);
            // if (data_n % 8 == 7)
            //     printf("\r\n");
        }
        printf("\n");
    }
    break;
    case 'r':
        if (str_p->part[2] == NULL)
            return;
        if (str_p->part[3] == NULL)
            return;

        char    *p_str = str_p->part[2];
        uint8_t *ptr   = (void *)(FLASH_BASE + strtol(p_str, &p_str, 16));
        size_t   len   = atoi(str_p->part[3]);
        if (len > CMD_FLASH_BUFF_LEN)
            len = CMD_FLASH_BUFF_LEN;

        if (str_p->part[4] != NULL)
        {
            p_str = str_p->part[4];
            char *p_str_log;
            for (size_t data_n = 0; data_n < len; data_n++)
            {
                p_str_log          = p_str;
                flash_buff[data_n] = strtol(p_str, &p_str, 16);
                if (p_str_log == p_str)
                {
                    len = data_n;
                    break;
                }
            }
        }

        bool b_check = memcmp(flash_buff, ptr, len) == 0;

        // printf("read");
        printf("%s", cmd_str_flash);
        printf(",r");
        printf(",%p", ptr);
        printf(",%d", len);
        printf(",%d", b_check);
        // printf("\r\n");
        printf(",");
        for (size_t data_n = 0; data_n < len; data_n++)
        {
            printf("%02X ", ptr[data_n]);
            // if (data_n % 8 == 7)
            //     printf("\r\n");
        }
        printf("\n");
    }
    uart_tx_channel_undo();
}

static void cmd_imu(h_str_pointers_t *str_p)
{
    uart_tx_channel_set(cmd_uart_ch);
    printf("@db,IMU set\n");
    printf("@db,%s = %s\n", str_p->part[1], str_p->part[2]);
    imu_value_set(str_p->part[1], str_p->part[2]);
}

static void cmd_debug(h_str_pointers_t *str_p)
{
    uart_tx_channel_set(cmd_uart_ch);
    // printf("cmd_debug\r\n");
    if (str_p->part[1] == NULL)
        return;
    if (strstr(str_p->part[1], "dev") != NULL)
    {
        h_dev_debug_t h_flag;
        if (str_p->part[2] != NULL)
        {
            atox_n(str_p->part[2], &h_flag, sizeof(h_dev_debug_t));
            dev_printf_debug_set(&h_flag);
        }
        else
        {
            dev_printf_debug_get(&h_flag);
        }
        printf("%s", cmd_str_debug);
        printf(",dev");
        printf(",%02X", h_flag.byte);
        printf("\n");
    }
    else if (strstr(str_p->part[1], "imu") != NULL)
    {
        live_imu = atoi(str_p->part[2]);
        imu_ch   = cmd_uart_ch;
        printf("@db,IMU dataset %d\r\n", live_imu);
    }
    else if (strstr(str_p->part[1], "multiplier") != NULL)
    {
        multiplier_set(atof(str_p->part[2]));
        printf("@db,water multiplier set: %d\r\n", atoi(str_p->part[2]));
    }
    else if (strstr(str_p->part[1], "reserve") != NULL)
    { // set from system config
        reserve_set(atoi(str_p->part[2]));
        printf("@db,water reserve threshold set: %d\r\n", atoi(str_p->part[2]));
    }
    else if (strstr(str_p->part[1], "air") != NULL)
    {
        ratio_air = atof(str_p->part[2]);
        printf("@db,air ratio = %f\r\n", ratio_air);
    }
    else if (strstr(str_p->part[1], "water") != NULL)
    {
        ratio_water = atof(str_p->part[2]);
        printf("@db,water ratio = %f\r\n", ratio_water);
    }
    else if (strstr(str_p->part[1], "adc") != NULL)
    {
        water_ch  = cmd_uart_ch;
        adc_print = !adc_print;
        printf("@db,Water ADC print = %d\r\n", adc_print);
    }
    else if (strstr(str_p->part[1], "reset") != NULL && strstr(str_p->part[2], "down") != NULL)
    {
        printf("@db,Restarting Downhole");
        HAL_NVIC_SystemReset();
    }
    else if (strcmp(str_p->part[1], "radar") == 0)
    {
        if (strcmp(str_p->part[2], "status") == 0)
        {
            can_send(CAN_CMD, CAN_STATUS);
        }
        else if (strcmp(str_p->part[2], "start") == 0)
        {
            can_send(CAN_CMD, CAN_START);
        }
        else if (strcmp(str_p->part[2], "stop") == 0)
        {
            can_send(CAN_CMD, CAN_STOP);
        }
        else if (strcmp(str_p->part[2], "power") == 0)
        {
            uint8_t message[2] = { CAN_POWER, atoi(str_p->part[3]) };
            can_send_array(CAN_CMD, message, 2);
        }
        else if (strcmp(str_p->part[2], "cal") == 0)
        {
            can_send(CAN_CMD, CAN_CAL);
        }
    }

    uart_tx_channel_undo();
}

/* ==========================================================================
   BACKGROUND STATE MACHINES (FIFO / FLASH)
   ========================================================================== */

static void cmd_fifo_multi_w_process(void)
{
    typedef enum step_
    {
        STEP_IDLE,
        STEP_DATA_RAND,
        STEP_WRITE_SET,
        STEP_WRITE_WAIT,
        STEP_FINISH,
    } step_t;
    static step_t               step     = STEP_IDLE;
    static h_flash_fifo_order_t h_fifo_w = {
        .data_size = CMD_LOG_BUFF_SIZE,
        .p_data    = fifo_buff_w,
    };
    static flash_fifo_res_t res;
    static size_t           ok_count;

    switch (step)
    {
    case STEP_IDLE:
        if (ff_write_count <= 0)
            break;

        srand(HAL_GetTick());
        ok_count = 0;

        printf("> cmd:fifo_multi_w,start");
        printf(",%d", h_flash_fifo_1.fifo_front);
        printf(",%d", h_flash_fifo_1.fifo_back);
        printf(",%d", flash_fifo_pack_n_get_saved(&h_flash_fifo_1));
        printf(",%d", flash_fifo_pack_n_get_free(&h_flash_fifo_1));
        printf("\r\n");
        step = STEP_DATA_RAND;
    case STEP_DATA_RAND:
        for (size_t data_n = 0; data_n < CMD_LOG_BUFF_SIZE; data_n++)
            fifo_buff_w[data_n] = rand();

        // if (check_count >= 12286)
        //     h_flash_fifo_1.h_debug.b_write = true;
        step = STEP_WRITE_SET;
    case STEP_WRITE_SET:
        res = flash_fifo_write(&h_flash_fifo_1, &h_fifo_w);
        switch (res)
        {
        case FLASH_FIFO_RES_BUSY:
            break;
        case FLASH_FIFO_RES_ERROR_PARA:
        case FLASH_FIFO_RES_NO_SPACE:
        case FLASH_FIFO_RES_NOT_ERASED:
            step = STEP_FINISH;
            break;
        case FLASH_FIFO_RES_OK:
            step = STEP_WRITE_WAIT;
            break;
        }
        break;
    case STEP_WRITE_WAIT:
        if (h_flash_fifo_1.h_write.b_busy)
            break;
        res = h_flash_fifo_1.h_write.res;
        if (res == FLASH_FIFO_RES_OK)
        {
            ok_count++;
            if (--ff_write_count > 0)
            {
                step = STEP_DATA_RAND;
                break;
            }
        }
        step = STEP_FINISH;
    case STEP_FINISH:
        // h_flash_fifo_1.h_debug.b_write = true;
        ff_write_count = 0;
        printf("> cmd:fifo_multi_w,finish");
        printf(",%d", res);
        printf(",%d", ok_count);
        printf(",%d", h_flash_fifo_1.fifo_back);
        printf(",%d", flash_fifo_pack_n_get_saved(&h_flash_fifo_1));
        printf(",%d", flash_fifo_pack_n_get_free(&h_flash_fifo_1));
        printf("\n");
        step = STEP_IDLE;
    }
}

static void cmd_fifo_check_process(void)
{
    typedef enum step_
    {
        STEP_IDLE,
        STEP_CHECK,
        STEP_FINISH,
    } step_t;
    static step_t                   step    = STEP_IDLE;
    const h_flash_fifo_cfg_t *const p_h_cfg = &h_flash_fifo_1.h_cfg;
    static size_t                   check_seq;
    static flash_fifo_res_t         res;
    static size_t                   ok_count;

    switch (step)
    {
    case STEP_IDLE:
        if (b_ff_check == false)
            break;

        ok_count  = 0;
        check_seq = h_flash_fifo_1.fifo_front;

        printf("> cmd:fifo_check,start");
        printf(",%d", h_flash_fifo_1.fifo_front);
        printf(",%d", h_flash_fifo_1.fifo_back);
        printf(",%d", flash_fifo_pack_n_get_saved(&h_flash_fifo_1));
        printf(",%d", flash_fifo_pack_n_get_free(&h_flash_fifo_1));
        printf("\r\n");
        step = STEP_CHECK;
    case STEP_CHECK:
        if (check_seq == h_flash_fifo_1.fifo_back)
        {
            step = STEP_FINISH;
            break;
        }

        res = flash_fifo_available_get(&h_flash_fifo_1, check_seq);
        if (res == FLASH_FIFO_RES_OK)
        {
            check_seq = FLASH_FIFO_FIFO_SEQ_NEXT(p_h_cfg, check_seq);
            ok_count++;
        }
        else
        {
            step = STEP_FINISH;
        }
        break;
    case STEP_FINISH:
        b_ff_check = false;
        printf("> cmd:fifo_check,finish");
        printf(",%d", res);
        printf(",%d", ok_count);
        printf(",%d", check_seq);
        printf("\n");
        step = STEP_IDLE;
    }
}

static void cmd_fifo_multi_d_process(void)
{
    typedef enum step_
    {
        STEP_IDLE,
        STEP_DELETE_SET,
        STEP_DELETE_WAIT,
        STEP_FINISH,
    } step_t;
    static step_t           step = STEP_IDLE;
    static flash_fifo_res_t res;
    static size_t           ok_count;

    switch (step)
    {
    case STEP_IDLE:
        if (ff_delete_count <= 0)
            break;

        srand(HAL_GetTick());
        ok_count = 0;

        // if (check_count >= 12286)
        //     h_flash_fifo_1.h_debug.b_write = true;

        printf("> cmd:fifo_multi_d,start");
        printf(",%d", h_flash_fifo_1.fifo_front);
        printf(",%d", h_flash_fifo_1.fifo_back);
        printf(",%d", flash_fifo_pack_n_get_saved(&h_flash_fifo_1));
        printf(",%d", flash_fifo_pack_n_get_free(&h_flash_fifo_1));
        printf("\r\n");
        step = STEP_DELETE_SET;
    case STEP_DELETE_SET:
        res = flash_fifo_delete_front(&h_flash_fifo_1);
        switch (res)
        {
        case FLASH_FIFO_RES_BUSY:
            break;
        case FLASH_FIFO_RES_NO_DATA:
        case FLASH_FIFO_RES_ERROR_PARA:
            step = STEP_FINISH;
            break;
        case FLASH_FIFO_RES_OK:
            step = STEP_DELETE_WAIT;
            break;
        }
        break;
    case STEP_DELETE_WAIT:
        if (h_flash_fifo_1.b_delete_busy)
            break;
        ok_count++;
        if (--ff_delete_count > 0)
        {
            step = STEP_DELETE_SET;
            break;
        }
        step = STEP_FINISH;
    case STEP_FINISH:
        // h_flash_fifo_1.h_debug.b_delete = true;
        ff_delete_count = 0;
        printf("> cmd:fifo_multi_d,finish");
        printf(",%d", res);
        printf(",%d", ok_count);
        printf(",%d", h_flash_fifo_1.fifo_back);
        printf(",%d", flash_fifo_pack_n_get_saved(&h_flash_fifo_1));
        printf(",%d", flash_fifo_pack_n_get_free(&h_flash_fifo_1));
        printf("\n");
        step = STEP_IDLE;
    }
}

/* ==========================================================================
   PUBLIC API & INTERFACE
   ========================================================================== */

void cmd_init(void)
{
    printf("command init,");
    printf("ver:%d.%d\r\n", CMD_VER_MAIN, CMD_VER_SUB);
    uart_tx_channel_set(UART_DEBUG);
    // printf("uart_command\r\n");
    // printf("uart_delimiters:%s|\r\n", delimiters);
    // printf("uart_EndChar:%s|\r\n", EndChar);
    // printf("uart_command:%s|\r\n", cmd_str_example);
    for (uart_select_t ch_n = UART_FIRST; ch_n < UART_NUMBER; ch_n++)
    {
        string_command_set(&h_str[ch_n]);
    }
    // string_command_set(&h_str[UART_DEBUG]);
}

void cmd_process(void)
{
    for (uart_select_t ch_n = UART_FIRST; ch_n < UART_NUMBER; ch_n++)
    {
        cmd_uart_process(ch_n);
    }
    // cmd_uart_process(UART_DEBUG, &h_str_q[UART_DEBUG]);
    cmd_fifo_multi_w_process();
    cmd_fifo_check_process();
    cmd_fifo_multi_d_process();
}

void cmd_print_bottom(uart_select_t channel)
{
    uart_tx_channel_set(channel);
    printf("%s", cmd_str_bottom);
    printf(",%d", dev_dump_get());
    printf("\n");
    uart_tx_channel_undo();
}

void cmd_print_water(uart_select_t channel)
{
    uart_tx_channel_set(channel);
    printf("%s", cmd_str_water);
    printf(",%d", dev_water_det_get());
    printf("\n");
    uart_tx_channel_undo();
}

void cmd_print_init_g_log(uart_select_t channel)
{
    uart_tx_channel_set(channel);
    bool b_busy = imu_g_log_busy_get(0);
    b_busy |= imu_g_log_busy_get(1);
    printf("%s,g", cmd_str_init);
    printf(",%d", b_busy);
    printf("\n");
    uart_tx_channel_undo();
}

void cmd_print_log_save(uart_select_t channel)
{
    uart_tx_channel_set(channel);
    printf("%s,save", cmd_str_log);
    printf(",%d", dev_log_save_busy_get());
    printf(",%d", dev_log_save_overflow_get());
    printf("\n");
    uart_tx_channel_undo();
}

void cmd_print_log_report_busy(uart_select_t channel)
{
    uart_tx_channel_set(channel);
    // printf("%s,report", cmd_str_log);
    // printf(",%d", dev_log_report_busy_get());
    // printf("\n");
    if (dev_log_report_busy_get())
        printf("@db,Exporting flash\n");
    else
        printf("@da,done\n"); // data export complete
    uart_tx_channel_undo();
}

void cmd_print_log_report_data(uart_select_t channel)
{ // logEdit
    uart_tx_channel_set(channel);
    h_dev_log_t h_log;
    dev_log_data_get(&h_log);
    // printf("%s,data", cmd_str_log);
    printf("@da");
    printf(",%d", h_log.time);
    h_dev_log_imu_t *p_h_imu = &h_log.h_imu[0];
    printf(":%d", p_h_imu->acc_x);
    printf(":%d", p_h_imu->acc_y);
    printf(":%d", p_h_imu->acc_z);
    printf(":%d", p_h_imu->angle_acc);
    printf(":%d", p_h_imu->angle_gyro);
    p_h_imu = &h_log.h_imu[1];
    printf(":%d", p_h_imu->acc_x);
    printf(":%d", p_h_imu->acc_y);
    printf(":%d", p_h_imu->acc_z);
    printf(":%d", p_h_imu->angle_acc);
    printf(":%d", p_h_imu->angle_gyro);
    printf("\n");
    uart_tx_channel_undo();
}

void cmd_print_log_erase(uart_select_t channel)
{
    uart_tx_channel_set(channel);
    printf("%s,erase", cmd_str_log);
    printf(",%d", dev_log_erase_busy_get());
    printf("\n");
    uart_tx_channel_undo();
}

void cmd_print_wake(uart_select_t channel)
{
    uart_tx_channel_set(channel);
    printf("@db,%s\n", cmd_str_wake);
    uart_tx_channel_undo();
}

void cmd_print_flash_fifo_write_finish(uart_select_t channel)
{
    if (b_ff_write_busy == false)
        return;
    b_ff_write_busy = false;
    uart_tx_channel_set(channel);
    printf("%s", cmd_str_flash);
    printf(",pf"); // program finish.
    size_t data_size = h_flash_fifo_w.data_size;
    printf(",%d", data_size);
    printf(",%d", h_log_header_w.data_size);
    printf(",%d", h_log_header_w.mark);
    printf(",%d", h_log_header_w.serial_seq);
    printf(",%d", h_log_header_w.crc);
    printf(",");
    for (size_t data_n = 0; data_n < data_size; data_n++)
    {
        printf("%02X ", fifo_buff_w[data_n]);
    }
    printf("\n");
    uart_tx_channel_undo();
}

void cmd_print_flash_fifo_delete_finish(uart_select_t channel)
{
    if (b_ff_delete_busy == false)
        return;
    b_ff_delete_busy = false;
    uart_tx_channel_set(channel);
    printf("%s,df", cmd_str_flash_fifo);
    printf(",%d", ff_delete_seq);
    printf("\n");
    uart_tx_channel_undo();
}

void cmd_print_flash_fifo_mark_finish(uart_select_t channel)
{
    if (b_ff_mark_set_busy == false)
        return;
    b_ff_mark_set_busy = false;
    uart_tx_channel_set(channel);
    printf("%s,mf", cmd_str_flash_fifo);
    printf(",%d", h_ff_mark.seq);
    printf(",%d", h_ff_mark.mark);
    printf("\n");
    uart_tx_channel_undo();
}
