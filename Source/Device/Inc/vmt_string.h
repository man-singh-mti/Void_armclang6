#ifndef __VMT_STRING_H__
#define __VMT_STRING_H__

// #pragma anon_unions
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* parameter */
#define STRING_VER_GENERATION (0)
#define STRING_VER_MAIN       (1)
#define STRING_VER_SUB        (7)

#define STR_SECTION_MAX (16)

/* variable */
typedef struct h_str_pointers_
{
    char *head, *sub, *part[STR_SECTION_MAX], delimiters[STR_SECTION_MAX], *end;
} h_str_pointers_t;

typedef struct h_str_cmd_
{
    const char *ptr;
    size_t      len;
    void (*callback)(h_str_pointers_t *p_h_str_p);
} h_str_cmd_t;

typedef struct h_string_ h_string_t;
struct h_string_
{
    /* configuration */
    uint8_t      id;
    h_str_cmd_t *p_h_cmd;
    char        *p_cmd_head;
    uint8_t      cmd_n;
    char        *p_delimiters, *p_end_char;
    void (*cmd_find_cb)(uint8_t id);

    /* internal parameter */
    size_t           cmd_len_max;
    size_t           check_n;
    h_str_pointers_t h_str_p;
};

/* macro */
#define STR_H_STR_CMD_LEN_GET(__H_STR_CMD)      (sizeof(__H_STR_CMD) / sizeof(h_str_cmd_t))
#define STR_H_STR_CMD_HEAD_LEN_GET(__H_STR_CMD) (STR_H_STR_CMD_LEN_GET(__H_STR_CMD) + 1)

/* function */
bool string_command_set(h_string_t *str_h);

void string_decoder_by_end(h_string_t *p_h_str, char *p_str, size_t len);

char *string_find_end(h_string_t *str_h, char *str);
/**
 * @param  str_h: pointer to a string_HandleType structure
 * @param  str:   Pointer to string
 * @retval returns a command_p numbering,
 *         or _string_CMD_n_max_ if not found.
 */
char  string_find_head(h_string_t *str_h, char *str);
char *string_find_head_first_char(h_string_t *str_h, char *str);
void  string_section(h_string_t *str_h, char *str);

char    *strtok_user(char *str, const char *delimiters);
uint32_t atox(const char *data);
void     atox_n(const char *p_str, void *p_data, size_t size);
void     atox_n_b_end(const char *p_str, void *p_data, size_t size);
char    *str_find_hex(const char *p_str_in, size_t *str_out_len);
void     flip(void *p_buff, size_t buff_len);

#endif /*__VMT_STRING_H__ */
