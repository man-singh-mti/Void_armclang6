#include "vmt_string.h"

/* function declaration */
static void string_check_n_update(h_string_t *p_h_str, char *p_str, size_t len);

/* function define */
bool string_command_set(h_string_t *p_h_str)
{
    h_str_cmd_t *command_h;
    for (size_t i = 0; i < p_h_str->cmd_n; i++)
    {
        command_h = &p_h_str->p_h_cmd[i];
        if (command_h->ptr[0] == '\0')
            return false;

        p_h_str->p_cmd_head[i] = command_h->ptr[0];
        command_h->len         = strlen(command_h->ptr);
        if (command_h->len > p_h_str->cmd_len_max)
            p_h_str->cmd_len_max = command_h->len;
    }
    return true;
}

void string_decoder_by_end(h_string_t *p_h_str, char *p_str, size_t len)
{
    // uint16_t len = strlen(p_str);
    char              command_sequence = p_h_str->cmd_n;
    h_str_pointers_t *p_h_str_p        = &p_h_str->h_str_p;
    bool              b_cmd_not_find   = true;

    // UR_CH_DEBUG();
    // printf("%s|\r\n", p_str);

    do
    {
        size_t check_n;
        for (check_n = 0; check_n < len; check_n++)
        {
            if (p_str[check_n] != 0x0)
                break;
        }
        if (check_n != 0)
        {
            p_h_str->check_n = check_n;
            if (p_h_str->cmd_find_cb != NULL)
                p_h_str->cmd_find_cb(p_h_str->id);
            return;
        }

        string_find_end(p_h_str, p_str);
        // printf("%p|%s\r\n", p_str, p_str);
        // printf("%p|%s\r\n", str_h->str_p.end + 1, str_h->str_p.end + 1);

        command_sequence = string_find_head(p_h_str, p_str);
        // printf("%p|%s\r\n", str_h->str_p.head, str_h->str_p.head);
        // printf("%p|%s\r\n", str_h->str_p.sub, str_h->str_p.sub);

        if (p_h_str->h_str_p.end == NULL)
            break;
        if (command_sequence == p_h_str->cmd_n)
            break;

        b_cmd_not_find = false;
        string_check_n_update(p_h_str, p_str, len);

        if (p_h_str->p_delimiters[0] != '\0')
            string_section(p_h_str, p_h_str->h_str_p.sub);
        // for (int i = 0; i < _string_section_max_; i++)
        //     printf("%d|%p|%c|%s\r\n", i, str_h->str_p.part[i], str_h->str_p.delimiters[i], str_h->str_p.part[i]);

        if (p_h_str->p_h_cmd[command_sequence].callback != NULL)
            p_h_str->p_h_cmd[command_sequence].callback(p_h_str_p);
    }
    while (0);

    if (b_cmd_not_find)
        string_check_n_update(p_h_str, p_str, len);

    // for (int i = 0; i < len; i++)
    //     printf("%c", p_str[i]);
    // printf("\r\n");
}

static void string_check_n_update(h_string_t *p_h_str, char *p_str, size_t len)
{
    h_str_pointers_t *p_h_str_p = &p_h_str->h_str_p;
    do
    {
        if (p_h_str_p->end != NULL)
        {
            p_h_str->check_n = (p_h_str_p->end + 1) - p_str;
            break;
        }
        if (p_h_str_p->head != NULL)
        {
            p_h_str->check_n = p_h_str_p->head - p_str;
            break;
        }
        /* found head first char */
        char  *p_next      = p_str;
        size_t cmd_len_max = p_h_str->cmd_len_max;
        if (len >= cmd_len_max)
            p_next = p_str + len - cmd_len_max + 1;
        p_next = string_find_head_first_char(p_h_str, p_next);
        if (p_next != NULL)
        {
            /* hold head first char */
            p_h_str->check_n = p_next - p_str;
            break;
        }
        p_h_str->check_n = len;
    }
    while (0);

    if (p_h_str->cmd_find_cb != NULL)
        p_h_str->cmd_find_cb(p_h_str->id);
}

char *string_find_end(h_string_t *str_h, char *str)
{
    char *ptr = str;

    ptr = strpbrk(ptr, str_h->p_end_char);
    if (ptr != NULL)
        *ptr = '\0';
    str_h->h_str_p.end = ptr;
    return str_h->h_str_p.end;
}

/**
 * @param  str_h: pointer to a string_HandleType structure
 * @param  str:   Pointer to string
 * @retval returns a command numbering,
 *         or _string_command_max_ if not found.
 */
char string_find_head(h_string_t *str_h, char *str)
{
    char *ptr = str;

    while (1)
    {
        ptr = strpbrk(ptr, str_h->p_cmd_head);
        if (ptr == NULL)
        {
            str_h->h_str_p.head = NULL;
            str_h->h_str_p.sub  = NULL;
            break;
        }

        for (int i = 0; i < str_h->cmd_n; i++)
        {
            if (*ptr != str_h->p_h_cmd[i].ptr[0])
                continue;

            if (memcmp(ptr, str_h->p_h_cmd[i].ptr, str_h->p_h_cmd[i].len) == 0)
            {
                str_h->h_str_p.head = ptr;
                str_h->h_str_p.sub  = ptr + str_h->p_h_cmd[i].len;
                return i;
            }
        }

        ptr++;
    }
    return str_h->cmd_n;
}
char *string_find_head_first_char(h_string_t *str_h, char *str)
{
    return strpbrk(str, str_h->p_cmd_head);
}

void string_section(h_string_t *str_h, char *str)
{
    char *ptr    = str;
    char  part_n = 0;

    memset(str_h->h_str_p.part, NULL, sizeof(str_h->h_str_p.part));
    memset(str_h->h_str_p.delimiters, '\0', sizeof(str_h->h_str_p.delimiters));
    str_h->h_str_p.part[part_n] = str;

    while (part_n + 1 < STR_SECTION_MAX)
    {
        ptr = str_h->h_str_p.part[part_n];
        part_n++;

        ptr = strpbrk(ptr, str_h->p_delimiters);
        if (ptr == NULL)
        {
            str_h->h_str_p.part[part_n] = ptr;
            break;
        }
        else
        {
            str_h->h_str_p.delimiters[part_n] = *ptr;
            *ptr                              = '\0';
            str_h->h_str_p.part[part_n]       = ptr + 1;
        }
    }
}

char *strtok_user(char *str, const char *delimiters)
{
    static char *sp  = NULL;
    int          i   = 0;
    int          len = strlen(delimiters);

    //    UR_CH_DEBUG();
    //    if (len == 0)
    //        printf("delimiters are empty\r\n");

    if (!str && !sp)
        return NULL;
    if (str && !sp)
        sp = str;

    /* find the start of the substring, skip delimiters */
    char *p_start = sp;
    while (1)
    {
        for (i = 0; i < len; i++)
        {
            if (*p_start == delimiters[i])
            {
                p_start++;
                break;
            }
        }

        if (i == len)
        {
            sp = p_start;
            break;
        }
    }

    /* return NULL if nothing left */
    if (*sp == '\0')
    {
        sp = NULL;
        return sp;
    }

    /* find the end of the substring,
     and replace the delimiter with null */
    while (*sp != '\0')
    {
        for (i = 0; i < len; i++)
        {
            if (*sp == delimiters[i])
            {
                *sp = '\0';
                break;
            }
        }

        sp++;
        if (i < len)
            break;
    }

    return p_start;
}

uint32_t atox(const char *data)
{
    int      i   = 0;
    uint32_t hex = 0;
    char    *p;
    p = strstr(data, "0x");
    if (p == NULL)
        p = strstr(data, "0X");
    else
    {
        p += 2;
        while (i >= 0)
        {
            if (*(p + i) >= '0' && *(p + i) <= '9')
            {
                hex <<= 4;
                hex += *(p + i) - '0';
                i++;
            }
            else if (*(p + i) >= 'a' && *(p + i) <= 'f')
            {
                hex <<= 4;
                hex += *(p + i) - 'a' + 10;
                i++;
            }
            else if (*(p + i) >= 'A' && *(p + i) <= 'F')
            {
                hex <<= 4;
                hex += *(p + i) - 'A' + 10;
                i++;
            }
            else
            {
                i = -1;
            }
        }
    }
    return hex;
}

void atox_n(const char *p_str, void *p_data, size_t size)
{
    memset(p_data, 0x0, size);

    size_t str_hex_len;
    char  *p_str_hex = str_find_hex(p_str, &str_hex_len);

    uint8_t  data_u4   = 0x0;
    uint8_t *p_data_u8 = p_data;
    size_t   data_u8_n = 0;
    bool     b_data_h  = true;
    char     ascii;
    for (size_t hex_char_n = 0; hex_char_n < str_hex_len; hex_char_n++)
    {
        ascii = p_str_hex[hex_char_n];
        if ((ascii >= '0') && (ascii <= '9'))
        {
            data_u4 = ascii - '0' + 0x0;
        }
        else if ((ascii >= 'a') && (ascii <= 'f'))
        {
            data_u4 = ascii - 'a' + 0xA;
        }
        else if ((ascii >= 'A') && (ascii <= 'F'))
        {
            data_u4 = ascii - 'A' + 0xA;
        }
        else
        {
            break;
        }

        if (b_data_h)
        {
            p_data_u8[data_u8_n] = data_u4 << 4;
        }
        else
        {
            p_data_u8[data_u8_n] |= data_u4;
            data_u8_n++;
        }
        b_data_h ^= true;
        if (data_u8_n >= size)
            break;
    }
}

void atox_n_b_end(const char *p_str, void *p_data, size_t size)
{
    memset(p_data, 0x0, size);

    size_t str_hex_len;
    char  *p_str_hex = str_find_hex(p_str, &str_hex_len);

    uint8_t  data_u4   = 0x0;
    uint8_t *p_data_u8 = p_data;
    size_t   data_u8_n = 0;
    bool     b_data_l  = true;
    char     ascii;
    for (size_t hex_char_n = 0; hex_char_n < str_hex_len; hex_char_n++)
    {
        ascii = p_str_hex[str_hex_len - 1 - hex_char_n];
        if ((ascii >= '0') && (ascii <= '9'))
        {
            data_u4 = ascii - '0' + 0x0;
        }
        else if ((ascii >= 'a') && (ascii <= 'f'))
        {
            data_u4 = ascii - 'a' + 0xA;
        }
        else if ((ascii >= 'A') && (ascii <= 'F'))
        {
            data_u4 = ascii - 'A' + 0xA;
        }
        else
        {
            break;
        }

        if (b_data_l)
        {
            p_data_u8[data_u8_n] = data_u4;
        }
        else
        {
            p_data_u8[data_u8_n] |= data_u4 << 4;
            data_u8_n++;
        }
        b_data_l ^= true;
        if (data_u8_n >= size)
            break;
    }
}

char *str_find_hex(const char *p_str_in, size_t *str_out_len)
{
    *str_out_len    = 0;
    char *p_str_out = strstr(p_str_in, "0x");
    if (p_str_out != NULL)
    {
        p_str_out += strlen("0x");
    }
    else
    {
        p_str_out = strstr(p_str_in, "0X");
        if (p_str_out != NULL)
        {
            p_str_out += strlen("0X");
        }
        else
        {
            p_str_out = (char *)p_str_in;
        }
    }
    const char str_key[] = "0123456789abcdefABCDEF";
    p_str_out            = strpbrk(p_str_out, str_key);
    if (p_str_out == NULL)
        return p_str_out;

    size_t str_key_len = strlen(str_key);
    for (; memchr(str_key, p_str_out[*str_out_len], str_key_len) != NULL; (*str_out_len)++)
    {
    }
    return p_str_out;
}

void flip(void *p_buff, size_t buff_len)
{
    static uint8_t *p_buff_u8;
    static size_t   len_half;
    static size_t   byte_end;
    static uint8_t  temp_u8;

    p_buff_u8 = p_buff;
    len_half  = buff_len / 2;
    byte_end  = buff_len - 1;
    for (size_t byte_n = 0; byte_n < len_half; byte_n++)
    {
        temp_u8                      = p_buff_u8[byte_n];
        p_buff_u8[byte_n]            = p_buff_u8[byte_end - byte_n];
        p_buff_u8[byte_end - byte_n] = temp_u8;
    }
}
