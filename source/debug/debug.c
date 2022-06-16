

#include <stdarg.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "debug.h"
#include <stdbool.h>
#include <string.h>

#include "usart.h"

typedef enum
{
    DEBUG_MESSAGE_INFO,
    DEBUG_MESSAGE_WARN,
    DEBUG_MESSAGE_ERROR,
    DEBUG_MESSAGE_REPLY
} DebugMessageType;

typedef struct _dbg
{
    char key[10];
    DebugHandler handler;
    bool enable_error_log;
    bool enable_warning;
    bool enable_info;

    struct _dbg *pNext;
} DebugNode;

typedef struct
{
    bool enable_error_log;
    bool enable_warning;
    bool enable_info;

    xSemaphoreHandle lock;

    UsartHandle port;

    DebugNode *pHead;

    char reply[512];
    uint8_t buffer[128];
    uint8_t index;
} DebugManager;

DebugManager dbg_man;

void DebugTask(void *param);
void DebugKeyHandler(char *reply, const char **param, uint16_t count);
void DebugSendByte(uint8_t byte);

void DebugInit()
{
    dbg_man.lock = xSemaphoreCreateBinary();
    xSemaphoreGive(dbg_man.lock);

    DebugNode *node = pvPortMalloc(sizeof(DebugNode));
    node->handler = DebugKeyHandler;
    node->enable_error_log = true;
    node->enable_info = true;
    node->enable_warning = true;
    strcpy(node->key, "DEBUG");
    node->pNext = NULL;

    dbg_man.pHead = node;

    dbg_man.enable_error_log = true;
    dbg_man.enable_info = true;
    dbg_man.enable_warning = true;
    dbg_man.port = InitUsart(COM1, 115200, 0, 128);
    xTaskCreate(DebugTask, "Debug", 1024, NULL, 3, NULL);
}

DebugChannel DebugRegister(const char *key, DebugHandler handler)
{
    DebugNode *node = pvPortMalloc(sizeof(DebugNode));
    node->handler = handler;
    strcpy(node->key, key);
    node->pNext = NULL;
    node->pNext = dbg_man.pHead;
    dbg_man.pHead = node;

    node->enable_error_log = true;
    node->enable_info = true;
    node->enable_warning = true;

    return node;
}

void DebugSendKey(DebugMessageType type, const char *module)
{
    // DebugSendByte(0x1b);
    // DebugSendByte('[');
    // switch (type)
    // {
    // case DEBUG_MESSAGE_INFO:
    //     DebugSendByte('0');
    //     break;

    // case DEBUG_MESSAGE_WARN:
    //     DebugSendByte('3');
    //     DebugSendByte('3');
    //     break;

    // case DEBUG_MESSAGE_ERROR:
    //     DebugSendByte('3');
    //     DebugSendByte('1');
    //     break;

    // case DEBUG_MESSAGE_REPLY:
    //     DebugSendByte('3');
    //     DebugSendByte('2');
    //     break;

    // default:
    //     break;
    // }
    // DebugSendByte('m');
    UsartSendByte(dbg_man.port, '>');
    UsartSendString(dbg_man.port, module, strlen(module));
    UsartSendByte(dbg_man.port, '[');
    switch (type)
    {
    case DEBUG_MESSAGE_INFO:
        UsartSendString(dbg_man.port, "INFO", 4);
        break;
    case DEBUG_MESSAGE_WARN:
        UsartSendString(dbg_man.port, "WARN", 4);
        break;

    case DEBUG_MESSAGE_ERROR:
        UsartSendString(dbg_man.port, "ERROR", 5);
        break;

    case DEBUG_MESSAGE_REPLY:
        UsartSendString(dbg_man.port, "REPLY", 5);
        break;

    default:
        break;
    }

    UsartSendByte(dbg_man.port, ']');
    UsartSendByte(dbg_man.port, ':');
    UsartSendByte(dbg_man.port, ' ');
}

void DebugError(DebugChannel ch, const char *error)
{
    if (ch == NULL)
        return;
    DebugNode *nch = (DebugNode *)ch;
    if (dbg_man.enable_error_log != true || nch->enable_error_log != true)
        return;
    if (xSemaphoreTake(dbg_man.lock, 10000) != pdFALSE)
    {
        DebugSendKey(DEBUG_MESSAGE_ERROR, nch->key);
        UsartSendString(dbg_man.port, error, strlen(error));
        xSemaphoreGive(dbg_man.lock);
    }
}

void DebugInfo(DebugChannel ch, const char *info)
{
    if (ch == NULL)
        return;
    DebugNode *nch = (DebugNode *)ch;
    if (dbg_man.enable_info != true || nch->enable_info != true)
        return;

    if (xSemaphoreTake(dbg_man.lock, 10000) != pdFALSE)
    {
        DebugSendKey(DEBUG_MESSAGE_INFO, nch->key);
        UsartSendString(dbg_man.port, info, strlen(info));
        xSemaphoreGive(dbg_man.lock);
    }
}

void DebugWarning(DebugChannel ch, const char *warning)
{
    if (ch == NULL)
        return;
    DebugNode *nch = (DebugNode *)ch;
    if (dbg_man.enable_warning != true || nch->enable_warning != true)
        return;
    if (xSemaphoreTake(dbg_man.lock, 10000) != pdFALSE)
    {
        DebugSendKey(DEBUG_MESSAGE_WARN, nch->key);
        UsartSendString(dbg_man.port, warning, strlen(warning));
        xSemaphoreGive(dbg_man.lock);
    }
}

void HandleInputKey(char *str)
{
    char *ptr = str;
    uint8_t len = strlen(str);
    char *lst[20];
    uint8_t count = 0;
    for (int i = 0; i < len; i++)
    {
        if (ptr[i] == ' ')
        {
            ptr[i] = 0;
            i++;
            if (count > 19)
                return;
            lst[count++] = &ptr[i];
        }
    }

    DebugNode *node = dbg_man.pHead;
    bool flag = 0;

    while (node != NULL)
    {
        //DebugSendString(node->key);
        if (strcmp(node->key, str) == 0)
        {
            if (node->handler != NULL)
            {
                if (strcmp(lst[0], "ERROR") == 0)
                {
                    if (strcmp(lst[1], "ON") == 0)
                    {
                        node->enable_error_log = true;
                        strcpy(dbg_man.reply, "Error log of <");
                        strcat(dbg_man.reply, node->key);
                        strcat(dbg_man.reply, "> turned on.\r\n");
                    }
                    else if (strcmp(lst[1], "OFF") == 0)
                    {
                        node->enable_error_log = false;
                        strcpy(dbg_man.reply, "Error log of <");
                        strcat(dbg_man.reply, node->key);
                        strcat(dbg_man.reply, "> turned off.\r\n");
                    }
                    else
                    {
                        strcpy(dbg_man.reply, "Unknown option for ");
                        strcat(dbg_man.reply, node->key);
                        strcat(dbg_man.reply, " -> ");
                        strcat(dbg_man.reply, lst[0]);
                        strcat(dbg_man.reply, " command.\r\n");
                    }
                }
                else if (strcmp(lst[0], "WARN") == 0)
                {
                    if (strcmp(lst[1], "ON") == 0)
                    {
                        node->enable_warning = true;
                        strcpy(dbg_man.reply, "Warning log of <");
                        strcat(dbg_man.reply, node->key);
                        strcat(dbg_man.reply, "> turned on.\r\n");
                    }
                    else if (strcmp(lst[1], "OFF") == 0)
                    {
                        node->enable_warning = false;
                        strcpy(dbg_man.reply, "Warning log of <");
                        strcat(dbg_man.reply, node->key);
                        strcat(dbg_man.reply, "> turned off.\r\n");
                    }
                    else
                    {
                        strcpy(dbg_man.reply, "Unknown option for ");
                        strcat(dbg_man.reply, node->key);
                        strcat(dbg_man.reply, " -> ");
                        strcat(dbg_man.reply, lst[0]);
                        strcat(dbg_man.reply, " command.\r\n");
                    }
                }
                else if (strcmp(lst[0], "INFO") == 0)
                {
                    if (strcmp(lst[1], "ON") == 0)
                    {
                        node->enable_info = true;
                        strcpy(dbg_man.reply, "Info log of <");
                        strcat(dbg_man.reply, node->key);
                        strcat(dbg_man.reply, "> turned on.\r\n");
                    }
                    else if (strcmp(lst[1], "OFF") == 0)
                    {
                        node->enable_info = false;
                        strcpy(dbg_man.reply, "Info log of <");
                        strcat(dbg_man.reply, node->key);
                        strcat(dbg_man.reply, "> turned off.\r\n");
                    }
                    else
                    {
                        strcpy(dbg_man.reply, "Unknown option for ");
                        strcat(dbg_man.reply, node->key);
                        strcat(dbg_man.reply, " -> ");
                        strcat(dbg_man.reply, lst[0]);
                        strcat(dbg_man.reply, " command.\r\n");
                    }
                }
                else if(strcmp(lst[0], "ALL") == 0)
                {
                    if (strcmp(lst[1], "ON") == 0)
                    {
                        node->enable_info = true;
                        node->enable_warning = true;
                        node->enable_error_log = true;
                        strcpy(dbg_man.reply, "Info, Warning and Error log of <");
                        strcat(dbg_man.reply, node->key);
                        strcat(dbg_man.reply, "> turned on.\r\n");
                    }
                    else if (strcmp(lst[1], "OFF") == 0)
                    {
                        node->enable_info = false;
                        node->enable_warning = false;
                        node->enable_error_log = false;
                        strcpy(dbg_man.reply, "Info, Warning and Error log of <");
                        strcat(dbg_man.reply, node->key);
                        strcat(dbg_man.reply, "> turned off.\r\n");
                    }
                    else
                    {
                        strcpy(dbg_man.reply, "Unknown option for ");
                        strcat(dbg_man.reply, node->key);
                        strcat(dbg_man.reply, " -> ");
                        strcat(dbg_man.reply, lst[0]);
                        strcat(dbg_man.reply, " command.\r\n");
                    }
                }
                else
                {
                    memset(dbg_man.reply, 0, 512);
                    node->handler((char *)dbg_man.reply, (const char **)lst, count);
                }

                flag = true;
                break;
            }
        }
        node = node->pNext;
    }

    if (flag == false)
        strcpy((char *)dbg_man.reply, "Command module not registerd or Not implemented.\r\n");

    if (xSemaphoreTake(dbg_man.lock, 1000) != pdFALSE)
    {
        DebugSendKey(DEBUG_MESSAGE_REPLY, node->key);
        UsartSendString(dbg_man.port, (const char *)dbg_man.reply, strlen((char *)dbg_man.reply));
        xSemaphoreGive(dbg_man.lock);
    }
}

void DebugTask(void *param)
{
    while (true)
    {
        uint8_t data = 0;
        if (UsartReceiveByte(dbg_man.port, &data) != pdFALSE)
        {
            if (data == '\n')
            {
                if (dbg_man.index != 0)
                {
                    HandleInputKey((char *)dbg_man.buffer);
                    memset(dbg_man.buffer, 0, 128);
                    dbg_man.index = 0;
                }
            }
            else
            {
                dbg_man.buffer[dbg_man.index++] = data;
                if (dbg_man.index >= 128)
                {
                    memset(dbg_man.buffer, 0, 128);
                    dbg_man.index = 0;
                }
            }
        }
    }
}

void DebugKeyHandler(char *reply, const char **param, uint16_t count)
{
    if (count == 2)
    {
        if (strcmp(param[0], "ERROR") == 0)
        {
            if (strcmp(param[1], "ON") == 0)
            {
                dbg_man.enable_error_log = true;
                strcpy(reply, "Error logging turned on!\r\n");
            }
            else if (strcmp(param[1], "OFF") == 0)
            {
                dbg_man.enable_error_log = false;
                strcpy(reply, "Error logging turned off!\r\n");
            }
            else
            {
                strcpy(reply, "Unknown error log setting!\r\n");
            }
        }
        else if (strcmp(param[0], "WARN") == 0)
        {
            if (strcmp(param[1], "ON") == 0)
            {
                dbg_man.enable_warning = true;
                strcpy(reply, "Warning logging turned on!\r\n");
            }
            else if (strcmp(param[1], "OFF") == 0)
            {
                dbg_man.enable_warning = false;
                strcpy(reply, "Warning logging turned off!\r\n");
            }
            else
            {
                strcpy(reply, "Unknown error log setting!\r\n");
            }
        }
        else if (strcmp(param[0], "INFO") == 0)
        {
            if (strcmp(param[1], "ON") == 0)
            {
                dbg_man.enable_info = true;
                strcpy(reply, "Info logging turned on!\r\n");
            }
            else if (strcmp(param[1], "OFF") == 0)
            {
                dbg_man.enable_info = false;
                strcpy(reply, "Info logging turned off!\r\n");
            }
            else
            {
                strcpy(reply, "Unknown info log setting!\r\n");
            }
        }
    }
}

#define PAD_RIGHT 1
#define PAD_ZERO 2

static void printchar(char **out, unsigned int c)
{
    UsartSendByte(dbg_man.port, (uint8_t)c);
}

static int prints(char **out, const char *string, int width, int pad)
{
    register int pc = 0, padchar = ' ';

    if (width > 0)
    {
        register int len = 0;
        register const char *ptr;
        for (ptr = string; *ptr; ++ptr)
            ++len;
        if (len >= width)
            width = 0;
        else
            width -= len;
        if (pad & PAD_ZERO)
            padchar = '0';
    }
    if (!(pad & PAD_RIGHT))
    {
        for (; width > 0; --width)
        {
            printchar(out, padchar);
            ++pc;
        }
    }
    for (; *string; ++string)
    {
        printchar(out, *string);
        ++pc;
    }
    for (; width > 0; --width)
    {
        printchar(out, padchar);
        ++pc;
    }

    return pc;
}

/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 12

static int printi(char **out, int i, int b, int sg, int width, int pad, int letbase)
{
    char print_buf[PRINT_BUF_LEN];
    register char *s;
    register int t, neg = 0, pc = 0;
    register unsigned int u = i;

    if (i == 0)
    {
        print_buf[0] = '0';
        print_buf[1] = '\0';
        return prints(out, print_buf, width, pad);
    }

    if (sg && b == 10 && i < 0)
    {
        neg = 1;
        u = -i;
    }

    s = print_buf + PRINT_BUF_LEN - 1;
    *s = '\0';

    while (u)
    {
        t = u % b;
        if (t >= 10)
            t += letbase - '0' - 10;
        *--s = t + '0';
        u /= b;
    }

    if (neg)
    {
        if (width && (pad & PAD_ZERO))
        {
            printchar(out, '-');
            ++pc;
            --width;
        }
        else
        {
            *--s = '-';
        }
    }

    return pc + prints(out, s, width, pad);
}

static int print(char **out, const char *format, va_list args)
{
    register int width, pad;
    register int pc = 0;
    char scr[2];

    for (; *format != 0; ++format)
    {
        if (*format == '%')
        {
            ++format;
            width = pad = 0;
            if (*format == '\0')
                break;
            if (*format == '%')
                goto out;
            if (*format == '-')
            {
                ++format;
                pad = PAD_RIGHT;
            }
            while (*format == '0')
            {
                ++format;
                pad |= PAD_ZERO;
            }
            for (; *format >= '0' && *format <= '9'; ++format)
            {
                width *= 10;
                width += *format - '0';
            }
            if (*format == 's')
            {
                register char *s = (char *)va_arg(args, int);
                pc += prints(out, s ? s : "(null)", width, pad);
                continue;
            }
            if (*format == 'd')
            {
                pc += printi(out, va_arg(args, int), 10, 1, width, pad, 'a');
                continue;
            }
            if (*format == 'x')
            {
                pc += printi(out, va_arg(args, int), 16, 0, width, pad, 'a');
                continue;
            }
            if (*format == 'X')
            {
                pc += printi(out, va_arg(args, int), 16, 0, width, pad, 'A');
                continue;
            }
            if (*format == 'u')
            {
                pc += printi(out, va_arg(args, int), 10, 0, width, pad, 'a');
                continue;
            }
            if (*format == 'c')
            {
                /* char are converted to int then pushed on the stack */
                scr[0] = (char)va_arg(args, int);
                scr[1] = '\0';
                pc += prints(out, scr, width, pad);
                continue;
            }
        }
        else
        {
        out:
            printchar(out, *format);
            ++pc;
        }
    }
    if (out)
        **out = '\0';
    va_end(args);
    return pc;
}

int printf(const char *format, ...)
{
    va_list args;

    va_start(args, format);
    return print(0, format, args);
}

void DebugPrintf(DebugChannel ch, const char *format, ...)
{

    if (ch == NULL)
        return;
    DebugNode *nch = (DebugNode *)ch;
    if (dbg_man.enable_info != true || nch->enable_info != true)
        return;
    if (xSemaphoreTake(dbg_man.lock, 10000) != pdFALSE)
    {
        DebugSendKey(DEBUG_MESSAGE_INFO, nch->key);
        va_list args;

        va_start(args, format);
        print(0, format, args);
        xSemaphoreGive(dbg_man.lock);
    }
}

void DebugWarnPrintf(DebugChannel ch, const char *format, ...)
{
    if (ch == NULL)
        return;
    DebugNode *nch = (DebugNode *)ch;
    if (dbg_man.enable_warning != true || nch->enable_warning != true)
        return;
    if (xSemaphoreTake(dbg_man.lock, 10000) != pdFALSE)
    {
        DebugSendKey(DEBUG_MESSAGE_WARN, nch->key);
        va_list args;

        va_start(args, format);
        print(0, format, args);
        xSemaphoreGive(dbg_man.lock);
    }
}

void DebugErrorPrintf(DebugChannel ch, const char *format, ...)
{
    if (ch == NULL)
        return;
    DebugNode *nch = (DebugNode *)ch;
    if (dbg_man.enable_error_log != true || nch->enable_error_log != true)
        return;
    if (xSemaphoreTake(dbg_man.lock, 10000) != pdFALSE)
    {
        DebugSendKey(DEBUG_MESSAGE_ERROR, nch->key);
        va_list args;

        va_start(args, format);
        print(0, format, args);
        xSemaphoreGive(dbg_man.lock);
    }
}