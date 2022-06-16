

#ifndef _DEBUG_INCLUDE_H_
#define _DEBUG_INCLUDE_H_

#include <stdint.h>


typedef void * DebugChannel;

typedef void (*DebugHandler)(char *reply, const char **param_list, uint16_t count);

void DebugInit();
DebugChannel DebugRegister(const char *key, DebugHandler handler);

void DebugError(DebugChannel ch, const char *error);
void DebugInfo(DebugChannel ch, const char *info);
void DebugWarning(DebugChannel ch, const char *warning);
void DebugPrintf(DebugChannel ch, const char *format, ...);
void DebugErrorPrintf(DebugChannel ch, const char *format, ...);
void DebugWarnPrintf(DebugChannel ch, const char *format, ...);
#endif // _DEBUG_INCLUDE_H_