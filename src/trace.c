#include "trace.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#define TRACE_BUFFER_SIZE 16384

char buffer[TRACE_BUFFER_SIZE];
uint32_t pointer;

void trace_log(const char *format, ...) {
  va_list args;
  va_start(args, format);

  pointer += vsprintf(buffer + pointer, format, args);

  va_end(args);
}

void trace_print() { write(1, buffer, pointer); }

void trace_clear() { pointer = 0; }
