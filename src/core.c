#include "bnd-core.h"
#include <stdio.h>
#include <stdarg.h>

#define MAX_MESSAGE_SIZE 512

char error_message_buffer[MAX_MESSAGE_SIZE];
bnd_error_callback error_callback = NULL;

void raise_error(bnd_error type, void *data, const char *template, ...) {
  if (error_callback == NULL) {
    return;
  }

  va_list list;
  va_start(list, template);
  vsnprintf(error_message_buffer, MAX_MESSAGE_SIZE, template, list);
  va_end(list);

  error_callback(type, error_message_buffer, data);
}

void bnd_register_error_callback(bnd_error_callback callback) { error_callback = callback; }
