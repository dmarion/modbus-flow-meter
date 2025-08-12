#include <stdarg.h>
#include <stdint.h>
#include "main.h"

static inline void out(char c) { usart2_write(c); }

static void out_str(const char *s) {
  while (*s)
    out(*s++);
}

static void out_hex32(uint32_t v) {
  static const char hex[] = "0123456789abcdef";
  int started = 0;
  for (int shift = 28; shift >= 0; shift -= 4) {
    uint8_t nib = (v >> shift) & 0xF;
    if (nib || started || shift == 0) {
      out(hex[nib]);
      started = 1;
    }
  }
}

static void out_u32(uint32_t v) {
  static const uint32_t pow10[] = {1000000000u,
                                   100000000u,
                                   10000000u,
                                   1000000u,
                                   100000u,
                                   10000u,
                                   1000u,
                                   100u,
                                   10u,
                                   1u};
  int started = 0;
  for (int i = 0; i < 10; ++i) {
    uint32_t d = 0, base = pow10[i];
    while (v >= base) {
      v -= base;
      d++;
    } // no division
    if (d || started || i == 9) {
      out((char)('0' + d));
      started = 1;
    }
  }
}

void printf(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  while (*fmt) {
    if (*fmt == '%') {
      ++fmt;
      switch (*fmt) {
      case 'u':
        out_u32(va_arg(ap, uint32_t));
        break;
      case 'x':
        out_hex32(va_arg(ap, uint32_t));
        break;
      case 'c':
        out((char)va_arg(ap, int));
        break;
      case 's': {
        const char *s = va_arg(ap, const char *);
        out_str(s ? s : "(null)");
      } break;
      case '%':
        out('%');
        break;
      default:
        out('%');
        out(*fmt);
        break;
      }
    } else {
      out(*fmt);
    }
    if (*fmt)
      ++fmt;
  }
  va_end(ap);
}
