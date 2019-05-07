#ifndef _PUTS
#define _PUTS

#define LITEUART_BASE 0xE0001800
#define RXTX 0
#define TXFULL 1
#define EVENTSTATUS 2
#define EVENTPENDING 3
#define EVENTENABLE 4

#if 0
#define MSG(n, ...) special_printf(n,__FILE__, __LINE__, (char*)__func__, __VA_ARGS__)
#define DBG() MSG(3, "")
#define DBGMSG(...) MSG(3, __VA_ARGS__)
#else
#define MSG(n, ...)
#define DBG()
#define DBGMSG(...)
#endif

static inline void liteuart_putc(char c) {
        volatile uint32_t *uart = (uint32_t*)LITEUART_BASE;
	int timeout = 60000;
        while (uart[TXFULL] && timeout--);
        uart[RXTX] = c;
}

static inline void liteuart_puts(char *s) {
       char *ptr = s;
       int max_chars = 512;
       while ((ptr[0] != 0) && (max_chars-- > 0)) {
           if (ptr[0] == '\n')
              liteuart_putc('\r');
           liteuart_putc(ptr++[0]);
       }
}

void dbgprintf(char const *, ...);
void special_printf(uint32_t type, char *fl, unsigned int n, char *fn, char const *fmt, ...);

#endif
