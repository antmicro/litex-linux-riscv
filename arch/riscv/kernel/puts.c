#include <linux/init.h>
#include <linux/mm.h>
#include <linux/memblock.h>
#include <linux/sched.h>
#include <linux/initrd.h>
#include <linux/console.h>
#include <linux/screen_info.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/sched/task.h>
#include <linux/swiotlb.h>

#include <asm/setup.h>
#include <asm/sections.h>
#include <asm/pgtable.h>
#include <asm/smp.h>
#include <asm/sbi.h>
#include <asm/tlbflush.h>
#include <asm/thread_info.h>
#include <utils/puts.h>

static int first = 1;

void special_printf(uint32_t type, char *fl, unsigned int n, char *fn, char const *fmt, ...) {
    char s[255];
    char s2[512];
    va_list arg;
    va_start(arg, fmt);
    vsprintf(s, fmt, arg);
    va_end(arg);
    if (first) {
       first = 0;
       liteuart_puts("\n\n\n\n|\n| >>> ***\n|\n");
    }
    switch (type) {
            case 0:
   	        sprintf(s2, "\e[1;49;31m|\e[0m \e[1;49;37m>>>\e[0m \e[1;49;31mINFO   \e[0m \e[1;49;37m>>>\e[0m (\e[0;49;36m%s:%u:%s\e[0m) %s\n",fl,n,fn,s);
		break;
	    case 1:
	        sprintf(s2, "\e[0;49;34m|\e[0m \e[1;49;37m>>>\e[0m \e[0;49;34mWARNING \e[0m \e[1;49;37m>>>\e[0m (\e[0;49;36m%s:%u:%s\e[0m) %s\n",fl,n,fn,s);
                break;
	    case 2:
	        sprintf(s2, "\e[1;49;31m|\e[0m \e[1;49;37m>>>\e[0m \e[1;49;31mERROR   \e[0m \e[1;49;37m>>>\e[0m (\e[0;49;36m%s:%u:%s\e[0m) %s\n",fl,n,fn,s);
                break;
	    case 3:
	    default:
	        sprintf(s2, "| \e[1;49;37m>>>\e[0m DEBUG    \e[1;49;37m>>>\e[0m (\e[0;49;36m%s:%u:%s\e[0m)  %s\n",fl,n,fn,s);
                break;
    }
    liteuart_puts(s2);
}

void dbgprintf(char const *fmt, ...) {
    char s[255];
    va_list arg;
    va_start(arg, fmt);
    vsprintf(s, fmt, arg);
    va_end(arg);
    liteuart_puts(s);
}
