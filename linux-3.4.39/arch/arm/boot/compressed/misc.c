/*
 * misc.c
 * 
 * This is a collection of several routines from gzip-1.0.3 
 * adapted for Linux.
 *
 * malloc by Hannu Savolainen 1993 and Matthias Urlichs 1994
 *
 * Modified for ARM Linux by Russell King
 *
 * Nicolas Pitre <nico@visuaide.com>  1999/04/14 :
 *  For this code to run directly from Flash, all constant variables must
 *  be marked with 'const' and all other variables initialized at run-time 
 *  only.  This way all non constant variables will end up in the bss segment,
 *  which should point to addresses in RAM and cleared to 0 on start.
 *  This allows for a much quicker boot time.
 */

unsigned int __machine_arch_type;

#include <linux/compiler.h>	/* for inline */
#include <linux/types.h>
#include <linux/linkage.h>

static void putstr(const char *ptr);
extern void error(char *x);

#include <mach/uncompress.h>

#ifdef CONFIG_DEBUG_ICEDCC

#if defined(CONFIG_CPU_V6) || defined(CONFIG_CPU_V6K) || defined(CONFIG_CPU_V7)

static void icedcc_putc(int ch)
{
	int status, i = 0x4000000;

	do {
		if (--i < 0)
			return;

		asm volatile ("mrc p14, 0, %0, c0, c1, 0" : "=r" (status));
	} while (status & (1 << 29));

	asm("mcr p14, 0, %0, c0, c5, 0" : : "r" (ch));
}


#elif defined(CONFIG_CPU_XSCALE)

static void icedcc_putc(int ch)
{
	int status, i = 0x4000000;

	do {
		if (--i < 0)
			return;

		asm volatile ("mrc p14, 0, %0, c14, c0, 0" : "=r" (status));
	} while (status & (1 << 28));

	asm("mcr p14, 0, %0, c8, c0, 0" : : "r" (ch));
}

#else

static void icedcc_putc(int ch)
{
	int status, i = 0x4000000;

	do {
		if (--i < 0)
			return;

		asm volatile ("mrc p14, 0, %0, c0, c0, 0" : "=r" (status));
	} while (status & 2);

	asm("mcr p14, 0, %0, c1, c0, 0" : : "r" (ch));
}

#endif

#define putc(ch)	icedcc_putc(ch)
#endif
/*******************************************************/
/* baudrate rest value */
union br_rest {
        unsigned short  slot;           /* udivslot */
        unsigned char   value;          /* ufracval */
};

struct s5p_uart {
        unsigned int    ulcon;
        unsigned int    ucon;
        unsigned int    ufcon;
        unsigned int    umcon;
        unsigned int    utrstat;
        unsigned int    uerstat;
        unsigned int    ufstat;
        unsigned int    umstat;
        unsigned char   utxh;
        unsigned char   res1[3];
        unsigned char   urxh;
        unsigned char   res2[3];
        unsigned int    ubrdiv;
        union br_rest   rest;
        unsigned char   res3[0xffd0];
};
#define RX_FIFO_COUNT_MASK      0xff
#define RX_FIFO_FULL_MASK       (1 << 8)
#define TX_FIFO_FULL_MASK       (1 << 24)
#define S5P6818_UART0		(0xC00A1000)
#define writeb(c,addr) *((unsigned char volatile *)(addr)) = (c)
#define readl(addr) *((unsigned char volatile *)(addr))
static int serial_err_check(const int dev_index, int op)
{
        struct s5p_uart *const uart = (unsigned long volatile *)S5P6818_UART0;
        unsigned int mask;

        /*
         * UERSTAT
         * Break Detect [3]
         * Frame Err    [2] : receive operation
         * Parity Err   [1] : receive operation
         * Overrun Err  [0] : receive operation
         */
        if (op)
                mask = 0x8;
        else
                mask = 0xf;

        return readl(&uart->uerstat) & mask;
}
/*
 * Output a single byte to the serial port.
 */
static void serial_putc_dev(const char c, const int dev_index)
{
        struct s5p_uart *const uart = (unsigned long volatile *)S5P6818_UART0;


        /* wait for room in the tx FIFO */
        while ((readl(&uart->ufstat) & TX_FIFO_FULL_MASK)) {
                if (serial_err_check(dev_index, 1))
                        return;
        }

        writeb(c, &uart->utxh);

        /* If \n, also do \r */
        if (c == '\n')
        	writeb('\r', &uart->utxh);
}


#define putc(c) serial_putc_dev(c, 0)


/*******************************************************/
static void putstr(const char *ptr)
{
	char c;

	while ((c = *ptr++) != '\0') {
		if (c == '\n')
			putc('\r');
		putc(c);
	}

	flush();
}

/*
 * gzip declarations
 */
extern char input_data[];
extern char input_data_end[];

unsigned char *output_data;

unsigned long free_mem_ptr;
unsigned long free_mem_end_ptr;

#ifndef arch_error
#define arch_error(x)
#endif

void error(char *x)
{
	arch_error(x);

	putstr("\n\n");
	putstr(x);
	putstr("\n\n -- System halted");

	while(1);	/* Halt */
}

asmlinkage void __div0(void)
{
	error("Attempting division by 0!");
}

extern int do_decompress(u8 *input, int len, u8 *output, void (*error)(char *x));


void
decompress_kernel(unsigned long output_start, unsigned long free_mem_ptr_p,
		unsigned long free_mem_ptr_end_p,
		int arch_id)
{
	int ret;

	output_data		= (unsigned char *)output_start;
	free_mem_ptr		= free_mem_ptr_p;
	free_mem_end_ptr	= free_mem_ptr_end_p;
	__machine_arch_type	= arch_id;

	arch_decomp_setup();

	putstr("Uncompressing Linux...");
	ret = do_decompress(input_data, input_data_end - input_data,
			    output_data, error);
	if (ret)
		error("decompressor returned an error");
	else
		putstr(" done, booting the kernel.\n");
}
