/*
 * dccwaved.c DCC Waveform Driver for the RaspberryPi
 * Copyright (c) 2020 Ian Hartwig
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <getopt.h>
#include <math.h>
#include <bcm_host.h>

#include "mailbox.h"

#define DMY 255 // Used to represent an invalid P1 pin, or unmapped servo

#define NUM_P1PINS  40
#define NUM_P5PINS  8

#define MAX_SERVOS  32  /* Only 21 really, but this lets you map servo IDs
         * to P1 pins, if you want to
         */
#define MAX_MEMORY_USAGE  (16*1024*1024)  /* Somewhat arbitrary limit of 16MB */

#define DEFAULT_CYCLE_TIME_US 20000
#define DEFAULT_STEP_TIME_US  10
#define DEFAULT_SERVO_MIN_US  500
#define DEFAULT_SERVO_MAX_US  2500

#define DEVFILE     "/dev/dccwave"
#define CFGFILE     "/dev/dccwave-cfg"

#define PAGE_SIZE   4096
#define PAGE_SHIFT    12
#define BITS_IN_BYTE  8

#define DMA_CHAN_SIZE   0x100
#define DMA_CHAN_MIN    0
#define DMA_CHAN_MAX    6  // don't support dma ch 15 or lite (7-14)
#define DMA_CHAN_DEFAULT  5
#define DMA_CHAN_PI4    7

#define DMA_BASE_OFFSET   0x00007000
#define DMA_LEN     DMA_CHAN_SIZE * (DMA_CHAN_MAX+1)
#define PWM_BASE_OFFSET   0x0020C000
#define PWM_LEN     0x28
#define CLK_BASE_OFFSET         0x00101000
#define CLK_LEN     0xA8
#define GPIO_BASE_OFFSET  0x00200000
#define GPIO_LEN    0x100
#define PCM_BASE_OFFSET   0x00203000
#define PCM_LEN     0x24

#define DMA_VIRT_BASE   (periph_virt_base + DMA_BASE_OFFSET)
#define PWM_VIRT_BASE   (periph_virt_base + PWM_BASE_OFFSET)
#define CLK_VIRT_BASE   (periph_virt_base + CLK_BASE_OFFSET)
#define GPIO_VIRT_BASE    (periph_virt_base + GPIO_BASE_OFFSET)
#define PCM_VIRT_BASE   (periph_virt_base + PCM_BASE_OFFSET)

#define PWM_PHYS_BASE   (periph_phys_base + PWM_BASE_OFFSET)
#define PCM_PHYS_BASE   (periph_phys_base + PCM_BASE_OFFSET)
#define GPIO_PHYS_BASE    (periph_phys_base + GPIO_BASE_OFFSET)

#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

// DMA Control Block Transfer Information
#define DMA_NO_WIDE_BURSTS  (1<<26)
#define DMA_WAIT_RESP   (1<<3)
#define DMA_D_DREQ    (1<<6)
#define DMA_PER_MAP(x)    ((x)<<16)
#define DMA_TDMODE  (1<<1)
#define DMA_TD_LEN(x,y)   (((y)<<16)|(x))
#define DMA_STRIDE(s,d)   (((d)<<16)|(s))

// DMA Control and Status (CS) Reg
#define DMA_RESET   (1<<31)
#define DMA_ERROR   (1<<8)
#define DMA_INT     (1<<2)
#define DMA_END     (1<<1)
#define DMA_ACTIVE  (1<<0)

// DMA Debug Reg
#define DMA_LITE    (1 << 28)

// DMA Regs
#define DMA_CS      (0x00/4)
#define DMA_CONBLK_AD   (0x04/4)
#define DMA_SOURCE_AD   (0x0c/4)
#define DMA_NEXTCONBK   (0x1c/4)
#define DMA_DEBUG   (0x20/4)
#define DMA_ENABLE  (0xff0/4)

#define GPIO_FSEL0    (0x00/4)
#define GPIO_SET0   (0x1c/4)
#define GPIO_CLR0   (0x28/4)
#define GPIO_LEV0   (0x34/4)
#define GPIO_PULLEN   (0x94/4)
#define GPIO_PULLCLK    (0x98/4)

#define GPIO_MODE_IN    0
#define GPIO_MODE_OUT   1
#define GPIO_MODE_ALT0  4
#define GPIO_MODE_ALT1  5
#define GPIO_MODE_ALT2  6
#define GPIO_MODE_ALT3  7
#define GPIO_MODE_ALT4  3
#define GPIO_MODE_ALT5  2

#define PWM_CTL     (0x00/4)
#define PWM_STA     (0x04/4)
#define PWM_DMAC    (0x08/4)
#define PWM_RNG1    (0x10/4)
#define PWM_FIFO    (0x18/4)
#define PWM_FIFO_WIDTH  32

#define PWMCLK_CNTL   40
#define PWMCLK_DIV    41

#define PWMCTL_MODE1    (1<<1)
#define PWMCTL_PWEN1    (1<<0)
#define PWMCTL_CLRF   (1<<6)
#define PWMCTL_USEF1    (1<<5)

#define PWMDMAC_ENAB    (1<<31)
#define PWMDMAC_PANIC(x)  ((x)<<8)
#define PWMDMAC_DREQ(x)   ((x)<<0)

#define PCM_CS_A    (0x00/4)
#define PCM_FIFO_A    (0x04/4)
#define PCM_MODE_A    (0x08/4)
#define PCM_RXC_A   (0x0c/4)
#define PCM_TXC_A   (0x10/4)
#define PCM_DREQ_A    (0x14/4)
#define PCM_INTEN_A   (0x18/4)
#define PCM_INT_STC_A   (0x1c/4)
#define PCM_GRAY    (0x20/4)

#define PCMCLK_CNTL   38
#define PCMCLK_DIV    39

#define PLLDFREQ_MHZ_DEFAULT  500
#define PLLDFREQ_MHZ_PI4  750

#define DELAY_VIA_PWM   0
#define DELAY_VIA_PCM   1

#define ROUNDUP(val, blksz) (((val)+((blksz)-1)) & ~(blksz-1))

typedef struct {
  uint32_t info;
  uint32_t src;
  uint32_t dst;
  uint32_t length;
  uint32_t stride;
  uint32_t next;
  uint32_t pad[2];
} dma_cb_t;

// DCC Packet Structure
#define DCC_ADDR_IDLE        (0xFF)
#define DCC_ADDR_LONG        (0x3<<6)
#define DCC_ADDR_LONG_MAX    (0x3FFF)
#define DCC_ADDR_SHORT_MAX   (0x7F)  // 127
#define DCC_CMD_IDLE         (0x0<<5)
#define DCC_CMD_SPD_DIR_REV  (0x2<<5)
#define DCC_CMD_SPD_DIR_FWD  (0x3<<5)
#define DCC_CMD_SPD_EXT      ((0x1<<5)|0x1F)  // 128 step mode
#define DCC_CMD_FN_GP_1      (0x4<<5)
#define DCC_CMD_FN_GP_2      (0x5<<5)
#define DCC_CMD_FN_GP_3      ((0x5<<5)|(0x1<<4))  // call it gp 3 for f9-12
#define DCC_CMD_FN_MASK      (0xF<<4)
#define DCC_CMD_MASK         (0x7<<5)
#define DCC_VAL_IDLE         (0x0)
#define DCC_VAL_MODE_REG     0
#define DCC_VAL_MODE_EXT     1
#define DCC_VAL_SPD_MASK     (0x0F) // only use the last 4 bits for 14 step mode

// DCC Line Coding
// 1 = 58us high, 58us low = 0b10
// 0 = 100+us high, 100+us low = 0b1100
// MSB is first bit transmitted
// MSB is first bit serialized by RPi PWM 
//   (the 0x0F in 0x0F070301 is sent on the line first)
#define DCC_CODE_ONE         (0x2)
#define DCC_CODE_ONE_LEN     2
#define DCC_CODE_ZERO        (0xC)
#define DCC_CODE_ZERO_LEN    4  // line coder only handles when this is 2x one
// longest packet we support has 210 bits encoded (no 3 byte inst)
// 11_1111_1111 0 0000_0000 0 0000_0000 0 0000_0000 0 00000_0000 0 00000_0000 1
// preamble       addr1       addr2       inst1       inst2        chk_sum
#define DCC_CODE_UI32_LEN     7  // 224 bits
#define DCC_PCK_UI8_LEN       5  // uncoded packet length

// DCC Intermediate Encoding
// fixed-length definition for 
typedef struct {
  uint16_t addr;
  uint8_t cmd;
  uint8_t pad1;
  uint16_t val;
  uint8_t val_mode;
  uint8_t chk_sum;
  uint32_t line_code[DCC_CODE_UI32_LEN];
} dcc_pkt_t;

// DMAable DCC Entry
#define DCC_DMA_BUF_LEN 64  // assuming ~120msg/s this gives ~.5s latency max
typedef struct {
  dma_cb_t dma_cb;  // first as must be 32-byte aligned
  dcc_pkt_t dcc_pkt;
  uint32_t next_id;  // for id-based linked list
} dcc_dma_entry_t;


/* Define which P1 header pins to use by default.  These are the eight standard
 * GPIO pins (those coloured green in the diagram on this page:
 *    http://elinux.org/Rpi_Low-level_peripherals
 *
 * Which P1 header pins are actually used can be overridden via command line
 * parameter '--p1pins=...'.
 */

static char *default_p1_pins = "7,11,12,13,15,16,18,22";
static char *default_p5_pins = "";

static uint8_t rev1_p1pin2gpio_map[] = {
  DMY,  // P1-1   3v3
  DMY,  // P1-2   5v
  0,  // P1-3   GPIO 0 (SDA)
  DMY,  // P1-4   5v
  1,  // P1-5   GPIO 1 (SCL)
  DMY,  // P1-6   Ground
  4,  // P1-7   GPIO 4 (GPCLK0)
  14, // P1-8   GPIO 14 (TXD)
  DMY,  // P1-9   Ground
  15, // P1-10  GPIO 15 (RXD)
  17, // P1-11  GPIO 17
  18, // P1-12  GPIO 18 (PCM_CLK)
  21, // P1-13  GPIO 21
  DMY,  // P1-14  Ground
  22, // P1-15  GPIO 22
  23, // P1-16  GPIO 23
  DMY,  // P1-17  3v3
  24, // P1-18  GPIO 24
  10, // P1-19  GPIO 10 (MOSI)
  DMY,  // P1-20  Ground
  9,  // P1-21  GPIO 9 (MISO)
  25, // P1-22  GPIO 25
  11, // P1-23  GPIO 11 (SCLK)
  8,  // P1-24  GPIO 8 (CE0)
  DMY,  // P1-25  Ground
  7,  // P1-26  GPIO 7 (CE1)
};

static uint8_t rev1_p5pin2gpio_map[] = {
  DMY,  // (P5-1 on rev 2 boards)
  DMY,  // (P5-2 on rev 2 boards)
  DMY,  // (P5-3 on rev 2 boards)
  DMY,  // (P5-4 on rev 2 boards)
  DMY,  // (P5-5 on rev 2 boards)
  DMY,  // (P5-6 on rev 2 boards)
  DMY,  // (P5-7 on rev 2 boards)
  DMY,  // (P5-8 on rev 2 boards)
};

static uint8_t rev2_p1pin2gpio_map[] = {
  DMY,  // P1-1   3v3
  DMY,  // P1-2   5v
  2,  // P1-3   GPIO 2 (SDA)
  DMY,  // P1-4   5v
  3,  // P1-5   GPIO 3 (SCL)
  DMY,  // P1-6   Ground
  4,  // P1-7   GPIO 4 (GPCLK0)
  14, // P1-8   GPIO 14 (TXD)
  DMY,  // P1-9   Ground
  15, // P1-10  GPIO 15 (RXD)
  17, // P1-11  GPIO 17
  18, // P1-12  GPIO 18 (PCM_CLK)
  27, // P1-13  GPIO 27
  DMY,  // P1-14  Ground
  22, // P1-15  GPIO 22
  23, // P1-16  GPIO 23
  DMY,  // P1-17  3v3
  24, // P1-18  GPIO 24
  10, // P1-19  GPIO 10 (MOSI)
  DMY,  // P1-20  Ground
  9,  // P1-21  GPIO 9 (MISO)
  25, // P1-22  GPIO 25
  11, // P1-23  GPIO 11 (SCLK)
  8,  // P1-24  GPIO 8 (CE0)
  DMY,  // P1-25  Ground
  7,  // P1-26  GPIO 7 (CE1)
};

static uint8_t rev2_p5pin2gpio_map[] = {
  DMY,  // P5-1   5v0
  DMY,  // P5-2   3v3
  28, // P5-3   GPIO 28 (I2C0_SDA)
  29, // P5-4   GPIO 29 (I2C0_SCL)
  30, // P5-5   GPIO 30
  31, // P5-6   GPIO 31
  DMY,  // P5-7   Ground
  DMY,  // P5-8   Ground
};

static uint8_t bplus_p1pin2gpio_map[] = {
  DMY,  // P1-1   3v3
  DMY,  // P1-2   5v
  2,  // P1-3   GPIO 2 (SDA)
  DMY,  // P1-4   5v
  3,  // P1-5   GPIO 3 (SCL)
  DMY,  // P1-6   Ground
  4,  // P1-7   GPIO 4 (GPCLK0)
  14, // P1-8   GPIO 14 (TXD)
  DMY,  // P1-9   Ground
  15, // P1-10  GPIO 15 (RXD)
  17, // P1-11  GPIO 17
  18, // P1-12  GPIO 18 (PCM_CLK)
  27, // P1-13  GPIO 27
  DMY,  // P1-14  Ground
  22, // P1-15  GPIO 22
  23, // P1-16  GPIO 23
  DMY,  // P1-17  3v3
  24, // P1-18  GPIO 24
  10, // P1-19  GPIO 10 (MOSI)
  DMY,  // P1-20  Ground
  9,  // P1-21  GPIO 9 (MISO)
  25, // P1-22  GPIO 25
  11, // P1-23  GPIO 11 (SCLK)
  8,  // P1-24  GPIO 8 (CE0)
  DMY,  // P1-25  Ground
  7,  // P1-26  GPIO 7 (CE1)
  DMY,  // P1-27  ID_SD
  DMY,  // P1-28  ID_SC
  5,  // P1-29  GPIO 5
  DMY,  // P1-30  Ground
  6,  // P1-31  GPIO 5
  12, // P1-32  GPIO 12
  13, // P1-33  GPIO 13
  DMY,  // P1-34  Ground
  19, // P1-35  GPIO 19
  16, // P1-36  GPIO 16
  26, // P1-37  GPIO 26
  20, // P1-38  GPIO 20
  DMY,  // P1-39  Ground
  21, // P1-40  GPIO 21
};

// bcm_host_get_model_type() return values to name mapping
static const char *model_names[] = {
  "A", "B", "A+", "B+", "2B", "Alpha", "CM", "CM2", "3B", "Zero", "CM3",
  "Custom", "ZeroW", "3B+", "3A+", "FPGA", "CM3+", "4B"
};
#define NUM_MODELS  (sizeof(model_names)/sizeof(*model_names))

// cycle_time_us is the pulse cycle time per servo, in microseconds.
// Typically it should be 20ms, or 20000us.

// step_time_us is the pulse width increment granularity, again in microseconds.
// Setting step_time_us too low will likely cause problems as the DMA controller
// will use too much memory bandwidth.  10us is a good value, though you
// might be ok setting it as low as 2us.

static int cycle_time_us;
static int step_time_us;

static uint8_t servo2gpio[MAX_SERVOS];
static uint32_t gpiomode[MAX_SERVOS];
static int restore_gpio_modes;
static int is_debug;
static int delay_hw = DELAY_VIA_PWM;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *pcm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_base_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;

static uint32_t plldfreq_mhz;
static int dma_chan;
static int invert = 0;
static int num_samples;
static int num_cbs;
static int num_pages;

static int board_model;
static int gpio_cfg;

static uint32_t periph_phys_base;
static uint32_t periph_virt_base;
static uint32_t dram_phys_base;
static uint32_t mem_flag;

static char *gpio_desc[] = {
  "Unknown",
  "P1 (26 pins)",
  "P1 (26 pins), P5 (8 pins)",
  "P1 (40 pins)"
};

static struct {
  int handle;   /* From mbox_open() */
  uint32_t size;    /* Required size */
  unsigned mem_ref; /* From mem_alloc() */
  unsigned bus_addr;  /* From mem_lock() */
  uint8_t *virt_addr; /* From mapmem() */
} mbox;

static dcc_pkt_t dcc_pkt_tmp;
static uint32_t dcc_dma_next_id = 0;

  
static void gpio_set_mode(uint32_t gpio, uint32_t mode);
static void dcc_dma_make_cb(
  dcc_dma_entry_t *dcc_dma_buf, int this_id, int next_id);
static void dcc_make_line_code(dcc_pkt_t *pkt);
static void dcc_dma_update(dcc_dma_entry_t *dcc_dma_buf, dcc_pkt_t tmp_pkt);
static void dcc_dma_init(dcc_dma_entry_t *dcc_dma_buf);
static void dcc_dma_hw_config();


static void
udelay(int us)
{
  struct timespec ts = { 0, us * 1000 };

  nanosleep(&ts, NULL);
}

static void
terminate(int dummy)
{
  // int i;

  // clean up dma regs
  if (dma_reg && mbox.virt_addr) {
    dma_reg[DMA_CS] = DMA_RESET;
    udelay(10);
  }
  // clean up pwm regs
  if (pwm_reg) {
    pwm_reg[PWM_CTL] = 0;
    udelay(10);
  }
  // if (restore_gpio_modes) {
  //   for (i = 0; i < MAX_SERVOS; i++) {
  //     if (servo2gpio[i] != DMY)
  //       gpio_set_mode(servo2gpio[i], gpiomode[i]);
  //   }
  // }
  if (mbox.virt_addr != NULL) {
    unmapmem(mbox.virt_addr, mbox.size);
    mem_unlock(mbox.handle, mbox.mem_ref);
    mem_free(mbox.handle, mbox.mem_ref);
    if (mbox.handle >= 0)
      mbox_close(mbox.handle);
  }

  unlink(DEVFILE);
  unlink(CFGFILE);
  exit(1);
}

static void
fatal(char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  va_end(ap);
  terminate(0);
}

static uint32_t gpio_get_mode(uint32_t gpio)
{
  uint32_t fsel = gpio_reg[GPIO_FSEL0 + gpio/10];

  return (fsel >> ((gpio % 10) * 3)) & 7;
}

static void
gpio_set_mode(uint32_t gpio, uint32_t mode)
{
  uint32_t fsel = gpio_reg[GPIO_FSEL0 + gpio/10];

  fsel &= ~(7 << ((gpio % 10) * 3));
  fsel |= mode << ((gpio % 10) * 3);
  gpio_reg[GPIO_FSEL0 + gpio/10] = fsel;
}

static void
gpio_set(int gpio, int level)
{
  if (level)
    gpio_reg[GPIO_SET0] = 1 << gpio;
  else
    gpio_reg[GPIO_CLR0] = 1 << gpio;
}

static uint32_t
mem_virt_to_phys(void *virt)
{
  uint32_t offset = (uint8_t *)virt - mbox.virt_addr;

  return mbox.bus_addr + offset;
}

static void *
map_peripheral(uint32_t base, uint32_t len)
{
  int fd = open("/dev/mem", O_RDWR|O_SYNC);
  void * vaddr;

  if (fd < 0)
    fatal("dccwaved: Failed to open /dev/mem: %m\n");
  vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
  if (vaddr == MAP_FAILED)
    fatal("dccwaved: Failed to map peripheral at 0x%08x: %m\n", base);
  close(fd);

  return vaddr;
}

static void
setup_sighandlers(void)
{
  int i;

  // Catch all signals possible - it is vital we kill the DMA engine
  // on process exit!
  for (i = 0; i < 64; i++) {
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = terminate;
    sigaction(i, &sa, NULL);
  }
}

static void
dcc_dma_make_cb(dcc_dma_entry_t *dcc_dma_buf, int this_id, int next_id)
{
  dma_cb_t *cb = &(dcc_dma_buf[this_id].dma_cb);
  uint32_t *line_code = &(dcc_dma_buf[this_id].dcc_pkt.line_code[0]);
  dma_cb_t *next_cb = &(dcc_dma_buf[next_id].dma_cb);
  uint32_t phys_fifo_addr;
  phys_fifo_addr = PWM_PHYS_BASE + (PWM_FIFO*sizeof(uint32_t));
  cb->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(5) | DMA_TDMODE;
  cb->src = mem_virt_to_phys(line_code);
  cb->dst = phys_fifo_addr;
  // copy whole dcc packet to fifo at once
  cb->length = DMA_TD_LEN(sizeof(uint32_t), DCC_CODE_UI32_LEN);
  // inc source addr (buffer) but not dest (fifo)
  cb->stride = DMA_STRIDE(sizeof(uint32_t), 0);
  cb->next = mem_virt_to_phys(next_cb);
}

static inline int
dcc_cmd_is_spd(uint8_t cmd) {
  return (cmd == DCC_CMD_SPD_DIR_REV || cmd == DCC_CMD_SPD_DIR_FWD);
}

static inline int
dcc_cmd_is_func(uint8_t cmd) {
  return ((cmd & DCC_CMD_MASK) == DCC_CMD_FN_GP_1 ||
          (cmd & DCC_CMD_MASK) == DCC_CMD_FN_GP_2);
}

static void
dcc_make_line_code(dcc_pkt_t *pkt)
{
  // calculate contents
  int pkt_code_len = 0;
  uint8_t pkt_code[DCC_PCK_UI8_LEN];
  // short addr
  if (pkt->addr < DCC_ADDR_SHORT_MAX || pkt->addr == DCC_ADDR_IDLE) {
    pkt_code[pkt_code_len] = pkt->addr;
    pkt_code_len++;
  }
  // long addr (up to 14 bit) with msb in the first byte bit 5
  else {
    pkt_code[pkt_code_len] = DCC_ADDR_LONG |
      ((pkt->addr & DCC_ADDR_LONG_MAX) >> sizeof(uint8_t));
    pkt_code_len++;
    pkt_code[pkt_code_len] = (uint8_t) pkt->addr;
    pkt_code_len++;
  }
  // short speed commands
  if (dcc_cmd_is_spd(pkt->cmd) && pkt->val_mode == DCC_VAL_MODE_REG) {
    pkt_code[pkt_code_len] = pkt->cmd | (pkt->val & DCC_VAL_SPD_MASK);
    pkt_code_len++;
  }
  // long speed commands
  else if (dcc_cmd_is_spd(pkt->cmd)) {
    pkt_code[pkt_code_len] = DCC_CMD_SPD_EXT;  // TODO encode direction
    pkt_code_len++;
    pkt_code[pkt_code_len] = pkt->val;
    pkt_code_len++;
  }
  else if (dcc_cmd_is_func(pkt->cmd)) {
    pkt_code[pkt_code_len] = pkt->cmd;  // TODO
    pkt_code_len++;
  }
  else {
    pkt_code[pkt_code_len] = DCC_CMD_IDLE;
    pkt_code_len++;
  }
  // xor check sum
  for (int i=0; i < pkt_code_len; i++) {
    pkt_code[pkt_code_len] ^= pkt_code[i];
  }
  pkt_code_len++;
  for (int i=0; i < pkt_code_len; i++) {
    printf("pkt_code[%d]: 0x%x\n", i, pkt_code[i]);
  }
  // write out the line code
  memset(&(pkt->line_code), 0, sizeof(pkt->line_code));
  int line_code_idx, line_code_shift, pkt_code_idx, pkt_code_shift;
  int bits_per_word = sizeof(uint32_t) * BITS_IN_BYTE;
  int pkt_bits_rem = sizeof(pkt_code) * BITS_IN_BYTE;
  int line_bits_rem = sizeof(pkt->line_code) * BITS_IN_BYTE;
  for (; pkt_bits_rem > 0; pkt_bits_rem--) {
    // start from last bit to send since line code is variable length
    // use integer div with offset as floor
    line_code_idx = (line_bits_rem - 1) / bits_per_word;
    line_code_shift = ((line_code_idx + 1) * bits_per_word) - line_bits_rem;
    pkt_code_idx = (pkt_bits_rem - 1) / BITS_IN_BYTE;
    pkt_code_shift = ((pkt_code_idx + 1) * BITS_IN_BYTE) - pkt_bits_rem;
    printf("pkt_bits_rem: %d\n", pkt_bits_rem);
    printf("line_code_idx: %d\n", line_code_idx);
    printf("line_code_shift: %d\n", line_code_shift);
    if(pkt_code[pkt_code_idx] >> pkt_code_shift == 1) {
      pkt->line_code[line_code_idx] |= (DCC_CODE_ONE << line_code_shift);
      line_bits_rem -= DCC_CODE_ONE_LEN;
    }
    else {
      pkt->line_code[line_code_idx] |= (DCC_CODE_ZERO << line_code_shift);
      line_bits_rem -= DCC_CODE_ZERO_LEN;
      // also handle carry over of longer zeros
      int carry_shift = bits_per_word - line_code_shift;
      if (carry_shift < DCC_CODE_ZERO_LEN) {
        pkt->line_code[line_code_idx-1] |= (DCC_CODE_ZERO >> carry_shift);
      }
    }
  }
  for (; line_bits_rem > 0;) {
    // and fill the rest of the buffer with 1s, which includes the preamble
    line_code_idx = (line_bits_rem - 1) / bits_per_word;
    line_code_shift = ((line_code_idx + 1) * bits_per_word) - line_bits_rem;
    pkt->line_code[line_code_idx] |= (DCC_CODE_ONE << line_code_shift);
    line_bits_rem -= DCC_CODE_ONE_LEN;
  }
}

static inline int
dcc_cmd_func_group(uint8_t cmd) {
  if ((cmd & DCC_CMD_MASK) == DCC_CMD_FN_GP_1) {
    return 1;
  }
  else if ((cmd & DCC_CMD_MASK) == DCC_CMD_FN_GP_2) {
    if ((cmd & DCC_CMD_FN_MASK) == DCC_CMD_FN_GP_3) {
      return 3;
    }
    else {
      return 2;
    }
  }
  else {
    return 0;
  }
}

static void
dcc_dma_update(dcc_dma_entry_t *dcc_dma_buf, dcc_pkt_t tmp_pkt) {
  int prev_id = 0;
  int this_id = 0;
  // look for matching addr and command
  // also keep track of previous entry
  for (; this_id < dcc_dma_next_id; this_id++) {
    if (dcc_dma_buf[this_id].dcc_pkt.addr == tmp_pkt.addr) {
      uint8_t this_cmd = dcc_dma_buf[this_id].dcc_pkt.cmd;
      uint8_t tmp_cmd = tmp_pkt.cmd;
      // matching command details
      if ((dcc_cmd_is_spd(tmp_cmd) && dcc_cmd_is_spd(this_cmd)) ||
          dcc_cmd_func_group(tmp_cmd) == dcc_cmd_func_group(this_cmd)) {
        break;
      }
    }
    prev_id = this_id;
  }

  // insert new when we can't find a match
  if (this_id == dcc_dma_next_id) {
    // make sure there's room
    if (dcc_dma_next_id == DCC_DMA_BUF_LEN) {
      if (is_debug) {
        printf("DCC DMA command buffer out of room!");
      }
      return;
    }
    if (is_debug) {
      printf("DCC DMA creating %d\n", this_id);
    }
    // prepare entry
    dcc_dma_buf[this_id].dcc_pkt = tmp_pkt;
    dcc_make_line_code(&(dcc_dma_buf[this_id].dcc_pkt));
    dcc_dma_buf[this_id].next_id = 0;  // always add to end wrapping around
    dcc_dma_buf[prev_id].next_id = this_id;
    dcc_dma_make_cb(dcc_dma_buf, this_id, 0);
    dcc_dma_make_cb(dcc_dma_buf, prev_id, this_id);
    dcc_dma_next_id++;
  }

  // remove, update, reinsert from dma if matched
  else {
    if (is_debug) {
      printf("DCC DMA updating %d\n", this_id);
    }
    uint32_t next_id = dcc_dma_buf[this_id].next_id;
    dcc_dma_make_cb(dcc_dma_buf, prev_id, next_id);
    // now safe, update entry
    dcc_dma_buf[this_id].dcc_pkt = tmp_pkt;
    dcc_make_line_code(&(dcc_dma_buf[this_id].dcc_pkt));
    // reinsert
    dcc_dma_make_cb(dcc_dma_buf, prev_id, this_id);
  }
}

static void
dcc_dma_init(dcc_dma_entry_t *dcc_dma_buf)
{
  // zero dma chain
  memset(dcc_dma_buf, 0, sizeof(dcc_dma_entry_t)*DCC_DMA_BUF_LEN);
  // make idle to start off the dma chain
  dcc_pkt_t tmp_pkt;
  tmp_pkt.addr = DCC_ADDR_IDLE;
  tmp_pkt.cmd = DCC_CMD_IDLE;
  tmp_pkt.val = DCC_VAL_IDLE;
  dcc_dma_update(dcc_dma_buf, tmp_pkt);
}

static void
dcc_dma_hw_config(dcc_dma_entry_t *dcc_dma_buf)
{
  // Initialise PWM
  if (pwm_reg[PWM_CTL] & PWMCTL_PWEN1) {
    fatal("dccwaved: pwm channel 1 already enabled!");
  }
  pwm_reg[PWM_CTL] = 0;
  udelay(10);
  clk_reg[PWMCLK_CNTL] = 0x5A000006;    // Source=PLLD (500MHz or 750MHz on Pi4)
  udelay(100);
  clk_reg[PWMCLK_DIV] = 0x5A000000 | (plldfreq_mhz<<12);  // set pwm div to give 1MHz
  udelay(100);
  clk_reg[PWMCLK_CNTL] = 0x5A000016;    // Source=PLLD and enable
  udelay(100);
  pwm_reg[PWM_RNG1] = PWM_FIFO_WIDTH;   // Range=FIFO width for serializer mode
  udelay(10);
  pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_PANIC(2) | PWMDMAC_DREQ(4);
  udelay(10);
  pwm_reg[PWM_CTL] = PWMCTL_CLRF; // clear fifo
  udelay(10);
  pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1 | PWMCTL_MODE1;    // ch 1 fifo, enable, serializer mode
  udelay(10);

  // Initialise the DMA
  if (dma_reg[DMA_CS] & DMA_ACTIVE) {
    fatal("dccwaved: dma channel %d already active!", dma_chan);
  }
  if (dma_reg[DMA_DEBUG] & DMA_LITE) {
    fatal("dccwaved: cannot run on dma lite channels!");
  }
  dma_reg[DMA_CS] = DMA_RESET;
  udelay(10);
  dma_reg[DMA_CS] = DMA_INT | DMA_END;
  dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(&(dcc_dma_buf->dma_cb));
  dma_reg[DMA_DEBUG] = 7; // clear debug error flags
  dma_reg[DMA_CS] = 0x10880001; // go, mid priority, wait for outstanding writes
}

// static void
// do_status(char *filename)
// {
//   uint32_t last;
//   int status = -1;
//   char *p;
//   int fd;
//   const char *dma_dead = "ERROR: DMA not running\n";

//   while (*filename == ' ')
//     filename++;
//   p = filename + strlen(filename) - 1;
//   while (p > filename && (*p == '\n' || *p == '\r' || *p == ' '))
//     *p-- = '\0';

//   last = dma_reg[DMA_CONBLK_AD];
//   udelay(step_time_us*2);
//   if (dma_reg[DMA_CONBLK_AD] != last)
//     status = 0;
//   if ((fd = open(filename, O_WRONLY|O_CREAT, 0666)) >= 0) {
//     if (status == 0)
//       write(fd, "OK\n", 3);
//     else
//       write(fd, dma_dead, strlen(dma_dead));
//     close(fd);
//   } else {
//     printf("Failed to open %s for writing: %m\n", filename);
//   }
// }

static void
do_debug(void)
{
  printf("\nDebug info:\n\n");
  // pwm regs
  printf("PWM_CTL: 0x%x\n", pwm_reg[PWM_CTL]);
  uint32_t pwm_sta = pwm_reg[PWM_STA];
  printf("PWM_STA: 0x%x\n", pwm_sta);
  printf("  STA2:  0x%x\n", !!(pwm_sta & (1<<10)));
  printf("  STA1:  0x%x\n", !!(pwm_sta & (1<<9)));
  printf("  GAPO2: 0x%x\n", !!(pwm_sta & (1<<5)));
  printf("  GAPO1: 0x%x\n", !!(pwm_sta & (1<<4)));
  printf("  RERR1: 0x%x\n", !!(pwm_sta & (1<<3)));
  printf("  WERR1: 0x%x\n", !!(pwm_sta & (1<<2)));
  printf("  EMPT1: 0x%x\n", !!(pwm_sta & (1<<1)));
  printf("  FULL1: 0x%x\n", !!(pwm_sta & (1<<0)));
  printf("PWM_DMAC: 0x%x\n", pwm_reg[PWM_DMAC]);

  // dma regs
  printf("DMA_ENABLE: 0x%x\n", dma_base_reg[DMA_ENABLE]);
  uint32_t dma_chan_cs = dma_reg[DMA_CS];
  printf("%d_CS: 0x%x\n", dma_chan, dma_chan_cs);
  printf("  ERROR: 0x%x\n", !!(dma_chan_cs & (1<<8)));
  printf("  DREQ:  0x%x\n", !!(dma_chan_cs & (1<<3)));
  printf("  END:   0x%x\n", !!(dma_chan_cs & DMA_END));
  printf("  ACTIVE: 0x%x\n", !!(dma_chan_cs & DMA_ACTIVE));
  printf("%d_NEXTCONBK: 0x%x\n", dma_chan, dma_reg[DMA_NEXTCONBK]);
  printf("%d_DEBUG: 0x%x\n", dma_chan, dma_reg[DMA_DEBUG]);
}

static void
go_go_go(void)
{
  // setup gpio 18 output for pwm0 (ch 1)
  gpio_set_mode(18, GPIO_MODE_ALT5);

  // start with debug info
  if (is_debug) {
    do_debug();
  }

  // open daemon char dev
  int fd;
  struct timeval tv;
  static char line[128];
  int nchars = 0;

  if ((fd = open(DEVFILE, O_RDWR|O_NONBLOCK)) == -1)
    fatal("dccwaved: Failed to open %s: %m\n", DEVFILE);

  // process dameon char dev commands
  for (;;) {
    int n;
    fd_set ifds;
    FD_ZERO(&ifds);
    FD_SET(fd, &ifds);
    // look for next command
    if ((n = select(fd+1, &ifds, NULL, NULL, &tv)) != 1)
      continue;
    while (read(fd, line+nchars, 1) == 1) {
      if (line[nchars] == '\n') {
        line[++nchars] = '\0';
        nchars = 0;

        // dispatch command
        if (!strcmp(line, "debug\n")) {
          do_debug();
        } else {
          fprintf(stderr, "Got input: %s", line);          
        }
      } else {
        if (++nchars >= 126) {
          fprintf(stderr, "Input too long\n");
          nchars = 0;
        }
      }
    }
  }
}

// static void
// go_go_go(void)
// {
//   int fd;
//   struct timeval tv;
//   static char line[128];
//   int nchars = 0;

//   if ((fd = open(DEVFILE, O_RDWR|O_NONBLOCK)) == -1)
//     fatal("dccwaved: Failed to open %s: %m\n", DEVFILE);

//   for (;;) {
//     int n, width, servo;
//     fd_set ifds;
//     char width_arg[64];

//     FD_ZERO(&ifds);
//     FD_SET(fd, &ifds);
//     get_next_idle_timeout(&tv);
//     if ((n = select(fd+1, &ifds, NULL, NULL, &tv)) != 1)
//       continue;
//     while (read(fd, line+nchars, 1) == 1) {
//       if (line[nchars] == '\n') {
//         line[++nchars] = '\0';
//         nchars = 0;
//         if (line[0] == 'p' || line[0] == 'P') {
//           int hdr, pin, width;

//           n = sscanf(line+1, "%d-%d=%s", &hdr, &pin, width_arg);
//           if (n != 3) {
//             fprintf(stderr, "Bad input: %s", line);
//           } else if (hdr != 1 && hdr != 5) {
//             fprintf(stderr, "Invalid header P%d\n", hdr);
//           } else if (pin < 1 ||
//               (hdr == 1 && pin > NUM_P1PINS) ||
//               (hdr == 5 && pin > NUM_P5PINS)) {
//             fprintf(stderr, "Invalid pin number P%d-%d\n", hdr, pin);
//           } else if ((hdr == 1 && p1pin2servo[pin] == DMY) ||
//                (hdr == 5 && p5pin2servo[pin] == DMY)) {
//               fprintf(stderr, "P%d-%d is not mapped to a servo\n", hdr, pin);
//           } else {
//             if (hdr == 1) {
//               servo = p1pin2servo[pin];
//             } else {
//               servo = p5pin2servo[pin];
//             }
//             if ((width = parse_width(servo, width_arg)) < 0) {
//               fprintf(stderr, "Invalid width specified\n");
//             } else {
//               set_servo(servo, width);
//             }
//           }
//         } else {
//           n = sscanf(line, "%d=%s", &servo, width_arg);
//           if (!strcmp(line, "debug\n")) {
//             do_debug();
//           } else if (!strncmp(line, "status ", 7)) {
//             do_status(line + 7);
//           } else if (n != 2) {
//             fprintf(stderr, "Bad input: %s", line);
//           } else if (servo < 0 || servo >= MAX_SERVOS) {
//             fprintf(stderr, "Invalid servo number %d\n", servo);
//           } else if (servo2gpio[servo] == DMY) {
//             fprintf(stderr, "Servo %d is not mapped to a GPIO pin\n", servo);
//           } else if ((width = parse_width(servo, width_arg)) < 0) {
//             fprintf(stderr, "Invalid width specified\n");
//           } else {
//             set_servo(servo, width);
//           }
//         }
//       } else {
//         if (++nchars >= 126) {
//           fprintf(stderr, "Input too long\n");
//           nchars = 0;
//         }
//       }
//     }
//   }
// }

/* Determining the board revision is a lot more complicated than it should be
 * (see comments in wiringPi for details).  We will just look at the last two
 * digits of the Revision string and treat '00' and '01' as errors, '02' and
 * '03' as rev 1, and any other hex value as rev 2.  'Pi1 and Pi2 are
 * differentiated by the Hardware being BCM2708 or BCM2709.
 *
 * NOTE: These days we should just use bcm_host_get_model_type().
 */
static void
get_model_and_revision(void)
{
  char buf[128], revstr[128], modelstr[128];
  char *ptr, *end, *res;
  int board_revision;
  FILE *fp;

  revstr[0] = modelstr[0] = '\0';

  fp = fopen("/proc/cpuinfo", "r");

  if (!fp)
    fatal("Unable to open /proc/cpuinfo: %m\n");

  while ((res = fgets(buf, 128, fp))) {
    if (!strncasecmp("hardware", buf, 8))
      memcpy(modelstr, buf, 128);
    else if (!strncasecmp(buf, "revision", 8))
      memcpy(revstr, buf, 128);
  }
  fclose(fp);

  if (modelstr[0] == '\0')
    fatal("dccwaved: No 'Hardware' record in /proc/cpuinfo\n");
  if (revstr[0] == '\0')
    fatal("dccwaved: No 'Revision' record in /proc/cpuinfo\n");

  if (strstr(modelstr, "BCM2708"))
    board_model = 1;
  else if (strstr(modelstr, "BCM2709") || strstr(modelstr, "BCM2835"))
    board_model = 2;
  else
    fatal("dccwaved: Cannot parse the hardware name string\n");

  /* Revisions documented at http://elinux.org/RPi_HardwareHistory */
  ptr = revstr + strlen(revstr) - 3;
  board_revision = strtol(ptr, &end, 16);
  if (end != ptr + 2)
    fatal("dccwaved: Failed to parse Revision string\n");
  if (board_revision < 1)
    fatal("dccwaved: Invalid board Revision\n");
  else if (board_revision < 4)
    gpio_cfg = 1;
  else if (board_revision < 16)
    gpio_cfg = 2;
  else
    gpio_cfg = 3;

  if (bcm_host_is_model_pi4()) {
    plldfreq_mhz = PLLDFREQ_MHZ_PI4;
    dma_chan = DMA_CHAN_PI4;
  } else {
    plldfreq_mhz = PLLDFREQ_MHZ_DEFAULT;
    dma_chan = DMA_CHAN_DEFAULT;
  }

  periph_virt_base = bcm_host_get_peripheral_address();
  dram_phys_base = bcm_host_get_sdram_address();
  periph_phys_base = 0x7e000000;

  /*
   * See https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
   *
   * 1:  MEM_FLAG_DISCARDABLE = 1 << 0  // can be resized to 0 at any time. Use for cached data
   *     MEM_FLAG_NORMAL = 0 << 2   // normal allocating alias. Don't use from ARM
   * 4:  MEM_FLAG_DIRECT = 1 << 2   // 0xC alias uncached
   * 8:  MEM_FLAG_COHERENT = 2 << 2 // 0x8 alias. Non-allocating in L2 but coherent
   *     MEM_FLAG_L1_NONALLOCATING =  // Allocating in L2
   *       (MEM_FLAG_DIRECT | MEM_FLAG_COHERENT)
   * 16: MEM_FLAG_ZERO = 1 << 4   // initialise buffer to all zeros
   * 32: MEM_FLAG_NO_INIT = 1 << 5  // don't initialise (default is initialise to all ones
   * 64: MEM_FLAG_HINT_PERMALOCK = 1 << 6 // Likely to be locked for long periods of time
   *
   */
  if (board_model == 1) {
    mem_flag         = 0x0c;  /* MEM_FLAG_DIRECT | MEM_FLAG_COHERENT */
  } else {
    mem_flag         = 0x04;  /* MEM_FLAG_DIRECT */
  }
}

int
main(int argc, char **argv)
{
  // dcc dma output chain
  dcc_pkt_t dcc_pkt_tmp;
  int dcc_dma_next_id = 0;
  dcc_dma_entry_t *dcc_dma_buf;

  // program options
  int i;
  char *p1pins = default_p1_pins;
  char *p5pins = default_p5_pins;
  int p1first = 1, hadp1 = 0, hadp5 = 0;
  char *dma_chan_arg = NULL;
  char *p;
  int daemonize = 1;
  is_debug = 0;

  /* Option processing */
  setvbuf(stdout, NULL, _IOLBF, 0);
  while (1) {
    int c;
    int option_index;

    static struct option long_options[] = {
      { "help",         no_argument,       0, 'h' },
      { "invert",       no_argument,       0, 'i' },
      { "debug",        no_argument,       0, 'f' },
      { "dma-chan",     required_argument, 0, 'd' },
      { 0,              0,                 0, 0   }
    };

    c = getopt_long(argc, argv, "hifd:", long_options, &option_index);
    if (c == -1) {
      break;
    } else if (c =='d') {
      dma_chan_arg = optarg;
    } else if (c == 'f') {
      daemonize = 0;
      is_debug = 1;
    } else if (c == 'i') {
      invert = 1;
    } else if (c == 'h') {
      printf("\nUsage: %s <options>\n\n"
        "Options:\n"
        "  --invert, -i        inverts outputs\n"
        "  --debug, -f         run in debug mode, dont' daemonize\n"
        "  --dma-chan=N, -d=N  tells servod which dma channel to use, default %d\n",
        argv[0],
        DMA_CHAN_DEFAULT);
      exit(0);
    } else {
      fatal("Invalid parameter\n");
    }
  }
  get_model_and_revision();
  if (board_model == 1 && gpio_cfg == 1 && p5pins[0])
    fatal("Board model 1 revision 1 does not have a P5 header\n");
  if (board_model == 2 && p5pins[0])
    fatal("Board models 2 and later do not have a P5 header\n");

  if (dma_chan_arg) {
    dma_chan = strtol(dma_chan_arg, &p, 10);
    if (*dma_chan_arg < '0' || *dma_chan_arg > '9' ||
        *p || dma_chan < DMA_CHAN_MIN || dma_chan > DMA_CHAN_MAX)
      fatal("Invalid dma-chan specified\n");
  }

  num_pages = (DCC_DMA_BUF_LEN * sizeof(dcc_dma_entry_t) +
               PAGE_SIZE - 1) >> PAGE_SHIFT;
  if (num_pages > MAX_MEMORY_USAGE / PAGE_SIZE) {
    fatal("Using too much memory; reduce cycle-time or increase step-size\n");
  }

  {
    int bcm_model = bcm_host_get_model_type();

    if (bcm_model < NUM_MODELS)
      printf("\nBoard model:               %7s\n", model_names[bcm_model]);
    else
      printf("\nBoard model:               Unknown\n");
  }

  printf("GPIO configuration:            %s\n", gpio_desc[gpio_cfg]);
  printf("Using hardware:                %s\n", delay_hw == DELAY_VIA_PWM ? "PWM" : "PCM");
  printf("Using DMA channel:         %7d\n", dma_chan);
  printf("\n");

  // init_idle_timers();
  setup_sighandlers();

  dma_base_reg = map_peripheral(DMA_VIRT_BASE, DMA_LEN);
  dma_reg = dma_base_reg + dma_chan * DMA_CHAN_SIZE / sizeof(uint32_t);
  pwm_reg = map_peripheral(PWM_VIRT_BASE, PWM_LEN);
  pcm_reg = map_peripheral(PCM_VIRT_BASE, PCM_LEN);
  clk_reg = map_peripheral(CLK_VIRT_BASE, CLK_LEN);
  gpio_reg = map_peripheral(GPIO_VIRT_BASE, GPIO_LEN);

  /* Use the mailbox interface to the VC to ask for physical memory */
  // Use the mailbox interface to request memory from the VideoCore
  // We specifiy (-1) for the handle rather than calling mbox_open()
  // so multiple users can share the resource.
  mbox.handle = -1; // mbox_open();
  mbox.size = num_pages * 4096;
  mbox.mem_ref = mem_alloc(mbox.handle, mbox.size, 4096, mem_flag);
  if (mbox.mem_ref < 0) {
    fatal("Failed to alloc memory from VideoCore\n");
  }
  mbox.bus_addr = mem_lock(mbox.handle, mbox.mem_ref);
  if (mbox.bus_addr == ~0) {
    mem_free(mbox.handle, mbox.size);
    fatal("Failed to lock memory\n");
  }
  mbox.virt_addr = mapmem(BUS_TO_PHYS(mbox.bus_addr), mbox.size);
  dcc_dma_buf = (dcc_dma_entry_t *)mbox.virt_addr;

  // turnoff_mask = (uint32_t *)mbox.virt_addr;
  // turnon_mask = (uint32_t *)(mbox.virt_addr + num_samples * sizeof(uint32_t));
  // cb_base = (dma_cb_t *)(mbox.virt_addr +
  //   ROUNDUP(num_samples + MAX_SERVOS, 8) * sizeof(uint32_t));

  // put dma control block at the beginning of new block
  // cb_base = (dma_cb_t *)(mbox.virt_addr);
  // // and pwm serializer data just after
  // uint32_t *pwm_ser_base;
  // pwm_ser_base = (uint32_t *)(cb_base+1);
  // pwm_ser_base[0] = 0x0F0F0F0F;
  // pwm_ser_base[1] = 0x0F070301;
  // pwm_ser_base[2] = 0x05155555;
  // pwm_ser_base[3] = 0x01234567;
  // pwm_ser_base[4] = 0x89ABCDEF;
  // pwm_ser_base[5] = 0x0103070F;

  // init_ctrl_data();

  /* Start DMA hardware */
  printf("Reserved %d pages at 0x%x\n", num_pages, (void *)dcc_dma_buf);
  dcc_dma_init(dcc_dma_buf);
  printf("%d: 0x%x -> 0x%x\n",
         0,
         mem_virt_to_phys(&(dcc_dma_buf[0].dma_cb)),
         (uint32_t)dcc_dma_buf[0].dma_cb.next);
  printf("0x%x\n0x%x\n0x%x\n0x%x\n0x%x\n0x%x\n0x%x\n", 
         dcc_dma_buf[0].dcc_pkt.line_code[0],
         dcc_dma_buf[0].dcc_pkt.line_code[1],
         dcc_dma_buf[0].dcc_pkt.line_code[2],
         dcc_dma_buf[0].dcc_pkt.line_code[3],
         dcc_dma_buf[0].dcc_pkt.line_code[4],
         dcc_dma_buf[0].dcc_pkt.line_code[5],
         dcc_dma_buf[0].dcc_pkt.line_code[6]);
  dcc_dma_hw_config(dcc_dma_buf);

  unlink(DEVFILE);
  if (mkfifo(DEVFILE, 0666) < 0)
    fatal("dccwaved: Failed to create %s: %m\n", DEVFILE);
  if (chmod(DEVFILE, 0666) < 0)
    fatal("dccwaved: Failed to set permissions on %s: %m\n", DEVFILE);

  if (daemonize && daemon(0,1) < 0)
    fatal("dccwaved: Failed to daemonize process: %m\n");

  go_go_go();

  return 0;
}
