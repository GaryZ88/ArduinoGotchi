/*
 * TamaLIB - A hardware agnostic Tamagotchi P1 emulation library
 *
 * Copyright (C) 2021 Jean-Christophe Rona <jc@rona.fr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
 #include <avr/pgmspace.h>
#include "cpu.h"
#include "hw.h"
#include "hal.h"
//#include "rom_new.h"
#include "rom_12bit.h"

#define CPU_SPEED_RATIO      0
#define TICK_FREQUENCY        32768 // Hz

#define TIMER_1HZ_PERIOD      32768 // in ticks
#define TIMER_256HZ_PERIOD      128 // in ticks

#define MASK_4B         0xF00
#define MASK_6B         0xFC0
#define MASK_7B         0xFE0
#define MASK_8B         0xFF0
#define MASK_10B        0xFFC
#define MASK_12B        0xFFF

#define PCS         (pc & 0xFF)
#define PCSL          (pc & 0xF)
#define PCSH          ((pc >> 4) & 0xF)
#define PCP         ((pc >> 8) & 0xF)
#define PCB         ((pc >> 12) & 0x1)
#define TO_PC(bank, page, step)     ((step & 0xFF) | ((page & 0xF) << 8) | (bank & 0x1) << 12)
#define NBP         ((np >> 4) & 0x1)
#define NPP         (np & 0xF)
#define TO_NP(bank, page)     ((page & 0xF) | (bank & 0x1) << 4)
#define XHL         (x & 0xFF)
#define XL1         (x & 0xF)
#define XH1         ((x >> 4) & 0xF)
#define XP          ((x >> 8) & 0xF)
#define YHL         (y & 0xFF)
#define YL1         (y & 0xF)
#define YH1         ((y >> 4) & 0xF)
#define YP          ((y >> 8) & 0xF)
#define M(n)          get_memory(n)
#define SET_M(n, v)       set_memory(n, v)
#define RQ(i)         get_rq(i)
#define SET_RQ(i, v)        set_rq(i, v)
#define SPL1          (sp & 0xF)
#define SPH1          ((sp >> 4) & 0xF)

#define FLAG_C          (0x1 << 0)
#define FLAG_Z          (0x1 << 1)
#define FLAG_D          (0x1 << 2)
#define FLAG_I          (0x1 << 3)

#define C         !!(flags & FLAG_C)
#define Z         !!(flags & FLAG_Z)
#define D         !!(flags & FLAG_D)
#define I         !!(flags & FLAG_I)

#define SET_C()         {flags |= FLAG_C;}
#define CLEAR_C()       {flags &= ~FLAG_C;}
#define SET_Z()         {flags |= FLAG_Z;}
#define CLEAR_Z()       {flags &= ~FLAG_Z;}
#define SET_D()         {flags |= FLAG_D;}
#define CLEAR_D()       {flags &= ~FLAG_D;}
#define SET_I()         {flags |= FLAG_I;}
#define CLEAR_I()       {flags &= ~FLAG_I;}

#define REG_CLK_INT_FACTOR_FLAGS    0xF00
#define REG_SW_INT_FACTOR_FLAGS     0xF01
#define REG_PROG_INT_FACTOR_FLAGS   0xF02
#define REG_SERIAL_INT_FACTOR_FLAGS   0xF03
#define REG_K00_K03_INT_FACTOR_FLAGS    0xF04
#define REG_K10_K13_INT_FACTOR_FLAGS    0xF05
#define REG_CLOCK_INT_MASKS     0xF10
#define REG_SW_INT_MASKS      0xF11
#define REG_PROG_INT_MASKS      0xF12
#define REG_SERIAL_INT_MASKS      0xF13
#define REG_K00_K03_INT_MASKS     0xF14
#define REG_K10_K13_INT_MASKS     0xF15
#define REG_PROG_TIMER_DATA_L     0xF24
#define REG_PROG_TIMER_DATA_H     0xF25
#define REG_PROG_TIMER_RELOAD_DATA_L    0xF26
#define REG_PROG_TIMER_RELOAD_DATA_H    0xF27
#define REG_K00_K03_INPUT_PORT      0xF40
#define REG_K10_K13_INPUT_PORT      0xF42
#define REG_K40_K43_BZ_OUTPUT_PORT    0xF54
#define REG_CPU_OSC3_CTRL     0xF70
#define REG_LCD_CTRL        0xF71
#define REG_LCD_CONTRAST      0xF72
#define REG_SVD_CTRL        0xF73
#define REG_BUZZER_CTRL1      0xF74
#define REG_BUZZER_CTRL2      0xF75
#define REG_CLK_WD_TIMER_CTRL     0xF76
#define REG_SW_TIMER_CTRL     0xF77
#define REG_PROG_TIMER_CTRL     0xF78
#define REG_PROG_TIMER_CLK_SEL      0xF79

#define INPUT_PORT_NUM        2

typedef struct {
  //char *log;
  u12_t code;
  u12_t mask;
//  u12_t shift_arg1;
//  u12_t mask_arg1;      // != 0 only if there are two arguments
  u8_t cycles;
 // void (*cb0)(u8_t arg0, u8_t arg1);
} op_t0;

typedef struct {
  //char *log;
//  u12_t code;
//  u12_t mask;
//  u12_t shift_arg1;
//  u12_t mask_arg1;      // != 0 only if there are two arguments
//  u8_t cycles;
  void (*cb1)(u8_t arg0, u8_t arg1);
} op_t1;

typedef struct {
  u4_t states;
} input_port_t;

/* Registers */
static u13_t pc, next_pc;
static u12_t x, y;
static u4_t a, b;
static u5_t np;
static u8_t sp;

/* Flags */
static u4_t flags;

//static const u12_t *g_program = NULL;
static u4_t memory[MEMORY_SIZE];
//static u4_t io_memory[MEM_IO_SIZE];

static input_port_t inputs[INPUT_PORT_NUM] = {{0}};

//static u8_t maxNumber = 0;

/* Interrupts (in priority order) */
static interrupt_t interrupts[INT_SLOT_NUM] = {
  {0x0, 0x0, 0, 0x0C}, // Prog timer
  {0x0, 0x0, 0, 0x0A}, // Serial interface
  {0x0, 0x0, 0, 0x08}, // Input (K10-K13)
  {0x0, 0x0, 0, 0x06}, // Input (K00-K03)
  {0x0, 0x0, 0, 0x04}, // Stopwatch timer
  {0x0, 0x0, 0, 0x02}, // Clock timer
};

//static breakpoint_t *g_breakpoints = NULL;

static u32_t call_depth = 0;

static u32_t clk_timer_timestamp = 0; // in ticks
static u32_t prog_timer_timestamp = 0; // in ticks
static bool_t prog_timer_enabled = 0;
static u8_t prog_timer_data = 0;
static u8_t prog_timer_rld = 0;

static u32_t tick_counter = 0;
static u32_t ts_freq;
//static u8_t speed_ratio = 0;
static timestamp_t ref_ts;

/*
static state_t cpu_state = {
  .pc = &pc,
  .x = &x,
  .y = &y,
  .a = &a,
  .b = &b,
  .np = &np,
  .sp = &sp,
  .flags = &flags,

  .tick_counter = &tick_counter,
  .clk_timer_timestamp = &clk_timer_timestamp,
  .prog_timer_timestamp = &prog_timer_timestamp,
  .prog_timer_enabled = &prog_timer_enabled,
  .prog_timer_data = &prog_timer_data,
  .prog_timer_rld = &prog_timer_rld,

  .call_depth = &call_depth,

  .interrupts = interrupts,

  .memory = memory, 
}; */


void cpu_add_bp(breakpoint_t **list, u13_t addr)
{
/*  breakpoint_t *bp;

  bp = (breakpoint_t *) g_hal->malloc(sizeof(breakpoint_t));
  if (!bp) {
    g_hal->log(LOG_ERROR, "Cannot allocate memory for breakpoint 0x%04X!\n", addr);
    return;
  }

  bp->addr = addr;

  if (*list != NULL) {
    bp->next = *list;
  } else {
    bp->next = NULL;
  }

  *list = bp; */
}

void cpu_free_bp(breakpoint_t **list)
{
/*  breakpoint_t *bp = *list, *tmp;
  while (bp != NULL) {
    tmp = bp->next;
    g_hal->free(bp);
    bp = tmp;
  }
  *list = NULL; */
}
/*
void cpu_set_speed(u8_t speed)
{
  speed_ratio = speed;
}*/


void cpu_get_state(cpu_state_t *cpustate)
{
  cpustate->pc = pc;
  cpustate->x = x;
  cpustate->y = y;
  cpustate->a = a;
  cpustate->b = b;
  cpustate->np = np;
  cpustate->sp = sp;

  cpustate->flags = flags;
  cpustate->tick_counter = tick_counter;
  cpustate->clk_timer_timestamp = clk_timer_timestamp;
  cpustate->prog_timer_timestamp = prog_timer_timestamp;
  cpustate->prog_timer_enabled = prog_timer_enabled;
  cpustate->prog_timer_data = prog_timer_data;
  cpustate->prog_timer_rld = prog_timer_rld;
  cpustate->call_depth = call_depth;
  cpustate->memory = (u4_t *)memory;
  uint8_t i;
  for(i=0;i<6;i++) {
    cpustate->interrupts[i].factor_flag_reg = interrupts[i].factor_flag_reg;
    cpustate->interrupts[i].mask_reg = interrupts[i].mask_reg;
    cpustate->interrupts[i].triggered = interrupts[i].triggered;
    cpustate->interrupts[i].vector = interrupts[i].vector;
  }
}

void cpu_set_state(cpu_state_t *cpustate)
{
  pc = cpustate->pc;
  x = cpustate->x;
  y = cpustate->y;
  a = cpustate->a;
  b = cpustate->b;
  np = cpustate->np;
  sp = cpustate->sp;
  flags = cpustate->flags;
  tick_counter = cpustate->tick_counter;
  clk_timer_timestamp = cpustate->clk_timer_timestamp;
  prog_timer_timestamp = cpustate->prog_timer_timestamp;
  prog_timer_enabled = cpustate->prog_timer_enabled;
  prog_timer_data = cpustate->prog_timer_data;
  prog_timer_rld = cpustate->prog_timer_rld;
  call_depth = cpustate->call_depth;
  //memory = (u4_t *)cpustate->memory;
  uint8_t i;
  for(i=0;i<6;i++) {
    interrupts[i].factor_flag_reg = cpustate->interrupts[i].factor_flag_reg;
    interrupts[i].mask_reg = cpustate->interrupts[i].mask_reg;
    interrupts[i].triggered = cpustate->interrupts[i].triggered;
    interrupts[i].vector = cpustate->interrupts[i].vector;
  }
}

u32_t cpu_get_depth(void)
{
  return call_depth;
}

static void generate_interrupt(int_slot_t slot, u8_t bit)
{
  /* Set the factor flag no matter what */
  interrupts[slot].factor_flag_reg = interrupts[slot].factor_flag_reg | (0x1 << bit);

  /* Trigger the INT only if not masked */
  if (interrupts[slot].mask_reg & (0x1 << bit)) {
    interrupts[slot].triggered = 1;
  }
}

void cpu_set_input_pin(pin_t pin, pin_state_t state)
{
  /* Set the I/O */
  inputs[pin & 0x4].states = (inputs[pin & 0x4].states & ~(0x1 << (pin & 0x3))) | (state << (pin & 0x3));

  /* Trigger the interrupt (TODO: handle relation register) */
  if (state == PIN_STATE_LOW) {
    switch ((pin & 0x4) >> 2) {
      case 0:
        generate_interrupt(INT_K00_K03_SLOT, pin & 0x3);
        break;

      case 1:
        generate_interrupt(INT_K10_K13_SLOT, pin & 0x3);
        break;
    }
  }
}

void cpu_sync_ref_timestamp(void)
{
  ref_ts = g_hal->get_timestamp();
}

static u4_t get_io(u12_t n)
{
  u4_t tmp;

  switch (n) {
    case REG_CLK_INT_FACTOR_FLAGS:
      /* Interrupt factor flags (clock timer) */
      tmp = interrupts[INT_CLOCK_TIMER_SLOT].factor_flag_reg;
      interrupts[INT_CLOCK_TIMER_SLOT].factor_flag_reg = 0;
      return tmp;

    case REG_SW_INT_FACTOR_FLAGS:
      /* Interrupt factor flags (stopwatch) */
      tmp = interrupts[INT_STOPWATCH_SLOT].factor_flag_reg;
      interrupts[INT_STOPWATCH_SLOT].factor_flag_reg = 0;
      return tmp;

    case REG_PROG_INT_FACTOR_FLAGS:
      /* Interrupt factor flags (prog timer) */
      tmp = interrupts[INT_PROG_TIMER_SLOT].factor_flag_reg;
      interrupts[INT_PROG_TIMER_SLOT].factor_flag_reg = 0;
      return tmp;

    case REG_SERIAL_INT_FACTOR_FLAGS:
      /* Interrupt factor flags (serial) */
      tmp = interrupts[INT_SERIAL_SLOT].factor_flag_reg;
      interrupts[INT_SERIAL_SLOT].factor_flag_reg = 0;
      return tmp;

    case REG_K00_K03_INT_FACTOR_FLAGS:
      /* Interrupt factor flags (K00-K03) */
      tmp = interrupts[INT_K00_K03_SLOT].factor_flag_reg;
      interrupts[INT_K00_K03_SLOT].factor_flag_reg = 0;
      return tmp;

    case REG_K10_K13_INT_FACTOR_FLAGS:
      /* Interrupt factor flags (K10-K13) */
      tmp = interrupts[INT_K10_K13_SLOT].factor_flag_reg;
      interrupts[INT_K10_K13_SLOT].factor_flag_reg = 0;
      return tmp;

    case REG_CLOCK_INT_MASKS:
      /* Clock timer interrupt masks */
      return interrupts[INT_CLOCK_TIMER_SLOT].mask_reg;

    case REG_SW_INT_MASKS:
      /* Stopwatch interrupt masks */
      return interrupts[INT_STOPWATCH_SLOT].mask_reg & 0x3;

    case REG_PROG_INT_MASKS:
      /* Prog timer interrupt masks */
      return interrupts[INT_PROG_TIMER_SLOT].mask_reg & 0x1;

    case REG_SERIAL_INT_MASKS:
      /* Serial interface interrupt masks */
      return interrupts[INT_SERIAL_SLOT].mask_reg & 0x1;

    case REG_K00_K03_INT_MASKS:
      /* Input (K00-K03) interrupt masks */
      return interrupts[INT_K00_K03_SLOT].mask_reg;

    case REG_K10_K13_INT_MASKS:
      /* Input (K10-K13) interrupt masks */
      return interrupts[INT_K10_K13_SLOT].mask_reg;

    case REG_PROG_TIMER_DATA_L:
      /* Prog timer data (low) */
      return prog_timer_data & 0xF;

    case REG_PROG_TIMER_DATA_H:
      /* Prog timer data (high) */
      return (prog_timer_data >> 4) & 0xF;

    case REG_PROG_TIMER_RELOAD_DATA_L:
      /* Prog timer reload data (low) */
      return prog_timer_rld & 0xF;

    case REG_PROG_TIMER_RELOAD_DATA_H:
      /* Prog timer reload data (high) */
      return (prog_timer_rld >> 4) & 0xF;

    case REG_K00_K03_INPUT_PORT:
      /* Input port (K00-K03) */
      return inputs[0].states;

    case REG_K10_K13_INPUT_PORT:
      /* Input port (K10-K13) */
      return inputs[1].states;

    case REG_K40_K43_BZ_OUTPUT_PORT:
      /* Output port (R40-R43) */
      //return io_memory[n - MEM_IO_ADDR_OFS];
      return 0xf;
    case REG_CPU_OSC3_CTRL:
      /* CPU/OSC3 clocks switch, CPU voltage switch */
      //return io_memory[n - MEM_IO_ADDR_OFS];
      return 0;
    case REG_LCD_CTRL:
      /* LCD control */
      //return io_memory[n - MEM_IO_ADDR_OFS];
      return 0x8;
    case REG_LCD_CONTRAST:
      /* LCD contrast */
      break;

    case REG_SVD_CTRL:
      /* SVD */
      //return io_memory[n - MEM_IO_ADDR_OFS] & 0x7; // Voltage always OK
      return 0;
    case REG_BUZZER_CTRL1:
      /* Buzzer config 1 */
      //return memory[n - MEM_IO_ADDR_OFS];
      return 0;
    case REG_BUZZER_CTRL2:
      /* Buzzer config 2 */
      //return io_memory[n - MEM_IO_ADDR_OFS] & 0x3; // Buzzer ready
      return 0;
    case REG_CLK_WD_TIMER_CTRL:
      /* Clock/Watchdog timer reset */
      break;

    case REG_SW_TIMER_CTRL:
      /* Stopwatch stop/run/reset */
      break;

    case REG_PROG_TIMER_CTRL:
      /* Prog timer stop/run/reset */
      return !!prog_timer_enabled;

    case REG_PROG_TIMER_CLK_SEL:
      /* Prog timer clock selection */
      break;

    default:
      break;
      //g_hal->log(LOG_ERROR,   "Read from unimplemented I/O 0x%03X - PC = 0x%04X\n", n, pc);
  }

  return 0;
}

static void set_io(u12_t n, u4_t v)
{
  switch (n) {
    case REG_CLOCK_INT_MASKS:
      /* Clock timer interrupt masks */
      /* Assume 1Hz timer INT enabled (0x8) */
      interrupts[INT_CLOCK_TIMER_SLOT].mask_reg = v;
      break;

    case REG_SW_INT_MASKS:
      /* Stopwatch interrupt masks */
      /* Assume all INT disabled */
      interrupts[INT_STOPWATCH_SLOT].mask_reg = v;
      break;

    case REG_PROG_INT_MASKS:
      /* Prog timer interrupt masks */
      /* Assume Prog timer INT enabled (0x1) */
      interrupts[INT_PROG_TIMER_SLOT].mask_reg = v;
      break;

    case REG_SERIAL_INT_MASKS:
      /* Serial interface interrupt masks */
      /* Assume all INT disabled */
      interrupts[INT_K10_K13_SLOT].mask_reg = v;
      break;

    case REG_K00_K03_INT_MASKS:
      /* Input (K00-K03) interrupt masks */
      /* Assume all INT disabled */
      interrupts[INT_SERIAL_SLOT].mask_reg = v;
      break;

    case REG_K10_K13_INT_MASKS:
      /* Input (K10-K13) interrupt masks */
      /* Assume all INT disabled */
      interrupts[INT_K10_K13_SLOT].mask_reg = v;
      break;

    case REG_PROG_TIMER_RELOAD_DATA_L:
      /* Prog timer reload data (low) */
      prog_timer_rld = v | (prog_timer_rld & 0xF0);
      break;

    case REG_PROG_TIMER_RELOAD_DATA_H:
      /* Prog timer reload data (high) */
      prog_timer_rld = (prog_timer_rld & 0xF) | (v << 4);
      break;

    case REG_K00_K03_INPUT_PORT:
      /* Input port (K00-K03) */
      /* Write not allowed */
      break;

    case REG_K40_K43_BZ_OUTPUT_PORT:
      /* Output port (R40-R43) */
      //g_hal->log(LOG_INFO, "Output/Buzzer: 0x%X\n", v);
      hw_enable_buzzer(!(v & 0x8));
      break;

    case REG_CPU_OSC3_CTRL:
      /* CPU/OSC3 clocks switch, CPU voltage switch */
      /* Assume 32,768 OSC1 selected, OSC3 off, battery >= 3,1V (0x1) */
      break;

    case REG_LCD_CTRL:
      /* LCD control */
      break;

    case REG_LCD_CONTRAST:
      /* LCD contrast */
      /* Assume medium contrast (0x8) */
      break;

    case REG_SVD_CTRL:
      /* SVD */
      /* Assume battery voltage always OK (0x6) */
      break;

    case REG_BUZZER_CTRL1:
      /* Buzzer config 1 */
      hw_set_buzzer_freq(v & 0x7);
      break;

    case REG_BUZZER_CTRL2:
      /* Buzzer config 2 */
      break;

    case REG_CLK_WD_TIMER_CTRL:
      /* Clock/Watchdog timer reset */
      /* Ignore watchdog */
      break;

    case REG_SW_TIMER_CTRL:
      /* Stopwatch stop/run/reset */
      break;

    case REG_PROG_TIMER_CTRL:
      /* Prog timer stop/run/reset */
      if (v & 0x2) {
        prog_timer_data = prog_timer_rld;
      }

      if ((v & 0x1) && !prog_timer_enabled) {
        prog_timer_timestamp = tick_counter;
      }

      prog_timer_enabled = v & 0x1;
      break;

    case REG_PROG_TIMER_CLK_SEL:
      /* Prog timer clock selection */
      /* Assume 256Hz, output disabled */
      break;

    default:
      break;
      //g_hal->log(LOG_ERROR,   "Write 0x%X to unimplemented I/O 0x%03X - PC = 0x%04X\n", v, n, pc);
  }
}

static void set_lcd(u12_t n, u4_t v)
{
  u8_t i;
  u8_t seg, com0;

  seg = ((n & 0x7F) >> 1);
  com0 = (((n & 0x80) >> 7) * 8 + (n & 0x1) * 4);

  for (i = 0; i < 4; i++) {
    hw_set_lcd_pin(seg, com0 + i, (v >> i) & 0x1);
  }
}

/*
#define MEMORY_SIZE        1280 // 4096 x 4 bits (640 x 4 bits of RAM)

#define MEM_RAM_ADDR        0x000
#define MEM_RAM_SIZE        0x280    // 640
#define MEM_DISPLAY1_ADDR     0xE00  // 3584  0x280
#define MEM_DISPLAY1_ADDR_OFS 0xB80
#define MEM_DISPLAY1_SIZE     0x050  // 80    
#define MEM_DISPLAY2_ADDR     0xE80  // 3712  0x2D0
#define MEM_DISPLAY2_ADDR_OFS 0xBB0
#define MEM_DISPLAY2_SIZE     0x050 // 80
#define MEM_IO_ADDR       0xF00     // 3840   0x320
#define MEM_IO_ADDR_OFS 0xBE0
#define MEM_IO_SIZE       0x080     // 128    0x3a0

*/
/*
u8_t cpu_get_max_number() {
  return maxNumber;
}
*/
static u4_t get_memory(u12_t n)
{
  u4_t res = 0;
  

  if (n < MEM_RAM_SIZE) {
    /* RAM */
    //g_hal->log(LOG_MEMORY, "RAM              - ");
    //if (n > max_memory_addr_access) max_memory_addr_access = n;
    if ((n & 0x1)==0) {
      res = memory[n>>1] >> 4;
    } else {
      res = memory[n>>1] & 0b00001111;
    }
    
  } else if (n >= MEM_DISPLAY1_ADDR && n < (MEM_DISPLAY1_ADDR + MEM_DISPLAY1_SIZE)) {
    /* Display Memory 1 */
    //g_hal->log(LOG_MEMORY, "Display Memory 1 - ");
    //res = memory[n - MEM_DISPLAY1_ADDR_OFS];
      res = 0;  

  } else if (n >= MEM_DISPLAY2_ADDR && n < (MEM_DISPLAY2_ADDR + MEM_DISPLAY2_SIZE)) {
    /* Display Memory 2 */
    //g_hal->log(LOG_MEMORY, "Display Memory 2 - ");
    //res = memory[n - MEM_DISPLAY2_ADDR_OFS];
      res = 0;  
  } else if (n >= MEM_IO_ADDR && n < (MEM_IO_ADDR + MEM_IO_SIZE)) {
    /* I/O Memory */
    //g_hal->log(LOG_MEMORY, "I/O              - ");
    res = get_io(n);
  } else {
    //g_hal->log(LOG_ERROR,   "Read from invalid memory address 0x%03X - PC = 0x%04X\n", n, pc);
    return 0;
  }

  //g_hal->log(LOG_MEMORY, "Read  0x%X - Address 0x%03X - PC = 0x%04X\n", res, n, pc);

  return res;
}

static void set_memory(u12_t n, u4_t v)
{
  if (n < MEM_RAM_SIZE) {
    /* RAM */
    //g_hal->log(LOG_MEMORY, "RAM              - ");
    if ((n & 0x1)==0) {
      memory[n>>1] = (memory[n>>1] & 0x0F) | (v << 4);
    } else {
      memory[n>>1] = (memory[n>>1] & 0xF0) | v;
    }
    //memory[n] = v;
  } else if (n >= MEM_DISPLAY1_ADDR && n < (MEM_DISPLAY1_ADDR + MEM_DISPLAY1_SIZE)) {
    /* Display Memory 1 */
    set_lcd(n, v);
    //memory[n - MEM_DISPLAY1_ADDR_OFS] = v;
    //g_hal->log(LOG_MEMORY, "Display Memory 1 - ");
  } else if (n >= MEM_DISPLAY2_ADDR && n < (MEM_DISPLAY2_ADDR + MEM_DISPLAY2_SIZE)) {
    /* Display Memory 2 */
    set_lcd(n, v);
    //memory[n - MEM_DISPLAY2_ADDR_OFS] = v;
    //g_hal->log(LOG_MEMORY, "Display Memory 2 - ");
  } else if (n >= MEM_IO_ADDR && n < (MEM_IO_ADDR + MEM_IO_SIZE)) {
    /* I/O Memory */
    set_io(n, v);
    //g_hal->log(LOG_MEMORY, "I/O              - ");
  } else {
    //g_hal->log(LOG_ERROR,   "Write 0x%X to invalid memory address 0x%03X - PC = 0x%04X\n", v, n, pc);
    return;
  }
  //g_hal->log(LOG_MEMORY, "Write 0x%X - Address 0x%03X - PC = 0x%04X\n", v, n, pc);
}
/*
void cpu_refresh_hw(void)
{
  static const struct range {
    u12_t addr;
    u12_t size;
  } refresh_locs[] = {
    { MEM_DISPLAY1_ADDR, MEM_DISPLAY1_SIZE }, // Display Memory 1 
    { MEM_DISPLAY2_ADDR, MEM_DISPLAY2_SIZE }, // Display Memory 2 
    { REG_BUZZER_CTRL1, 1 }, // Buzzer frequency 
    { REG_K40_K43_BZ_OUTPUT_PORT, 1 }, // Buzzer enabled 

    { 0, 0 }, // end of list
  };

  for (int i = 0; refresh_locs[i].size != 0; i++) {
    for (u12_t n = refresh_locs[i].addr; n < (refresh_locs[i].addr + refresh_locs[i].size); n++) {
      set_memory(n, memory[n]);
    }
  }
}*/

static u4_t get_rq(u12_t rq)
{
  switch (rq & 0x3) {
    case 0x0: return a;
    case 0x1: return b;
    case 0x2: return M(x);
    case 0x3: return M(y);
  }
  return 0;
}

static void set_rq(u12_t rq, u4_t v)
{
  switch (rq & 0x3) {
    case 0x0: a = v; break;
    case 0x1: b = v; break;
    case 0x2: SET_M(x, v); break;
    case 0x3: SET_M(y, v); break;
  }
}

/* Instructions */
static void op_pset_cb(u8_t arg0, u8_t arg1)
{
  np = arg0;
}

static void op_jp_cb(u8_t arg0, u8_t arg1)
{
  next_pc = arg0 | (np << 8);
}

static void op_jp_c_cb(u8_t arg0, u8_t arg1)
{
  if (flags & FLAG_C) {
    next_pc = arg0 | (np << 8);
  }
}

static void op_jp_nc_cb(u8_t arg0, u8_t arg1)
{
  if (!(flags & FLAG_C)) {
    next_pc = arg0 | (np << 8);
  }
}

static void op_jp_z_cb(u8_t arg0, u8_t arg1)
{
  if (flags & FLAG_Z) {
    next_pc = arg0 | (np << 8);
  }
}

static void op_jp_nz_cb(u8_t arg0, u8_t arg1)
{
  if (!(flags & FLAG_Z)) {
    next_pc = arg0 | (np << 8);
  }
}

static void op_jpba_cb(u8_t arg0, u8_t arg1)
{
  next_pc = a | (b << 4) | (np << 8);
}

static void op_call_cb(u8_t arg0, u8_t arg1)
{
  pc = (pc + 1) & 0x1FFF; // This does not actually change the PC register
  SET_M(sp - 1, PCP);
  SET_M(sp - 2, PCSH);
  SET_M(sp - 3, PCSL);
  sp = (sp - 3) & 0xFF;
  next_pc = TO_PC(PCB, NPP, arg0);
  call_depth++;
}

static void op_calz_cb(u8_t arg0, u8_t arg1)
{
  pc = (pc + 1) & 0x1FFF; // This does not actually change the PC register
  SET_M(sp - 1, PCP);
  SET_M(sp - 2, PCSH);
  SET_M(sp - 3, PCSL);
  sp = (sp - 3) & 0xFF;
  next_pc = TO_PC(PCB, 0, arg0);
  call_depth++;
}

static void op_ret_cb(u8_t arg0, u8_t arg1)
{
  next_pc = M(sp) | (M(sp + 1) << 4) | (M(sp + 2) << 8) | (PCB << 12);
  sp = (sp + 3) & 0xFF;
  call_depth--;
}

static void op_rets_cb(u8_t arg0, u8_t arg1)
{
  next_pc = M(sp) | (M(sp + 1) << 4) | (M(sp + 2) << 8) | (PCB << 12);
  sp = (sp + 3) & 0xFF;
  next_pc = (pc + 1) & 0x1FFF;
  call_depth--;
}

static void op_retd_cb(u8_t arg0, u8_t arg1)
{
  next_pc = M(sp) | (M(sp + 1) << 4) | (M(sp + 2) << 8) | (PCB << 12);
  sp = (sp + 3) & 0xFF;
  SET_M(x, arg0 & 0xF);
  SET_M(x + 1, (arg0 >> 4) & 0xF);
  x = (x + 2) & 0xFFF;
  call_depth--;
}

static void op_nop5_cb(u8_t arg0, u8_t arg1)
{
}

static void op_nop7_cb(u8_t arg0, u8_t arg1)
{
}

static void op_halt_cb(u8_t arg0, u8_t arg1)
{
  g_hal->halt();
}

static void op_inc_x_cb(u8_t arg0, u8_t arg1)
{
  x = (x + 1) & 0xFFF;
}

static void op_inc_y_cb(u8_t arg0, u8_t arg1)
{
  y = (y + 1) & 0xFFF;
}

static void op_ld_x_cb(u8_t arg0, u8_t arg1)
{
  x = arg0 | (XP << 8);
}

static void op_ld_y_cb(u8_t arg0, u8_t arg1)
{
  y = arg0 | (YP << 8);
}

static void op_ld_xp_r_cb(u8_t arg0, u8_t arg1)
{
  x = XHL | (RQ(arg0) << 8);
}

static void op_ld_xh_r_cb(u8_t arg0, u8_t arg1)
{
  x = XL1 | (RQ(arg0) << 4) | (XP << 8);
}

static void op_ld_xl_r_cb(u8_t arg0, u8_t arg1)
{
  x = RQ(arg0) | (XH1 << 4) | (XP << 8);
}

static void op_ld_yp_r_cb(u8_t arg0, u8_t arg1)
{
  y = YHL | (RQ(arg0) << 8);
}

static void op_ld_yh_r_cb(u8_t arg0, u8_t arg1)
{
  y = YL1 | (RQ(arg0) << 4) | (YP << 8);
}

static void op_ld_yl_r_cb(u8_t arg0, u8_t arg1)
{
  y = RQ(arg0) | (YH1 << 4) | (YP << 8);
}

static void op_ld_r_xp_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, XP);
}

static void op_ld_r_xh_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, XH1);
}

static void op_ld_r_xl_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, XL1);
}

static void op_ld_r_yp_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, YP);
}

static void op_ld_r_yh_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, YH1);
}

static void op_ld_r_yl_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, YL1);
}

static void op_adc_xh_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = XH1 + arg0 + C;
  x = XL1 | ((tmp & 0xF) << 4)| (XP << 8);
  if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  if (!(tmp & 0xF)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_adc_xl_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = XL1 + arg0 + C;
  x = (tmp & 0xF) | (XH1 << 4) | (XP << 8);
  if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  if (!(tmp & 0xF)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_adc_yh_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = YH1 + arg0 + C;
  y = YL1 | ((tmp & 0xF) << 4)| (YP << 8);
  if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  if (!(tmp & 0xF)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_adc_yl_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = YL1 + arg0 + C;
  y = (tmp & 0xF) | (YH1 << 4) | (YP << 8);
  if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  if (!(tmp & 0xF)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_cp_xh_cb(u8_t arg0, u8_t arg1)
{
  if (XH1 < arg0) { SET_C(); } else { CLEAR_C(); }
  if (XH1 == arg0) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_cp_xl_cb(u8_t arg0, u8_t arg1)
{
  if (XL1 < arg0) { SET_C(); } else { CLEAR_C(); }
  if (XL1 == arg0) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_cp_yh_cb(u8_t arg0, u8_t arg1)
{
  if (YH1 < arg0) { SET_C(); } else { CLEAR_C(); }
  if (YH1 == arg0) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_cp_yl_cb(u8_t arg0, u8_t arg1)
{
  if (YL1 < arg0) { SET_C(); } else { CLEAR_C(); }
  if (YL1 == arg0) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_ld_r_i_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, arg1);
}

static void op_ld_r_q_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, RQ(arg1));
}

static void op_ld_a_mn_cb(u8_t arg0, u8_t arg1)
{
  a = M(arg0);
}

static void op_ld_b_mn_cb(u8_t arg0, u8_t arg1)
{
  b = M(arg0);
}

static void op_ld_mn_a_cb(u8_t arg0, u8_t arg1)
{
  SET_M(arg0, a);
}

static void op_ld_mn_b_cb(u8_t arg0, u8_t arg1)
{
  SET_M(arg0, b);
}

static void op_ldpx_mx_cb(u8_t arg0, u8_t arg1)
{
  SET_M(x, arg0);
  x = (x + 1) & 0xFFF;
}

static void op_ldpx_r_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, RQ(arg1));
  x = (x + 1) & 0xFFF;
}

static void op_ldpy_my_cb(u8_t arg0, u8_t arg1)
{
  SET_M(y, arg0);
  y = (y + 1) & 0xFFF;
}

static void op_ldpy_r_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, RQ(arg1));
  y = (y + 1) & 0xFFF;
}

static void op_lbpx_cb(u8_t arg0, u8_t arg1)
{
  SET_M(x, arg0 & 0xF);
  SET_M(x + 1, (arg0 >> 4) & 0xF);
  x = (x + 2) & 0xFFF;
}

static void op_set_cb(u8_t arg0, u8_t arg1)
{
  flags |= arg0;
}

static void op_rst_cb(u8_t arg0, u8_t arg1)
{
  flags &= arg0;
}

static void op_scf_cb(u8_t arg0, u8_t arg1)
{
  SET_C();
}

static void op_rcf_cb(u8_t arg0, u8_t arg1)
{
  CLEAR_C();
}

static void op_szf_cb(u8_t arg0, u8_t arg1)
{
  SET_Z();
}

static void op_rzf_cb(u8_t arg0, u8_t arg1)
{
  CLEAR_Z();
}

static void op_sdf_cb(u8_t arg0, u8_t arg1)
{
  SET_D();
}

static void op_rdf_cb(u8_t arg0, u8_t arg1)
{
  CLEAR_D();
}

static void op_ei_cb(u8_t arg0, u8_t arg1)
{
  SET_I();
}

static void op_di_cb(u8_t arg0, u8_t arg1)
{
  CLEAR_I();
}

static void op_inc_sp_cb(u8_t arg0, u8_t arg1)
{
  sp = (sp + 1) & 0xFF;
}

static void op_dec_sp_cb(u8_t arg0, u8_t arg1)
{
  sp = (sp - 1) & 0xFF;
}

static void op_push_r_cb(u8_t arg0, u8_t arg1)
{
  sp = (sp - 1) & 0xFF;
  SET_M(sp, RQ(arg0));
}

static void op_push_xp_cb(u8_t arg0, u8_t arg1)
{
  sp = (sp - 1) & 0xFF;
  SET_M(sp, XP);
}

static void op_push_xh_cb(u8_t arg0, u8_t arg1)
{
  sp = (sp - 1) & 0xFF;
  SET_M(sp, XH1);
}

static void op_push_xl_cb(u8_t arg0, u8_t arg1)
{
  sp = (sp - 1) & 0xFF;
  SET_M(sp, XL1);
}

static void op_push_yp_cb(u8_t arg0, u8_t arg1)
{
  sp = (sp - 1) & 0xFF;
  SET_M(sp, YP);
}

static void op_push_yh_cb(u8_t arg0, u8_t arg1)
{
  sp = (sp - 1) & 0xFF;
  SET_M(sp, YH1);
}

static void op_push_yl_cb(u8_t arg0, u8_t arg1)
{
  sp = (sp - 1) & 0xFF;
  SET_M(sp, YL1);
}

static void op_push_f_cb(u8_t arg0, u8_t arg1)
{
  sp = (sp - 1) & 0xFF;
  SET_M(sp, flags);
}

static void op_pop_r_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, M(sp));
  sp = (sp + 1) & 0xFF;
}

static void op_pop_xp_cb(u8_t arg0, u8_t arg1)
{
  x = XL1 | (XH1 << 4)| (M(sp) << 8);
  sp = (sp + 1) & 0xFF;
}

static void op_pop_xh_cb(u8_t arg0, u8_t arg1)
{
  x = XL1 | (M(sp) << 4)| (XP << 8);
  sp = (sp + 1) & 0xFF;
}

static void op_pop_xl_cb(u8_t arg0, u8_t arg1)
{
  x = M(sp) | (XH1 << 4)| (XP << 8);
  sp = (sp + 1) & 0xFF;
}

static void op_pop_yp_cb(u8_t arg0, u8_t arg1)
{
  y = YL1 | (YH1 << 4)| (M(sp) << 8);
  sp = (sp + 1) & 0xFF;
}

static void op_pop_yh_cb(u8_t arg0, u8_t arg1)
{
  y = YL1 | (M(sp) << 4)| (YP << 8);
  sp = (sp + 1) & 0xFF;
}

static void op_pop_yl_cb(u8_t arg0, u8_t arg1)
{
  y = M(sp) | (YH1 << 4)| (YP << 8);
  sp = (sp + 1) & 0xFF;
}

static void op_pop_f_cb(u8_t arg0, u8_t arg1)
{
  flags = M(sp);
  sp = (sp + 1) & 0xFF;
}

static void op_ld_sph_r_cb(u8_t arg0, u8_t arg1)
{
  sp = SPL1 | (RQ(arg0) << 4);
}

static void op_ld_spl_r_cb(u8_t arg0, u8_t arg1)
{
  sp = RQ(arg0) | (SPH1 << 4);
}

static void op_ld_r_sph_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, SPH1);
}

static void op_ld_r_spl_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, SPL1);
}

static void op_add_r_i_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = RQ(arg0) + arg1;
  if (D) {
    if (tmp >= 10) {
      SET_RQ(arg0, (tmp - 10) & 0xF);
      SET_C();
    } else {
      SET_RQ(arg0, tmp);
      CLEAR_C();
    }
  } else {
    SET_RQ(arg0, tmp & 0xF);
    if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  }
  if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_add_r_q_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = RQ(arg0) + RQ(arg1);
  if (D) {
    if (tmp >= 10) {
      SET_RQ(arg0, (tmp - 10) & 0xF);
      SET_C();
    } else {
      SET_RQ(arg0, tmp);
      CLEAR_C();
    }
  } else {
    SET_RQ(arg0, tmp & 0xF);
    if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  }
  if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_adc_r_i_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = RQ(arg0) + arg1 + C;
  if (D) {
    if (tmp >= 10) {
      SET_RQ(arg0, (tmp - 10) & 0xF);
      SET_C();
    } else {
      SET_RQ(arg0, tmp);
      CLEAR_C();
    }
  } else {
    SET_RQ(arg0, tmp & 0xF);
    if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  }
  if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_adc_r_q_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = RQ(arg0) + RQ(arg1) + C;
  if (D) {
    if (tmp >= 10) {
      SET_RQ(arg0, (tmp - 10) & 0xF);
      SET_C();
    } else {
      SET_RQ(arg0, tmp);
      CLEAR_C();
    }
  } else {
    SET_RQ(arg0, tmp & 0xF);
    if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  }
  if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_sub_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = RQ(arg0) - RQ(arg1);
  if (D) {
    if (tmp >> 4) {
      SET_RQ(arg0, (tmp - 6) & 0xF);
    } else {
      SET_RQ(arg0, tmp);
    }
  } else {
    SET_RQ(arg0, tmp & 0xF);
  }
  if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_sbc_r_i_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = RQ(arg0) - arg1 - C;
  if (D) {
    if (tmp >> 4) {
      SET_RQ(arg0, (tmp - 6) & 0xF);
    } else {
      SET_RQ(arg0, tmp);
    }
  } else {
    SET_RQ(arg0, tmp & 0xF);
  }
  if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_sbc_r_q_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = RQ(arg0) - RQ(arg1) - C;
  if (D) {
    if (tmp >> 4) {
      SET_RQ(arg0, (tmp - 6) & 0xF);
    } else {
      SET_RQ(arg0, tmp);
    }
  } else {
    SET_RQ(arg0, tmp & 0xF);
  }
  if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_and_r_i_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, RQ(arg0) & arg1);
  if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_and_r_q_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, RQ(arg0) & RQ(arg1));
  if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_or_r_i_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, RQ(arg0) | arg1);
  if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_or_r_q_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, RQ(arg0) | RQ(arg1));
  if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_xor_r_i_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, RQ(arg0) ^ arg1);
  if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_xor_r_q_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, RQ(arg0) ^ RQ(arg1));
  if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_cp_r_i_cb(u8_t arg0, u8_t arg1)
{
  if (RQ(arg0) < arg1) { SET_C(); } else { CLEAR_C(); }
  if (RQ(arg0) == arg1) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_cp_r_q_cb(u8_t arg0, u8_t arg1)
{
  if (RQ(arg0) < RQ(arg1)) { SET_C(); } else { CLEAR_C(); }
  if (RQ(arg0) == RQ(arg1)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_fan_r_i_cb(u8_t arg0, u8_t arg1)
{
  if (!(RQ(arg0) & arg1)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_fan_r_q_cb(u8_t arg0, u8_t arg1)
{
  if (!(RQ(arg0) & RQ(arg1))) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_rlc_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = (RQ(arg0) << 1) | C;
  if (RQ(arg0) & 0x8) { SET_C(); } else { CLEAR_C(); }
  SET_RQ(arg0, tmp & 0xF);
  /* No need to set Z (issue in DS) */
}

static void op_rrc_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = (RQ(arg0) >> 1) | (C << 3);
  if (RQ(arg0) & 0x1) { SET_C(); } else { CLEAR_C(); }
  SET_RQ(arg0, tmp & 0xF);
  /* No need to set Z (issue in DS) */
}

static void op_inc_mn_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = M(arg0) + 1;
  SET_M(arg0, tmp & 0xF);
  if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  if (!M(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_dec_mn_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = M(arg0) - 1;
  SET_M(arg0, tmp & 0xF);
  if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  if (!M(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static void op_acpx_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = M(x) + RQ(arg0) + C;
  if (D) {
    if (tmp >= 10) {
      SET_M(x, (tmp - 10) & 0xF);
      SET_C();
    } else {
      SET_M(x, tmp);
      CLEAR_C();
    }
  } else {
    SET_M(x, tmp & 0xF);
    if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  }
  if (!M(x)) { SET_Z(); } else { CLEAR_Z(); }
  x = (x + 1) & 0xFFF;
}

static void op_acpy_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = M(y) + RQ(arg0) + C;
  if (D) {
    if (tmp >= 10) {
      SET_M(y, (tmp - 10) & 0xF);
      SET_C();
    } else {
      SET_M(y, tmp);
      CLEAR_C();
    }
  } else {
    SET_M(y, tmp & 0xF);
    if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  }
  if (!M(y)) { SET_Z(); } else { CLEAR_Z(); }
  y = (y + 1) & 0xFFF;
}

static void op_scpx_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = M(x) - RQ(arg0) - C;
  if (D) {
    if (tmp >> 4) {
      SET_M(x, (tmp - 6) & 0xF);
    } else {
      SET_M(x, tmp);
    }
  } else {
    SET_M(x, tmp & 0xF);
  }
  if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  if (!M(x)) { SET_Z(); } else { CLEAR_Z(); }
  x = (x + 1) & 0xFFF;
}

static void op_scpy_cb(u8_t arg0, u8_t arg1)
{
  u8_t tmp;

  tmp = M(y) - RQ(arg0) - C;
  if (D) {
    if (tmp >> 4) {
      SET_M(y, (tmp - 6) & 0xF);
    } else {
      SET_M(y, tmp);
    }
  } else {
    SET_M(y, tmp & 0xF);
  }
  if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
  if (!M(y)) { SET_Z(); } else { CLEAR_Z(); }
  y = (y + 1) & 0xFFF;
}

static void op_not_cb(u8_t arg0, u8_t arg1)
{
  SET_RQ(arg0, ~RQ(arg0) & 0xF);
  if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

/* The E0C6S46 supported instructions */
static const op_t0 ops0[] PROGMEM = {
  {0xE40, MASK_7B       , 5 }, // PSET
  {0x000, MASK_4B       , 5 }, // JP
  {0x200, MASK_4B       , 5 }, // JP_C
  {0x300, MASK_4B       , 5 }, // JP_NC
  {0x600, MASK_4B       , 5 }, // JP_Z
  {0x700, MASK_4B       , 5 }, // JP_NZ
  {0xFE8, MASK_12B      , 5 }, // JPBA
  {0x400, MASK_4B       , 7 }, // CALL
  {0x500, MASK_4B       , 7 }, // CALZ
  {0xFDF, MASK_12B      , 7 }, // RET
  {0xFDE, MASK_12B      , 12}, // RETS
  {0x100, MASK_4B       , 12}, // RETD
  {0xFFB, MASK_12B      , 5 }, // NOP5
  {0xFFF, MASK_12B      , 7 }, // NOP7
  {0xFF8, MASK_12B      , 5 }, // HALT
  {0xEE0, MASK_12B      , 5 }, // INC_X
  {0xEF0, MASK_12B      , 5 }, // INC_Y
  {0xB00, MASK_4B       , 5 }, // LD_X
  {0x800, MASK_4B       , 5 }, // LD_Y
  {0xE80, MASK_10B      , 5 }, // LD_XP_R
  {0xE84, MASK_10B      , 5 }, // LD_XH_R
  {0xE88, MASK_10B      , 5 }, // LD_XL_R
  {0xE90, MASK_10B      , 5 }, // LD_YP_R
  {0xE94, MASK_10B      , 5 }, // LD_YH_R
  {0xE98, MASK_10B      , 5 }, // LD_YL_R
  {0xEA0, MASK_10B      , 5 }, // LD_R_XP
  {0xEA4, MASK_10B      , 5 }, // LD_R_XH
  {0xEA8, MASK_10B      , 5 }, // LD_R_XL
  {0xEB0, MASK_10B      , 5 }, // LD_R_YP
  {0xEB4, MASK_10B      , 5 }, // LD_R_YH
  {0xEB8, MASK_10B      , 5 }, // LD_R_YL
  {0xA00, MASK_8B       , 7 }, // ADC_XH
  {0xA10, MASK_8B       , 7 }, // ADC_XL
  {0xA20, MASK_8B       , 7 }, // ADC_YH
  {0xA30, MASK_8B       , 7 }, // ADC_YL
  {0xA40, MASK_8B       , 7 }, // CP_XH
  {0xA50, MASK_8B       , 7 }, // CP_XL
  {0xA60, MASK_8B       , 7 }, // CP_YH
  {0xA70, MASK_8B       , 7 }, // CP_YL
  {0xFA0, MASK_8B       , 5 }, // LD_A_MN
  {0xFB0, MASK_8B       , 5 }, // LD_B_MN
  {0xF80, MASK_8B       , 5 }, // LD_MN_A
  {0xF90, MASK_8B       , 5 }, // LD_MN_B
  {0xE60, MASK_8B       , 5 }, // LDPX_MX
  {0xE70, MASK_8B       , 5 }, // LDPY_MY
  {0x900, MASK_4B       , 5 }, // LBPX
  {0xF40, MASK_8B       , 7 }, // SET
  {0xF50, MASK_8B       , 7 }, // RST
  {0xF41, MASK_12B      , 7 }, // SCF
  {0xF5E, MASK_12B      , 7 }, // RCF
  {0xF42, MASK_12B      , 7 }, // SZF
  {0xF5D, MASK_12B      , 7 }, // RZF
  {0xF44, MASK_12B      , 7 }, // SDF
  {0xF5B, MASK_12B      , 7 }, // RDF
  {0xF48, MASK_12B      , 7 }, // EI
  {0xF57, MASK_12B      , 7 }, // DI
  {0xFDB, MASK_12B      , 5 }, // INC_SP
  {0xFCB, MASK_12B      , 5 }, // DEC_SP
  {0xFC0, MASK_10B      , 5 }, // PUSH_R
  {0xFC4, MASK_12B      , 5 }, // PUSH_XP
  {0xFC5, MASK_12B      , 5 }, // PUSH_XH
  {0xFC6, MASK_12B      , 5 }, // PUSH_XL
  {0xFC7, MASK_12B      , 5 }, // PUSH_YP
  {0xFC8, MASK_12B      , 5 }, // PUSH_YH
  {0xFC9, MASK_12B      , 5 }, // PUSH_YL
  {0xFCA, MASK_12B      , 5 }, // PUSH_F
  {0xFD0, MASK_10B      , 5 }, // POP_R
  {0xFD4, MASK_12B      , 5 }, // POP_XP
  {0xFD5, MASK_12B      , 5 }, // POP_XH
  {0xFD6, MASK_12B      , 5 }, // POP_XL
  {0xFD7, MASK_12B      , 5 }, // POP_YP
  {0xFD8, MASK_12B      , 5 }, // POP_YH
  {0xFD9, MASK_12B      , 5 }, // POP_YL
  {0xFDA, MASK_12B      , 5 }, // POP_F
  {0xFE0, MASK_10B      , 5 }, // LD_SPH_R
  {0xFF0, MASK_10B      , 5 }, // LD_SPL_R
  {0xFE4, MASK_10B      , 5 }, // LD_R_SPH
  {0xFF4, MASK_10B      , 5 }, // LD_R_SPL
  {0xC00, MASK_6B   , 7 }, // ADD_R_I
  {0xC40, MASK_6B   , 7 }, // ADC_R_I
  {0xB40, MASK_6B   , 7 }, // SBC_R_I
  {0xC80, MASK_6B   , 7 }, // AND_R_I
  {0xCC0, MASK_6B   , 7 }, // OR_R_I
  {0xD00, MASK_6B   , 7 }, // XOR_R_I
  {0xDC0, MASK_6B   , 7 }, // CP_R_I
  {0xD80, MASK_6B   , 7 }, // FAN_R_I
  {0xE00, MASK_6B   , 5 }, // LD_R_I
  {0xA80, MASK_8B   , 7 }, // ADD_R_Q
  {0xA90, MASK_8B   , 7 }, // ADC_R_Q
  {0xAA0, MASK_8B   , 7 }, // SUB
  {0xAB0, MASK_8B   , 7 }, // SBC_R_Q
  {0xAC0, MASK_8B   , 7 }, // AND_R_Q
  {0xAD0, MASK_8B   , 7 }, // OR_R_Q
  {0xAE0, MASK_8B   , 7 }, // XOR_R_Q
  {0xEC0, MASK_8B   , 5 }, // LD_R_Q
  {0xEE0, MASK_8B   , 5 }, // LDPX_R
  {0xEF0, MASK_8B   , 5 }, // LDPY_R
  {0xF00, MASK_8B   , 7 }, // CP_R_Q
  {0xF10, MASK_8B   , 7 }, // FAN_R_Q
  
  {0xAF0, MASK_8B       , 7 }, // RLC
  {0xE8C, MASK_10B      , 5 }, // RRC
  {0xF60, MASK_8B       , 7 }, // INC_MN
  {0xF70, MASK_8B       , 7 }, // DEC_MN
  {0xF28, MASK_10B      , 7 }, // ACPX
  {0xF2C, MASK_10B      , 7 }, // ACPY
  {0xF38, MASK_10B      , 7 }, // SCPX
  {0xF3C, MASK_10B      , 7 }, // SCPY
  {0xD0F, 0xFCF         , 7 }, // NOT
  {NULL, 0, 0, 0, NULL},
};

/* The E0C6S46 supported instructions */
static const op_t1 ops1[] PROGMEM = {
  {&op_pset_cb}, // PSET
  {&op_jp_cb}, // JP
  {&op_jp_c_cb}, // JP_C
  {&op_jp_nc_cb}, // JP_NC
  {&op_jp_z_cb}, // JP_Z
  {&op_jp_nz_cb}, // JP_NZ
  {&op_jpba_cb}, // JPBA
  {&op_call_cb}, // CALL
  {&op_calz_cb}, // CALZ
  {&op_ret_cb}, // RET
  {&op_rets_cb}, // RETS
  {&op_retd_cb}, // RETD
  {&op_nop5_cb}, // NOP5
  {&op_nop7_cb}, // NOP7
  {&op_halt_cb}, // HALT
  {&op_inc_x_cb}, // INC_X
  {&op_inc_y_cb}, // INC_Y
  {&op_ld_x_cb}, // LD_X
  {&op_ld_y_cb}, // LD_Y
  {&op_ld_xp_r_cb}, // LD_XP_R
  {&op_ld_xh_r_cb}, // LD_XH_R
  {&op_ld_xl_r_cb}, // LD_XL_R
  {&op_ld_yp_r_cb}, // LD_YP_R
  {&op_ld_yh_r_cb}, // LD_YH_R
  {&op_ld_yl_r_cb}, // LD_YL_R
  {&op_ld_r_xp_cb}, // LD_R_XP
  {&op_ld_r_xh_cb}, // LD_R_XH
  {&op_ld_r_xl_cb}, // LD_R_XL
  {&op_ld_r_yp_cb}, // LD_R_YP
  {&op_ld_r_yh_cb}, // LD_R_YH
  {&op_ld_r_yl_cb}, // LD_R_YL
  {&op_adc_xh_cb}, // ADC_XH
  {&op_adc_xl_cb}, // ADC_XL
  {&op_adc_yh_cb}, // ADC_YH
  {&op_adc_yl_cb}, // ADC_YL
  {&op_cp_xh_cb}, // CP_XH
  {&op_cp_xl_cb}, // CP_XL
  {&op_cp_yh_cb}, // CP_YH
  {&op_cp_yl_cb}, // CP_YL
  {&op_ld_a_mn_cb}, // LD_A_MN
  {&op_ld_b_mn_cb}, // LD_B_MN
  {&op_ld_mn_a_cb}, // LD_MN_A
  {&op_ld_mn_b_cb}, // LD_MN_B
  {&op_ldpx_mx_cb}, // LDPX_MX
  {&op_ldpy_my_cb}, // LDPY_MY
  {&op_lbpx_cb}, // LBPX
  {&op_set_cb}, // SET
  {&op_rst_cb}, // RST
  {&op_scf_cb}, // SCF
  {&op_rcf_cb}, // RCF
  {&op_szf_cb}, // SZF
  {&op_rzf_cb}, // RZF
  {&op_sdf_cb}, // SDF
  {&op_rdf_cb}, // RDF
  {&op_ei_cb}, // EI
  {&op_di_cb}, // DI
  {&op_inc_sp_cb}, // INC_SP
  {&op_dec_sp_cb}, // DEC_SP
  {&op_push_r_cb}, // PUSH_R
  {&op_push_xp_cb}, // PUSH_XP
  {&op_push_xh_cb}, // PUSH_XH
  {&op_push_xl_cb}, // PUSH_XL
  {&op_push_yp_cb}, // PUSH_YP
  {&op_push_yh_cb}, // PUSH_YH
  {&op_push_yl_cb}, // PUSH_YL
  {&op_push_f_cb}, // PUSH_F
  {&op_pop_r_cb}, // POP_R
  {&op_pop_xp_cb}, // POP_XP
  {&op_pop_xh_cb}, // POP_XH
  {&op_pop_xl_cb}, // POP_XL
  {&op_pop_yp_cb}, // POP_YP
  {&op_pop_yh_cb}, // POP_YH
  {&op_pop_yl_cb}, // POP_YL
  {&op_pop_f_cb}, // POP_F
  {&op_ld_sph_r_cb}, // LD_SPH_R
  {&op_ld_spl_r_cb}, // LD_SPL_R
  {&op_ld_r_sph_cb}, // LD_R_SPH
  {&op_ld_r_spl_cb}, // LD_R_SPL
  {&op_add_r_i_cb}, // ADD_R_I
  {&op_adc_r_i_cb}, // ADC_R_I
  {&op_sbc_r_i_cb}, // SBC_R_I
  {&op_and_r_i_cb}, // AND_R_I
  {&op_or_r_i_cb}, // OR_R_I
  {&op_xor_r_i_cb}, // XOR_R_I
  {&op_cp_r_i_cb}, // CP_R_I
  {&op_fan_r_i_cb}, // FAN_R_I
  {&op_ld_r_i_cb}, // LD_R_I
  {&op_add_r_q_cb}, // ADD_R_Q
  {&op_adc_r_q_cb}, // ADC_R_Q
  {&op_sub_cb}, // SUB
  {&op_sbc_r_q_cb}, // SBC_R_Q
  {&op_and_r_q_cb}, // AND_R_Q
  {&op_or_r_q_cb}, // OR_R_Q
  {&op_xor_r_q_cb}, // XOR_R_Q
  {&op_ld_r_q_cb}, // LD_R_Q
  {&op_ldpx_r_cb}, // LDPX_R
  {&op_ldpy_r_cb}, // LDPY_R
  {&op_cp_r_q_cb}, // CP_R_Q
  {&op_fan_r_q_cb}, // FAN_R_Q
  
  {&op_rlc_cb}, // RLC
  {&op_rrc_cb}, // RRC
  {&op_inc_mn_cb}, // INC_MN
  {&op_dec_mn_cb}, // DEC_MN
  {&op_acpx_cb}, // ACPX
  {&op_acpy_cb}, // ACPY
  {&op_scpx_cb}, // SCPX
  {&op_scpy_cb}, // SCPY
  {&op_not_cb}, // NOT
  {NULL}
};
  
u12_t getShiftArg0(u12_t code, u12_t mask) {
  if (mask==MASK_6B || mask==0xFCF) return 4;
  if (code==0xA80 || code==0xA90 || code==0xAA0  || code==0xAB0 || code==0xAC0 || code==0xAD0 || code==0xAE0 || code==0xEC0 || code==0xEE0 || code==0xEF0 || code==0xF00 || code==0xF10) return 2;
  return 0;  
}

u12_t getMaskArg0(u12_t shiftArg, u12_t mask) {
  if (mask==MASK_6B) return 0x030;
  if (mask==MASK_8B && shiftArg==2) return 0x00C;
  return 0;
}

static timestamp_t wait_for_cycles(timestamp_t since, u8_t cycles) {
  timestamp_t deadline;

  tick_counter += cycles;

  if (CPU_SPEED_RATIO == 0) {
    /* Emulation will be as fast as possible */
    return g_hal->get_timestamp();
  }

  deadline = since + (cycles * ts_freq)/(TICK_FREQUENCY * CPU_SPEED_RATIO);
  g_hal->sleep_until(deadline);

  return deadline;
}

static void process_interrupts(void)
{
  u8_t i;

  /* Process interrupts in priority order */
  for (i = 0; i < INT_SLOT_NUM; i++) {
    if (interrupts[i].triggered) {
      //printf("IT %u !\n", i);
      SET_M(sp - 1, PCP);
      SET_M(sp - 2, PCSH);
      SET_M(sp - 3, PCSL);
      sp = (sp - 3) & 0xFF;
      CLEAR_I();
      np = TO_NP(NBP, 1);
      pc = TO_PC(PCB, 1, interrupts[i].vector);
      call_depth++;

      ref_ts = wait_for_cycles(ref_ts, 12);
      interrupts[i].triggered = 0;
    }
  }
}

static void print_state(u8_t op_num, u12_t op, u13_t addr)
{
}

//static char logMsg[40];

void cpu_reset(void)
{
  u13_t i;

  /* Registers and variables init */
  pc = TO_PC(0, 1, 0x00); // PC starts at bank 0, page 1, step 0
  np = TO_NP(0, 1); // NP starts at page 1
  a = 0; // undef
  b = 0; // undef
  x = 0; // undef
  y = 0; // undef
  sp = 0; // undef
  flags = 0;

  //sprintf(logMsg, "Start pc 1:0x%04X, %d", pc, pc); g_hal->log(LOG_ERROR, logMsg);

  /* Init RAM to zeros */
  for (i = 0; i < MEMORY_SIZE; i++) {
    memory[i] = 0;
  }
  /*for (i = 0; i < MEM_IO_SIZE; i++) {
    io_memory[i] = 0;
  } */ 

  //io_memory[REG_K40_K43_BZ_OUTPUT_PORT - MEM_IO_ADDR_OFS] = 0xF; // Output port (R40-R43)
  //io_memory[REG_LCD_CTRL - MEM_IO_ADDR_OFS] = 0x8; // LCD control
  /* TODO: Input relation register */

  cpu_sync_ref_timestamp();
}

bool_t cpu_init(u32_t freq)
{
  //g_program = program;
  //g_breakpoints = breakpoints;
  ts_freq = freq;
  cpu_reset();
  return 0;
}

void cpu_release(void)
{
}

u12_t getProgramOpCode(u12_t pc) {
  u12_t i = pc >> 1;  // divided by 2
  if ((pc & 0x1)==0) {   // if pc is a even number
    return (pgm_read_byte_near(g_program_b12+i+i+i) << 4) | ((pgm_read_byte_near(g_program_b12+i+i+i+1) >> 4) & 0xF);
  } 
  return (pgm_read_byte_near(g_program_b12+i+i+i+1) << 8) | pgm_read_byte_near(g_program_b12+i+i+i+2);
}

/*
typedef struct {
  u12_t code;
  u12_t mask;
  u8_t cycles;
} op_t0;
*/

int cpu_step(void)
{
  u12_t op;
  u8_t i;
  //breakpoint_t *bp = g_breakpoints;
  static u8_t previous_cycles = 0;

  op = getProgramOpCode(pc);

  //op_t0 *ops = (op_t0 *)pgm_read_ptr_near(ops0);

  /* Lookup the OP code */
  for (i = 0; pgm_read_byte_near(&ops0[i].cycles) != 0; i++) {
    if ((op & pgm_read_word_near(&ops0[i].mask)) == pgm_read_word_near(&ops0[i].code)) {
      break;
    }
  }

 //sprintf(logMsg, "op-code 0x%X (pc = 0x%04X)", op, pc); g_hal->log(LOG_ERROR, logMsg);

  if (pgm_read_byte_near(&ops0[i].cycles) == 0) {
    //printf(logMsg, "Unknown op-code 0x%X (pc = 0x%04X)\n", op, pc); g_hal->log(LOG_ERROR, logMsg);
    return 1;
  }

  op_t0 ops;
  ops.code=pgm_read_word_near(&ops0[i].code);
  ops.mask=pgm_read_word_near(&ops0[i].mask);
  ops.cycles=pgm_read_byte_near(&ops0[i].cycles);

  next_pc = (pc + 1) & 0x1FFF;

  /* Display the operation along with the current state of the processor */
  print_state(i, op, pc);

  /* Match the speed of the real processor
   * NOTE: For better accuracy, the final wait should happen here, however
   * the downside is that all interrupts will likely be delayed by one OP
   */
  ref_ts = wait_for_cycles(ref_ts, previous_cycles);

  op_t1 ops11;
  ops11.cb1 = pgm_read_ptr_near(&ops1[i].cb1);

  /* Process the OP code */
  if (ops11.cb1 != NULL) {
    u12_t shiftArg0 = getShiftArg0(ops.code,ops.mask);
    u12_t maskArg0 = getMaskArg0(shiftArg0,ops.mask);
    if (maskArg0 != 0) {
      /* Two arguments */
      //ops[i].cb((op & ops[i].mask_arg0) >> ops[i].shift_arg0, op & ~(ops[i].mask | ops[i].mask_arg0));
      ops11.cb1((op & maskArg0) >> shiftArg0, op & ~(ops.mask | maskArg0));
    } else {
      /* One arguments */
      //ops[i].cb((op & ~ops[i].mask) >> ops[i].shift_arg0, 0);
      ops11.cb1((op & ~ops.mask) >> shiftArg0, 0);
    }
  }

  /* Prepare for the next instruction */
  pc = next_pc;
  previous_cycles = ops.cycles;

  if (i > 0) {
    /* OP code is not PSET, reset NP */
    np = (pc >> 8) & 0x1F;
  }

  /* Handle timers using the internal tick counter */
  if (tick_counter - clk_timer_timestamp >= TIMER_1HZ_PERIOD) {
    do {
      clk_timer_timestamp += TIMER_1HZ_PERIOD;
    } while (tick_counter - clk_timer_timestamp >= TIMER_1HZ_PERIOD);

    generate_interrupt(INT_CLOCK_TIMER_SLOT, 3);
  }

  if (prog_timer_enabled && tick_counter - prog_timer_timestamp >= TIMER_256HZ_PERIOD) {
    do {
      prog_timer_timestamp += TIMER_256HZ_PERIOD;
      prog_timer_data--;

      if (prog_timer_data == 0) {
        prog_timer_data = prog_timer_rld;
        generate_interrupt(INT_PROG_TIMER_SLOT, 0);
      }
    } while (tick_counter - prog_timer_timestamp >= TIMER_256HZ_PERIOD);
  }

  /* Check if there is any pending interrupt */
  if (I && i > 0) { // Do not process interrupts after a PSET operation
    process_interrupts();
  }

  /* Check if we could pause the execution */
  /*while (bp != NULL) {
    if (bp->addr == pc) {
      return 1;
    }

    bp = bp->next;
  } */

  return 0;
}
