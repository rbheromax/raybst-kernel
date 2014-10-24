/*
 * sec_debug.h
 *
 * header file supporting debug functions for Samsung device
 *
 * COPYRIGHT(C) Samsung Electronics Co., Ltd. 2006-2011 All Right Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#ifndef SEC_DEBUG_H
#define SEC_DEBUG_H

#include <linux/sched.h>
#include <linux/semaphore.h>

#define NORMAL_REBOOT   0x776655AA
#define RECOVERY_MODE   0x77665502
#define FASTBOOT_MODE   0x77665500
#define SECDEBUG_UNKNOWN_RESET_MODE   0x776655D0
#define SECDEBUG_KERNEL_PANIC_MODE    0x776655D1
#define SECDEBUG_FORCED_UPLOAD_MODE   0x776655D2
#define SECDEBUG_MODEM_UPLOAD_MODE   0x776655D3

extern void set_restart_reason(unsigned);

#ifdef CONFIG_SEC_DEBUG
extern int sec_debug_init(void);
extern int sec_debug_dump_stack(void);
extern void sec_debug_hw_reset(void);
extern void sec_debug_check_crash_key(unsigned int code, int value);
extern void sec_getlog_supply_fbinfo(void *p_fb, u32 res_x, u32 res_y, u32 bpp,
		u32 frames);
extern void sec_getlog_supply_meminfo(u32 size0, u32 addr0, u32 size1,
		u32 addr1);
extern void sec_getlog_supply_loggerinfo(void *p_main, void *p_radio,
		void *p_events, void *p_system);
extern void sec_getlog_supply_kloginfo(void *klog_buf);
extern void sec_gaf_supply_rqinfo(unsigned short curr_offset,
				  unsigned short rq_offset);
extern int sec_debug_is_enabled(void);
#else
static inline int sec_debug_init(void)
{
}
static inline int sec_debug_dump_stack(void) {}
static inline void sec_debug_hw_reset(void) {}
static inline void sec_debug_check_crash_key(unsigned int code, int value) {}

static inline void sec_getlog_supply_fbinfo(void *p_fb, u32 res_x, u32 res_y,
					    u32 bpp, u32 frames)
{
}
static inline void sec_getlog_supply_meminfo(u32 size0, u32 addr0, u32 size1,
					     u32 addr1)
{
}

static inline void sec_getlog_supply_loggerinfo(void *p_main,
						void *p_radio, void *p_events,
						void *p_system)
{
}

static inline void sec_getlog_supply_kloginfo(void *klog_buf)
{
}

static inline void sec_gaf_supply_rqinfo(unsigned short curr_offset,
					 unsigned short rq_offset)
{
}

static inline int sec_debug_is_enabled(void) {return 0; }
#endif

#ifdef CONFIG_SEC_DEBUG_SCHED_LOG
extern void sec_debug_task_sched_log_short_msg(char *msg);
extern void sec_debug_task_sched_log(int cpu, struct task_struct *task);
extern void sec_debug_irq_sched_log(unsigned int irq, void *fn, int en);
extern void sec_debug_irq_sched_log_end(void);
extern void sec_debug_timer_log(unsigned int type, int int_lock, void *fn);
extern void sec_debug_sched_log_init(void);
#define secdbg_sched_msg(fmt, ...) \
	do { \
		char ___buf[16]; \
		snprintf(___buf, sizeof(___buf), fmt, ##__VA_ARGS__); \
		sec_debug_task_sched_log_short_msg(___buf); \
	} while (0)
#else
static inline void sec_debug_task_sched_log(int cpu, struct task_struct *task)
{
}
static inline void sec_debug_irq_sched_log(unsigned int irq, void *fn, int en)
{
}
static inline void sec_debug_irq_sched_log_end(void)
{
}
static inline void sec_debug_timer_log(unsigned int type,
						int int_lock, void *fn)
{
}
static inline void sec_debug_sched_log_init(void)
{
}
#define secdbg_sched_msg(fmt, ...)
#endif
#ifdef CONFIG_SEC_DEBUG_IRQ_EXIT_LOG
extern void sec_debug_irq_enterexit_log(unsigned int irq,
						unsigned long long start_time);
#else
static inline void sec_debug_irq_enterexit_log(unsigned int irq,
						unsigned long long start_time)
{
}
#endif

#ifdef CONFIG_SEC_DEBUG_SCHED_LOG
#define SCHED_LOG_MAX 1024

struct irq_log {
	unsigned long long time;
	int irq;
	void *fn;
	int en;
	int preempt_count;
	void *context;
};

struct irq_exit_log {
	unsigned int irq;
	unsigned long long time;
	unsigned long long end_time;
	unsigned long long elapsed_time;
};

struct sched_log {
	unsigned long long time;
	char comm[TASK_COMM_LEN];
	pid_t pid;
};


struct timer_log {
	unsigned long long time;
	unsigned int type;
	int int_lock;
	void *fn;
};
#endif	/* CONFIG_SEC_DEBUG_SCHED_LOG */

/* for sec debug level */
#define KERNEL_SEC_DEBUG_LEVEL_LOW	(0x574F4C44)
#define KERNEL_SEC_DEBUG_LEVEL_MID	(0x44494D44)
#define KERNEL_SEC_DEBUG_LEVEL_HIGH	(0x47494844)
extern bool kernel_sec_set_debug_level(int level);
extern int kernel_sec_get_debug_level(void);

extern struct class *sec_class;

__weak void dump_all_task_info(void);
__weak void dump_cpu_stat(void);

#ifdef CONFIG_SEC_DEBUG_SUBSYS
extern void sec_debug_subsys_fill_fbinfo(int idx, void *fb, u32 xres,
				u32 yres, u32 bpp, u32 color_mode);

#define SEC_DEBUG_SUBSYS_MAGIC0 0xFFFFFFFF
#define SEC_DEBUG_SUBSYS_MAGIC1 0x5ECDEB6
#define SEC_DEBUG_SUBSYS_MAGIC2 0x14F014F0
 /* high word : major version
  * low word : minor version
  * minor version changes should not affect LK behavior
  */
#define SEC_DEBUG_SUBSYS_MAGIC3 0x00010002


#define TZBSP_CPU_COUNT           2
/* CPU context for the monitor. */
struct tzbsp_dump_cpu_ctx_s {
	unsigned int mon_lr;
	unsigned int mon_spsr;
	unsigned int usr_r0;
	unsigned int usr_r1;
	unsigned int usr_r2;
	unsigned int usr_r3;
	unsigned int usr_r4;
	unsigned int usr_r5;
	unsigned int usr_r6;
	unsigned int usr_r7;
	unsigned int usr_r8;
	unsigned int usr_r9;
	unsigned int usr_r10;
	unsigned int usr_r11;
	unsigned int usr_r12;
	unsigned int usr_r13;
	unsigned int usr_r14;
	unsigned int irq_spsr;
	unsigned int irq_r13;
	unsigned int irq_r14;
	unsigned int svc_spsr;
	unsigned int svc_r13;
	unsigned int svc_r14;
	unsigned int abt_spsr;
	unsigned int abt_r13;
	unsigned int abt_r14;
	unsigned int und_spsr;
	unsigned int und_r13;
	unsigned int und_r14;
	unsigned int fiq_spsr;
	unsigned int fiq_r8;
	unsigned int fiq_r9;
	unsigned int fiq_r10;
	unsigned int fiq_r11;
	unsigned int fiq_r12;
	unsigned int fiq_r13;
	unsigned int fiq_r14;
};

struct tzbsp_dump_buf_s {
	unsigned int sc_status[TZBSP_CPU_COUNT];
	struct tzbsp_dump_cpu_ctx_s sc_ns[TZBSP_CPU_COUNT];
	struct tzbsp_dump_cpu_ctx_s sec;
};

struct core_reg_info {
	char name[12];
	unsigned int value;
};

struct sec_debug_subsys_excp {
	char type[16];
	char task[16];
	char file[32];
	int line;
	char msg[256];
	struct core_reg_info core_reg[64];
};

struct sec_debug_subsys_excp_scorpion {
	char pc_sym[64];
	char lr_sym[64];
	char panic_caller[64];
	char panic_msg[128];
	char thread[32];
};

struct sec_debug_subsys_log {
	unsigned int idx_paddr;
	unsigned int log_paddr;
	unsigned int size;
};

struct rgb_bit_info {
	unsigned char r_off;
	unsigned char r_len;
	unsigned char g_off;
	unsigned char g_len;
	unsigned char b_off;
	unsigned char b_len;
	unsigned char a_off;
	unsigned char a_len;
};

struct var_info {
	char name[16];
	int sizeof_type;
	unsigned int var_paddr;
};
struct sec_debug_subsys_simple_var_mon {
	int idx;
	struct var_info var[32];
};

struct sec_debug_subsys_fb {
	unsigned int fb_paddr;
	int xres;
	int yres;
	int bpp;
	struct rgb_bit_info rgb_bitinfo;
};

struct sec_debug_subsys_sched_log {
	unsigned int sched_idx_paddr;
	unsigned int sched_buf_paddr;
	unsigned int sched_struct_sz;
	unsigned int sched_array_cnt;
	unsigned int irq_idx_paddr;
	unsigned int irq_buf_paddr;
	unsigned int irq_struct_sz;
	unsigned int irq_array_cnt;
	unsigned int irq_exit_idx_paddr;
	unsigned int irq_exit_buf_paddr;
	unsigned int irq_exit_struct_sz;
	unsigned int irq_exit_array_cnt;
};

struct sec_debug_subsys_data {
	char name[16];
	char state[16];
	struct sec_debug_subsys_log log;
	struct sec_debug_subsys_excp excp;
	struct sec_debug_subsys_simple_var_mon var_mon;
};

struct sec_debug_subsys_data_modem {
	char name[16];
	char state[16];
	struct sec_debug_subsys_log log;
	struct sec_debug_subsys_excp excp;
	struct sec_debug_subsys_simple_var_mon var_mon;
};

struct sec_debug_subsys_data_scorpion {
	char name[16];
	char state[16];
	int nr_cpus;
	struct sec_debug_subsys_log log;
	struct sec_debug_subsys_excp_scorpion excp;
	struct sec_debug_subsys_simple_var_mon var_mon;
	struct tzbsp_dump_buf_s **tz_core_dump;
	struct sec_debug_subsys_fb fb_info;
	struct sec_debug_subsys_sched_log sched_log;
};

struct sec_debug_subsys_private {
	struct sec_debug_subsys_data_scorpion scorpion;
	struct sec_debug_subsys_data_modem modem;
};

struct sec_debug_subsys {
	unsigned int magic[4];
	struct sec_debug_subsys_data_scorpion *scorpion;
	struct sec_debug_subsys_data_modem *modem;

	struct sec_debug_subsys_private priv;
};

extern int sec_debug_subsys_add_var_mon(char *name, unsigned int size,
	unsigned int addr);
#define SEC_DEBUG_SUBSYS_ADD_VAR_TO_MONITOR(var) \
	sec_debug_subsys_add_var_mon(#var, sizeof(var), \
		(unsigned int)__pa(&var))
#define SEC_DEBUG_SUBSYS_ADD_STR_TO_MONITOR(pstr) \
	sec_debug_subsys_add_var_mon(#pstr, -1, (unsigned int)__pa(pstr))

extern int get_fbinfo(int fb_num, unsigned int *fb_paddr, unsigned int *xres,
		unsigned int *yres, unsigned int *bpp,
		unsigned char *roff, unsigned char *rlen,
		unsigned char *goff, unsigned char *glen,
		unsigned char *boff, unsigned char *blen,
		unsigned char *aoff, unsigned char *alen);
extern unsigned int msm_shared_ram_phys;
extern char *get_kernel_log_buf_paddr(void);
extern char *get_fb_paddr(void);
extern unsigned int get_wdog_regsave_paddr(void);

extern unsigned int get_last_pet_paddr(void);
extern void sec_debug_subsys_set_kloginfo(unsigned int *idx_paddr,
	unsigned int *log_paddr, unsigned int *size);
int sec_debug_save_die_info(const char *str, struct pt_regs *regs);
int sec_debug_save_panic_info(const char *str, unsigned int caller);

extern uint32_t global_pvs;
#endif

#endif	/* SEC_DEBUG_H */
