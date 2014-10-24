/*
 * sec_debug.c
 *
 * driver supporting debug functions for Samsung device
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

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/ctype.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>
#include <mach/sec_debug.h>
#include <mach/proc_comm.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/sec_param.h>
#include "pm.h"
#include <linux/moduleparam.h>

/* To ensure panic is called only once */
unsigned in_panic ;

struct sec_debug_mmu_reg_t {
	int SCTLR;
	int TTBR0;
	int TTBR1;
	int TTBCR;
	int DACR;
	int DFSR;
	int DFAR;
	int IFSR;
	int IFAR;
	int DAFSR;
	int IAFSR;
	int PMRRR;
	int NMRRR;
	int FCSEPID;
	int CONTEXT;
	int URWTPID;
	int UROTPID;
	int POTPIDR;
};

/* ARM CORE regs mapping structure */
struct sec_debug_core_t {
	/* COMMON */
	unsigned int r0;
	unsigned int r1;
	unsigned int r2;
	unsigned int r3;
	unsigned int r4;
	unsigned int r5;
	unsigned int r6;
	unsigned int r7;
	unsigned int r8;
	unsigned int r9;
	unsigned int r10;
	unsigned int r11;
	unsigned int r12;

	/* SVC */
	unsigned int r13_svc;
	unsigned int r14_svc;
	unsigned int spsr_svc;

	/* PC & CPSR */
	unsigned int pc;
	unsigned int cpsr;

	/* USR/SYS */
	unsigned int r13_usr;
	unsigned int r14_usr;

	/* FIQ */
	unsigned int r8_fiq;
	unsigned int r9_fiq;
	unsigned int r10_fiq;
	unsigned int r11_fiq;
	unsigned int r12_fiq;
	unsigned int r13_fiq;
	unsigned int r14_fiq;
	unsigned int spsr_fiq;

	/* IRQ */
	unsigned int r13_irq;
	unsigned int r14_irq;
	unsigned int spsr_irq;

	/* MON */
	unsigned int r13_mon;
	unsigned int r14_mon;
	unsigned int spsr_mon;

	/* ABT */
	unsigned int r13_abt;
	unsigned int r14_abt;
	unsigned int spsr_abt;

	/* UNDEF */
	unsigned int r13_und;
	unsigned int r14_und;
	unsigned int spsr_und;
};

/* enable sec_debug feature */
static unsigned int enable = 1;
static unsigned enable_user = 1;
static char sec_build_info[100];
static unsigned int secdbg_paddr;
static unsigned int secdbg_size;
static unsigned int sec_subsys_paddr;
static unsigned int sec_subsys_size;

module_param_named(enable, enable, uint, 0644);

static char *sec_build_time[] = {
	__DATE__,
	__TIME__
};

#ifdef CONFIG_SEC_DEBUG_SCHED_LOG
struct sec_debug_log {
	atomic_t idx_sched[CONFIG_NR_CPUS];
	struct sched_log sched[CONFIG_NR_CPUS][SCHED_LOG_MAX] ;

	atomic_t idx_irq[CONFIG_NR_CPUS];
	struct irq_log irq[CONFIG_NR_CPUS][SCHED_LOG_MAX] ;

	atomic_t idx_irq_exit[CONFIG_NR_CPUS];
	struct irq_exit_log irq_exit[CONFIG_NR_CPUS][SCHED_LOG_MAX] ;

	atomic_t idx_timer[CONFIG_NR_CPUS];
	struct timer_log timer_log[CONFIG_NR_CPUS][SCHED_LOG_MAX] ;
};

struct sec_debug_log *secdbg_log;

#endif	/* CONFIG_SEC_DEBUG_SCHED_LOG */

#ifdef CONFIG_SEC_DEBUG_SUBSYS
struct sec_debug_subsys *secdbg_subsys;
struct sec_debug_subsys_data_scorpion *secdbg_scorpion;
#endif

struct sec_debug_core_t sec_debug_core_reg;
struct sec_debug_mmu_reg_t sec_debug_mmu_reg;

/* for sec debug level */
unsigned int sec_dbg_level;
static int __init sec_debug_level(char *str)
{
	get_option(&str, &sec_dbg_level);
	return 0;
}
early_param("level", sec_debug_level);

bool kernel_sec_set_debug_level(int level)
{
	if (!(level == KERNEL_SEC_DEBUG_LEVEL_LOW
			|| level == KERNEL_SEC_DEBUG_LEVEL_MID
			|| level == KERNEL_SEC_DEBUG_LEVEL_HIGH)) {
		pr_notice(KERN_NOTICE "(kernel_sec_set_debug_level) The debug"
				"value is invalid(0x%x)!! Set default"
				"level(LOW)\n", level);
		sec_dbg_level = KERNEL_SEC_DEBUG_LEVEL_LOW;
		return -EINVAL;
	}

	sec_dbg_level = level;

	switch (level) {
	case KERNEL_SEC_DEBUG_LEVEL_LOW:
		enable = 0;
		enable_user = 0;
		break;
	case KERNEL_SEC_DEBUG_LEVEL_MID:
		enable = 1;
		enable_user = 0;
		break;
	case KERNEL_SEC_DEBUG_LEVEL_HIGH:
		enable = 1;
		enable_user = 1;
		break;
	default:
		enable = 1;
		enable_user = 1;
	}

	/* write to param */
	sec_set_param(param_index_debuglevel, &sec_dbg_level);

	pr_notice(KERN_NOTICE "(kernel_sec_set_debug_level)"
			"The debug value is 0x%x !!\n", level);

	return 1;
}
EXPORT_SYMBOL(kernel_sec_set_debug_level);

int kernel_sec_get_debug_level(void)
{
	sec_get_param(param_index_debuglevel, &sec_dbg_level);

	if (!(sec_dbg_level == KERNEL_SEC_DEBUG_LEVEL_LOW
			|| sec_dbg_level == KERNEL_SEC_DEBUG_LEVEL_MID
			|| sec_dbg_level == KERNEL_SEC_DEBUG_LEVEL_HIGH)) {
		/*In case of invalid debug level, default (debug level low)*/
		pr_notice(KERN_NOTICE "(%s) The debug value is"
				"invalid(0x%x)!! Set default level(LOW)\n",
				__func__, sec_dbg_level);
		sec_dbg_level = KERNEL_SEC_DEBUG_LEVEL_LOW;
		sec_set_param(param_index_debuglevel, &sec_dbg_level);
	}
	return sec_dbg_level;
}
EXPORT_SYMBOL(kernel_sec_get_debug_level);

/* core reg dump function*/
static void sec_debug_save_core_reg(struct sec_debug_core_t *core_reg)
{
	/* we will be in SVC mode when we enter this function. Collect
	   SVC registers along with cmn registers. */
	asm("str r0, [%0,#0]\n\t"	/* R0 is pushed first to core_reg */
	    "mov r0, %0\n\t"		/* R0 will be alias for core_reg */
	    "str r1, [r0,#4]\n\t"	/* R1 */
	    "str r2, [r0,#8]\n\t"	/* R2 */
	    "str r3, [r0,#12]\n\t"	/* R3 */
	    "str r4, [r0,#16]\n\t"	/* R4 */
	    "str r5, [r0,#20]\n\t"	/* R5 */
	    "str r6, [r0,#24]\n\t"	/* R6 */
	    "str r7, [r0,#28]\n\t"	/* R7 */
	    "str r8, [r0,#32]\n\t"	/* R8 */
	    "str r9, [r0,#36]\n\t"	/* R9 */
	    "str r10, [r0,#40]\n\t"	/* R10 */
	    "str r11, [r0,#44]\n\t"	/* R11 */
	    "str r12, [r0,#48]\n\t"	/* R12 */
	    /* SVC */
	    "str r13, [r0,#52]\n\t"	/* R13_SVC */
	    "str r14, [r0,#56]\n\t"	/* R14_SVC */
	    "mrs r1, spsr\n\t"		/* SPSR_SVC */
	    "str r1, [r0,#60]\n\t"
	    /* PC and CPSR */
	    "sub r1, r15, #0x4\n\t"	/* PC */
	    "str r1, [r0,#64]\n\t"
	    "mrs r1, cpsr\n\t"		/* CPSR */
	    "str r1, [r0,#68]\n\t"
	    /* SYS/USR */
	    "mrs r1, cpsr\n\t"		/* switch to SYS mode */
	    "and r1, r1, #0xFFFFFFE0\n\t"
	    "orr r1, r1, #0x1f\n\t"
	    "msr cpsr,r1\n\t"
	    "str r13, [r0,#72]\n\t"	/* R13_USR */
	    "str r14, [r0,#76]\n\t"	/* R14_USR */
	    /* FIQ */
	    "mrs r1, cpsr\n\t"		/* switch to FIQ mode */
	    "and r1,r1,#0xFFFFFFE0\n\t"
	    "orr r1,r1,#0x11\n\t"
	    "msr cpsr,r1\n\t"
	    "str r8, [r0,#80]\n\t"	/* R8_FIQ */
	    "str r9, [r0,#84]\n\t"	/* R9_FIQ */
	    "str r10, [r0,#88]\n\t"	/* R10_FIQ */
	    "str r11, [r0,#92]\n\t"	/* R11_FIQ */
	    "str r12, [r0,#96]\n\t"	/* R12_FIQ */
	    "str r13, [r0,#100]\n\t"	/* R13_FIQ */
	    "str r14, [r0,#104]\n\t"	/* R14_FIQ */
	    "mrs r1, spsr\n\t"		/* SPSR_FIQ */
	    "str r1, [r0,#108]\n\t"
		/* IRQ */
	    "mrs r1, cpsr\n\t"		/* switch to IRQ mode */
	    "and r1, r1, #0xFFFFFFE0\n\t"
	    "orr r1, r1, #0x12\n\t"
	    "msr cpsr,r1\n\t"
	    "str r13, [r0,#112]\n\t"	/* R13_IRQ */
	    "str r14, [r0,#116]\n\t"	/* R14_IRQ */
	    "mrs r1, spsr\n\t"		/* SPSR_IRQ */
	    "str r1, [r0,#120]\n\t"
	    /* MON */
	    "mrs r1, cpsr\n\t"		/* switch to monitor mode */
	    "and r1, r1, #0xFFFFFFE0\n\t"
	    "orr r1, r1, #0x16\n\t"
	    "msr cpsr,r1\n\t"
	    "str r13, [r0,#124]\n\t"	/* R13_MON */
	    "str r14, [r0,#128]\n\t"	/* R14_MON */
	    "mrs r1, spsr\n\t"		/* SPSR_MON */
	    "str r1, [r0,#132]\n\t"
	    /* ABT */
	    "mrs r1, cpsr\n\t"		/* switch to Abort mode */
	    "and r1, r1, #0xFFFFFFE0\n\t"
	    "orr r1, r1, #0x17\n\t"
	    "msr cpsr,r1\n\t"
	    "str r13, [r0,#136]\n\t"	/* R13_ABT */
	    "str r14, [r0,#140]\n\t"	/* R14_ABT */
	    "mrs r1, spsr\n\t"		/* SPSR_ABT */
	    "str r1, [r0,#144]\n\t"
	    /* UND */
	    "mrs r1, cpsr\n\t"		/* switch to undef mode */
	    "and r1, r1, #0xFFFFFFE0\n\t"
	    "orr r1, r1, #0x1B\n\t"
	    "msr cpsr,r1\n\t"
	    "str r13, [r0,#148]\n\t"	/* R13_UND */
	    "str r14, [r0,#152]\n\t"	/* R14_UND */
	    "mrs r1, spsr\n\t"		/* SPSR_UND */
	    "str r1, [r0,#156]\n\t"
	    /* restore to SVC mode */
	    "mrs r1, cpsr\n\t"		/* switch to SVC mode */
	    "and r1, r1, #0xFFFFFFE0\n\t"
	    "orr r1, r1, #0x13\n\t"
	    "msr cpsr,r1\n\t" :		/* output */
	    : "r"(core_reg)			/* input */
	    : "%r0", "%r1"		/* clobbered registers */
	);

	return;
}

static void sec_debug_save_mmu_reg(struct sec_debug_mmu_reg_t *mmu_reg)
{
	asm("mrc    p15, 0, r1, c1, c0, 0\n\t"	/* SCTLR */
	    "str r1, [%0]\n\t"
	    "mrc    p15, 0, r1, c2, c0, 0\n\t"	/* TTBR0 */
	    "str r1, [%0,#4]\n\t"
	    "mrc    p15, 0, r1, c2, c0,1\n\t"	/* TTBR1 */
	    "str r1, [%0,#8]\n\t"
	    "mrc    p15, 0, r1, c2, c0,2\n\t"	/* TTBCR */
	    "str r1, [%0,#12]\n\t"
	    "mrc    p15, 0, r1, c3, c0,0\n\t"	/* DACR */
	    "str r1, [%0,#16]\n\t"
	    "mrc    p15, 0, r1, c5, c0,0\n\t"	/* DFSR */
	    "str r1, [%0,#20]\n\t"
	    "mrc    p15, 0, r1, c6, c0,0\n\t"	/* DFAR */
	    "str r1, [%0,#24]\n\t"
	    "mrc    p15, 0, r1, c5, c0,1\n\t"	/* IFSR */
	    "str r1, [%0,#28]\n\t"
	    "mrc    p15, 0, r1, c6, c0,2\n\t"	/* IFAR */
	    "str r1, [%0,#32]\n\t"
	    /* Don't populate DAFSR and RAFSR */
	    "mrc    p15, 0, r1, c10, c2,0\n\t"	/* PMRRR */
	    "str r1, [%0,#44]\n\t"
	    "mrc    p15, 0, r1, c10, c2,1\n\t"	/* NMRRR */
	    "str r1, [%0,#48]\n\t"
	    "mrc    p15, 0, r1, c13, c0,0\n\t"	/* FCSEPID */
	    "str r1, [%0,#52]\n\t"
	    "mrc    p15, 0, r1, c13, c0,1\n\t"	/* CONTEXT */
	    "str r1, [%0,#56]\n\t"
	    "mrc    p15, 0, r1, c13, c0,2\n\t"	/* URWTPID */
	    "str r1, [%0,#60]\n\t"
	    "mrc    p15, 0, r1, c13, c0,3\n\t"	/* UROTPID */
	    "str r1, [%0,#64]\n\t"
	    "mrc    p15, 0, r1, c13, c0,4\n\t"	/* POTPIDR */
	    "str r1, [%0,#68]\n\t" :		/* output */
	    : "r"(mmu_reg)			/* input */
	    : "%r1", "memory"			/* clobbered register */
	);
}

static void sec_debug_save_context(void)
{
	unsigned long flags;
	local_irq_save(flags);
	sec_debug_save_mmu_reg(&sec_debug_mmu_reg);
	sec_debug_save_core_reg(&sec_debug_core_reg);
	pr_emerg("(%s) context saved(CPU:%d)\n", __func__,
			smp_processor_id());
	local_irq_restore(flags);
}

static int sec_debug_normal_reboot_handler(struct notifier_block *nb,
		unsigned long l, void *p)
{
	set_restart_reason(NORMAL_REBOOT);
	return 0;
}

void sec_debug_hw_reset(void)
{
	pr_emerg("(%s) %s\n", __func__, sec_build_info);
	pr_emerg("(%s) rebooting...\n", __func__);
	flush_cache_all();
	outer_flush_all();

	#ifdef CONFIG_SEC_DEBUG
	/* During panic AP saves restart reason by itself*/
	kernel_secdebug_restart();
	#else
	machine_restart("sec_debug_hw_reset");
	#endif

	pr_emerg("Cannot reboot...");

	while (1)
		;
}
EXPORT_SYMBOL(sec_debug_hw_reset);

static int sec_debug_panic_handler(struct notifier_block *nb,
		unsigned long l, void *buf)
{
	unsigned int len;

	len = strnlen(buf, 16);

	if (!in_panic) {
		if (!strncmp(buf, "Crash Key", len))
			set_restart_reason(SECDEBUG_FORCED_UPLOAD_MODE);
		else if (!strncmp(buf, "modem crash", len))
			set_restart_reason(SECDEBUG_MODEM_UPLOAD_MODE);
		else
			set_restart_reason(SECDEBUG_KERNEL_PANIC_MODE);
		sec_debug_dump_stack();
		in_panic = 1;
	}

	sec_debug_hw_reset();
	return 0;
}

/*
 * Called from dump_stack()
 * This function call does not necessarily mean that a fatal error
 * had occurred. It may be just a warning.
 */
int sec_debug_dump_stack(void)
{
	if (!enable)
		return -EPERM;

	sec_debug_save_context();

	/* flush L1 from each core.
	   L2 will be flushed later before reset. */
	flush_cache_all();
	return 0;
}
EXPORT_SYMBOL(sec_debug_dump_stack);

void sec_debug_check_crash_key(unsigned int code, int value)
{
	static enum { NONE, STEP1, STEP2} state = NONE;

	if (!enable)
		return;

	switch (state) {
	case NONE:
		if (code == KEY_VOLUMEUP && value)
			state = STEP1;
		else
			state = NONE;
		break;
	case STEP1:
		if (code == KEY_VOLUMEDOWN && value)
			state = STEP2;
		else
			state = NONE;
		break;
	case STEP2:
		if (code == KEY_CAMERA_FOCUS && value) {
			dump_all_task_info();
			dump_cpu_stat();
			pr_info("Crash key..\n");
			panic("Crash Key");
		} else
			state = NONE;
		break;
	}
}

static struct notifier_block nb_reboot_block = {
	.notifier_call = sec_debug_normal_reboot_handler
};

static struct notifier_block nb_panic_block = {
	.notifier_call = sec_debug_panic_handler,
};

static void sec_debug_set_build_info(void)
{
	char *p = sec_build_info;
	strncat(p, "Kernel Build Info : ", 20);
	strncat(p, "Date:", 5);
	strncat(p, sec_build_time[0], 12);
	strncat(p, "Time:", 5);
	strncat(p, sec_build_time[1], 9);
}

#ifdef CONFIG_SEC_DEBUG_SCHED_LOG
static int __init __init_sec_debug_log(void)
{
	int i;
	struct sec_debug_log *vaddr;
	int size;

	if (secdbg_paddr == 0 || secdbg_size == 0) {
		pr_info("%s: sec debug buffer not provided. Using kmalloc..\n",
			__func__);
		size = sizeof(struct sec_debug_log);
		vaddr = kmalloc(size, GFP_KERNEL);
	} else {
		size = secdbg_size;
		vaddr = ioremap_nocache(secdbg_paddr, secdbg_size);
	}

	pr_info("%s: vaddr=0x%x paddr=0x%x size=0x%x "
		"sizeof(struct sec_debug_log)=0x%x\n", __func__,
		(unsigned int)vaddr, secdbg_paddr, secdbg_size,
		sizeof(struct sec_debug_log));

	if ((vaddr == NULL) || (sizeof(struct sec_debug_log) > size)) {
		pr_info("%s: ERROR! init failed!\n", __func__);
		return -EFAULT;
	}

	for (i = 0; i < CONFIG_NR_CPUS; i++) {
		atomic_set(&(vaddr->idx_sched[i]), -1);
		atomic_set(&(vaddr->idx_irq[i]), -1);
		atomic_set(&(vaddr->idx_irq_exit[i]), -1);
		atomic_set(&(vaddr->idx_timer[i]), -1);
	}

	secdbg_log = vaddr;

	pr_info("%s: init done\n", __func__);

	return 0;
}
#endif

#ifdef CONFIG_SEC_DEBUG_SUBSYS
int sec_debug_save_die_info(const char *str, struct pt_regs *regs)
{
	if (!secdbg_scorpion)
		return -ENOMEM;
	snprintf(secdbg_scorpion->excp.pc_sym,
			sizeof(secdbg_scorpion->excp.pc_sym),
			"%pS", (void *)regs->ARM_pc);
	snprintf(secdbg_scorpion->excp.lr_sym,
			sizeof(secdbg_scorpion->excp.lr_sym),
			"%pS", (void *)regs->ARM_lr);

	return 0;
}

int sec_debug_save_panic_info(const char *str, unsigned int caller)
{
	if (!secdbg_scorpion)
		return -ENOMEM;
	snprintf(secdbg_scorpion->excp.panic_caller,
		sizeof(secdbg_scorpion->excp.panic_caller), "%pS",
							(void *)caller);
	snprintf(secdbg_scorpion->excp.panic_msg,
		sizeof(secdbg_scorpion->excp.panic_msg), "%s", str);
	snprintf(secdbg_scorpion->excp.thread,
		sizeof(secdbg_scorpion->excp.thread), "%s:%d", current->comm,
		task_pid_nr(current));

	return 0;
}

int sec_debug_subsys_add_var_mon(char *name, unsigned int size, unsigned int pa)
{
	if (!secdbg_scorpion)
		return -ENOMEM;

	if (secdbg_scorpion->var_mon.idx >
			ARRAY_SIZE(secdbg_scorpion->var_mon.var))
		return -ENOMEM;

	strlcpy(secdbg_scorpion->var_mon.var[secdbg_scorpion->var_mon.idx].name,
		name, sizeof(secdbg_scorpion->var_mon.var[0].name));
	secdbg_scorpion->var_mon.var[secdbg_scorpion->var_mon.idx].sizeof_type
									= size;
	secdbg_scorpion->var_mon.var[secdbg_scorpion->var_mon.idx].var_paddr
									= pa;

	secdbg_scorpion->var_mon.idx++;

	return 0;
}

int sec_debug_subsys_init(void)
{
	struct sec_debug_subsys *vaddr;
	struct sec_debug_subsys *paddr;

	secdbg_subsys = NULL;
	secdbg_scorpion = NULL;

	if (sec_subsys_paddr == 0 || sec_subsys_size == 0) {
		pr_info("%s: sec subsys buffer not provided!\n", __func__);
		return -ENOMEM;
	} else {
		vaddr = ioremap_nocache(sec_subsys_paddr, sec_subsys_size);
		paddr = (struct sec_debug_subsys *) sec_subsys_paddr;
	}

	pr_info("%s: vaddr=0x%x paddr=0x%x size=0x%x "
		"sizeof(struct sec_debug_subsys)=0x%x\n", __func__,
		(unsigned int)vaddr, sec_subsys_paddr, sec_subsys_size,
		sizeof(struct sec_debug_subsys));

	if ((vaddr == NULL) ||
		(sizeof(struct sec_debug_subsys) > sec_subsys_size)) {
		pr_info("%s: ERROR! init failed!\n", __func__);
		return -EFAULT;
	}

	secdbg_subsys = vaddr;

	/*clear the priv data*/
	memset(&secdbg_subsys->priv, 0, sizeof(secdbg_subsys->priv));

	secdbg_scorpion = &secdbg_subsys->priv.scorpion;

	/*just storing the calculated physical address*/
	/*no memory access using physical address*/
	secdbg_subsys->scorpion = &paddr->priv.scorpion;
	secdbg_subsys->modem = &paddr->priv.modem;

	pr_info("%s: scorpion(%x) modem(%x)\n", __func__,
		(unsigned int)secdbg_subsys->scorpion,
		(unsigned int)secdbg_subsys->modem);

	strncpy(secdbg_scorpion->name, "Scorpion",
		sizeof(secdbg_scorpion->name));
	strncpy(secdbg_scorpion->state, "Init",
		sizeof(secdbg_scorpion->state));
	secdbg_scorpion->nr_cpus = CONFIG_NR_CPUS;

	sec_debug_subsys_set_kloginfo(&secdbg_scorpion->log.idx_paddr,
		&secdbg_scorpion->log.log_paddr, &secdbg_scorpion->log.size);

	secdbg_scorpion->tz_core_dump =
		(struct tzbsp_dump_buf_s **)NULL;
	get_fbinfo(0, &secdbg_scorpion->fb_info.fb_paddr,
		&secdbg_scorpion->fb_info.xres,
		&secdbg_scorpion->fb_info.yres,
		&secdbg_scorpion->fb_info.bpp,
		&secdbg_scorpion->fb_info.rgb_bitinfo.r_off,
		&secdbg_scorpion->fb_info.rgb_bitinfo.r_len,
		&secdbg_scorpion->fb_info.rgb_bitinfo.g_off,
		&secdbg_scorpion->fb_info.rgb_bitinfo.g_len,
		&secdbg_scorpion->fb_info.rgb_bitinfo.b_off,
		&secdbg_scorpion->fb_info.rgb_bitinfo.b_len,
		&secdbg_scorpion->fb_info.rgb_bitinfo.a_off,
		&secdbg_scorpion->fb_info.rgb_bitinfo.a_len);

	SEC_DEBUG_SUBSYS_ADD_STR_TO_MONITOR(linux_banner);

	if (secdbg_paddr) {
		secdbg_scorpion->sched_log.sched_idx_paddr = secdbg_paddr +
			offsetof(struct sec_debug_log, idx_sched);
		secdbg_scorpion->sched_log.sched_buf_paddr = secdbg_paddr +
			offsetof(struct sec_debug_log, sched);
		secdbg_scorpion->sched_log.sched_struct_sz =
			sizeof(struct sched_log);
		secdbg_scorpion->sched_log.sched_array_cnt = SCHED_LOG_MAX;

		secdbg_scorpion->sched_log.irq_idx_paddr = secdbg_paddr +
			offsetof(struct sec_debug_log, idx_irq);
		secdbg_scorpion->sched_log.irq_buf_paddr = secdbg_paddr +
			offsetof(struct sec_debug_log, irq);
		secdbg_scorpion->sched_log.irq_struct_sz =
			sizeof(struct irq_log);
		secdbg_scorpion->sched_log.irq_array_cnt = SCHED_LOG_MAX;

		secdbg_scorpion->sched_log.irq_exit_idx_paddr = secdbg_paddr +
			offsetof(struct sec_debug_log, idx_irq_exit);
		secdbg_scorpion->sched_log.irq_exit_buf_paddr = secdbg_paddr +
			offsetof(struct sec_debug_log, irq_exit);
		secdbg_scorpion->sched_log.irq_exit_struct_sz =
			sizeof(struct irq_exit_log);
		secdbg_scorpion->sched_log.irq_exit_array_cnt = SCHED_LOG_MAX;
	}

	/* fill magic nubmer last to ensure data integrity when the magic
	 * numbers are written
	 */
	secdbg_subsys->magic[0] = SEC_DEBUG_SUBSYS_MAGIC0;
	secdbg_subsys->magic[1] = SEC_DEBUG_SUBSYS_MAGIC1;
	secdbg_subsys->magic[2] = SEC_DEBUG_SUBSYS_MAGIC2;
	secdbg_subsys->magic[3] = SEC_DEBUG_SUBSYS_MAGIC3;
	return 0;
}
late_initcall(sec_debug_subsys_init);

static int __init sec_subsys_setup(char *str)
{
	unsigned size = memparse(str, &str);

	pr_emerg("%s: str=%s\n", __func__, str);

	if (size && (*str == '@')) {
		sec_subsys_paddr = (unsigned int)memparse(++str, NULL);
		sec_subsys_size = size;
	}

	pr_emerg("%s: sec_subsys_paddr = 0x%x\n", __func__, sec_subsys_paddr);
	pr_emerg("%s: sec_subsys_size = 0x%x\n", __func__, sec_subsys_size);

	return 1;
}

__setup("sec_subsys=", sec_subsys_setup);

#endif

int __init sec_debug_init(void)
{
	pr_emerg("%s: enable=%d\n", __func__, enable);

	register_reboot_notifier(&nb_reboot_block);
	atomic_notifier_chain_register(&panic_notifier_list, &nb_panic_block);

	if (!enable)
		return -EPERM;

#ifdef CONFIG_SEC_DEBUG_SCHED_LOG
	__init_sec_debug_log();
#endif

	sec_debug_set_build_info();

	return 0;
}

int sec_debug_is_enabled(void)
{
	return enable;
}

#ifdef CONFIG_SEC_DEBUG_SCHED_LOG
void __sec_debug_task_sched_log(int cpu, struct task_struct *task,
						char *msg)
{
	unsigned i;

	if (!secdbg_log)
		return;

	if (!task && !msg)
		return;

	i = atomic_inc_return(&(secdbg_log->idx_sched[cpu]))
		& (SCHED_LOG_MAX - 1);
	secdbg_log->sched[cpu][i].time = cpu_clock(cpu);
	if (task) {
		strncpy(secdbg_log->sched[cpu][i].comm, task->comm,
			sizeof(secdbg_log->sched[cpu][i].comm));
		secdbg_log->sched[cpu][i].pid = task->pid;
	} else {
		strncpy(secdbg_log->sched[cpu][i].comm, msg,
			sizeof(secdbg_log->sched[cpu][i].comm));
		secdbg_log->sched[cpu][i].pid = -1;
	}
}

void sec_debug_task_sched_log_short_msg(char *msg)
{
	__sec_debug_task_sched_log(smp_processor_id(), NULL, msg);
}

void sec_debug_task_sched_log(int cpu, struct task_struct *task)
{
	__sec_debug_task_sched_log(cpu, task, NULL);
}

void sec_debug_timer_log(unsigned int type, int int_lock, void *fn)
{
	int cpu = smp_processor_id();
	unsigned i;

	if (!secdbg_log)
		return;

	i = atomic_inc_return(&(secdbg_log->idx_timer[cpu]))
		& (SCHED_LOG_MAX - 1);
	secdbg_log->timer_log[cpu][i].time = cpu_clock(cpu);
	secdbg_log->timer_log[cpu][i].type = type;
	secdbg_log->timer_log[cpu][i].int_lock = int_lock;
	secdbg_log->timer_log[cpu][i].fn = (void *)fn;
}

void sec_debug_irq_sched_log(unsigned int irq, void *fn, int en)
{
	int cpu = smp_processor_id();
	unsigned i;

	if (!secdbg_log)
		return;

	i = atomic_inc_return(&(secdbg_log->idx_irq[cpu]))
		& (SCHED_LOG_MAX - 1);
	secdbg_log->irq[cpu][i].time = cpu_clock(cpu);
	secdbg_log->irq[cpu][i].irq = irq;
	secdbg_log->irq[cpu][i].fn = (void *)fn;
	secdbg_log->irq[cpu][i].en = en;
	secdbg_log->irq[cpu][i].preempt_count = preempt_count();
	secdbg_log->irq[cpu][i].context = &cpu;
}

#ifdef CONFIG_SEC_DEBUG_IRQ_EXIT_LOG
void sec_debug_irq_enterexit_log(unsigned int irq,
					unsigned long long start_time)
{
	int cpu = smp_processor_id();
	unsigned i;

	if (!secdbg_log)
		return;

	i = atomic_inc_return(&(secdbg_log->idx_irq_exit[cpu]))
		& (SCHED_LOG_MAX - 1);
	secdbg_log->irq_exit[cpu][i].time = start_time;
	secdbg_log->irq_exit[cpu][i].end_time = cpu_clock(cpu);
	secdbg_log->irq_exit[cpu][i].irq = irq;
	secdbg_log->irq_exit[cpu][i].elapsed_time =
		secdbg_log->irq_exit[cpu][i].end_time - start_time;
}
#endif
#endif  /* CONFIG_SEC_DEBUG_SCHED_LOG */

static int __init sec_dbg_setup(char *str)
{
	unsigned size = memparse(str, &str);

	pr_emerg("%s: str=%s\n", __func__, str);

	if (size && (size == roundup_pow_of_two(size)) && (*str == '@')) {
		secdbg_paddr = (unsigned int)memparse(++str, NULL);
		secdbg_size = size;
	}

	pr_emerg("%s: secdbg_paddr = 0x%x\n", __func__, secdbg_paddr);
	pr_emerg("%s: secdbg_size = 0x%x\n", __func__, secdbg_size);

	return 1;
}

__setup("sec_dbg=", sec_dbg_setup);
