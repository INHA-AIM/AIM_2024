cmd_/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/spi/src/xspi.o := gcc -Wp,-MMD,/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/spi/src/.xspi.o.d  -nostdinc -isystem /usr/lib/gcc/x86_64-linux-gnu/9/include -I./arch/x86/include -I./arch/x86/include/generated  -I./include -I./arch/x86/include/uapi -I./arch/x86/include/generated/uapi -I./include/uapi -I./include/generated/uapi -include ./include/linux/compiler-version.h -include ./include/linux/kconfig.h -I./ubuntu/include -include ./include/linux/compiler_types.h -D__KERNEL__ -fmacro-prefix-map=./= -Wall -Wundef -Werror=strict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -fshort-wchar -fno-PIE -Werror=implicit-function-declaration -Werror=implicit-int -Werror=return-type -Wno-format-security -std=gnu89 -mno-sse -mno-mmx -mno-sse2 -mno-3dnow -mno-avx -fcf-protection=none -m64 -falign-jumps=1 -falign-loops=1 -mno-80387 -mno-fp-ret-in-387 -mpreferred-stack-boundary=3 -mskip-rax-setup -mtune=generic -mno-red-zone -mcmodel=kernel -DCONFIG_X86_X32_ABI -Wno-sign-compare -fno-asynchronous-unwind-tables -mindirect-branch=thunk-extern -mindirect-branch-register -mfunction-return=thunk-extern -fno-jump-tables -fno-delete-null-pointer-checks -Wno-frame-address -Wno-format-truncation -Wno-format-overflow -Wno-address-of-packed-member -O2 --param=allow-store-data-races=0 -Wframe-larger-than=1024 -fstack-protector-strong -Wimplicit-fallthrough=5 -Wno-main -Wno-unused-but-set-variable -Wno-unused-const-variable -fno-omit-frame-pointer -fno-optimize-sibling-calls -fno-stack-clash-protection -g -pg -mrecord-mcount -mfentry -DCC_USING_FENTRY -Wdeclaration-after-statement -Wvla -Wno-pointer-sign -Wno-stringop-truncation -Wno-array-bounds -Wno-stringop-overflow -Wno-restrict -Wno-maybe-uninitialized -Wno-alloc-size-larger-than -fno-strict-overflow -fno-stack-check -fconserve-stack -Werror=date-time -Werror=incompatible-pointer-types -Werror=designated-init -Wno-packed-not-aligned -DLINUX=1 -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/../include -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/hw -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/util -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/kvaser -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/kvaser/pwm -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/kvaser/pciefd -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/kvaser/spi_flash -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/kvaser/hydra_flash -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/altera/inc -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/altera/inc/sys -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/altera/HAL/inc -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/kvaser/spi/sf2_spi -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/common/src -I/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/spi/src -Wall -Wno-date-time -D_DEBUG=0 -DDEBUG=0 -DWIN32=0  -fsanitize=bounds -fsanitize=shift -fsanitize=bool -fsanitize=enum  -DMODULE  -DKBUILD_BASENAME='"xspi"' -DKBUILD_MODNAME='"kvpciefd"' -D__KBUILD_MODNAME=kmod_kvpciefd -c -o /home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/spi/src/xspi.o /home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/spi/src/xspi.c

source_/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/spi/src/xspi.o := /home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/spi/src/xspi.c

deps_/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/spi/src/xspi.o := \
  include/linux/compiler-version.h \
    $(wildcard include/config/CC_VERSION_TEXT) \
  include/linux/kconfig.h \
    $(wildcard include/config/CPU_BIG_ENDIAN) \
    $(wildcard include/config/BOOGER) \
    $(wildcard include/config/FOO) \
  include/linux/compiler_types.h \
    $(wildcard include/config/HAVE_ARCH_COMPILER_H) \
    $(wildcard include/config/CC_HAS_ASM_INLINE) \
  include/linux/compiler_attributes.h \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/RETPOLINE) \
    $(wildcard include/config/ARCH_USE_BUILTIN_BSWAP) \
    $(wildcard include/config/KCOV) \
  /home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/spi/src/xspi.h \
  /home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/common/src/xil_types.h \
  include/linux/types.h \
    $(wildcard include/config/HAVE_UID16) \
    $(wildcard include/config/UID16) \
    $(wildcard include/config/ARCH_DMA_ADDR_T_64BIT) \
    $(wildcard include/config/PHYS_ADDR_T_64BIT) \
    $(wildcard include/config/64BIT) \
    $(wildcard include/config/ARCH_32BIT_USTAT_F_TINODE) \
  include/uapi/linux/types.h \
  arch/x86/include/generated/uapi/asm/types.h \
  include/uapi/asm-generic/types.h \
  include/asm-generic/int-ll64.h \
  include/uapi/asm-generic/int-ll64.h \
  arch/x86/include/uapi/asm/bitsperlong.h \
  include/asm-generic/bitsperlong.h \
  include/uapi/asm-generic/bitsperlong.h \
  include/uapi/linux/posix_types.h \
  include/linux/stddef.h \
  include/uapi/linux/stddef.h \
  include/linux/compiler_types.h \
  arch/x86/include/asm/posix_types.h \
    $(wildcard include/config/X86_32) \
  arch/x86/include/uapi/asm/posix_types_64.h \
  include/uapi/asm-generic/posix_types.h \
  /home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/common/src/xil_assert.h \
  include/linux/bug.h \
    $(wildcard include/config/GENERIC_BUG) \
    $(wildcard include/config/BUG_ON_DATA_CORRUPTION) \
  arch/x86/include/asm/bug.h \
    $(wildcard include/config/DEBUG_BUGVERBOSE) \
  include/linux/stringify.h \
  include/linux/instrumentation.h \
    $(wildcard include/config/DEBUG_ENTRY) \
    $(wildcard include/config/STACK_VALIDATION) \
  include/asm-generic/bug.h \
    $(wildcard include/config/BUG) \
    $(wildcard include/config/GENERIC_BUG_RELATIVE_POINTERS) \
    $(wildcard include/config/SMP) \
    $(wildcard include/config/CFI_CLANG) \
    $(wildcard include/config/MODULES) \
  include/linux/compiler.h \
    $(wildcard include/config/TRACE_BRANCH_PROFILING) \
    $(wildcard include/config/PROFILE_ALL_BRANCHES) \
  arch/x86/include/generated/asm/rwonce.h \
  include/asm-generic/rwonce.h \
  include/linux/kasan-checks.h \
    $(wildcard include/config/KASAN_GENERIC) \
    $(wildcard include/config/KASAN_SW_TAGS) \
  include/linux/kcsan-checks.h \
    $(wildcard include/config/KCSAN) \
    $(wildcard include/config/KCSAN_IGNORE_ATOMICS) \
  include/linux/once_lite.h \
  include/linux/panic.h \
    $(wildcard include/config/PANIC_TIMEOUT) \
  include/linux/printk.h \
    $(wildcard include/config/MESSAGE_LOGLEVEL_DEFAULT) \
    $(wildcard include/config/CONSOLE_LOGLEVEL_DEFAULT) \
    $(wildcard include/config/CONSOLE_LOGLEVEL_QUIET) \
    $(wildcard include/config/EARLY_PRINTK) \
    $(wildcard include/config/PRINTK) \
    $(wildcard include/config/PRINTK_INDEX) \
    $(wildcard include/config/DYNAMIC_DEBUG) \
    $(wildcard include/config/DYNAMIC_DEBUG_CORE) \
  include/linux/stdarg.h \
  include/linux/init.h \
    $(wildcard include/config/HAVE_ARCH_PREL32_RELOCATIONS) \
    $(wildcard include/config/STRICT_KERNEL_RWX) \
    $(wildcard include/config/STRICT_MODULE_RWX) \
    $(wildcard include/config/LTO_CLANG) \
  include/linux/kern_levels.h \
  include/linux/linkage.h \
    $(wildcard include/config/ARCH_USE_SYM_ANNOTATIONS) \
  include/linux/export.h \
    $(wildcard include/config/MODVERSIONS) \
    $(wildcard include/config/MODULE_REL_CRCS) \
    $(wildcard include/config/TRIM_UNUSED_KSYMS) \
  arch/x86/include/asm/linkage.h \
    $(wildcard include/config/X86_64) \
    $(wildcard include/config/X86_ALIGNMENT_16) \
    $(wildcard include/config/RETHUNK) \
    $(wildcard include/config/SLS) \
  include/linux/cache.h \
    $(wildcard include/config/ARCH_HAS_CACHE_LINE_SIZE) \
  include/uapi/linux/kernel.h \
  include/uapi/linux/sysinfo.h \
  include/linux/const.h \
  include/vdso/const.h \
  include/uapi/linux/const.h \
  arch/x86/include/asm/cache.h \
    $(wildcard include/config/X86_L1_CACHE_SHIFT) \
    $(wildcard include/config/X86_INTERNODE_CACHE_SHIFT) \
    $(wildcard include/config/X86_VSMP) \
  include/linux/ratelimit_types.h \
  include/linux/bits.h \
  include/vdso/bits.h \
  include/linux/build_bug.h \
  include/uapi/linux/param.h \
  arch/x86/include/generated/uapi/asm/param.h \
  include/asm-generic/param.h \
    $(wildcard include/config/HZ) \
  include/uapi/asm-generic/param.h \
  include/linux/spinlock_types.h \
    $(wildcard include/config/PREEMPT_RT) \
    $(wildcard include/config/DEBUG_LOCK_ALLOC) \
  include/linux/spinlock_types_raw.h \
    $(wildcard include/config/DEBUG_SPINLOCK) \
  arch/x86/include/asm/spinlock_types.h \
  include/asm-generic/qspinlock_types.h \
    $(wildcard include/config/NR_CPUS) \
  include/asm-generic/qrwlock_types.h \
  arch/x86/include/uapi/asm/byteorder.h \
  include/linux/byteorder/little_endian.h \
  include/uapi/linux/byteorder/little_endian.h \
  include/linux/swab.h \
  include/uapi/linux/swab.h \
  arch/x86/include/uapi/asm/swab.h \
  include/linux/byteorder/generic.h \
  include/linux/lockdep_types.h \
    $(wildcard include/config/PROVE_RAW_LOCK_NESTING) \
    $(wildcard include/config/PREEMPT_LOCK) \
    $(wildcard include/config/LOCKDEP) \
    $(wildcard include/config/LOCK_STAT) \
  include/linux/rwlock_types.h \
  include/linux/dynamic_debug.h \
    $(wildcard include/config/JUMP_LABEL) \
  include/linux/jump_label.h \
    $(wildcard include/config/HAVE_ARCH_JUMP_LABEL_RELATIVE) \
  arch/x86/include/asm/jump_label.h \
  arch/x86/include/asm/asm.h \
    $(wildcard include/config/KPROBES) \
  arch/x86/include/asm/extable_fixup_types.h \
  arch/x86/include/asm/nops.h \
  /home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/common/src/xil_types.h \
  /home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/common/src/xstatus.h \
  /home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/common/src/xbasic_types.h \
  /home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/spi/src/xspi_l.h \
  /home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/common/src/xil_io.h \
  /home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/common/src/xstatus.h \
  /home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/spi/src/xspi_i.h \

/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/spi/src/xspi.o: $(deps_/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/spi/src/xspi.o)

$(deps_/home/autonav/AIM_2024/dependencies/linuxcan/pciefd/drivers/xilinx/spi/src/xspi.o):
