#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x32e21920, "module_layout" },
	{ 0x26087692, "kmalloc_caches" },
	{ 0xf78d370, "vCanCleanup" },
	{ 0xe9ff7061, "vCanTime" },
	{ 0xb3378a7b, "pv_ops" },
	{ 0xd57c78, "vCanRemoveCardChannel" },
	{ 0x679e43d1, "queue_empty" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xfb578fc5, "memset" },
	{ 0xfaa20ff6, "queue_front" },
	{ 0xbee80d0, "vCanDispatchEvent" },
	{ 0xa7bfbf2f, "current_task" },
	{ 0x3274f5c3, "set_capability_value" },
	{ 0xe6cf5658, "queue_wakeup_on_space" },
	{ 0x220f6eb0, "queue_pop" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x62ea9509, "vCanInitData" },
	{ 0x8a94440d, "vCanGetCardInfo2" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xb8b9f817, "kmalloc_order_trace" },
	{ 0x92997ed8, "_printk" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0xf35141b2, "kmem_cache_alloc_trace" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xb72fe76b, "vCanGetCardInfo" },
	{ 0x37a0cba, "kfree" },
	{ 0x88830556, "vCanInit" },
	{ 0x2f3fe71d, "vCanFlushSendBuffer" },
	{ 0x30372d96, "queue_release" },
	{ 0xcce6352d, "vCanAddCardChannel" },
};

MODULE_INFO(depends, "kvcommon");


MODULE_INFO(srcversion, "F691342F17CCFB55EDFB3B3");
