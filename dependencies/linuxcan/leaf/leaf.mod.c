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
	{ 0x2d3385d3, "system_wq" },
	{ 0x26087692, "kmalloc_caches" },
	{ 0xfe2fd6f8, "queue_init" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xf78d370, "vCanCleanup" },
	{ 0xa02aea3a, "queue_length" },
	{ 0x2da2081e, "usb_kill_urb" },
	{ 0x15388bbc, "convert_vcan_to_hydra_cmd" },
	{ 0x45561738, "softSyncLoc2Glob" },
	{ 0xb3378a7b, "pv_ops" },
	{ 0x42501bb9, "softSyncAddMember" },
	{ 0x2da00799, "kthread_create_on_node" },
	{ 0xd57c78, "vCanRemoveCardChannel" },
	{ 0xaad8c7d6, "default_wake_function" },
	{ 0x679e43d1, "queue_empty" },
	{ 0x6782eeca, "queue_push" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x25974000, "wait_for_completion" },
	{ 0x2289f050, "ticks_init" },
	{ 0xfb578fc5, "memset" },
	{ 0xfaa20ff6, "queue_front" },
	{ 0x87d7787f, "queue_add_wait_for_space" },
	{ 0xbee80d0, "vCanDispatchEvent" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0xa7bfbf2f, "current_task" },
	{ 0xdf85ea06, "usb_deregister" },
	{ 0x6adf812a, "ticks_to_64bit_ns" },
	{ 0x3274f5c3, "set_capability_value" },
	{ 0x9166fada, "strncpy" },
	{ 0xe6cf5658, "queue_wakeup_on_space" },
	{ 0x220f6eb0, "queue_pop" },
	{ 0x8c03d20c, "destroy_workqueue" },
	{ 0x7212d1e6, "usb_free_coherent" },
	{ 0x952664c5, "do_exit" },
	{ 0x42160169, "flush_workqueue" },
	{ 0xb6bf1355, "softSyncHandleTRef" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x37fcf336, "module_put" },
	{ 0x62ea9509, "vCanInitData" },
	{ 0xe06e0338, "usb_submit_urb" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x228fca22, "usb_bulk_msg" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xb8b9f817, "kmalloc_order_trace" },
	{ 0x92997ed8, "_printk" },
	{ 0x45937b6f, "wake_up_process" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0xf35141b2, "kmem_cache_alloc_trace" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x42903cc4, "set_capability_mask" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0x3c401726, "softSyncRemoveMember" },
	{ 0x37a0cba, "kfree" },
	{ 0x88830556, "vCanInit" },
	{ 0x2f3fe71d, "vCanFlushSendBuffer" },
	{ 0xf63cc4cc, "usb_register_driver" },
	{ 0x10fa71db, "queue_remove_wait_for_space" },
	{ 0x608741b5, "__init_swait_queue_head" },
	{ 0x244ab863, "queue_back" },
	{ 0x55555880, "queue_reinit" },
	{ 0x30372d96, "queue_release" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0xa6257a2f, "complete" },
	{ 0x6f4cc9ff, "usb_alloc_coherent" },
	{ 0x7f02188f, "__msecs_to_jiffies" },
	{ 0xcce6352d, "vCanAddCardChannel" },
	{ 0x4a3ad70e, "wait_for_completion_timeout" },
	{ 0xc94a6b37, "get_usb_root_hub_id" },
	{ 0xb8b75f24, "usb_free_urb" },
	{ 0x49cd25ed, "alloc_workqueue" },
	{ 0x1cfae7df, "try_module_get" },
	{ 0x8a7330c7, "usb_alloc_urb" },
};

MODULE_INFO(depends, "kvcommon");

MODULE_ALIAS("usb:v0BFDp000Ad*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp000Bd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp000Cd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp000Ed*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp000Fd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0010d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0011d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0012d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0013d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0016d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0017d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0018d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0019d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp001Ad*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp001Bd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp001Cd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp001Dd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0020d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0022d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0023d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0027d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0026d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0120d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0121d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0122d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0123d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0124d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0126d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0127d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0128d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "3C2DBF1F043F65C252260C4");
