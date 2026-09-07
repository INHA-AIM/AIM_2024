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
	{ 0xeb233a45, "__kmalloc" },
	{ 0xf78d370, "vCanCleanup" },
	{ 0x5021bd81, "_raw_write_lock_irqsave" },
	{ 0xfb14ff3c, "pci_disable_device" },
	{ 0xf288fcdc, "pci_release_regions" },
	{ 0xb3378a7b, "pv_ops" },
	{ 0xeb078aee, "_raw_write_unlock_irqrestore" },
	{ 0xd57c78, "vCanRemoveCardChannel" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x679e43d1, "queue_empty" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xfb578fc5, "memset" },
	{ 0xfaa20ff6, "queue_front" },
	{ 0xbee80d0, "vCanDispatchEvent" },
	{ 0xbd1fc1a3, "pci_iounmap" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0xa7bfbf2f, "current_task" },
	{ 0x3274f5c3, "set_capability_value" },
	{ 0xe6cf5658, "queue_wakeup_on_space" },
	{ 0x220f6eb0, "queue_pop" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x800473f, "__cond_resched" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x62ea9509, "vCanInitData" },
	{ 0xb1342cdb, "_raw_read_lock_irqsave" },
	{ 0x8a94440d, "vCanGetCardInfo2" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xb8b9f817, "kmalloc_order_trace" },
	{ 0x92997ed8, "_printk" },
	{ 0xdf2ebb87, "_raw_read_unlock_irqrestore" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0x233f929e, "pci_unregister_driver" },
	{ 0xf35141b2, "kmem_cache_alloc_trace" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0xb72fe76b, "vCanGetCardInfo" },
	{ 0x37a0cba, "kfree" },
	{ 0x95b52781, "pci_request_regions" },
	{ 0x88830556, "vCanInit" },
	{ 0xf2bb7d9, "__pci_register_driver" },
	{ 0x2f3fe71d, "vCanFlushSendBuffer" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x608741b5, "__init_swait_queue_head" },
	{ 0x55555880, "queue_reinit" },
	{ 0x30372d96, "queue_release" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0xa6257a2f, "complete" },
	{ 0x518022d6, "pci_iomap" },
	{ 0xcce6352d, "vCanAddCardChannel" },
	{ 0x4a453f53, "iowrite32" },
	{ 0x8d929026, "pci_enable_device" },
	{ 0x4a3ad70e, "wait_for_completion_timeout" },
	{ 0xa78af5f3, "ioread32" },
	{ 0xc1514a3b, "free_irq" },
};

MODULE_INFO(depends, "kvcommon");


MODULE_INFO(srcversion, "3B3CD464C9A4E13AD531812");
