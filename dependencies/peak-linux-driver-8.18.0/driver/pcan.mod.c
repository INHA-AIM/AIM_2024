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
	{ 0x367fcc51, "module_layout" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x85bd1608, "__request_region" },
	{ 0x58f94a7a, "kmalloc_caches" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xc4f0da12, "ktime_get_with_offset" },
	{ 0xf90a1e85, "__x86_indirect_thunk_r8" },
	{ 0x67a1c6c2, "pci_free_irq_vectors" },
	{ 0x8dab710, "pci_write_config_word" },
	{ 0xc22aaf3c, "single_open" },
	{ 0x77358855, "iomem_resource" },
	{ 0xc769c289, "dma_set_mask" },
	{ 0x62ae48af, "single_release" },
	{ 0x49296222, "usb_reset_endpoint" },
	{ 0xb114006f, "pci_disable_device" },
	{ 0x1b565747, "i2c_transfer" },
	{ 0x20000329, "simple_strtoul" },
	{ 0xffeedf6a, "delayed_work_timer_fn" },
	{ 0xb20dd93f, "seq_printf" },
	{ 0xb43f9365, "ktime_get" },
	{ 0x8f08370a, "usb_kill_urb" },
	{ 0x8d5ebc6b, "remove_proc_entry" },
	{ 0x4ce2483a, "device_destroy" },
	{ 0xf0d8f0d, "__register_chrdev" },
	{ 0xefb76610, "driver_for_each_device" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0x3c609325, "pci_release_regions" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0x9fa7184a, "cancel_delayed_work_sync" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x293ce16, "dma_free_attrs" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0xdb5115c4, "device_create_with_groups" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0xc4bcd79c, "seq_read" },
	{ 0x81fcfa48, "pv_ops" },
	{ 0xee0c0e31, "dma_set_coherent_mask" },
	{ 0x15ba50a6, "jiffies" },
	{ 0xe2d5255a, "strcmp" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xe525bb91, "dma_get_required_mask" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x344967ed, "param_ops_charp" },
	{ 0x66ddf8c0, "pci_set_master" },
	{ 0x34a794e, "pci_alloc_irq_vectors_affinity" },
	{ 0x543561f8, "_dev_warn" },
	{ 0xfb578fc5, "memset" },
	{ 0xdbdf6c92, "ioport_resource" },
	{ 0x1e1e140e, "ns_to_timespec64" },
	{ 0xacab1a72, "pci_iounmap" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x85b33882, "current_task" },
	{ 0x37befc70, "jiffies_to_msecs" },
	{ 0x4e89151d, "usb_deregister" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xfef216eb, "_raw_spin_trylock" },
	{ 0x95483448, "sysfs_remove_file_from_group" },
	{ 0x449ad0a7, "memcmp" },
	{ 0x9ec6ca96, "ktime_get_real_ts64" },
	{ 0x536af521, "class_unregister" },
	{ 0xde80cd09, "ioremap" },
	{ 0x1edb69d6, "ktime_get_raw_ts64" },
	{ 0x277d6f8, "usb_set_interface" },
	{ 0x9166fada, "strncpy" },
	{ 0xc4de20ea, "usb_control_msg" },
	{ 0x27c5b4e, "pci_read_config_word" },
	{ 0x670ecece, "__x86_indirect_thunk_rbx" },
	{ 0x4aa2d6e8, "dma_alloc_attrs" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0xc38c83b8, "mod_timer" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x8ff68d13, "__class_register" },
	{ 0x76b1ce1d, "_dev_err" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x800473f, "__cond_resched" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x167c5967, "print_hex_dump" },
	{ 0xba451227, "i2c_del_adapter" },
	{ 0xbb74ffcc, "_dev_info" },
	{ 0xc090cb6, "usb_submit_urb" },
	{ 0x6383b27c, "__x86_indirect_thunk_rdx" },
	{ 0xb2fcb56d, "queue_delayed_work_on" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x7361851e, "usb_reset_device" },
	{ 0xb341d9ba, "usb_bulk_msg" },
	{ 0x1000e51, "schedule" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xe97e5c7b, "usb_clear_halt" },
	{ 0x92997ed8, "_printk" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x1035c7c2, "__release_region" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0x45648c, "pci_unregister_driver" },
	{ 0xcc5005fe, "msleep_interruptible" },
	{ 0xcbb0ae81, "kmem_cache_alloc_trace" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x8914ccdc, "param_ops_byte" },
	{ 0xdfbfa97, "pci_irq_vector" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xf6ebc03b, "net_ratelimit" },
	{ 0x82ee90dc, "timer_delete_sync" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0xd823f987, "seq_lseek" },
	{ 0x37a0cba, "kfree" },
	{ 0x69acdf38, "memcpy" },
	{ 0x43e76b7a, "pci_request_regions" },
	{ 0x49ba14bc, "param_array_ops" },
	{ 0xedc03953, "iounmap" },
	{ 0xce536bb4, "__pci_register_driver" },
	{ 0xf0fca0ae, "usb_register_driver" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x467c6bab, "sysfs_add_file_to_group" },
	{ 0xd2ba368d, "i2c_bit_add_bus" },
	{ 0x656e4a6e, "snprintf" },
	{ 0xb0e602eb, "memmove" },
	{ 0xad4e8f27, "pci_iomap" },
	{ 0x28cce246, "param_ops_ushort" },
	{ 0xaaf8f76a, "proc_create" },
	{ 0xe3e557a3, "usb_get_current_frame_number" },
	{ 0x5e515be6, "ktime_get_ts64" },
	{ 0x7f02188f, "__msecs_to_jiffies" },
	{ 0x986aec2c, "pci_enable_device" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0xe426891a, "param_ops_ulong" },
	{ 0x49a5d567, "param_ops_uint" },
	{ 0x77b6f9cd, "usb_free_urb" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0x626418fb, "usb_alloc_urb" },
	{ 0xc1514a3b, "free_irq" },
};

MODULE_INFO(depends, "i2c-algo-bit");

MODULE_ALIAS("pci:v0000001Cd00000001sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000003sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000004sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000005sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000006sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000007sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000008sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000009sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000002sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000000Asv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000010sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000013sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000014sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000016sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000017sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000018sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000019sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000001Asv*sd*bc*sc*i*");
MODULE_ALIAS("usb:v0C72p000Cd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p000Dd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0012d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0011d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0013d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0014d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "05D9F3722F526359885CEB0");
