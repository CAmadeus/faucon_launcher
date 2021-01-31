/*
 * Copyright (c) 2018 naehrwert
 *
 * Copyright (c) 2018-2020 CTCaer
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <stdlib.h>

#include <memory_map.h>

#include "config.h"
#include <gfx/di.h>
#include <gfx_utils.h>
#include "gfx/logos.h"
#include "gfx/tui.h"
#include "hos/hos.h"
#include "hos/secmon_exo.h"
#include "hos/sept.h"
#include <ianos/ianos.h>
#include <libs/compr/blz.h>
#include <libs/fatfs/ff.h>
#include <mem/heap.h>
#include <mem/minerva.h>
#include <mem/sdram.h>
#include <power/bq24193.h>
#include <power/max17050.h>
#include <power/max77620.h>
#include <power/max7762x.h>
#include <rtc/max77620-rtc.h>
#include <soc/bpmp.h>
#include <soc/fuse.h>
#include <soc/hw_init.h>
#include <soc/i2c.h>
#include <soc/t210.h>
#include <soc/uart.h>
#include "storage/emummc.h"
#include "storage/nx_emmc.h"
#include <storage/nx_sd.h>
#include <storage/sdmmc.h>
#include <utils/btn.h>
#include <utils/dirlist.h>
#include <utils/list.h>
#include <utils/util.h>

#include "frontend/fe_emmc_tools.h"
#include "frontend/fe_tools.h"
#include "frontend/fe_info.h"

#define IDENT(x) x
#define XSTR(x) #x
#define STR(x) XSTR(x)
#define PATH(x,y, z) STR(IDENT(x)IDENT(y)IDENT(z))
 
#define FAUCON_BASE_DIR ../../falcon-tools/
#define FAUCON_BASE_FILE /tsec_fw.h

#include PATH(FAUCON_BASE_DIR,TARGET_FAUCON,FAUCON_BASE_FILE)

hekate_config h_cfg;
boot_cfg_t __attribute__((section ("._boot_cfg"))) b_cfg;
const volatile ipl_ver_meta_t __attribute__((section ("._ipl_version"))) ipl_ver = {
	.magic = BL_MAGIC,
	.version = (BL_VER_MJ + '0') | ((BL_VER_MN + '0') << 8) | ((BL_VER_HF + '0') << 16),
	.rsvd0 = 0,
	.rsvd1 = 0
};

volatile nyx_storage_t *nyx_str = (nyx_storage_t *)NYX_STORAGE_ADDR;

void emmcsn_path_impl(char *path, char *sub_dir, char *filename, sdmmc_storage_t *storage)
{
	sdmmc_storage_t storage2;
	sdmmc_t sdmmc;
	char emmcSN[9];
	bool init_done = false;

	memcpy(path, "backup", 7);
	f_mkdir(path);

	if (!storage)
	{
		if (!sdmmc_storage_init_mmc(&storage2, &sdmmc, SDMMC_BUS_WIDTH_8, SDHCI_TIMING_MMC_HS400))
			memcpy(emmcSN, "00000000", 9);
		else
		{
			init_done = true;
			itoa(storage2.cid.serial, emmcSN, 16);
		}
	}
	else
		itoa(storage->cid.serial, emmcSN, 16);

	u32 sub_dir_len = strlen(sub_dir);   // Can be a null-terminator.
	u32 filename_len = strlen(filename); // Can be a null-terminator.

	memcpy(path + strlen(path), "/", 2);
	memcpy(path + strlen(path), emmcSN, 9);
	f_mkdir(path);
	memcpy(path + strlen(path), sub_dir, sub_dir_len + 1);
	if (sub_dir_len)
		f_mkdir(path);
	memcpy(path + strlen(path), "/", 2);
	memcpy(path + strlen(path), filename, filename_len + 1);

	if (init_done)
		sdmmc_storage_end(&storage2);
}

void check_power_off_from_hos()
{
	// Power off on AutoRCM wakeup from HOS shutdown. For modchips/dongles.
	u8 hosWakeup = i2c_recv_byte(I2C_5, MAX77620_I2C_ADDR, MAX77620_REG_IRQTOP);
	if (hosWakeup & MAX77620_IRQ_TOP_RTC_MASK)
	{
		sd_end();

		// Stop the alarm, in case we injected too fast.
		max77620_rtc_stop_alarm();

		if (h_cfg.autohosoff == 1)
		{
			gfx_clear_grey(0x1B);
			u8 *BOOTLOGO = (void *)malloc(0x4000);
			blz_uncompress_srcdest(BOOTLOGO_BLZ, SZ_BOOTLOGO_BLZ, BOOTLOGO, SZ_BOOTLOGO);
			gfx_set_rect_grey(BOOTLOGO, X_BOOTLOGO, Y_BOOTLOGO, 326, 544);

			display_backlight_brightness(10, 5000);
			display_backlight_brightness(100, 25000);
			msleep(600);
			display_backlight_brightness(0, 20000);
		}
		power_off();
	}
}

// This is a safe and unused DRAM region for our payloads.
#define RELOC_META_OFF      0x7C
#define PATCHED_RELOC_SZ    0x94
#define PATCHED_RELOC_STACK 0x40007000
#define PATCHED_RELOC_ENTRY 0x40010000
#define EXT_PAYLOAD_ADDR    0xC0000000
#define RCM_PAYLOAD_ADDR    (EXT_PAYLOAD_ADDR + ALIGN(PATCHED_RELOC_SZ, 0x10))
#define COREBOOT_END_ADDR   0xD0000000
#define CBFS_DRAM_EN_ADDR   0x4003e000
#define  CBFS_DRAM_MAGIC    0x4452414D // "DRAM"

static void *coreboot_addr;

void reloc_patcher(u32 payload_dst, u32 payload_src, u32 payload_size)
{
	memcpy((u8 *)payload_src, (u8 *)IPL_LOAD_ADDR, PATCHED_RELOC_SZ);

	volatile reloc_meta_t *relocator = (reloc_meta_t *)(payload_src + RELOC_META_OFF);

	relocator->start = payload_dst - ALIGN(PATCHED_RELOC_SZ, 0x10);
	relocator->stack = PATCHED_RELOC_STACK;
	relocator->end   = payload_dst + payload_size;
	relocator->ep    = payload_dst;

	if (payload_size == 0x7000)
	{
		memcpy((u8 *)(payload_src + ALIGN(PATCHED_RELOC_SZ, 0x10)), coreboot_addr, 0x7000); //Bootblock
		*(vu32 *)CBFS_DRAM_EN_ADDR = CBFS_DRAM_MAGIC;
	}
}

bool is_ipl_updated(void *buf, char *path, bool force)
{
	return true;
}

int dump_tsec_dmem_to_file(void *falcon_dmem)
{
	int res = 1;

	if (sd_mount())
	{
		char path[64];
		emmcsn_path_impl(path, "/dumps", "tsec_dmem.bin", NULL);

		res = sd_save_to_file(falcon_dmem, 0x4000, path);
		sd_unmount();
	}

	return res;
}

void launch_tsec_firmware(const void *fw, u32 fw_size)
{
	gfx_clear_partial_grey(0x1B, 0, 1256);
	gfx_con_setpos(0, 0);

	u8 keys[0x10 * 2];
	memset(keys, 0x00, 0x20);

	// Prepare the TSEC exploit context which holds result data.
	u8 falcon_dmem[0x4000];
	tsec_exploit_ctxt_t ctx;
	ctx.fw = fw;
	ctx.size = fw_size;
	ctx.dmem = falcon_dmem;

	// Launch the TSEC firmware and wait for the result.
	int res = 0;
	if (tsec_launch_exploit(keys, &ctx, true) < 0)
	{
		res = -1;
	}

	// Print contents of TSEC Mailboxes for better debugging.
	gfx_printf("\n%kMailbox 0: %k%X\n", 0xFF00DDFF, 0xFFCCCCCC, ctx.mailbox0);
	gfx_printf("%kMailbox 1: %k%X\n\n", 0xFF00DDFF, 0xFFCCCCCC, ctx.mailbox1);

	// Pretty-print the TSEC key that was dumped from the SOR1 HDCP MMIO register.
	gfx_printf("%kTSEC key: %k", 0xFF00DDFF, 0xFFCCCCCC);
	for (u32 i = 0; i < 0x10; i++)
	{
		gfx_printf("%02X", keys[i]);
	}
	gfx_puts("\n\n");

	// If an exception occurred, print error details.
	if (res < 0)
	{
		EPRINTFARGS("ERROR %X\n", res);
		gfx_printf("\nLast traced Program Counter: %08X\n", ctx.trace_pc);
		if ((ctx.scp_sec_error >> 31) & 1)
		{
			gfx_puts("\nSCP error occurred! No further details available?\n");
			// TODO: Is this true?          ^
		}
		else
		{
			gfx_printf("\nFaulting Program Counter: %08X\n", ctx.exci & 0xFFFFF);
			gfx_puts("\nException Clause: ");
			switch ((ctx.exci >> 20) & 0xF)
			{
				case 0: WPRINTF("Trap 0"); break;
				case 1: WPRINTF("Trap 1"); break;
				case 2: WPRINTF("Trap 2"); break;
				case 3: WPRINTF("Trap 3"); break;
				case 8: WPRINTF("Invalid Opcode"); break;
				case 9: WPRINTF("Authentication Entry"); break;
				case 10: WPRINTF("Page Fault (Miss)"); break;
				case 11: WPRINTF("Page Fault (Multihit)"); break;
				case 15: WPRINTF("Breakpoint Hit"); break;
				default: gfx_puts("Unknown\n"); break;
			}
		}
	}

	gfx_puts("\n\nPress POWER to dump DMEM to SD card.\n");
	gfx_puts("Press Volume Up to dump the key.\n");
	gfx_puts("Press any buttons to power off.\n");

	// Wait for user input to process requested actions.
	u32 btn = btn_wait();
	if (btn & BTN_POWER)
	{
		if (!dump_tsec_dmem_to_file(falcon_dmem))
		{
			gfx_puts("\nDone! Press any buttons to power off.\n");
		}

		btn_wait();
	}
	else if (btn & BTN_VOL_UP)
	{
		if (sd_mount())
		{
			char path[64];
			emmcsn_path_impl(path, "/dumps", "key.bin", NULL);
			if (!sd_save_to_file(keys, 0x10, path))
				gfx_puts("\nDone!\nPress any buttons to power off.\n");
			sd_unmount();
		}

		btn_wait();
	}

	power_off();
}

void launch_custom_tsec_firmware()
{
	launch_tsec_firmware(tsec_fw_bin, tsec_fw_bin_length);
}


static void _check_low_battery()
{
	int enough_battery;
	int batt_volt = 3500;
	int charge_status = 0;

	bq24193_get_property(BQ24193_ChargeStatus, &charge_status);
	max17050_get_property(MAX17050_AvgVCELL, &batt_volt);

	enough_battery = charge_status ? 3250 : 3000;

	if (batt_volt > enough_battery)
		goto out;

	// Prepare battery icon resources.
	u8 *battery_res = malloc(ALIGN(SZ_BATTERY_EMPTY, 0x1000));
	blz_uncompress_srcdest(BATTERY_EMPTY_BLZ, SZ_BATTERY_EMPTY_BLZ, battery_res, SZ_BATTERY_EMPTY);

	u8 *battery_icon = malloc(0x95A);  // 21x38x3
	u8 *charging_icon = malloc(0x2F4); // 21x12x3
	u8 *no_charging_icon = calloc(0x2F4, 1);

	memcpy(charging_icon, battery_res, 0x2F4);
	memcpy(battery_icon, battery_res + 0x2F4, 0x95A);

	u32 battery_icon_y_pos = 1280 - 16 - Y_BATTERY_EMPTY_BATT;
	u32 charging_icon_y_pos = 1280 - 16 - Y_BATTERY_EMPTY_BATT - 12 - Y_BATTERY_EMPTY_CHRG;
	free(battery_res);

	charge_status = !charge_status;

	u32 timer = 0;
	bool screen_on = false;
	while (true)
	{
		bpmp_msleep(250);

		// Refresh battery stats.
		int current_charge_status = 0;
		bq24193_get_property(BQ24193_ChargeStatus, &current_charge_status);
		max17050_get_property(MAX17050_AvgVCELL, &batt_volt);
		enough_battery = current_charge_status ? 3250 : 3000;

		if (batt_volt > enough_battery)
			break;

		// Refresh charging icon.
		if (screen_on && (charge_status != current_charge_status))
		{
			if (current_charge_status)
				gfx_set_rect_rgb(charging_icon, X_BATTERY_EMPTY, Y_BATTERY_EMPTY_CHRG, 16, charging_icon_y_pos);
			else
				gfx_set_rect_rgb(no_charging_icon, X_BATTERY_EMPTY, Y_BATTERY_EMPTY_CHRG, 16, charging_icon_y_pos);
		}

		// Check if it's time to turn off display.
		if (screen_on && timer < get_tmr_ms())
		{
			if (!current_charge_status)
			{
				max77620_low_battery_monitor_config(true);
				power_off();
			}

			display_end();
			screen_on = false;
		}

		// Check if charging status changed or Power button was pressed.
		if ((charge_status != current_charge_status) || (btn_wait_timeout_single(0, BTN_POWER) & BTN_POWER))
		{
			if (!screen_on)
			{
				display_init();
				u32 *fb = display_init_framebuffer_pitch();
				gfx_init_ctxt(fb, 720, 1280, 720);

				gfx_set_rect_rgb(battery_icon, X_BATTERY_EMPTY, Y_BATTERY_EMPTY_BATT, 16, battery_icon_y_pos);
				if (current_charge_status)
					gfx_set_rect_rgb(charging_icon, X_BATTERY_EMPTY, Y_BATTERY_EMPTY_CHRG, 16, charging_icon_y_pos);
				else
					gfx_set_rect_rgb(no_charging_icon, X_BATTERY_EMPTY, Y_BATTERY_EMPTY_CHRG, 16, charging_icon_y_pos);

				display_backlight_pwm_init();
				display_backlight_brightness(100, 1000);

				screen_on = true;
			}

			timer = get_tmr_ms() + 15000;
		}

		// Check if forcefully continuing.
		if (btn_read_vol() == (BTN_VOL_UP | BTN_VOL_DOWN))
			break;

		charge_status = current_charge_status;
	}

	display_end();

	free(battery_icon);
	free(charging_icon);
	free(no_charging_icon);

out:
	// Re enable Low Battery Monitor shutdown.
	max77620_low_battery_monitor_config(true);
}

ment_t ment_top[] = {
	MDEF_HANDLER("Launch", launch_custom_tsec_firmware),
	MDEF_END()
};

menu_t menu_top = { ment_top, "hekate - CTCaer mod v5.3.0", 0, 0 };

extern void pivot_stack(u32 stack_top);

void ipl_main()
{
	// Do initial HW configuration. This is compatible with consecutive reruns without a reset.
	config_hw();

	// Pivot the stack so we have enough space.
	pivot_stack(IPL_STACK_TOP);

	// Tegra/Horizon configuration goes to 0x80000000+, package2 goes to 0xA9800000, we place our heap in between.
	heap_init(IPL_HEAP_START);

#ifdef DEBUG_UART_PORT
	uart_send(DEBUG_UART_PORT, (u8 *)"hekate: Hello!\r\n", 16);
	uart_wait_idle(DEBUG_UART_PORT, UART_TX_IDLE);
#endif

	// Check if battery is enough.
	_check_low_battery();

	// Set bootloader's default configuration.
	set_default_configuration();

	sd_mount();

	// Save sdram lp0 config.
	if (!ianos_loader("bootloader/sys/libsys_lp0.bso", DRAM_LIB, (void *)sdram_get_params_patched()))
		h_cfg.errors |= ERR_LIBSYS_LP0;

	// Train DRAM and switch to max frequency.
	if (minerva_init())
		h_cfg.errors |= ERR_SYSOLD_MTC;

	display_init();

	u32 *fb = display_init_framebuffer_pitch();
	gfx_init_ctxt(fb, 720, 1280, 720);

	gfx_con_init();

	display_backlight_pwm_init();
	//display_backlight_brightness(h_cfg.backlight, 1000);

	// Overclock BPMP.
	bpmp_clk_rate_set(BPMP_CLK_DEFAULT_BOOST);

	// Load emuMMC configuration from SD.
	emummc_load_cfg();

	minerva_change_freq(FREQ_800);

	while (true)
		tui_do_menu(&menu_top);

	// Halt BPMP if we managed to get out of execution.
	while (true)
		bpmp_halt();
}
