/*
 * tda1997x - NXP HDMI receiver
 *
 * Copyright 2017 Tim Harvey <tharvey@gateworks.com>
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef _TDA1997X_
#define _TDA1997X_

/* Platform Data */
struct tda1997x_platform_data {
	u32 vidout_bus_width;
	u8 vidout_port_cfg[9];
	int max_pixel_rate; /* (MP/sec) */
	/* pin polarity (1=invert) */
	bool vidout_inv_de;
	bool vidout_inv_hs;
	bool vidout_inv_vs;
	bool vidout_inv_pclk;
	/* clock delays (0=-8, 1=-7 ... 15=+7 pixels) */
	u8 vidout_delay_hs;
	u8 vidout_delay_vs;
	u8 vidout_delay_de;
	u8 vidout_delay_pclk;
	/* sync selections (controls how sync pins are derived) */
	u8 vidout_sel_hs;
	u8 vidout_sel_vs;
	u8 vidout_sel_de;

	/* Audio Port Output */
	u8 audout_fmt;          /* output data format */
	u8 audout_sysclk;       /* clock frequency */
	u8 audout_layout;       /* physical bus layout (if not auto) */
	bool audout_layoutauto; /* audio layout dictated by pkt header */
	bool audout_invert_clk; /* data valid on rising edge of BCLK */
	bool audio_auto_mute;   /* enable hardware audio auto-mute */
};

#endif
