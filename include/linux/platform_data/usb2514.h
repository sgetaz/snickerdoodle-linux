#ifndef __USB2514_H__
#define __USB2514_H__

#define USB2514_I2C_NAME	"usb2514"

#define USB2514_OFF_PORT1	(1 << 1)
#define USB2514_OFF_PORT2	(1 << 2)
#define USB2514_OFF_PORT3	(1 << 3)
#define USB2514_OFF_PORT4	(1 << 4)

enum usb2514_mode {
	USB2514_MODE_UNKNOWN,
	USB2514_MODE_HUB,
	USB2514_MODE_STANDBY,
};

struct usb2514_platform_data {
	enum usb2514_mode	initial_mode;
	u8	port_off_mask;
	int	gpio_reset;
};

#endif
