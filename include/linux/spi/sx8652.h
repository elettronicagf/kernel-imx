/* linux/spi/sx8652.h */

/* Touchscreen characteristics vary between boards and models.  The
 * platform_data for the device's "struct device" holds this information.
 *
 * It's OK if the min/max values are zero.
 */

struct sx8652_platform_data {
	u16	model;			/* 8652 */

	u16	x_plate_ohms;
	u16	y_plate_ohms;

	u16	x_min, x_max;
	u16	y_min, y_max;
	u16	pressure_min, pressure_max;

	int	gpio_pendown;		/* the GPIO used to decide the pendown
					 * state if get_pendown_state == NULL
					 */
	int gpio_reset;	 
	int	(*get_pendown_state)(void);
	int (*get_pendown_irq)(void);
	void	(*wait_for_sync)(void);
	u8 ts_wires;
};
