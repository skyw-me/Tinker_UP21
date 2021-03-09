#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/can/platform/mcp251x.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

int can0_int_io_requested = 0;

struct spi_device *can0;

/** configurations **/
const int can0_int = 23;

static struct mcp251x_platform_data mcp251x_info = {
    .oscillator_frequency = 16000000,
};

static struct spi_board_info spi_device_info = {
    .modalias = "mcp2515",

    // from up_board.c
    .bus_num = 2,

    // mcp2515 max speed
    .max_speed_hz = 8 * 1000 * 1000,

    .platform_data = &mcp251x_info,
    .irq = -1,
};
/********************/

static int __init mcp2515_init(void)
{
    int ret;
    struct spi_master *master;

    printk("mcp2515_can: init.\n");

    spi_device_info.bus_num = 2;
    master = spi_busnum_to_master(spi_device_info.bus_num);
    if (!master) {
        printk("mcp2515_can: MASTER not found.\n");
        return -ENODEV;
    }

    ///////////////can0///////////////
    // request gpio
    ret = gpio_request(can0_int, "sysfs");

    if (ret) {
        printk("mcp2515_can: could not request gpio %d\n", can0_int);
        gpio_free(can0_int);
        return ret;
    }
    can0_int_io_requested = 1;

    gpio_direction_input(can0_int);

    // init spi
    int irq = gpio_to_irq(can0_int);
    printk("mcp2515_can: irq for pin %d is %d\n", can0_int, irq);
    spi_device_info.irq = irq;
    spi_device_info.chip_select = 0;

    // create a new slave device, given the master and device info
    can0 = spi_new_device(master, &spi_device_info);
    if (!can0) {
        printk("mcp2515_can: FAILED to create slave.\n");
        gpio_free(can0_int);
        return -ENODEV;
    }

    printk("mcp2515_can: CAN0 created!\n");
    return 0;
}

static void __exit mcp2515_exit(void)
{
    printk("mcp2515_can: exit\n");

    if (can0) {
        spi_unregister_device(can0);
    }
    if (can0_int_io_requested) {
        gpio_free(can0_int);
    }
}

module_init(mcp2515_init);
module_exit(mcp2515_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("One");
MODULE_DESCRIPTION("MCP2515 CAN module");
