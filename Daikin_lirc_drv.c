/*
 * daikin_rpi.c
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/timekeeping.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_data/bcm2708.h>

#define DAIKIN_DRIVER_NAME "daikin_rpi"
#define RBUF_LEN 256
#define LIRC_TRANSMITTER_LATENCY 50

#ifndef MAX_UDELAY_MS
#define MAX_UDELAY_US 5000
#else
#define MAX_UDELAY_US (MAX_UDELAY_MS*1000)
#endif

#define INVALID -1
#define dprintk(fmt, args...)     \
        do {       \
                if (debug)     \
                        printk(KERN_DEBUG DAIKIN_DRIVER_NAME ": " \
                               fmt, ## args);   \
        } while (0)

/* Note: These defaults are pretty brutally overwritten by the devtree stuff. */
#define DEFAULT_GPIO_IN_PIN 22
#define DEFAULT_GPIO_OUT_PIN 23
#define DAIKIN_CARRIER_FREQ 33333


#define BURST_LENGTH 360
#define MARK_LENGTH 1800
#define SPACE_LENGTH 720
#define START_BURST 5000
#define START_SPACE 2100
#define PACKET_SPACE 29400
#define FIRST_PACKET_LEN (56*2)
#define SECOND_PACKET_LEN (104*2)
#define START_BIT_LEN 2
#define END_BIT_LEN 1
#define PACKET_SPACE_LEN 1
#define DAIK_PACKET_SIZE (START_BIT_LEN+FIRST_PACKET_LEN+END_BIT_LEN+PACKET_SPACE_LEN+START_BIT_LEN+SECOND_PACKET_LEN+END_BIT_LEN)

typedef union {

    struct __attribute__((packed)) {
        unsigned unknown1 : 32;
        unsigned unknown1b : 8;
        unsigned on_off : 1;
        unsigned unknown2 : 3;
        unsigned mode : 3;
        unsigned unknown3 : 1;
        unsigned swing : 1;
        unsigned unknown4 : 2;
        unsigned powerful : 1;
        unsigned unknown5 : 21;
        unsigned temperature : 5;
        unsigned unknown6 : 2;
        unsigned fanspeed : 3;
        unsigned fanauto : 1;
        unsigned unknown7 : 12;
        unsigned checksum : 8;
    }
    bf;
    unsigned char packet[13];
} daikin_command_t;




/* module parameters */

/* set the default pull behaviour for input pin */
static int gpio_out_pin = DEFAULT_GPIO_OUT_PIN;
/* enable debugging messages */
static bool debug = 0;
/* -1 = auto, 0 = active high, 1 = active low */
static int sense = -1;
/* use softcarrier by default */
static bool softcarrier = 1;
/* 0 = do not invert output, 1 = invert output */
static bool invert = 0;
/* Transmit mask */
unsigned int tx_mask = 0xFFFFFFFF; /* All transmitters selected as default */

struct gpio_chip *gpiochip;


/* forward declarations */
static long send_pulse(unsigned long length);
static void send_space(long length);
static void daikin_rpi_exit(void);

static struct platform_device *daikin_rpi_dev;
static struct timeval lasttv = {0, 0};

static spinlock_t lock;

/* initialized/set in init_timing_params() */
static unsigned int freq = DAIKIN_CARRIER_FREQ;
static unsigned int duty_cycle = 50;
static unsigned long period;
static unsigned long pulse_width;
static unsigned long space_width;
/* SYSFS */
static struct kobject *daikin_kobject;
/* Daikin */
daikin_command_t daikin_cmd;

int charbuf2array(const unsigned char *src, size_t srclen, int *dest, int dest_from, int dest_max) {
    int i, j;
    int destindex = dest_from;
    unsigned char c;
    for (i = 0; i < srclen && destindex < dest_max; i++) {
        c = src[i];
        for (j = 0; j < 8 && destindex < dest_max; j++) {
            dest[destindex++] = BURST_LENGTH;
            if (c & 0x01) {
                dest[destindex++] = MARK_LENGTH;
            } else {
                dest[destindex++] = SPACE_LENGTH;
            }
            c >>= 1;
        }
    }
    return destindex;

}

void daikin_command_initialize(daikin_command_t *cmd) {
    unsigned char second_packet[13] = {17, 218, 39, 0, 211, 65, 0, 0, 0, 28, 3, 8, 77}; //packet initializer
    int i;
    for (i = 0; i < 13; i++) {
        cmd->packet[i] = second_packet[i];
    }
}

void daikin_command_calculate_checksum(daikin_command_t *cmd) {
    unsigned char sum = 0;
    int i;
    for (i = 0; i < 12; i++) {
        sum += cmd->packet[i];
    }
    cmd->bf.checksum = sum;
}

static ssize_t store_on_off(struct kobject *kobj, struct kobj_attribute *attr, const char *buff, size_t count) {
    int onoff;
    if (kstrtoint(buff, 10, &onoff) == 0) {
        if (onoff == 0 || onoff == 1) daikin_cmd.bf.on_off = onoff;
    }
    return count;
}

static ssize_t show_on_off(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    return sprintf(buf, "%u", daikin_cmd.bf.on_off);
}

static ssize_t store_mode(struct kobject *kobj, struct kobj_attribute *attr, const char *buff, size_t count) {

    switch (buff[0]) {
        case 'H': //Heat
        case 'h':
            daikin_cmd.bf.mode = 1;
            break;
        case 'C': //Cool
        case 'c':
            daikin_cmd.bf.mode = 2;
            break;
        case 'D': //Dehumidify
        case 'd':
            daikin_cmd.bf.mode = 3;
            break;
        case 'A': //Automatic
        case 'a':
            daikin_cmd.bf.mode = 4;
            break;
        default:
            break;
    }
    return count;
}

static ssize_t show_mode(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    char c = 'E';
    switch (daikin_cmd.bf.mode) {
        case 1:
            c = 'H';
            break;
        case 2:
            c = 'C';
            break;
        case 3:
            c = 'D';
            break;
        case 4:
            c = 'A';
            break;
        default:
            break;
    }
    return sprintf(buf, "%c", c);
}

static ssize_t store_swing(struct kobject *kobj, struct kobj_attribute *attr, const char *buff, size_t count) {
    int swing;
    if (kstrtoint(buff, 10, &swing) == 0) {
        if (swing == 0 || swing == 1) daikin_cmd.bf.swing = swing;
    }
    return count;
}

static ssize_t show_swing(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    return sprintf(buf, "%u", daikin_cmd.bf.swing);
}

static ssize_t store_powerful(struct kobject *kobj, struct kobj_attribute *attr, const char *buff, size_t count) {
    int powerful;
    if (kstrtoint(buff, 10, &powerful) == 0) {
        if (powerful == 0 || powerful == 1) daikin_cmd.bf.powerful = powerful;
    }
    return count;
}

static ssize_t show_powerful(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    return sprintf(buf, "%u", daikin_cmd.bf.powerful);
}

static ssize_t store_temperature(struct kobject *kobj, struct kobj_attribute *attr, const char *buff, size_t count) {
    int temperature;
    if (kstrtoint(buff, 10, &temperature) == 0) {
        if (temperature > 15 && temperature < 30) daikin_cmd.bf.temperature = temperature - 10; //add proper min and max temp, make it mode-dependent
    }
    return count;
}

static ssize_t show_temperature(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    return sprintf(buf, "%u", daikin_cmd.bf.temperature);
}

static ssize_t store_fanspeed(struct kobject *kobj, struct kobj_attribute *attr, const char *buff, size_t count) {
    int fanspeed;
    if (kstrtoint(buff, 10, &fanspeed) == 0) {
        if (fanspeed > 0 && fanspeed < 5) {
            daikin_cmd.bf.fanspeed = fanspeed + 2;
            daikin_cmd.bf.fanauto = 0;
        }
    } else if (buff[0] == 'A' || buff[0] == 'a') {
        daikin_cmd.bf.fanspeed = 2;
        daikin_cmd.bf.fanauto = 1;
    }
    return count;
}

static ssize_t show_fanspeed(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    if (daikin_cmd.bf.fanspeed == 2) {
        return sprintf(buf, "A");
    }//automatic
    return sprintf(buf, "%u", daikin_cmd.bf.fanspeed);
}

static void safe_udelay(unsigned long usecs) {
    while (usecs > MAX_UDELAY_US) {
        udelay(MAX_UDELAY_US);
        usecs -= MAX_UDELAY_US;
    }
    udelay(usecs);
}

static unsigned long read_current_us(void) {
    struct timespec now;
    getnstimeofday(&now);
    return (now.tv_sec * 1000000) + (now.tv_nsec / 1000);
}

static int init_timing_params(unsigned int new_duty_cycle,
        unsigned int new_freq) {
    if (new_freq > 0) {
        if (1000 * 1000000L / new_freq * new_duty_cycle / 100 <=
                LIRC_TRANSMITTER_LATENCY)
            return -EINVAL;
        if (1000 * 1000000L / new_freq * (100 - new_duty_cycle) / 100 <=
                LIRC_TRANSMITTER_LATENCY)
            return -EINVAL;
        duty_cycle = new_duty_cycle;
        freq = new_freq;
        period = 1000 * 1000000L / freq;
        pulse_width = period * duty_cycle / 100;
        space_width = period - pulse_width;
    } else {
        duty_cycle = INVALID;
        freq = 0;
        period = INVALID;
        pulse_width = INVALID;
        space_width = INVALID;
    }
    dprintk("in init_timing_params, freq=%d pulse=%ld, "
            "space=%ld\n", freq, pulse_width, space_width);
    return 0;
}

static long send_pulse_softcarrier(unsigned long length) {
    int flag;

    unsigned long actual, target;
    unsigned long actual_us, initial_us, target_us;

    length *= 1000;

    actual = 0;
    target = 0;
    flag = 0;
    actual_us = read_current_us();
    while (actual < length) {
        gpiochip->set(gpiochip, gpio_out_pin, !(flag ^ invert));
        target += flag ? space_width : pulse_width;
        initial_us = actual_us;
        target_us = actual_us + (target - actual) / 1000;
        /*
         * Note - we've checked in ioctl that the pulse/space
         * widths are big enough so that d is > 0
         */
        if ((int) (target_us - actual_us) > 0)
            udelay(target_us - actual_us);
        actual_us = read_current_us();
        actual += (actual_us - initial_us) * 1000;
        flag = !flag;
    }
    return (actual - length) / 1000;
}

static long send_pulse(unsigned long length) {

    if (length <= 0)
        return 0;
    if (softcarrier && freq > 0) {
        return send_pulse_softcarrier(length);
    } else {
        gpiochip->set(gpiochip, gpio_out_pin, !invert);

        safe_udelay(length);
        return 0;
    }
}

static void send_space(long length) {

    gpiochip->set(gpiochip, gpio_out_pin, invert);
    if (length <= 0)
        return;
    safe_udelay(length);
}

static int is_right_chip(struct gpio_chip *chip, void *data) {
    dprintk("is_right_chip %s %d\n", chip->label, strcmp(data, chip->label));

    if (strcmp(data, chip->label) == 0)
        return 1;
    return 0;
}

static inline int read_bool_property(const struct device_node *np,
        const char *propname,
        bool *out_value) {
    u32 value = 0;
    int err = of_property_read_u32(np, propname, &value);
    if (err == 0)
        *out_value = (value != 0);
    return err;
}

static void read_pin_settings(struct device_node *node) {
    u32 pin;
    u32 function;
    int err;
    of_property_read_u32_index(node, "brcm,pins", 0, &pin);
    err = of_property_read_u32_index(
            node,
            "brcm,function",
            0,
            &function);
    if (err == 0) {
        if (function == 1) /* Output */
            gpio_out_pin = pin;
    }
}

static int init_port(void) {
    struct device_node *node;
    node = daikin_rpi_dev->dev.of_node;
    gpiochip = gpiochip_find("bcm2708_gpio", is_right_chip);
    /*
     * Because of the lack of a setpull function, only support
     * pinctrl-bcm2835 if using device tree.
     */
    if (!gpiochip && node)
        gpiochip = gpiochip_find("pinctrl-bcm2835", is_right_chip);

    if (!gpiochip) {
        pr_err(DAIKIN_DRIVER_NAME ": gpio chip not found!\n");
        return -ENODEV;
    }

    if (node) {
        struct device_node *pins_node;
        pins_node = of_parse_phandle(node, "pinctrl-0", 0);
        if (!pins_node) {
            printk(KERN_ERR DAIKIN_DRIVER_NAME
                    ": pinctrl settings not found!\n");
            return -EINVAL;
        }
        read_pin_settings(pins_node);

        of_property_read_u32(node, "rpi,sense", &sense);

        read_bool_property(node, "rpi,softcarrier", &softcarrier);

        read_bool_property(node, "rpi,invert", &invert);

        read_bool_property(node, "rpi,debug", &debug);

    } else {
        return -EINVAL;
    }
    gpiochip->set(gpiochip, gpio_out_pin, invert);
    return 0;
}

static void daikin_write(int* wbuf, int count) {
    int i;
    unsigned long flags;
    long delta = 0;
    if (count % 2 == 0)
        return;
    spin_lock_irqsave(&lock, flags);
    for (i = 0; i < count; i++) {
        if (i % 2)
            send_space(wbuf[i] - delta);
        else
            delta = send_pulse(wbuf[i]);
    }

    gpiochip->set(gpiochip, gpio_out_pin, invert);
    spin_unlock_irqrestore(&lock, flags);
}

static ssize_t store_send(struct kobject *kobj, struct kobj_attribute *attr, const char *buff, size_t count) {
    unsigned char first_packet[7] = {17, 218, 39, 240, 13, 0, 15};
    int* packet_buffer;
    int packet_index = 0;
    packet_buffer = kmalloc(sizeof (int)*DAIK_PACKET_SIZE, GFP_KERNEL);
    if (packet_buffer != NULL) {
        daikin_command_calculate_checksum(&daikin_cmd);
        packet_buffer[packet_index++] = START_BURST;
        packet_buffer[packet_index++] = START_SPACE;
        packet_index = charbuf2array(first_packet, sizeof (first_packet), packet_buffer, packet_index, DAIK_PACKET_SIZE);
        packet_buffer[packet_index++] = BURST_LENGTH; //stop bit
        packet_buffer[packet_index++] = PACKET_SPACE; //stop bit
        packet_buffer[packet_index++] = START_BURST;
        packet_buffer[packet_index++] = START_SPACE;
        packet_index = charbuf2array(daikin_cmd.packet, sizeof (daikin_cmd.packet), packet_buffer, packet_index, DAIK_PACKET_SIZE);
        packet_buffer[packet_index++] = BURST_LENGTH; //stop bit
        do_gettimeofday(&lasttv);
        /* initialize pulse/space widths */
        init_timing_params(duty_cycle, freq);

        daikin_write(packet_buffer, packet_index);
        kfree(packet_buffer);
    }
    return count;
}


static const struct of_device_id daikin_rpi_of_match[] = {
    { .compatible = "rpi,daikin-rpi",},
    {},
};
MODULE_DEVICE_TABLE(of, daikin_rpi_of_match);

static struct platform_driver daikin_rpi_driver = {
    .driver =
    {
        .name = DAIKIN_DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(daikin_rpi_of_match),
    },
};

static int __init daikin_rpi_init(void) {
    struct device_node *node;
    int result;

    /* Init read buffer. */
    result = platform_driver_register(&daikin_rpi_driver);
    if (result) {
        printk(KERN_ERR DAIKIN_DRIVER_NAME
                ": daikin register returned %d\n", result);
        goto exit_buffer_free;
    }
    node = of_find_compatible_node(NULL, NULL,
            daikin_rpi_of_match[0].compatible);
    if (node) {
        /* DT-enabled */
        printk("DT enabled\n");
        daikin_rpi_dev = of_find_device_by_node(node);
        WARN_ON(daikin_rpi_dev->dev.of_node != node);
        of_node_put(node);
    } else {
        daikin_rpi_dev = platform_device_alloc(DAIKIN_DRIVER_NAME, 0);
        if (!daikin_rpi_dev) {
            result = -ENOMEM;
            goto exit_driver_unregister;
        }
        result = platform_device_add(daikin_rpi_dev);
        if (result)
            goto exit_device_put;
    }
    return 0;

exit_device_put:
    platform_device_put(daikin_rpi_dev);

exit_driver_unregister:
    platform_driver_unregister(&daikin_rpi_driver);

exit_buffer_free:
    return result;
}

static void daikin_rpi_exit(void) {
    if (!daikin_rpi_dev->dev.of_node)
        platform_device_unregister(daikin_rpi_dev);
    platform_driver_unregister(&daikin_rpi_driver);
}


static struct kobj_attribute daikin_attributes[] = {
    __ATTR(on_off, (S_IWUSR | S_IRUGO), show_on_off, store_on_off),
    __ATTR(mode, (S_IWUSR | S_IRUGO), show_mode, store_mode),
    __ATTR(swing, (S_IWUSR | S_IRUGO), show_swing, store_swing),
    __ATTR(powerful, (S_IWUSR | S_IRUGO), show_powerful, store_powerful),
    __ATTR(temperature, (S_IWUSR | S_IRUGO), show_temperature, store_temperature),
    __ATTR(fanspeed, (S_IWUSR | S_IRUGO), show_fanspeed, store_fanspeed),
    __ATTR(send, (S_IWUSR | S_IRUGO), NULL, store_send)
};

int daikin_sysfs_init(void) {
    int i;
    printk(KERN_INFO "DAIKIN: starting sysfs...");
    daikin_kobject = kobject_create_and_add("daikin_ac", NULL);
    for (i = 0; i < (sizeof (daikin_attributes) / sizeof (daikin_attributes[0])); i++) {
        if (sysfs_create_file(daikin_kobject, &(daikin_attributes[i].attr))) {
            dprintk("failed to create daikin sysfs!\n");

            return -1;
        }

    }
    printk(KERN_INFO "DAIKIN: starting sysfs done.");
    return 0;
}

void daikin_sysfs_exit(void) {

    printk(KERN_INFO "DAIKIN: stopping sysfs...");
    kobject_put(daikin_kobject);
    printk(KERN_INFO "DAIKIN: stopping sysfs done.");
}

static int __init daikin_rpi_init_module(void) {
    int result;

    result = daikin_rpi_init();
    if (result)
        return result;
    result = daikin_sysfs_init();
    if (result)
        return result;

    result = init_port();
    if (result < 0)
        goto exit_rpi;
    dprintk("gpio_out_pin = %d\n", gpio_out_pin);
    daikin_command_initialize(&daikin_cmd);
    return 0;

exit_rpi:
    daikin_rpi_exit();

    return result;
}

static void __exit daikin_rpi_exit_module(void) {

    gpio_free(gpio_out_pin);
    daikin_rpi_exit();
    daikin_sysfs_exit();
    printk(KERN_INFO DAIKIN_DRIVER_NAME ": cleaned up module\n");
}

module_init(daikin_rpi_init_module);
module_exit(daikin_rpi_exit_module);

MODULE_DESCRIPTION("Daikin Air conditioner IR driver based on lirc_rpi.");
MODULE_LICENSE("GPL");


// tx_mask is deliberately not made available as module parameter;
// it is a user parameter, not a hardware configuration parameter.
