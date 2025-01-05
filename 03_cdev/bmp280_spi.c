#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>

#define DEVICE_NAME_TEMP "bmp280_temperature"
#define DEVICE_NAME_PRESSURE "bmp280_pressure"
#define CLASS_NAME "bmp280"



// Declarații funcții
static int bmp280_probe(struct platform_device *pdev);
static int bmp280_remove(struct platform_device *pdev);

static ssize_t bmp280_temp_read(struct file *file, char __user *buffer, size_t len, loff_t *offset);
static ssize_t bmp280_pressure_read(struct file *file, char __user *buffer, size_t len, loff_t *offset);

static struct of_device_id bmp280_driver_ids[] = {
    {
        .compatible = "mta-labs, bmp280-spi"
    }, { /* sentinel */ }
};

static struct platform_driver bmp280_driver = {
    .probe = bmp280_probe,
    .remove = bmp280_remove,
    .driver = {
        .name = "bmp280_spi_driver",
        .of_match_table = bmp280_driver_ids
    }
};

static int temp_major;
static int pressure_major;

static struct spi_device *spi_dev;

static struct class *bmp280_class;
static struct device *temp_device;
static struct device *pressure_device;
static char temp_buffer[] = "Temperature: 25.5 C\n";
static char pressure_buffer[] = "Pressure: 1013 hPa\n";


// Structurile pentru operațiile dispozitivelor
static const struct file_operations temp_fops = {
    .owner = THIS_MODULE,
    .read = bmp280_temp_read
};

static const struct file_operations pressure_fops = {
    .owner = THIS_MODULE,
    .read = bmp280_pressure_read
};

int bmp280_probe(platform_device *pdev)
{
    struct device *dev = pdev->dev;
    int spiMaxFreq, spiBitsPerWord, ret;
    
    // Check if <spi-max-frequency> property is configured in device tree
    if(device_property_present(dev,"spi-max-frequency"))
    {
        // Get <spi-max-frequency> property actual value from device tree and store in spiMaxFreq
        ret = device_property_read_u32(dev,"spi-max-frequency",&spiMaxFreq);
        if(ret) {
            pr_err("bmp280_driver: Error: Unable to read <spi-max-frequency> property value\n");
            return -1;
        }
    }
    else {
        pr_info("bmp280_driver: <spi-max-frequency> property not configured in device tree\n");
        spiMaxFreq = 1000000; // Set default value to 1MHz
    }

    // Check if <spi-bits-per-word> property is configured in device tree
    if(device_property_present(dev,"spi-bits-per-word"))
    {
        ret = device_property_read_u32(dev,"spi-bits-per-word",&spiBitsPerWord);
        if(ret) {
            pr_err("bmp280_driver: Error: Unable to read <spi-bits-per-word> property value\n");
            return -1;
        }
    }
    else {
        pr_info("bmp280_driver: <spi-bits-per-word> property not configured in device tree\n");
        spiBitsPerWord = 8; // 1MHz
    }
    

    return 0;
}

int bmp280_remove(platform_device *pdev)
{
    return 0;
}

ssize_t bmp280_temp_read(struct file *file, char __user *buffer, size_t len, loff_t *offset)
{
    int to_copy, not_copied, delta;

    to_copy = min(len, (size_t)strlen(temp_buffer) - *offset);
    not_copied = copy_to_user(buffer, temp_buffer + *offset, to_copy);
    delta = to_copy - not_copied;
    *offset += delta;
    return delta; 
}

ssize_t bmp280_pressure_read (struct file *file, char __user *buffer, size_t len, loff_t *offset)
{
    int to_copy, not_copied, delta;

    to_copy = min(len, (size_t)strlen(pressure_buffer) - *offset);
    not_copied = copy_to_user(buffer, pressure_buffer + *offset, to_copy);
    delta = to_copy - not_copied;
    *offset += delta;
    return delta;
}


static int bmp280_init(void)
{
    // Register temperature device 
    temp_major = register_chrdev(0, DEVICE_NAME_TEMP, &temp_fops);
    if (temp_major < 0) {
        pr_err("Failed to register temperature device\n");
        return temp_major;
    }

    // Register pressure device
    pressure_major = register_chrdev(0, DEVICE_NAME_PRESSURE, &pressure_fops);
    if (pressure_major < 0) {
        pr_err("Failed to register pressure device\n");
        return pressure_major;
    }

    // Create device class
    bmp280_class = class_create(CLASS_NAME);
    if (IS_ERR(bmp280_class)) {
        pr_err("Failed to create device class\n");
        unregister_chrdev(temp_major, DEVICE_NAME_TEMP);
        unregister_chrdev(pressure_major, DEVICE_NAME_PRESSURE);
        return PTR_ERR(bmp280_class);
    }

    // Create temperature device node
    temp_device = device_create(bmp280_class, NULL, MKDEV(temp_major, 0), NULL, DEVICE_NAME_TEMP);
    if (IS_ERR(temp_device)) {
        pr_err("Failed to create temperature device node\n");
        class_destroy(bmp280_class);
        unregister_chrdev(temp_major, DEVICE_NAME_TEMP);
        unregister_chrdev(pressure_major, DEVICE_NAME_PRESSURE);
        return PTR_ERR(temp_device);
    }

    // Create pressure device node
    pressure_device = device_create(bmp280_class, NULL, MKDEV(pressure_major, 0), NULL, DEVICE_NAME_PRESSURE);
    if (IS_ERR(pressure_device)) {
        pr_err("Failed to create pressure device node\n");
        device_destroy(bmp280_class, MKDEV(temp_major, 0));
        class_destroy(bmp280_class);
        unregister_chrdev(temp_major, DEVICE_NAME_TEMP);
        unregister_chrdev(pressure_major, DEVICE_NAME_PRESSURE);
        return PTR_ERR(pressure_device);
    }
    
    pr_info("BMP280 module initialized: temp_major=%d, pressure_major=%d\n", temp_major, pressure_major);

    return 0;
}

static void bmp280_deinit(void)
{
     // Remove device nodes and class
    device_destroy(bmp280_class, MKDEV(temp_major, 0));
    device_destroy(bmp280_class, MKDEV(pressure_major, 0));
    class_destroy(bmp280_class);

    // Unregister character devices
    unregister_chrdev(temp_major, DEVICE_NAME_TEMP);
    unregister_chrdev(pressure_major, DEVICE_NAME_PRESSURE);
    pr_info("BMP280 module exited\n");
}



module_init(bmp280_init);
module_exit(bmp280_deinit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Device driver used for registering a character device.");
MODULE_AUTHOR("Leonard");
