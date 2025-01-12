#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/property.h>
#include <linux/of_device.h>

#define DEVICE_NAME "bmp280_sensor"
#define CLASS_NAME "bmp280_sensor_class"
#define BMP280_REG_CHIPID 0xD0
#define REG_CTRL_MEAS 0xF4
#define BMP280_REG_PRESS_MSB 0xF7
#define REG_CONFIG 0xF5
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_CHIP_ID 0x58


// Functions declaration
static int bmp280_probe(struct spi_device *client);
static void bmp280_remove(struct spi_device *client);
static ssize_t bmp280_device_read(struct file *file, char __user *buffer, size_t len, loff_t *offset);
static int bmp280_write_register(uint8_t reg, uint8_t value);
static int bmp280_read_register(uint8_t reg, uint8_t *data, size_t len);
static int read_calibration_data(void);



// Device tree bindings
static struct of_device_id bmp280_of_ids[] = {
    {
        .compatible = "mta-labs,bmp280-spi",
    }, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bmp280_of_ids);

// Driver configuration
static struct spi_driver bmp280_driver = {
    .probe = bmp280_probe,
    .remove = bmp280_remove,
    .driver = {
        .name = "bmp280-spi",
        .of_match_table = bmp280_of_ids,
    },
};

static struct bmp280_calib {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} calib;


static int driver_major;

static struct spi_device *spi_dev;

static struct class *bmp280_class;
static struct device *bmp280_device;

// Structurile pentru operațiile dispozitivelor
static const struct file_operations bmp280_fops = {
    .read = bmp280_device_read
};


int bmp280_probe(struct spi_device *client)
{    
    spi_dev = client;
    int ret;
    uint8_t id;

    // Initiate the SPI bus
    ret = spi_setup(spi_dev);
    if(ret < 0) {
		pr_err("BMP280 driver: Unable to setup SPI bus\n");
		return -1;
    }
    else
    {
        pr_info("BMP280 driver: SPI bus OK\n");
    }

    // Read Chip ID
    ret = bmp280_read_register(BMP280_REG_CHIPID,&id,sizeof(id));
	if (ret < 0) {
        pr_err("BMP280 driver: Failed to get ChipID\n");
        return ret;
    }
    else
    {
        pr_info("BMP280 driver: Chip ID: 0x%x\n", id);
    }

    // BMP280 configuration for normal mode, oversampling temp x1, oversampling pressure x4 
    if (bmp280_write_register(REG_CTRL_MEAS, 0x2F) < 0) {
        pr_err("BMP280 driver: Unable to set measuring operation config\n");
        return -1;
    }

    // BMP280 configuration for 0.5ms Tstdby, IIR filter coef = 16 and 4 wires SPI mode
    if (bmp280_write_register(REG_CONFIG, 0xA0) < 0) {
        pr_err("BMP280 driver: Unable to set sensor configuration parameters\n");
        return -1;
    }

    // Get calibration data
    if (read_calibration_data() < 0) {
        pr_err("BMP280 driver: Failed to read calibration data\n");
        return -1;
    }

    pr_info("BMP280 driver: Successfully probed\n");

    return 0;
}

// SPI reading from register function
int bmp280_read_register(uint8_t reg, uint8_t *data, size_t length) {
    uint8_t tx = reg | 0x80; //Set first bit to 1 to indicate a read operation
    uint8_t rx[length];
    struct spi_transfer transfer[] = {
        {
            .tx_buf = &tx,
            .len = sizeof(tx),
        },
        {
            .rx_buf = rx,
            .len = length,
        },
    };
    
    int ret = spi_sync_transfer(spi_dev, transfer, ARRAY_SIZE(transfer));
    if (ret < 0) {
        pr_err("BMP280 driver: Failed to read register 0x%02x: %d\n", reg, ret);
        return ret;
    }

    memcpy(data, rx, length);

    return 0;
}


// SPI writing to register function
static int bmp280_write_register(uint8_t reg, uint8_t value) {
    uint8_t tx[] = {reg & 0x7F, value};
    struct spi_transfer transfer[] = {
        {
            .tx_buf = tx,
            .len = sizeof(tx),
        }
    };

    struct spi_message msg;
    spi_message_init(&msg);
    spi_message_add_tail(&transfer[0], &msg);

    if (spi_sync(spi_dev, &msg) < 0) {
        pr_err("BMP280: Failed to read register 0x%x\n", reg);
        return -EIO;
    }

    return 0;
}

void bmp280_remove(struct spi_device *client)
{
    pr_info("BMP280 driver: Successfully removed\n");
}

// Temperature compesantion function
int32_t compensate_temperature(int32_t raw_temp, int32_t *t_fine) {
    int32_t var1, var2, T;
    var1 = ((((raw_temp >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    var2 = (((((raw_temp >> 4) - ((int32_t)calib.dig_T1)) * ((raw_temp >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
            ((int32_t)calib.dig_T3)) >>
           14;
    *t_fine = var1 + var2;
    T = (*t_fine * 5 + 128) >> 8;
    return T;
}

// Pressure compensation function
uint32_t compensate_pressure(int32_t adc_P, int32_t t_fine) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;

    if (var1 == 0) {
        return 0;
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    return (uint32_t)p;
}

// Raw data reading function
static int bmp280_read_raw_data(int *raw_temp, int *raw_press) {
    
    uint8_t buffer[6];
    if (bmp280_read_register(BMP280_REG_PRESS_MSB, buffer, 6) < 0) {
        pr_err("Failed to read temperature/pressure\n");
        return -1;
    }
    
    *raw_temp = (buffer[3] << 12) | (buffer[4] << 4) | (buffer[5] >> 4);
    *raw_press = (buffer[0] << 12) | (buffer[1] << 4) | (buffer[2] >> 4);

    return 0;
}

ssize_t bmp280_device_read(struct file *file, char __user *buffer, size_t len, loff_t *offset)
{
    int raw_temp, raw_press, t_fine;
    char result[64];

    // Get the raw data values for temperature and pressure
    bmp280_read_raw_data(&raw_temp, &raw_press);

    // Apply the compensation data and get the env real values
    int32_t temperature = compensate_temperature(raw_temp, &t_fine);
    int32_t pressure = compensate_pressure(raw_press, t_fine);

    // Build the output string
    snprintf(result, sizeof(result), "Temperature: %d.%02d °C\nPressure: %d.%02d hPa\n",
    temperature / 100, temperature % 100,pressure / 25600,(pressure % 25600) * 100 / 25600);

    int to_copy, not_copied, delta;
    
    // copy the data from kernel space to userspace
    to_copy = min(len, (size_t)strlen(result) - *offset);
    not_copied = copy_to_user(buffer, result + *offset, to_copy);
    delta = to_copy - not_copied;
    *offset += delta;
    return delta; 
}

static int read_calibration_data(void) {
    uint8_t calib_data[24];
    if (bmp280_read_register(0x88, calib_data, 24) < 0) {
        return -1;
    }

    calib.dig_T1 = calib_data[0] | (calib_data[1] << 8);
    calib.dig_T2 = calib_data[2] | (calib_data[3] << 8);
    calib.dig_T3 = calib_data[4] | (calib_data[5] << 8);
    calib.dig_P1 = calib_data[6] | (calib_data[7] << 8);
    calib.dig_P2 = calib_data[8] | (calib_data[9] << 8);
    calib.dig_P3 = calib_data[10] | (calib_data[11] << 8);
    calib.dig_P4 = calib_data[12] | (calib_data[13] << 8);
    calib.dig_P5 = calib_data[14] | (calib_data[15] << 8);
    calib.dig_P6 = calib_data[16] | (calib_data[17] << 8);
    calib.dig_P7 = calib_data[18] | (calib_data[19] << 8);
    calib.dig_P8 = calib_data[20] | (calib_data[21] << 8);
    calib.dig_P9 = calib_data[22] | (calib_data[23] << 8);

    return 0;
}


static int bmp280_init(void)
{   
    // Register platform driver
    if(spi_register_driver(&bmp280_driver)) {
        pr_err("BMP280 driver: Failed to register platform driver\n");
        return -1;
    }
    else {
        pr_info("BMP280 driver: Successfully registered platform driver\n");
    }

    // Register device
    driver_major = register_chrdev(0, DEVICE_NAME, &bmp280_fops);
    if (driver_major < 0) {
        pr_err("BMP280 driver: Failed to register temperature device\n");
        return driver_major;
    }

    // Create device class
    bmp280_class = class_create(THIS_MODULE,CLASS_NAME);
    if (IS_ERR(bmp280_class)) {
        pr_err("BMP280 driver: Failed to create device class\n");
        unregister_chrdev(driver_major, DEVICE_NAME);
        return PTR_ERR(bmp280_class);
    }

    // Create device node in /dev
    bmp280_device = device_create(bmp280_class, NULL, MKDEV(driver_major, 0), NULL, DEVICE_NAME);
    if (IS_ERR(bmp280_device)) {
        pr_err("BMP280 driver: Failed to create device node\n");
        class_destroy(bmp280_class);
        unregister_chrdev(driver_major, DEVICE_NAME);
        return PTR_ERR(bmp280_device);
    }

    pr_info("BMP280 driver: Successfully initialized. Major number: %d\n", driver_major);

    return 0;
}

static void bmp280_exit(void)
{
    // Remove device nodes and class
    device_destroy(bmp280_class, MKDEV(driver_major, 0));
    class_destroy(bmp280_class);

    // Unregister character device
    unregister_chrdev(driver_major, DEVICE_NAME);

    // Unregister platform driver
    spi_unregister_driver(&bmp280_driver);

    pr_info("BMP280 driver: Successfully exited\n");
}



//module_spi_driver(bmp280_driver);
module_init(bmp280_init);
module_exit(bmp280_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Device driver used for registering a character device.");
MODULE_AUTHOR("Leonard");
