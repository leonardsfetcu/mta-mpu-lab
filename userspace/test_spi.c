#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <math.h>

#define SPI_DEVICE "/dev/spidev0.0"
#define SPI_MODE SPI_MODE_0
#define SPI_SPEED 100000
#define SPI_BITS 8

#define REG_CHIPID 0xD0
#define REG_CTRL_MEAS 0xF4
#define REG_PRESS_MSB 0xF7
#define REG_CONFIG 0xF5
#define REG_RESET 0xE0
#define REG_TEMP_MSB 0xFA
#define BMP280_CHIP_ID 0x58

/* *    osrs_p[2:0] = x4 =      011
*       osrs_t[2:0] = x1 =      001
*       mode[1:0] = normal =    11
*       ==> ctrl_meas = 0010 1111 = 2F
*
*       t_stdby[2:0] = 0.5ms = 000
*       IIR_filter[2:0] = 16 = 100
*       ==> config = 000 100 00 = 10
* */

struct bmp280_calib {
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
};

int init_spi() {
    int spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) {
        perror("Failed to open SPI device");
        return -1;
    }

    uint8_t mode = SPI_MODE;
    uint32_t speed = SPI_SPEED;
    uint8_t bits = SPI_BITS;

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) == -1) {
        perror("Failed to set SPI mode");
        return -1;
    }
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
        perror("Failed to set bits per word");
        return -1;
    }
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
        perror("Failed to set SPI speed");
        return -1;
    }

    printf("SPI initialized: Mode %d, Speed %d Hz, Bits %d\n", mode, speed, bits);
    return spi_fd;
}

// Reading from register address function
int spi_read_register(int fd, uint8_t reg, uint8_t *data, size_t length) {
    uint8_t tx[1] = {reg | 0x80}; // Bitul MSB 1 pentru citire
    uint8_t rx[length + 1];

    struct spi_ioc_transfer transfer = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = length + 1,
        .speed_hz = SPI_SPEED,
        .bits_per_word = SPI_BITS,
        .cs_change = 0,
    };
    

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &transfer) < 0) {
        perror("Failed to read register");
        return -1;
    }
  
    memcpy(data, rx + 1, length); // Ignorăm primul byte
    return 0;
}

// Writing to register address function
int spi_write_register(int fd, uint8_t reg, uint8_t value) {
    
    uint8_t tx[] = {reg & 0x7F, value};

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .len = sizeof(tx),
        .speed_hz = SPI_SPEED,
        .bits_per_word = 8,
        .cs_change = 0
    };

    // Send data through SPI bus
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("Failed to write register");
        return -1;
    }

    return 0;
}

int read_calibration_data(int fd, struct bmp280_calib *calib) {
    uint8_t calib_data[24];
    if (spi_read_register(fd, 0x88, calib_data, 24) < 0) {
        return -1;
    }

    calib->dig_T1 = calib_data[0] | (calib_data[1] << 8);
    calib->dig_T2 = calib_data[2] | (calib_data[3] << 8);
    calib->dig_T3 = calib_data[4] | (calib_data[5] << 8);
    calib->dig_P1 = calib_data[6] | (calib_data[7] << 8);
    calib->dig_P2 = calib_data[8] | (calib_data[9] << 8);
    calib->dig_P3 = calib_data[10] | (calib_data[11] << 8);
    calib->dig_P4 = calib_data[12] | (calib_data[13] << 8);
    calib->dig_P5 = calib_data[14] | (calib_data[15] << 8);
    calib->dig_P6 = calib_data[16] | (calib_data[17] << 8);
    calib->dig_P7 = calib_data[18] | (calib_data[19] << 8);
    calib->dig_P8 = calib_data[20] | (calib_data[21] << 8);
    calib->dig_P9 = calib_data[22] | (calib_data[23] << 8);

    return 0;
}

int32_t compensate_temperature(int32_t adc_T, struct bmp280_calib *calib, int32_t *t_fine) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)calib->dig_T1 << 1))) * ((int32_t)calib->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib->dig_T1)) * ((adc_T >> 4) - ((int32_t)calib->dig_T1))) >> 12) *
            ((int32_t)calib->dig_T3)) >>
           14;
    *t_fine = var1 + var2;
    T = (*t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t compensate_pressure(int32_t adc_P, struct bmp280_calib *calib, int32_t t_fine) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib->dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib->dig_P5) << 17);
    var2 = var2 + (((int64_t)calib->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib->dig_P3) >> 8) + ((var1 * (int64_t)calib->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib->dig_P1) >> 33;

    if (var1 == 0) {
        return 0;
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib->dig_P7) << 4);
    return (uint32_t)p;
}

int main() {
    int fd = init_spi(SPI_DEVICE);
    if (fd < 0) {
        perror("Failed to open SPI device");
        return -1;
    }
    
    // Soft reset the sensor
    if (spi_write_register(fd,REG_RESET, 0xB6) < 0) {
        printf("Failed to reset the sensor\n");
        close(fd);
        return -1;
    }
    
    // Check sensor ID value
    uint8_t chip_id;
    if (spi_read_register(fd, REG_CHIPID, &chip_id, 1) < 0 || chip_id != BMP280_CHIP_ID) {
        printf("Failed to detect BMP280 sensor (Chip ID: 0x%02X)\n", chip_id);
        close(fd);
        return -1;
    }
    printf("BMP280 detected (Chip ID: 0x%02X)\n", chip_id);

    // Initialise sensor: configure normal mode + base filter + standby 1sec 
    if (spi_write_register(fd,REG_CTRL_MEAS, 0x2F) < 0) { // Normal mode, oversampling 1x
        return -1;
    }
    if (spi_write_register(fd,REG_CONFIG, 0x10) < 0) { // 0.5ms stdby, filter coef 16
        return -1;
    }

    struct bmp280_calib calib;
    if (read_calibration_data(fd, &calib) < 0) {
        printf("Failed to read calibration data\n");
        close(fd);
        return -1;
    }

    // Main loop
    while (1) {
        uint8_t data[6];
        if (spi_read_register(fd, REG_PRESS_MSB, data, 6) < 0) {
            printf("Failed to read sensor data\n");
            break;
        }

        int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
        int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

        int32_t t_fine;
        int32_t temperature = compensate_temperature(adc_T, &calib, &t_fine);
        uint32_t pressure = compensate_pressure(adc_P, &calib, t_fine);

        printf("Temperature: %.2f °C\nPressure: %.2f hPa\n\n", temperature / 100.0, pressure / 25600.0);
        usleep(1000000);
    }

    close(fd);
    return 0;
}
