# BMP280 Driver for CH32V003 with ch32fun

This project demonstrates how to use the BMP280 temperature and pressure sensor with the CH32V003 microcontroller with [CH32fun framework](https://github.com/cnlohr/ch32fun/tree/master). The project includes initialization of the I2C bus, reading data from the BMP280 sensor, and printing the temperature and pressure values to the console.

## Features

- Temperature and pressure measurement using BMP280 sensor
- I2C communication with the BMP280 sensor

## Hardware Requirements

- CH32V003 microcontroller
- BMP280 temperature and humidity sensor module

## Usage

 - Init I2C bus with `i2c_init()`
 - Declare variable of `BMP280Data` to store result
 - Run `BMP280_read()` to fetch sensor data

**BMP280Data** struct:
 - temperature - temperature in hundredths of degrees Celsius (e.g., 2234 means 22.34°C)
 - pressure - Compensated pressure in Pa (Pascal)

To convert pascals to millimeters of mercury, you divide the pressure in pascals by 133.322.

## Example Code

### main.c

The main application code, including initialization, data reading, and console output.

```c
// filepath: /home/pavlo/Documents/PlatformIO/Projects/BMP280-driver/src/main.c
#include "ch32fun.h"
#include "bmp280.h"

int main() {
    BMP280Data data;

    SystemInit();

    i2c_init();

    while (1)
    {   
        if(BMP280_read(&data) == BMP280_OK) {
            printf("Temperature: %d.%02d°C, Pressure: %d Pa\n", 
                   data.temperature / 100, data.temperature % 100, data.pressure);
        }
        else {
            printf("Error reading BMP280 sensor\n");
        }
        Delay_Ms(1000);
    }
}
```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
