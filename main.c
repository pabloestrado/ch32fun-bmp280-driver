# include "ch32fun.h"
# include "bmp280.h"


int main() {
    BMP280Data data;

    SystemInit();

    i2c_init();

    while (1)
    {   
        if(BMP280_read(&data) == BMP280_OK) {
            printf("Temperature: %d Pressure: %d\n", (int)data.temperature, (int)data.pressure);
        }
        else {
            printf("Error reading BMP280 sensor\n");
        }
        Delay_Ms(1000);
    }
    
}