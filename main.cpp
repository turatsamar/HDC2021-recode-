#include "mbed.h"
#include <string.h>
#include "hdc2021.h"

EventFlags event_flags;
I2C i2c(PB_9, PB_8);
InterruptIn drdy(PA_6);
BufferedSerial serial_port(USBTX, USBRX);
// main() runs in its own thread in the OS
static void log(const char* fmt, ...);


void dataReady(void)
{
    event_flags.set(1);
}


bool i2c_write(uint8_t addr, uint8_t reg, const uint8_t* src, size_t size)
{
    char data[size + 1];
    bool ok = false;

    do
    {
        data[0] = reg;
        memcpy(&data[1], src, size);

        if (0 != i2c.write(addr, (const char*)data, sizeof(data)))
            break;

        ok = true;

    } while (0);

    return ok;
}

bool i2c_read(uint8_t addr, uint8_t reg, uint8_t* dst, size_t size)
{
    bool ok = false;

    do
    {

        if (0 != i2c.write(addr, (const char*)&reg, 1))
            break;

        if(0 != i2c.read(addr, (char*)dst, size))
            break;

        ok = true;
    } while (0);

    return ok;
}

void wait_msec(int msec)
{
    wait_ns(msec * 1000);
}

void log(const char* fmt, ...)
{
    va_list args;
    char buffer[256];

    va_start(args, fmt);
    vsprintf(buffer, fmt, args);
    va_end(args);

    serial_port.write((const void*)buffer, strlen(buffer));
}

int main()
{
    HDC2021Err err;
    HDC2021Data data;
    uint8_t status;

    serial_port.set_baud(115200);
    serial_port.set_format(8, BufferedSerial::None, 1); //8bit NO-Parity STOP1

    log("heleo\n");
    i2c.frequency(100 * 1000);

    do
    {
        if ((err = hdc2021_init(0x80, i2c_write, i2c_read, wait_msec)) != HDC2021_ERR_NONE)
            break;

        hdc2021_setInterrupt(HDC2021_INT_EN_DRDY, HDC2021_INT_POL_LOW);

        if ((err = hdc2021_start(HDC2021_CC_1MIN)) != HDC2021_ERR_NONE)
            break;
        drdy.mode(PullNone);
        drdy.fall(&dataReady);

        do
        {
            event_flags.wait_all();
            event_flags.clear();

            if ((err = hdc2021_readData(&data)) == HDC2021_ERR_NONE)
            {
	            log("Temperature:%d\n", data.temperature);
	            log("Humidity:%d\n", data.humidity);
            }
            else
            {
	            log("HDC2021 Data Read Error:%s\n", hdc2021_getErrorString(err));
            }
            if ((err = hdc2021_readStatus(&status)) != HDC2021_ERR_NONE)
	            log("HDC2021 Status Read Error:%s\n", hdc2021_getErrorString(err));

            wait_msec(10);
        } while (1);
        
    } while (0);

	log("HDC2021 Error:%s\n", hdc2021_getErrorString(err));
}

