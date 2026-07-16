#include "sc7a20h.h"
#include <string.h>

/* Soft I2C pin definitions */
#define I2C_SDA_PORT GPIOB
#define I2C_SDA_PIN  GPIO_PIN_7
#define I2C_SCL_PORT GPIOB
#define I2C_SCL_PIN  GPIO_PIN_6

#define I2C_DELAY() for(volatile int _i=0;_i<30;_i++)

#define SDA_LOW()  HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET)
#define SDA_HIGH() HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET)
#define SDA_READ() HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN)
#define SCL_LOW()  HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET)
#define SCL_HIGH() HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET)

static void I2C_Start(void)
{
    SDA_HIGH();
    I2C_DELAY();
    SCL_HIGH();
    I2C_DELAY();
    SDA_LOW();
    I2C_DELAY();
    SCL_LOW();
    I2C_DELAY();
}

static void I2C_Stop(void)
{
    SDA_LOW();
    I2C_DELAY();
    SCL_HIGH();
    I2C_DELAY();
    SDA_HIGH();
    I2C_DELAY();
}

static int I2C_WriteByte(uint8_t data)
{
    for (int i = 0; i < 8; i++)
    {
        if (data & 0x80)
            SDA_HIGH();
        else
            SDA_LOW();
        data <<= 1;
        I2C_DELAY();
        SCL_HIGH();
        I2C_DELAY();
        SCL_LOW();
        I2C_DELAY();
    }
    SDA_HIGH();
    I2C_DELAY();
    SCL_HIGH();
    I2C_DELAY();
    int ack = SDA_READ() ? 1 : 0;
    SCL_LOW();
    I2C_DELAY();
    return ack;
}

static uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t data = 0;
    SDA_HIGH();
    for (int i = 0; i < 8; i++)
    {
        data <<= 1;
        SCL_HIGH();
        I2C_DELAY();
        if (SDA_READ()) data |= 1;
        SCL_LOW();
        I2C_DELAY();
    }
    if (ack)
        SDA_LOW();
    else
        SDA_HIGH();
    I2C_DELAY();
    SCL_HIGH();
    I2C_DELAY();
    SCL_LOW();
    I2C_DELAY();
    return data;
}

static int I2C_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    I2C_Start();
    if (I2C_WriteByte(dev_addr)) { I2C_Stop(); return 1; }
    if (I2C_WriteByte(reg))      { I2C_Stop(); return 1; }
    if (I2C_WriteByte(data))     { I2C_Stop(); return 1; }
    I2C_Stop();
    return 0;
}

static int I2C_ReadRegs(uint8_t dev_addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    I2C_Start();
    if (I2C_WriteByte(dev_addr)) { I2C_Stop(); return 1; }
    if (I2C_WriteByte(reg))      { I2C_Stop(); return 1; }
    I2C_Start();
    if (I2C_WriteByte(dev_addr | 1)) { I2C_Stop(); return 1; }
    for (uint8_t i = 0; i < len; i++)
        buf[i] = I2C_ReadByte(i < len - 1);
    I2C_Stop();
    return 0;
}

void SC7A20H_SoftI2C_Init(void)
{
    SDA_HIGH();
    SCL_HIGH();
}

#define SL_SC7A20H_FIFO_CTRL_REG   (unsigned char)0X2E
#define SL_SC7A20H_FIFO_SRC_REG    (unsigned char)0X2F
#define SL_SC7A20H_IIC_OUT_X_L     (unsigned char)0XA8
#define SL_SC7A20H_SPI_OUT_X_L     (unsigned char)0X28

#define SL_SC7A20H_INIT_REG1_NUM 8
static unsigned char SL_SC7A20H_INIT_REG1[SL_SC7A20H_INIT_REG1_NUM*3]=
{
    0x24,0x80,0x00,
    0x2E,0x00,0x00,
    0x1f,0x00,0x00,
    0x20,0x47,0x00,
    0x21,0x70,0x00,
    0x23,0x98,0x00,
    0x24,0x40,0x00,
    0x2E,0x4F,0x00,
};

signed char SC7A20H_Driver_Init(SC7A20H_Interface_t interface)
{
    unsigned char i=0;
    unsigned char id1=0;
    unsigned char id2=0;

    SC7A20H_SoftI2C_Init();

    I2C_WriteReg(SC7A20H_I2C_ADDR, 0x57, 0x00);
    for(i=0;i<SL_SC7A20H_INIT_REG1_NUM;i++)
    {
        I2C_WriteReg(SC7A20H_I2C_ADDR, SL_SC7A20H_INIT_REG1[3*i], SL_SC7A20H_INIT_REG1[3*i+1]);
    }

    I2C_ReadRegs(SC7A20H_I2C_ADDR, 0x0f, 1, &id1);
    I2C_ReadRegs(SC7A20H_I2C_ADDR, 0x70, 1, &id2);

    if (id1 == 0x11 || id2 == 0x11) return 0x11;
    if (id1 == 0 && id2 == 0) return 0x01;
    return (signed char)(id1 ? id1 : id2);
}

unsigned char SC7A20H_Read_FIFO_Buf(SC7A20H_Interface_t interface, signed short *x_buf, signed short *y_buf, signed short *z_buf)
{
    unsigned char  i=0;
    unsigned char  sc7a20_data[7];
    unsigned char  SL_FIFO_ACCEL_NUM;

    I2C_ReadRegs(SC7A20H_I2C_ADDR, SL_SC7A20H_FIFO_SRC_REG, 1, &SL_FIFO_ACCEL_NUM);

    if (SL_FIFO_ACCEL_NUM & 0x40)
        SL_FIFO_ACCEL_NUM = 32;
    else
        SL_FIFO_ACCEL_NUM = SL_FIFO_ACCEL_NUM & 0x1f;

    for(i=0;i<SL_FIFO_ACCEL_NUM;i++)
    {
        I2C_ReadRegs(SC7A20H_I2C_ADDR, SL_SC7A20H_IIC_OUT_X_L, 6, &sc7a20_data[1]);

        x_buf[i] =(signed short int)(((unsigned char)sc7a20_data[2] * 256 ) + (unsigned char)sc7a20_data[1]);
        y_buf[i] =(signed short int)(((unsigned char)sc7a20_data[4] * 256 ) + (unsigned char)sc7a20_data[3]);
        z_buf[i] =(signed short int)(((unsigned char)sc7a20_data[6] * 256 ) + (unsigned char)sc7a20_data[5]);
    }

    I2C_WriteReg(SC7A20H_I2C_ADDR, SL_SC7A20H_FIFO_CTRL_REG, 0X00);
    I2C_WriteReg(SC7A20H_I2C_ADDR, SL_SC7A20H_FIFO_CTRL_REG, 0X4F);

    return SL_FIFO_ACCEL_NUM;
}
