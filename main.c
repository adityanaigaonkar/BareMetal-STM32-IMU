#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

/*
========================================================
SELF CALIBRATING IMU SYSTEM
STM32F4 + MPU6050
FINAL PITCH FIX VERSION
========================================================

MPU6050 I2C:
PB8 -> SCL
PB9 -> SDA

UART:
PA9 -> TX

CONNECT:
PA9 -> ESP32 GPIO16
GND -> ESP32 GND

UART BAUD:
9600
========================================================
*/

#define MPU6050_ADDR  0x68

#define PWR_MGMT_1    0x6B

#define ACCEL_XOUT_H  0x3B
#define GYRO_XOUT_H   0x43

#define RAD_TO_DEG    57.295779f

char txBuffer[128];

int16_t ax, ay, az;
int16_t gx, gy;

float AccX, AccY, AccZ;

float GyroX, GyroY;

float accelRoll;
float accelPitch;

float roll  = 0.0f;
float pitch = 0.0f;

float gyroX_offset = 0.0f;
float gyroY_offset = 0.0f;

float roll_offset  = 0.0f;
float pitch_offset = 0.0f;

float dt = 0.01f;

/* =====================================================
   DELAY
===================================================== */

void delay_ms(uint32_t ms)
{
    volatile uint32_t i;

    while(ms--)
    {
        for(i = 0; i < 16000; i++);
    }
}

/* =====================================================
   UART1 INIT
===================================================== */

void uart1_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* PA9 = TX */

    GPIOA->MODER &= ~(3 << (9 * 2));

    GPIOA->MODER |=  (2 << (9 * 2));

    GPIOA->AFR[1] &= ~(0xF << ((9 - 8) * 4));

    GPIOA->AFR[1] |=  (7 << ((9 - 8) * 4));

    /* 9600 baud */

    USART1->BRR = 0x0683;

    USART1->CR1 |= USART_CR1_TE;

    USART1->CR1 |= USART_CR1_UE;
}

/* =====================================================
   UART SEND CHAR
===================================================== */

void uart_send_char(char c)
{
    while(!(USART1->SR & USART_SR_TXE));

    USART1->DR = c;
}

/* =====================================================
   UART SEND STRING
===================================================== */

void uart_send_string(char *str)
{
    while(*str)
    {
        uart_send_char(*str++);
    }
}

/* =====================================================
   I2C INIT
===================================================== */

void i2c1_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* PB8 SCL */

    GPIOB->MODER &= ~(3 << (8 * 2));
    GPIOB->MODER |=  (2 << (8 * 2));

    /* PB9 SDA */

    GPIOB->MODER &= ~(3 << (9 * 2));
    GPIOB->MODER |=  (2 << (9 * 2));

    GPIOB->OTYPER |= (1 << 8);
    GPIOB->OTYPER |= (1 << 9);

    GPIOB->AFR[1] |= (4 << 0);
    GPIOB->AFR[1] |= (4 << 4);

    I2C1->CR2 = 16;

    I2C1->CCR = 80;

    I2C1->TRISE = 17;

    I2C1->CR1 |= I2C_CR1_PE;
}

/* =====================================================
   I2C START
===================================================== */

void i2c_start(void)
{
    I2C1->CR1 |= I2C_CR1_START;

    while(!(I2C1->SR1 & I2C_SR1_SB));
}

/* =====================================================
   I2C STOP
===================================================== */

void i2c_stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

/* =====================================================
   I2C ADDRESS
===================================================== */

void i2c_address(uint8_t address)
{
    volatile uint32_t temp;

    I2C1->DR = address;

    while(!(I2C1->SR1 & I2C_SR1_ADDR));

    temp = I2C1->SR1;
    temp = I2C1->SR2;

    (void)temp;
}

/* =====================================================
   I2C WRITE
===================================================== */

void i2c_write(uint8_t data)
{
    while(!(I2C1->SR1 & I2C_SR1_TXE));

    I2C1->DR = data;
}

/* =====================================================
   I2C READ
===================================================== */

uint8_t i2c_read_nack(void)
{
    I2C1->CR1 &= ~I2C_CR1_ACK;

    while(!(I2C1->SR1 & I2C_SR1_RXNE));

    return I2C1->DR;
}

/* =====================================================
   MPU WRITE
===================================================== */

void mpu_write(uint8_t reg, uint8_t data)
{
    i2c_start();

    i2c_address(MPU6050_ADDR << 1);

    i2c_write(reg);

    i2c_write(data);

    i2c_stop();
}

/* =====================================================
   MPU READ
===================================================== */

uint8_t mpu_read(uint8_t reg)
{
    uint8_t data;

    i2c_start();

    i2c_address(MPU6050_ADDR << 1);

    i2c_write(reg);

    i2c_start();

    i2c_address((MPU6050_ADDR << 1) | 1);

    data = i2c_read_nack();

    i2c_stop();

    return data;
}

/* =====================================================
   MPU INIT
===================================================== */

void mpu_init(void)
{
    mpu_write(PWR_MGMT_1, 0x00);

    delay_ms(100);
}

/* =====================================================
   READ MPU
===================================================== */

void read_mpu(void)
{
    ax =
        (mpu_read(ACCEL_XOUT_H) << 8) |
         mpu_read(ACCEL_XOUT_H + 1);

    ay =
        (mpu_read(ACCEL_XOUT_H + 2) << 8) |
         mpu_read(ACCEL_XOUT_H + 3);

    az =
        (mpu_read(ACCEL_XOUT_H + 4) << 8) |
         mpu_read(ACCEL_XOUT_H + 5);

    gx =
        (mpu_read(GYRO_XOUT_H) << 8) |
         mpu_read(GYRO_XOUT_H + 1);

    gy =
        (mpu_read(GYRO_XOUT_H + 2) << 8) |
         mpu_read(GYRO_XOUT_H + 3);
}

/* =====================================================
   GYRO CALIBRATION
===================================================== */

void calibrate_gyro(void)
{
    long gx_sum = 0;
    long gy_sum = 0;

    for(int i = 0; i < 1000; i++)
    {
        read_mpu();

        gx_sum += gx;
        gy_sum += gy;

        delay_ms(2);
    }

    gyroX_offset =
        (gx_sum / 1000.0f) / 131.0f;

    gyroY_offset =
        (gy_sum / 1000.0f) / 131.0f;
}

/* =====================================================
   ZERO CALIBRATION
===================================================== */

void calibrate_zero_position(void)
{
    float rollSum  = 0.0f;
    float pitchSum = 0.0f;

    for(int i = 0; i < 500; i++)
    {
        read_mpu();

        AccX = ax / 16384.0f;
        AccY = ay / 16384.0f;
        AccZ = az / 16384.0f;

        accelRoll =
            atan2f(AccY, AccZ)
            * RAD_TO_DEG;

        accelPitch =
            atan2f(
                -AccX,
                sqrtf(
                    (AccY * AccY) +
                    (AccZ * AccZ)
                )
            )
            * RAD_TO_DEG;

        rollSum  += accelRoll;
        pitchSum += accelPitch;

        delay_ms(2);
    }

    roll_offset = (rollSum / 500.0f) + 1.8f;

    /* FINAL PITCH FIX */

    pitch_offset =
        (pitchSum / 500.0f) - 1.5f;
}

/* =====================================================
   MAIN
===================================================== */

int main(void)
{
    uart1_init();

    i2c1_init();

    delay_ms(100);

    uart_send_string("IMU START\r\n");

    mpu_init();

    uart_send_string("CALIBRATING...\r\n");

    calibrate_gyro();

    calibrate_zero_position();

    uart_send_string("SYSTEM READY\r\n");

    while(1)
    {
        read_mpu();

        AccX = ax / 16384.0f;
        AccY = ay / 16384.0f;
        AccZ = az / 16384.0f;

        GyroX =
            (gx / 131.0f)
            - gyroX_offset;

        GyroY =
            (gy / 131.0f)
            - gyroY_offset;

        /* REMOVE GYRO NOISE */

        if(fabs(GyroX) < 0.5f)
        {
            GyroX = 0.0f;
        }

        if(fabs(GyroY) < 0.5f)
        {
            GyroY = 0.0f;
        }

        /* ACCEL ANGLES */

        accelRoll =
            atan2f(AccY, AccZ)
            * RAD_TO_DEG;

        accelPitch =
            atan2f(
                -AccX,
                sqrtf(
                    (AccY * AccY) +
                    (AccZ * AccZ)
                )
            )
            * RAD_TO_DEG;

        /* COMPLEMENTARY FILTER */

        roll =
            0.98f * (roll + GyroX * dt) +
            0.02f * (accelRoll - roll_offset);

        pitch =
            0.97f * (pitch + GyroY * dt) +
            0.03f * (accelPitch - pitch_offset);

        /* AUTO RETURN TO ZERO */

        if(
            fabs(GyroX) < 0.15f &&
            fabs(GyroY) < 0.15f
        )
        {
            /* ROLL AUTO CENTER */

            if(roll > 0.0f)
            {
                roll -= 0.05f;
            }
            else if(roll < 0.0f)
            {
                roll += 0.05f;
            }

            /* PITCH AUTO CENTER */

            if(pitch > 0.0f)
            {
                pitch -= 0.08f;
            }
            else if(pitch < 0.0f)
            {
                pitch += 0.08f;
            }
        }

        /* HARD ZERO LOCK */

        if(fabs(roll) < 0.10f)
        {
            roll = 0.0f;
        }

        if(fabs(pitch) < 0.20f)
        {
            pitch = 0.0f;
        }

        /* UART OUTPUT */

        sprintf(
            txBuffer,
            "ROLL: %.2f  PITCH: %.2f\r\n",
            roll,
            pitch
        );

        uart_send_string(txBuffer);

        delay_ms(10);
    }
}
