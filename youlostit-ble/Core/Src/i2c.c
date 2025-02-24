/*
 * i2c.c
 *
 *  Created on: Feb 5, 2025
 *      Author: gisellemendoza
 */

#include <stdio.h>
#include "stm32l475xx.h"
#include "i2c.h"

#define TIMEOUT 1000000

void i2c_init() {
    // Disable I2C2 for configuration
    I2C2->CR1 &= ~I2C_CR1_PE;

    // Enable clocks for GPIOB and I2C2
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

    // Configure PB10 (SCL) and PB11 (SDA) for Alternate Function (AF4 for I2C)
    GPIOB->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11);
    GPIOB->MODER |= (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1);

    // Configure Alternate Function (AF4) for PB10 and PB11
    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL11);
    GPIOB->AFR[1] |= (GPIO_AFRH_AFSEL10_2 | GPIO_AFRH_AFSEL11_2);

    // Set PB10 and PB11 to Open-Drain mode
    GPIOB->OTYPER |= (GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11);

    // Enable Pull-up resistors for PB10 and PB11
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD10 | GPIO_PUPDR_PUPD11);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD10_1 | GPIO_PUPDR_PUPD11_1);

    // Set speed for PB10 and PB11
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED11);
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED10_0 | GPIO_OSPEEDR_OSPEED11_0);

    // Enable interrupts for TX, RX, and Transfer Complete
    I2C2->CR1 |= (I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_TCIE);

    // Configure Timing Register for 400 kHz I2C Fast Mode
    I2C2->TIMINGR = (0U << I2C_TIMINGR_PRESC_Pos) | // Prescaler = 0, using 4 MHz clock
    		(0xC7 << I2C_TIMINGR_SCLL_Pos) | // SCL low period = 199 (Tlow)
            (0xC3 << I2C_TIMINGR_SCLH_Pos) | // SCL high period = 195 (Thigh)
            (0x02 << I2C_TIMINGR_SDADEL_Pos) | // Data setup delay = 2
            (0x04 << I2C_TIMINGR_SCLDEL_Pos); // Data hold delay = 4

    // Enable I2C2 Peripheral
    I2C2->CR1 |= I2C_CR1_PE;
}

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len) {
    uint32_t timeout_counter = TIMEOUT;

    while ((I2C2->ISR & I2C_ISR_BUSY) && timeout_counter--) {
        if (timeout_counter == 0) {
            printf("I2C busy timeout\n");
            return 0; // Timeout error
        }
    }

    if (dir == 0) {  // WRITE operation
        I2C2->CR2 = (address << 1) | (0 << 10) | (len << 16);
        I2C2->CR2 |= I2C_CR2_START;

        for (uint8_t i = 0; i < len; i++) {
            while (!(I2C2->ISR & I2C_ISR_TXIS));  // Wait for TX buffer
            I2C2->TXDR = data[i];  // Send byte
        }

        while (!(I2C2->ISR & I2C_ISR_TC)) {
            if (I2C2->ISR & I2C_ISR_NACKF) {
                I2C2->ICR |= I2C_ICR_NACKCF;
                I2C2->CR2 |= I2C_CR2_STOP;
                return 0; // NACK received
            }
        }

        I2C2->CR2 |= I2C_CR2_STOP;
        return 1;
    } else {  // READ operation
        I2C2->CR2 = (address << 1) | (1 << 10) | (len << 16);
        I2C2->CR2 |= I2C_CR2_START;

        for (uint8_t i = 0; i < len; i++) {
            while (!(I2C2->ISR & I2C_ISR_RXNE));
            data[i] = I2C2->RXDR;
        }

        while (!(I2C2->ISR & I2C_ISR_TC)) {
            if (I2C2->ISR & I2C_ISR_NACKF) {
                I2C2->CR2 |= I2C_CR2_STOP;
                while (!(I2C2->ISR & I2C_ISR_STOPF));
                I2C2->ICR = I2C_ICR_STOPCF;
                return 0; // NACK received
            }
        }

        I2C2->CR2 |= I2C_CR2_STOP;
        return 1;
    }
}
