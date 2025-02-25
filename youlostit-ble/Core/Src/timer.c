/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"


void timer_init(TIM_TypeDef* timer)
{
	// TODO implement this
	// Enable the clock for Timer 2 in RCC
	    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // Enable TIM2 clock (APB1 bus)

	// Stop the timer and clear out any timer state & reset all counters
	    timer->CR1 &= ~TIM_CR1_CEN; // Disable the timer
	    timer->CNT = 0;            // Reset the counter
	   	timer->SR = 0;             // Clear any interrupt flags


	// Setup the timer to auto-reload when the max value is reached
	   	timer->ARR = 0xFFFF;       // Set the max count value

	// Enable the timer’s interrupt both internally and in the interrupt controller (NVIC)
	    timer->DIER |= TIM_DIER_UIE; // Update Interrupt Enable (UIE)
	    NVIC_EnableIRQ(TIM2_IRQn);  // Enable the interrupt in the NVIC
		NVIC_SetPriority(TIM2_IRQn, 1); // Set interrupt priority (lower number = higher priority)

	// Enable the timer
	   	timer->CR1 |= TIM_CR1_CEN; // Enable the timer (CEN)
}

void timer_reset(TIM_TypeDef* timer){
	// TODO implement this
	// Reset the counter of the specified timer to 0
	timer->CNT = 0;
}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
		// TODO implement this
		// Step 1: Calculate the timer's frequency based on the system clock
	    uint32_t system_clock = 8000000;  // Assuming system clock is 4 MHz (adjust based on actual clock)

	    // Step 2: Determine the prescaler value and the period for milliseconds
	    uint32_t prescaler = (system_clock / 1000) - 1;  // To get 1 ms ticks
	    uint32_t period = period_ms;  // Period in milliseconds

	    // Step 3: Set the prescaler and auto-reload register for the timer
	    timer->PSC = prescaler;  // Set the prescaler to divide the clock
	    timer->ARR = period - 1; // Set the period value for how many ticks it should wait

	    // Step 4: Enable the timer (done in the initialization)
}
