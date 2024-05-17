//====================================================================
// INCLUDE FILES
//====================================================================
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>

//====================================================================
// LEFT THRESHOLDS
//====================================================================
const float threshold_voltage_left_upper = 2.7;
const float threshold_voltage_left_lower = 2.4;
    
//====================================================================
// MIDDLE THRESHOLDS
//====================================================================
const float threshold_voltage_middle_upper = 2.8;
const float threshold_voltage_middle_lower = 2.5;

//====================================================================
// RIGHT THRESHOLDS
//====================================================================
const float threshold_voltage_right_upper = 2.7;
const float threshold_voltage_right_lower = 2.4;

//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void helloWorld();
void init(void);
void loop();
void low_power_mode_enabled(void);
void low_power_mode_disabled(void);

//====================================================================
// MAIN FUNCTION
//====================================================================
int main (void)
{
    helloWorld();
    init();

    while (1) {
        loop();
        // low_power_mode_enabled();
        low_power_mode_disabled();

    }
}							

//====================================================================
// FUNCTION DEFINITIONS
//====================================================================
void helloWorld(){
    init_LCD();
    lcd_command(CLEAR);
    lcd_putstring("Slayyy");
}

void init(void){
    // Enable the clock to GPIO Port B
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIOB clock

    GPIOA -> MODER |= GPIO_MODER_MODER3|GPIO_MODER_MODER7; // Set PA3-PA5 to analogue mode

    GPIOB -> MODER |= GPIO_MODER_MODER0_0|GPIO_MODER_MODER5_0|GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0; // Configure LED PB0, PB1, PB2, PB7 to outputs

    GPIOB -> ODR = 0x00; // Turn LEDs OFF

    // Configure PA11-PA13 as output
    GPIOA->MODER &= ~(3UL << (11 * 2)); // Clear bits for PA11
    GPIOA->MODER |= (1UL << (11 * 2));  // Set as general purpose output
    GPIOA->MODER &= ~(3UL << (12 * 2)); // Clear bits for PA12
    GPIOA->MODER |= (1UL << (12 * 2));  // Set as general purpose output
    GPIOA->MODER &= ~(3UL << (13 * 2)); // Clear bits for PA13
    GPIOA->MODER |= (1UL << (13 * 2));  // Set as general purpose output

    // Set initial output state
    GPIOA->ODR |= (1UL << 11); // Set PA11 high
    GPIOA->ODR |= (1UL << 12); // Set PA12 high
    GPIOA->ODR |= (1UL << 13); // Set PA13 high

    // Enable the clock to the ADC
    RCC -> APB2ENR |= RCC_APB2ENR_ADCEN;

    // Active ADC to be ready
    ADC1 -> CR |= ADC_CR_ADEN;
    while ( ((ADC1 -> ISR) & ADC_ISR_ADRDY) == 0 ); // wait for ADRDY flag to go high

    // Stop the ADC to configure parameters
    ADC1 -> CR &= ~ADC_CR_ADSTART;

    // ADC configuration
    ADC1 -> CFGR1 |= 0b00000000000000000000000000011000; // 12 bits
    ADC1 -> CFGR1 &= ~ADC_CFGR1_ALIGN; // right alignment
    ADC1 -> CFGR1 &= 0b00000000000000000010000000000000; // continuous mode
}

void loop(void) {
    uint16_t threshold_left_upper = threshold_voltage_left_upper / 3.3 * 4095; // Convert threshold voltage to ADC value (assuming 12-bit resolution)
    uint16_t threshold_middle_upper = threshold_voltage_middle_upper / 3.3 * 4095;
    uint16_t threshold_right_upper = threshold_voltage_right_upper / 3.3 * 4095;
    uint16_t threshold_left_lower = threshold_voltage_left_lower / 3.3 * 4095;
    uint16_t threshold_middle_lower = threshold_voltage_middle_lower / 3.3 * 4095;
    uint16_t threshold_right_lower = threshold_voltage_right_lower / 3.3 * 4095;
    
    //Select channel 3 and start ADC conversion
    ADC1->CHSELR = ADC_CHSELR_CHSEL3;
    ADC1->CR |= ADC_CR_ADSTART;
    
    // Wait for ADC conversion to complete
    while (!(ADC1->ISR & ADC_ISR_EOC));

    // Read ADC conversion result
    uint16_t adc_value_left = ADC1->DR;

    // Check if ADC value exceeds threshold for each channel
    if (adc_value_left > threshold_left_upper) {
        GPIOB->ODR |= (1UL << 7); // Turn on LED connected to PB7
        }

    else if (adc_value_left < threshold_left_lower){
        GPIOB->ODR &= ~(1UL << 7);
    }

    // Stop the ADC to configure parameters
    ADC1 -> CR &= ~ADC_CR_ADSTART;

    // Select channel 4 and start ADC conversion
    ADC1->CHSELR = ADC_CHSELR_CHSEL4;
    ADC1->CR |= ADC_CR_ADSTART;
    
    // Wait for ADC conversion to complete
    while (!(ADC1->ISR & ADC_ISR_EOC));

    // Read ADC conversion result
    uint16_t adc_value_middle = ADC1->DR;

    // Check if ADC value exceeds threshold for each channel
    if (adc_value_middle > threshold_middle_upper) {
        GPIOB->ODR |= (1UL << 6); // Turn on LED connected to PB6
        }
        
    else if (adc_value_middle < threshold_middle_lower){
        GPIOB->ODR &= ~(1UL << 6);
    }

    // Stop the ADC to configure parameters
    ADC1 -> CR &= ~ADC_CR_ADSTART;

    // Select channel 7 and start ADC conversion
    ADC1->CHSELR = ADC_CHSELR_CHSEL7;
    ADC1->CR |= ADC_CR_ADSTART;
    
    // Wait for ADC conversion to complete
    while (!(ADC1->ISR & ADC_ISR_EOC));

    // Read ADC conversion result
    uint16_t adc_value_right = ADC1->DR;

    // Check if ADC value exceeds threshold for each channel
    if (adc_value_right > threshold_right_upper) {
        GPIOB->ODR |= (1UL << 5); // Turn on LED connected to PB5
        }
        
    else if (adc_value_right < threshold_right_lower){
        GPIOB->ODR &= ~(1UL << 5);
    }

    // Stop the ADC
    ADC1 -> CR &= ~ADC_CR_ADSTART;
}

void low_power_mode_enabled(void){
    // 80% duty cycle - 20% reduction in power consumption
    GPIOB->ODR |= GPIO_ODR_0;
    GPIOA->ODR |= (1UL << 11); // Set PA11 high
    GPIOA->ODR |= (1UL << 12); // Set PA12 high
    GPIOA->ODR |= (1UL << 13); // Set PA13 high

    // adjust delay to change duty cycle
    delay(200);

    GPIOB->ODR &= ~GPIO_ODR_0;
    GPIOA->ODR &= ~(1UL << 11); // Set PA11 low
    GPIOA->ODR &= ~(1UL << 12); // Set PA12 low
    GPIOA->ODR &= ~(1UL << 13); // Set PA13 low

    delay(50); // f = 4kHz
    }

void low_power_mode_disabled(void){
    GPIOA->ODR |= (1UL << 11); // Set PA11 high
    GPIOA->ODR |= (1UL << 12); // Set PA12 high
    GPIOA->ODR |= (1UL << 13); // Set PA13 high
    }

//********************************************************************
// END OF PROGRAM
//********************************************************************
