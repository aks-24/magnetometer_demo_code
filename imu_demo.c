/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include <math.h> // Include math library for roundf

#define UART0_ID uart0
#define UART1_ID uart1
#define BAUD_RATE 115200
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1
#define UART1_TX_PIN 8
#define UART1_RX_PIN 9
#define POT_PIN 28

// Returns distance in cm, or -1 if read fails
int16_t getLidarData(uart_inst_t *UART1, uart_inst_t *UART0) {
    uint8_t temp[9];
    int bytes_read = 0;
    uint16_t distance = UINT16_MAX; // Use UINT16_MAX to indicate invalid/no reading initially

    // Attempt to read 9 bytes with a small timeout mechanism
    absolute_time_t timeout_time = make_timeout_time_ms(10); // 10ms timeout
    while (bytes_read < 9 && absolute_time_diff_us(get_absolute_time(), timeout_time) > 0) {
        if (uart_is_readable(UART1)) {
            temp[bytes_read] = uart_getc(UART1);
            bytes_read++;
        }
    }
    
    if (bytes_read == 9 && temp[0] == 0x59 && temp[1] == 0x59) {
        distance = temp[2] + (temp[3] << 8);
        // uint16_t strength = temp[4] + (temp[5] << 8);
        // int16_t temperature = ((temp[6] + (temp[7] << 8)) / 8) - 256;
        
        // Optionally forward raw data or processed data
        // uart_write_blocking(UART0, temp, 9); 
        
        // printf("distance = %5dcm, strength = %5d, temperature = %5d°C\n", 
        //        distance, strength, temperature);
        return (int16_t)distance; // Return valid distance
    } else {
        // Flush remaining buffer if header wasn't found or read wasn't complete
        while(uart_is_readable(UART1)) {
            uart_getc(UART1);
        }
        return -1; // Return -1 indicating error or incomplete read
    }
}

// Helper function for linear mapping
float map_value(uint16_t value, uint16_t in_min, uint16_t in_max, float out_min, float out_max) {
    // Perform linear mapping
    // Formula: y = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return (float)(value - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

int main() {
    stdio_init_all();

    // --- ADC Init ---
    adc_init();
    adc_gpio_init(POT_PIN);
    adc_select_input(2);

    // --- UART Init ---
    uart_init(UART0_ID, BAUD_RATE);
    uart_init(UART1_ID, BAUD_RATE);

    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART1_RX_PIN, GPIO_FUNC_UART);

    // --- Angle-Distance Map & Min Distance Tracking ---
    #define ANGLE_BUCKETS 37 // 0 to 180 degrees in 5-degree steps (180/5 + 1)
    uint16_t angle_distance_map[ANGLE_BUCKETS];
    // Initialize map with a value indicating no data (e.g., UINT16_MAX)
    for (int i = 0; i < ANGLE_BUCKETS; i++) {
        angle_distance_map[i] = UINT16_MAX; 
    }

    uint16_t min_distance = UINT16_MAX;
    float min_distance_angle = -1.0f; // Angle corresponding to min_distance

    absolute_time_t last_min_print_time = get_absolute_time();

    sleep_ms(2000); // Wait for sensors to stabilize
    printf("Starting main loop...\n");
    
    while (true) {
        absolute_time_t current_time = get_absolute_time();

        // --- Read Potentiometer and Calculate Angle ---
        uint16_t pot_value = adc_read();
        float angle = -1.0f; // Invalid angle initially

        if (pot_value < 600) {
            // printf("Potentiometer Angle: Too low\n"); 
            // Keep angle = -1.0f
        } else if (pot_value > 3500) {
            // printf("Potentiometer Angle: Too high\n");
            // Keep angle = -1.0f
        } else {
            // Map the value within the range 600-3500 to 0-180 degrees
            angle = map_value(pot_value, 600, 3500, 0.0f, 180.0f);
            // printf("Potentiometer Angle: %.2f degrees\n", angle); 
        }

        // --- Read Lidar ---
        int16_t current_distance = getLidarData(UART1_ID, UART0_ID);

        // --- Update Map and Min Distance ---
        if (angle >= 0.0f && current_distance >= 0) { // Check for valid angle and distance
            // Calculate the index for the angle map (0-36)
            int angle_index = (int)roundf(angle / 5.0f);
            // Clamp index to be safe
            if (angle_index < 0) angle_index = 0;
            if (angle_index >= ANGLE_BUCKETS) angle_index = ANGLE_BUCKETS - 1; 

            // Store the distance in the map
            angle_distance_map[angle_index] = (uint16_t)current_distance;
            // printf("Stored distance %d cm at angle bucket %d (%.1f deg)\n", current_distance, angle_index, angle_index * 5.0f);


            // Check for new minimum distance
            if ((uint16_t)current_distance < min_distance) {
                min_distance = (uint16_t)current_distance;
                min_distance_angle = angle; // Store the actual calculated angle
                // No longer print only on new minimum, handled by timed print below
            }
        } else {
             if (current_distance < 0) {
                 // Optional: print if lidar read failed
                 // printf("Lidar read failed.\n");
             }
        }

        // --- Timed Printing ---
        // Check if 5 seconds (5,000,000 us) have passed
        if (absolute_time_diff_us(last_min_print_time, current_time) >= 5000000) {
            if (min_distance != UINT16_MAX) {
                printf("*** Min Distance Update (0.2 Hz): %d cm at ~%.1f degrees ***\n", min_distance, min_distance_angle);
            } else {
                printf("*** Min Distance Update (0.2 Hz): No minimum recorded yet ***\n");
            }
            last_min_print_time = current_time; // Reset the timer
        } else {
            // Otherwise, print current reading if valid
            if (angle >= 0.0f && current_distance >= 0) {
                 printf("Current: Angle=%.1f deg, Dist=%d cm\n", angle, current_distance);
            }
        }

        sleep_ms(20); // Slightly longer delay might be good
    }

    return 0; // Should never reach here
}