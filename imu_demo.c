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
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h" // Required for multicore functions
#include "pico/sync.h"      // Required for spinlocks
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pt_cornell_rp2040_v1_3.h"

#define UART0_ID uart0
#define UART1_ID uart1
#define BAUD_RATE 115200
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1
#define UART1_TX_PIN 8
#define UART1_RX_PIN 9
#define POT_PIN 28

// === Fixed Point Macros ===
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

// === UART Config ===
#define UART0_ID uart0
#define UART1_ID uart1
#define BAUD_RATE 115200
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1
#define UART1_TX_PIN 8
#define UART1_RX_PIN 9

// === ADC Config ===
#define POT_PIN 28

// === Button Config ===
#define BUTTON_PIN 14

// === SPI/DAC Config ===
#define PIN_MISO 4
#define PIN_CS   13
#define PIN_SCK  10
#define PIN_MOSI 11
#define LDAC     2
#define SPI_PORT spi1
// DAC Control Bits (A-channel, 1x gain, active / B-channel, 1x gain, active)
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000
#define SPI_LOCK_NUM 0 // Spinlock number for SPI protection
spin_lock_t *spi_lock ; // Pointer to spinlock instance

// === Audio Synthesis (DDS) Config ===
#define two32 4294967296.0  // 2^32
#define Fs 50000 // ISR sample rate
#define DELAY 20 // ISR interval = 1/Fs = 20 microseconds
#define sine_table_size 256
fix15 sin_table[sine_table_size] ;

// === DDS Synthesis - Core 0 (Channel B - variable amplitude) ===
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs ; // 400 Hz beep
volatile fix15 current_amplitude_0 = 0 ; // Current amplitude envelope (modified in ISR 0)
volatile fix15 distance_amplitude_scale = int2fix15(1); // Scale factor from distance (updated by protothread)
volatile unsigned int STATE_0 = 0 ;      // 0: Beeping, 1: Silent Interval
volatile unsigned int count_0 = 0 ;      // Counter for timing within states
uint16_t DAC_data_0 ;                    // Value sent to SPI (channel B)
volatile int DAC_output_0 ;              // Intermediate DAC value (channel B)

// === DDS Synthesis - Core 1 (Channel A - fixed amplitude envelope) ===
volatile unsigned int phase_accum_main_1;
volatile unsigned int phase_incr_main_1 = (800.0*two32)/Fs ; // 800 Hz beep
volatile fix15 current_amplitude_1 = 0 ; // Current amplitude envelope (modified in ISR 1)
volatile unsigned int STATE_1 = 0 ;      // 0: Beeping, 1: Silent Interval
volatile unsigned int count_1 = 0 ;      // Counter for timing within states
uint16_t DAC_data_1 ;                    // Value sent to SPI (channel A)
volatile int DAC_output_1 ;              // Intermediate DAC value (channel A)

// === Interaural Time Difference (ITD) ===
volatile int delay_ticks_0 = 0; // Delay ticks for Core 0 / Channel B (Right Ear)
volatile int delay_ticks_1 = 0; // Delay ticks for Core 1 / Channel A (Left Ear)

// === Beep Envelope Timing === (in units of ISR ticks)
#define ATTACK_TIME             250
#define DECAY_TIME              250
#define BEEP_DURATION           10500 // Total time sound is generated
#define BEEP_REPEAT_INTERVAL    25000 // Time from beep end to next beep start (500ms)

// === Amplitude Control ===
fix15 max_amplitude = int2fix15(1) ; // Maximum possible amplitude envelope
fix15 attack_inc ;                   // Rate sound ramps up (calculated in main)
fix15 decay_inc ;                    // Rate sound ramps down (calculated in main)

// === Distance to Amplitude Mapping ===
#define MIN_DIST_VOL 20  // Distance (cm) for max volume
#define MAX_DIST_VOL 200 // Distance (cm) for min volume (zero)

// === Timer Alarms for ISRs ===
#define ALARM_NUM_0 0
#define ALARM_NUM_1 1
#define ALARM_IRQ_0 TIMER_IRQ_0
#define ALARM_IRQ_1 TIMER_IRQ_1
#define ISR_GPIO_0 15 // GPIO for Core 0 ISR timing (optional)
#define ISR_GPIO_1 16 // GPIO for Core 1 ISR timing (optional)

// === Lidar/Angle Mapping ===
#define ANGLE_BUCKETS 37 // 0 to 180 degrees in 5-degree steps (180/5 + 1)
uint16_t angle_distance_map[ANGLE_BUCKETS];
uint16_t min_distance = UINT16_MAX;
float min_distance_angle = -1.0f;
uint64_t last_min_print_time_us = 0; // For timed printing in protothread

#define head_diameter 0.144
#define v_sound 344

// === LED ===
#define LED 25 // Onboard LED

/* FSM States */
typedef enum {
    STATE_SCANNING,
    STATE_LOCKED
} system_state_t;

// === Function Prototypes (declare before use) ===
int16_t getLidarData(uart_inst_t *uart_id);
float map_value(uint16_t value, uint16_t in_min, uint16_t in_max, float out_min, float out_max);
static void alarm_irq_0(void);
static void alarm_irq_1(void);
void core1_entry();

// ==================================================
// === Lidar Data Acquisition Function ===
// ==================================================
// Returns distance in cm, or -1 if read fails
int16_t getLidarData(uart_inst_t *uart_id) {
    uint8_t temp[9];
    int bytes_read = 0;
    uint16_t distance = UINT16_MAX; 

    // Attempt to read 9 bytes with a small timeout mechanism
    absolute_time_t timeout_time = make_timeout_time_ms(10); // 10ms timeout for read
    while (bytes_read < 9 && absolute_time_diff_us(get_absolute_time(), timeout_time) > 0) {
        if (uart_is_readable(uart_id)) {
            temp[bytes_read] = uart_getc(uart_id);
            bytes_read++;
        }
    }
    
    if (bytes_read == 9 && temp[0] == 0x59 && temp[1] == 0x59) {
        distance = temp[2] + (temp[3] << 8);
        return (int16_t)distance; // Return valid distance
    } else {
        // Flush remaining buffer if header wasn't found or read wasn't complete
        while(uart_is_readable(uart_id)) {
            uart_getc(uart_id);
        }
        return -1; // Return -1 indicating error or incomplete read
    }
}

// ==================================================
// === Angle Mapping Helper Function ===
// ==================================================
float map_value(uint16_t value, uint16_t in_min, uint16_t in_max, float out_min, float out_max) {
    // Clamp input value
    if (value < in_min) value = in_min;
    if (value > in_max) value = in_max;
    // Perform linear mapping
    return (float)(value - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}


// ==================================================
// === Timer ISR Core 0 (Channel B - variable vol) ===
// ==================================================
// This is called at Fs (e.g., 50kHz)
static void alarm_irq_0(void) {
    // Assert GPIO for timing check (optional)
    gpio_put(ISR_GPIO_0, 1) ;

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM_0);
    // Rearm the alarm
    timer_hw->alarm[ALARM_NUM_0] = timer_hw->timerawl + DELAY ;

    // === Beep State Machine ===
    if (STATE_0 == 0) { // State 0: Beeping
        // DDS phase calculation
        phase_accum_main_0 += phase_incr_main_0;
        // Calculate basic sine output scaled by envelope
        int current_sample = fix2int15(multfix15(current_amplitude_0, sin_table[phase_accum_main_0 >> 24]));
        
        // Scale by distance factor
        int final_sample = fix2int15(multfix15(distance_amplitude_scale, int2fix15(current_sample)));

        // Add DC offset (DAC middle value)
        DAC_output_0 = final_sample + 2048;

        // Clamp output to 12-bit range (0-4095)
        if (DAC_output_0 > 4095) DAC_output_0 = 4095;
        else if (DAC_output_0 < 0) DAC_output_0 = 0;

        // === Amplitude Envelope Update ===
        // Ramp up amplitude during attack phase
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 += attack_inc;
            // Clamp amplitude to max_amplitude (safety)
            if (current_amplitude_0 > max_amplitude) current_amplitude_0 = max_amplitude;
        }
        // Ramp down amplitude during decay phase
        else if (count_0 >= (BEEP_DURATION - DECAY_TIME)) {
             // Ensure amplitude doesn't go negative
            if (current_amplitude_0 > decay_inc) {
                current_amplitude_0 -= decay_inc;
            } else {
                current_amplitude_0 = 0;
            }
        }
        // Sustain phase: amplitude held (implicitly, no change here)

        // Mask with DAC control bits (using channel B)
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xFFF)); // Ensure only 12 bits are used

        // === SPI Write with ITD Delay ===
        // If delay counter is active, decrement and skip SPI write
        if (delay_ticks_0 > 0) {
            delay_ticks_0--;
        } else {
            // Otherwise, write to DAC via SPI (with spinlock)
            spin_lock_unsafe_blocking(spi_lock) ;
            spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;
            spin_unlock_unsafe(spi_lock) ;
        }

        // Increment the counter for envelope timing
        count_0++;

        // Check for state transition: Beep finished?
        if (count_0 >= BEEP_DURATION) {
            STATE_0 = 1; // Transition to silent state
            count_0 = 0; // Reset counter for silent interval
            current_amplitude_0 = 0; // Ensure silence
            // Optionally clear DAC output immediately
            DAC_data_0 = (DAC_config_chan_B | 2048); // Midpoint
            spin_lock_unsafe_blocking(spi_lock) ;
            spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;
            spin_unlock_unsafe(spi_lock) ;
        }
    }
    else { // State 1: Silent Interval
        // Increment counter for silent interval timing
        count_0++;

        // Check for state transition: Silent interval finished?
        if (count_0 >= BEEP_REPEAT_INTERVAL) {
            STATE_0 = 0; // Transition back to beeping state
            count_0 = 0; // Reset counter for beep duration
            current_amplitude_0 = 0; // Start beep ramp-up from zero
        }
    }

    // De-assert GPIO for timing check
    gpio_put(ISR_GPIO_0, 0) ;
}

// ==================================================
// === Timer ISR Core 1 (Channel A - fixed vol) ===
// ==================================================
// This is called at Fs (e.g., 50kHz)
static void alarm_irq_1(void) {
    // Assert GPIO for timing check (optional)
    gpio_put(ISR_GPIO_1, 1) ;

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM_1);
    // Rearm the alarm
    timer_hw->alarm[ALARM_NUM_1] = timer_hw->timerawl + DELAY ;

    // === Beep State Machine ===
    if (STATE_1 == 0) { // State 0: Beeping
        // DDS phase calculation
        phase_accum_main_1 += phase_incr_main_1;
        // Calculate basic sine output scaled by envelope
        int current_sample = fix2int15(multfix15(current_amplitude_1, sin_table[phase_accum_main_1 >> 24]));
        
        // Scale by distance factor
        int final_sample = fix2int15(multfix15(distance_amplitude_scale, int2fix15(current_sample)));

        // Add DC offset (DAC middle value)
        DAC_output_1 = final_sample + 2048;

        // Clamp output to 12-bit range (0-4095)
        if (DAC_output_1 > 4095) DAC_output_1 = 4095;
        else if (DAC_output_1 < 0) DAC_output_1 = 0;

        // === Amplitude Envelope Update ===
        // Ramp up amplitude during attack phase
        if (count_1 < ATTACK_TIME) {
            current_amplitude_1 += attack_inc;
            // Clamp amplitude to max_amplitude (safety)
            if (current_amplitude_1 > max_amplitude) current_amplitude_1 = max_amplitude;
        }
        // Ramp down amplitude during decay phase
        else if (count_1 >= (BEEP_DURATION - DECAY_TIME)) {
             // Ensure amplitude doesn't go negative
            if (current_amplitude_1 > decay_inc) {
                current_amplitude_1 -= decay_inc;
            } else {
                current_amplitude_1 = 0;
            }
        }
        // Sustain phase: amplitude held (implicitly, no change here)

        // Mask with DAC control bits (using channel A)
        DAC_data_1 = (DAC_config_chan_A | (DAC_output_1 & 0xFFF)); // Ensure only 12 bits are used

        // === SPI Write with ITD Delay ===
        // If delay counter is active, decrement and skip SPI write
        if (delay_ticks_1 > 0) {
            delay_ticks_1--;
        } else {
             // Otherwise, write to DAC via SPI (with spinlock)
            spin_lock_unsafe_blocking(spi_lock) ;
            spi_write16_blocking(SPI_PORT, &DAC_data_1, 1) ;
            spin_unlock_unsafe(spi_lock) ;
        }

        // Increment the counter for envelope timing
        count_1++;

        // Check for state transition: Beep finished?
        if (count_1 >= BEEP_DURATION) {
            STATE_1 = 1; // Transition to silent state
            count_1 = 0; // Reset counter for silent interval
            current_amplitude_1 = 0; // Ensure silence
            // Optionally clear DAC output immediately
            DAC_data_1 = (DAC_config_chan_A | 2048); // Midpoint
            spin_lock_unsafe_blocking(spi_lock) ;
            spi_write16_blocking(SPI_PORT, &DAC_data_1, 1) ;
            spin_unlock_unsafe(spi_lock) ;
        }
    }
    else { // State 1: Silent Interval
        // Increment counter for silent interval timing
        count_1++;

        // Check for state transition: Silent interval finished?
        if (count_1 >= BEEP_REPEAT_INTERVAL) {
            STATE_1 = 0; // Transition back to beeping state
            count_1 = 0; // Reset counter for beep duration
            current_amplitude_1 = 0; // Start beep ramp-up from zero
        }
    }

    // De-assert GPIO for timing check
    gpio_put(ISR_GPIO_1, 0) ;
}

// ==================================================
// === Protothread for Lidar/Pot/FSM (Core 0) ===
// ==================================================
static PT_THREAD (protothread_lidar_pot(struct pt *pt))
{
    PT_BEGIN(pt);

    // FSM state variable
    static system_state_t current_state = STATE_SCANNING;
    // Timer variables for printing
    static uint64_t last_scan_print_time = 0; 
    static uint64_t last_lock_print_time = 0;
    // Variable to store calculated ITD for printing
    static float last_itd_calc_ms = 0.0f;

    printf("Protothread starting in SCANNING state.\n");

    while(1) {
        // === Read Button State ===
        // Simple check, assumes button pulls low when pressed
        // Add debouncing here if needed
        bool button_pressed = !gpio_get(BUTTON_PIN); 

        // === Read Potentiometer and Calculate Angle ===
        uint16_t pot_value = adc_read();
        float angle = -1.0f; 
        if (pot_value >= 600 && pot_value <= 3500) {
            angle = map_value(pot_value, 600, 3500, 0.0f, 180.0f);
        }

        // === Read Lidar ===
        int16_t current_distance = getLidarData(UART1_ID);

        // === FSM Logic ===
        if (current_state == STATE_SCANNING) {
            // --- Action: Update Map and Min Distance ---
            if (angle >= 0.0f && current_distance >= 0) { 
                int angle_index = (int)roundf(angle / 5.0f);
                if (angle_index < 0) angle_index = 0;
                if (angle_index >= ANGLE_BUCKETS) angle_index = ANGLE_BUCKETS - 1;
                angle_distance_map[angle_index] = (uint16_t)current_distance;

                if ((uint16_t)current_distance < min_distance) {
                    min_distance = (uint16_t)current_distance;
                    min_distance_angle = angle; 
                }
            }
            
            // --- Action: Update Audio Scale based on CURRENT Distance ---
            if (current_distance >= 0) {
                float dist_f = (float)current_distance;
                float scale_f = 0.0f;
                if (dist_f < MIN_DIST_VOL) dist_f = MIN_DIST_VOL;
                if (dist_f > MAX_DIST_VOL) dist_f = MAX_DIST_VOL;
                scale_f = 1.0f - ((dist_f - MIN_DIST_VOL) / (MAX_DIST_VOL - MIN_DIST_VOL));
                distance_amplitude_scale = float2fix15(scale_f);
            } else {
                distance_amplitude_scale = 0; // Mute on Lidar error
            }

            // --- Action: Timed Printing ---
            uint64_t current_time_us = PT_GET_TIME_usec();
            if (current_time_us - last_scan_print_time >= 5000000) { // Print min every 5s
                 if (min_distance != UINT16_MAX) {
                     printf("[SCAN] Min Dist: %d cm at ~%.1f deg\n", min_distance, min_distance_angle);
                 } else {
                     printf("[SCAN] Min Dist: No minimum recorded yet\n");
                 }
                 last_scan_print_time = current_time_us; 
            } else { // Print current reading otherwise
                 if (angle >= 0.0f && current_distance >= 0) {
                      printf("[SCAN] Angle:%.1f | Dist:%d | Scale:%.2f\n", angle, current_distance, fix2float15(distance_amplitude_scale));
                 } 
            }

            // --- State Transition ---
            if (button_pressed) {
                current_state = STATE_LOCKED;
                printf("Button Pressed: Transitioning to LOCKED state.\n");
                // Reset lock print timer when entering state
                last_lock_print_time = PT_GET_TIME_usec(); 
                // Optional: Set audio scale based on min_distance immediately
                if (min_distance != UINT16_MAX) {
                    float dist_f = (float)min_distance;
                    // --- Calculate and Set ITD Delay --- 
                    if (min_distance_angle >= 0.0f) { 
                        // Convert pot angle (0-180) to ITD angle (-90 to +90)
                        float theta_deg = min_distance_angle - 90.0f;
                        float theta_rad = theta_deg * M_PI / 180.0f;

                        // Calculate ITD
                        float itd_seconds = (head_diameter / v_sound) * sinf(theta_rad);
                        last_itd_calc_ms = itd_seconds * 1000.0f; // Store for printing

                        // Calculate delay ticks (absolute value)
                        int total_delay_ticks = (int)roundf(fabsf(itd_seconds) * Fs);

                        // Assign delay to the appropriate channel
                        if (itd_seconds > 0.00001f) { // Sound arrives at Left ear first, delay Right (Core 0)
                            delay_ticks_0 = total_delay_ticks;
                            delay_ticks_1 = 0;
                        } else if (itd_seconds < -0.00001f) { // Sound arrives at Right ear first, delay Left (Core 1)
                            delay_ticks_1 = total_delay_ticks;
                            delay_ticks_0 = 0;
                        } else { // Directly ahead
                            delay_ticks_0 = 0;
                            delay_ticks_1 = 0;
                        }
                        printf("[LOCK] Angle: %.1f deg => ITD: %.3f ms => Delay L:%d R:%d ticks\n", 
                               min_distance_angle, last_itd_calc_ms, delay_ticks_1, delay_ticks_0);
                    } else {
                        // No valid angle found during scan
                        delay_ticks_0 = 0;
                        delay_ticks_1 = 0;
                         last_itd_calc_ms = 0.0f;
                        printf("[LOCK] No valid angle for ITD calculation.\n");
                    }
                } else {
                    distance_amplitude_scale = 0; // Mute if no min found
                    delay_ticks_0 = 0; // No delay if muted
                    delay_ticks_1 = 0;
                    last_itd_calc_ms = 0.0f;
                    printf("[LOCK] No minimum distance found, audio muted.\n");
                }
            }

        } else if (current_state == STATE_LOCKED) {
            // --- Action: Update Audio Scale based on RECORDED Min Distance ---
            // (Recalculate each time in case min_distance was initially UINT16_MAX)
             if (min_distance != UINT16_MAX) {
                float dist_f = (float)min_distance; // Use the stored minimum
                float scale_f = 0.0f;
                if (dist_f < MIN_DIST_VOL) dist_f = MIN_DIST_VOL;
                if (dist_f > MAX_DIST_VOL) dist_f = MAX_DIST_VOL;
                scale_f = 1.0f - ((dist_f - MIN_DIST_VOL) / (MAX_DIST_VOL - MIN_DIST_VOL));
                distance_amplitude_scale = float2fix15(scale_f);
            } else {
                distance_amplitude_scale = 0; // Keep muted if no min found
            }
            // NOTE: No map updates, no min distance updates in this state.

            // --- Action: Periodic Printing of Min Distance ---
             uint64_t current_time_us = PT_GET_TIME_usec();
             if (current_time_us - last_lock_print_time >= 1000000) { // Print every 1 second (1Hz)
                 if (min_distance != UINT16_MAX) {
                     printf("[LOCKED] Closest: %d cm @ %.1f deg (ITD: %.3f ms)\n", 
                            min_distance, min_distance_angle, last_itd_calc_ms);
                 } else {
                     printf("[LOCKED] No minimum distance recorded during scan.\n");
                 }
                 last_lock_print_time = current_time_us; 
             }

            // --- State Transition --- 
            // Check if button is pressed again to return to scanning
            if (button_pressed) {
                current_state = STATE_SCANNING;
                printf("Button Pressed: Returning to SCANNING state.\n");
                printf("Resetting minimum distance and angle map.\n");

                // Reset min distance tracking
                min_distance = UINT16_MAX;
                min_distance_angle = -1.0f;

                // Reset the angle-distance map
                for (int i = 0; i < ANGLE_BUCKETS; i++) {
                    angle_distance_map[i] = UINT16_MAX; 
                }
                
                // Reset ITD delays
                delay_ticks_0 = 0;
                delay_ticks_1 = 0;
                last_itd_calc_ms = 0.0f;
                
                // Reset scan print timer when entering state
                last_scan_print_time = PT_GET_TIME_usec(); 
            }

        } // End of FSM states

        // Common yield for the protothread
        PT_YIELD_usec(20000); // Yield for approx 20 ms
    } // End while(1)
    PT_END(pt);
}

// ==================================================
// === Core 1 Entry Point ===
// ==================================================
void core1_entry() {
    // Setup Timer ISR for Core 1 (Alarm 1, handling Channel A)
    printf("Core 1: Setting up Timer ISR 1...\n");
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM_1) ;
    irq_set_exclusive_handler(ALARM_IRQ_1, alarm_irq_1) ;
    irq_set_enabled(ALARM_IRQ_1, true) ;
    timer_hw->alarm[ALARM_NUM_1] = timer_hw->timerawl + DELAY ;
    printf("Core 1: Timer ISR 1 Enabled.\n");

    // Core 1 does nothing else in this version, just runs the ISR.
    // If you wanted Core 1 threads, you'd add pt_add_thread() here
    // and pt_schedule_start;
    while(1){
        tight_loop_contents(); // Keeps Core 1 active
    }
}

// ==================================================
// === Core 0 Main Entry Point ===
// ==================================================
int main() {
    stdio_init_all();
    printf("FSM Lidar/Potentiometer Audio Feedback System Initializing...\n");

    // === Initialize Hardware ===
    gpio_init(LED) ; gpio_set_dir(LED, GPIO_OUT) ; gpio_put(LED, 0) ;
    adc_init(); adc_gpio_init(POT_PIN); adc_select_input(2); 
    uart_init(UART0_ID, BAUD_RATE); uart_init(UART1_ID, BAUD_RATE); 
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART); gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART); gpio_set_function(UART1_RX_PIN, GPIO_FUNC_UART);
    printf("Core 0: UART Initialized.\n");
    
    // --- Button ---
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN); // Enable pull-up, button press pulls low
    printf("Core 0: Button GPIO Initialized (Pin %d).\n", BUTTON_PIN);

    spi_init(SPI_PORT, 20000000); 
    spi_set_format(SPI_PORT, 16, 0, 0, 0); 
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI); gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS); gpio_set_dir(PIN_CS, GPIO_OUT); gpio_put(PIN_CS, 1); 
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI); 
    printf("Core 0: SPI Initialized.\n");
    spi_lock = spin_lock_init(SPI_LOCK_NUM) ;
    printf("Core 0: SPI Spinlock Initialized (Lock %d).\n", SPI_LOCK_NUM);
    gpio_init(LDAC) ; gpio_set_dir(LDAC, GPIO_OUT) ; gpio_put(LDAC, 0) ;
    printf("Core 0: LDAC Initialized.\n");
    gpio_init(ISR_GPIO_0) ; gpio_set_dir(ISR_GPIO_0, GPIO_OUT); gpio_put(ISR_GPIO_0, 0) ;
    gpio_init(ISR_GPIO_1) ; gpio_set_dir(ISR_GPIO_1, GPIO_OUT); gpio_put(ISR_GPIO_1, 0) ;
    
    // === Initialize Synthesis Data ===
    for (int ii = 0; ii < sine_table_size; ii++){
        sin_table[ii] = float2fix15(2047.0 * sin((float)ii * 2.0 * M_PI / (float)sine_table_size));
    }
    printf("Core 0: Sine Table Built.\n");
    attack_inc = (ATTACK_TIME > 0) ? divfix(max_amplitude, int2fix15(ATTACK_TIME)) : max_amplitude;
    decay_inc = (DECAY_TIME > 0) ? divfix(max_amplitude, int2fix15(DECAY_TIME)) : max_amplitude;
    printf("Core 0: Envelope Increments Calculated.\n");
    for (int i = 0; i < ANGLE_BUCKETS; i++) { angle_distance_map[i] = UINT16_MAX; }
    printf("Core 0: Angle-Distance Map Initialized.\n");

    // === Launch Core 1 ===
    printf("Core 0: Launching Core 1...\n");
    multicore_launch_core1(core1_entry);
    sleep_ms(10); // Small delay to allow Core 1 to start up

    // === Setup Core 0 Timer ISR ===
    printf("Core 0: Setting up Timer ISR 0...\n");
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM_0) ;
    irq_set_exclusive_handler(ALARM_IRQ_0, alarm_irq_0) ;
    irq_set_enabled(ALARM_IRQ_0, true) ;
    timer_hw->alarm[ALARM_NUM_0] = timer_hw->timerawl + DELAY ;
    printf("Core 0: Timer ISR 0 Enabled.\n");

    // === Setup Core 0 Protothreads ===
    pt_add_thread(protothread_lidar_pot);
    printf("Core 0: Protothread Added.\n");

    // === Start Core 0 Scheduler ===
    printf("Core 0: Starting Protothread Scheduler...\n");
    gpio_put(LED, 1); 
    pt_schedule_start; // This function never returns

    return 0; 
}