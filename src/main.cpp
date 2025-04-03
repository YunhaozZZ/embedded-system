//Group58: Aadi Pinglay avp5581, Yisai Wang yw7794, Pengcheng Chen pc3452, Yunhao Si ys6264
// main.cpp
// Team Members: Pengcheng Chen, Yisai Wang, Yunhao Si, Aadi Pinglay
// RT Embedded Challenge Spring 2024 - Tremor Challenge
// Objective: Detect Parkinsonian tremor using STM32F429 Discovery board

#include "mbed.h"
#include "arm_math.h"
#include "drivers/LCD_DISCO_F429ZI.h"


// Define Gyroscope's settings
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23 
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

#define CTRL_REG3 0x22 
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define OUT_X_L 0x28

#define SPI_FLAG 1
#define DATA_READY_FLAG 2

#define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)
#define FILTER_COEFFICIENT 0.2f // Adjust this value as needed

#define BUFFER_SIZE 256
#define FFT_LENGTH BUFFER_SIZE
#define SAMPLE_RATE 100
#define TREMOR_MIN_FREQ 3.0f // Parkinson's tremor minimum frequency
#define TREMOR_MAX_FREQ 6.0f // Parkinson's tremor maximum frequency
#define Pi 3.1415926

EventFlags flags;// EventFlags object declaration


// spi callback function
void spi_cb(int event) {
    flags.set(SPI_FLAG);
}

// data ready callback function
void data_cb() {
    flags.set(DATA_READY_FLAG);
}

// Initialize the LCD screen
LCD_DISCO_F429ZI lcd;

// Circular buffer to store gyro values
struct GyroData {
    float x, y, z;
};

GyroData gyroBuffer[BUFFER_SIZE];
int bufferHead = 0;
int bufferTail = 0;

void storeGyroData(float gx, float gy, float gz) {
    gyroBuffer[bufferHead].x = gx;
    gyroBuffer[bufferHead].y = gy;
    gyroBuffer[bufferHead].z = gz;
    bufferHead = (bufferHead + 1) % BUFFER_SIZE;

    // If the buffer becomes full, overwrite the oldest data
    if (bufferHead == bufferTail) {
        bufferTail = (bufferTail + 1) % BUFFER_SIZE;
    }
}

void performFFT(float* data, float* fftOutput) {
    arm_cfft_radix4_instance_f32 fft_instance;
    arm_cfft_radix4_init_f32(&fft_instance, FFT_LENGTH, 0, 1);

    // Apply a Hamming window before performing FFT
    for (int i = 0; i < FFT_LENGTH; i++) {
        float window = 0.54 - 0.46 * cos(2 * PI * i / (FFT_LENGTH - 1));
        data[i * 2] *= window;
    }

    arm_cfft_radix4_f32(&fft_instance, data);

    // Calculate the magnitude of the FFT output
    for (int i = 0; i < FFT_LENGTH; i++) {
        fftOutput[i] = sqrt(data[2 * i] * data[2 * i] + data[2 * i + 1] * data[2 * i + 1]);
    }
}

float calculateFrequency(float* fftOutput, int fftLength, float sampleRate, float &intensity) {
    float maxMagnitude = 0.0f;
    int maxFreqIndex = 0;
    for (int i = 0; i < fftLength / 2; i++) {
        if (fftOutput[i] > maxMagnitude) {
            maxMagnitude = fftOutput[i];
            maxFreqIndex = i;
        }
    }
    intensity = 20 * log10(maxMagnitude); // Convert magnitude to dB
    return (float)maxFreqIndex * sampleRate / (float)fftLength;
}

int main() {
    //spi initialization
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    uint8_t write_buf[32], read_buf[32];

    //interrupt initialization
    InterruptIn int2(PA_2, PullDown);
    int2.rise(&data_cb);
    
    //spi format and frequency
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Write to control registers
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[1] = 0xFF;

    //Polling for data ready flag
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {
        flags.set(DATA_READY_FLAG);
    }

    // LPF definitions
    float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f;

    // HPF definitions
    float high_pass_gx = 0.0f, high_pass_gy = 0.0f, high_pass_gz = 0.0f;

    // FFT input and output buffers
    float fftInput[FFT_LENGTH * 2];
    float fftOutput[FFT_LENGTH];
  
    while (1) {
        int16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;

        flags.wait_all(DATA_READY_FLAG);
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Process raw data
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        gx = ((float)raw_gx) * SCALING_FACTOR;
        gy = ((float)raw_gy) * SCALING_FACTOR;
        gz = ((float)raw_gz) * SCALING_FACTOR;

        // Apply Simple low-pass filter
        filtered_gx = FILTER_COEFFICIENT * gx + (1 - FILTER_COEFFICIENT) * filtered_gx;
        filtered_gy = FILTER_COEFFICIENT * gy + (1 - FILTER_COEFFICIENT) * filtered_gy;
        filtered_gz = FILTER_COEFFICIENT * gz + (1 - FILTER_COEFFICIENT) * filtered_gz;

        // FFT input and output buffers
        high_pass_gx = gx - filtered_gx;
        high_pass_gy = gy - filtered_gy;
        high_pass_gz = gz - filtered_gz;

        printf("filtered gx: %0.02f \n",filtered_gx);
        printf("filtered gy: %0.02f \n",filtered_gy);
        printf("filtered gz: %0.02f \n",filtered_gz);

        // Store the gyro values in the circular buffer
        storeGyroData(filtered_gx, filtered_gy, filtered_gz);
        
        // Display gyro values on the LCD
        lcd.Clear(LCD_COLOR_BLACK);
        lcd.SetTextColor(LCD_COLOR_WHITE);
        lcd.SetBackColor(LCD_COLOR_BLACK);
        lcd.SetFont(&Font16);

        char displayBuffer[50];
        sprintf(displayBuffer, "X: %.2f", gx);
        lcd.DisplayStringAt(0, LINE(0), (uint8_t *)displayBuffer, CENTER_MODE);

        sprintf(displayBuffer, "Y: %.2f", gy);
        lcd.DisplayStringAt(0, LINE(2), (uint8_t *)displayBuffer, CENTER_MODE);

        sprintf(displayBuffer, "Z: %.2f", gz);
        lcd.DisplayStringAt(0, LINE(4), (uint8_t *)displayBuffer, CENTER_MODE);

        // Perform FFT on high-pass filtered data
        for (int i = 0; i < FFT_LENGTH; i++) {
            int bufferIndex = (bufferTail + i) % BUFFER_SIZE;
            fftInput[i * 2] = gyroBuffer[bufferIndex].x; // Using the data after filter 
            fftInput[i * 2 + 1] = 0.0f;
        }

        float intensity;
        performFFT(fftInput, fftOutput);

        // Calculate the current detected frequency
        float currentFrequency = calculateFrequency(fftOutput, FFT_LENGTH, SAMPLE_RATE, intensity);

        lcd.SetFont(&Font12);
        sprintf(displayBuffer, "Current Freq: %.2f Hz", currentFrequency);
        lcd.DisplayStringAt(0, LINE(7), (uint8_t *)displayBuffer, CENTER_MODE);

        // Check if the current frequency is within the tremor range
        if (currentFrequency >= TREMOR_MIN_FREQ && currentFrequency <= TREMOR_MAX_FREQ) {
            lcd.SetTextColor(LCD_COLOR_GREEN);
            lcd.DisplayStringAt(0, LINE(11), (uint8_t *)"Parkinsonian Tremor Detected", CENTER_MODE);
            sprintf(displayBuffer, "Intensity: %.2f dB", intensity);
            lcd.DisplayStringAt(0, LINE(13), (uint8_t *)displayBuffer, CENTER_MODE);
        } else {
            lcd.SetTextColor(LCD_COLOR_RED);
            lcd.DisplayStringAt(0, LINE(12), (uint8_t *)"No Parkinsonian Tremor", CENTER_MODE);
            sprintf(displayBuffer, "Intensity: %.2f dB", intensity);
            lcd.DisplayStringAt(0, LINE(13), (uint8_t *)displayBuffer, CENTER_MODE);
        }

        thread_sleep_for(100);
    }
}