# Ultimate Analog Guide for ESP32

This project provides a comprehensive guide and implementation for improving the accuracy of the ESP32's internal Analog-to-Digital Converter (ADC). It covers various calibration techniques to overcome the inherent non-linearity and variability of the ESP32 ADC.

## Features

- **Multiple Calibration Methods:**
  - **Single-Point Calibration:** Quick correction for a specific voltage value.
  - **Two-Point (Linear) Calibration:** Removes slope and offset errors across a specific range.
  - **Quadratic (Non-Linear) Calibration:** Uses 2nd-order polynomial regression to account for ADC non-linearity (best for 3+ measurement points).
  - **ESP-IDF Native Calibration:** Utilizes the built-in `esp_adc_cal` characterization.
  - **Map-based Calibration:** Simple linear approximation using `map()`.
- **Voltage & Current Measurement:**
  - Includes logic for voltage dividers (voltage measurement).
  - Includes logic for Hall-effect current sensors (e.g., ACS712).
- **Cramer's Rule Implementation:** Custom 3x3 determinant calculation for solving quadratic systems of equations.

## Supported Hardware

- **Microcontroller:** ESP32 (Successive Approximation Register ADC).
- **Voltage Divider:** Configurable R1 and R2 values.
- **Current Sensor:** Logic for 5A version (185mV/A) current sensors.

## Calibration Methods Explained

### 1. Single-Point Calibration
Ideal for simple applications where you only need accuracy around a specific target voltage. It calculates a `Calibration Factor` based on a known "True" voltage vs. the measured "Meter" voltage.

### 2. Two-Point (Linear) Calibration
Calculates a slope ($a$) and offset ($b$) using the formula $y = ax + b$. This is highly effective for removing systematic gain and offset errors.

### 3. Quadratic (Non-Linear) Calibration
The most accurate method for the ESP32. It solves a system of equations to find coefficients $\alpha, \beta, \gamma$ for the curve:
$$V_{calibrated} = \alpha V_{measure}^2 + \beta V_{measure} + \gamma$$
This accounts for the "S-curve" non-linearity typical of the ESP32 ADC.

## Getting Started

### Prerequisites
- [PlatformIO](https://platformio.org/) installed in VS Code.
- An ESP32 development board (e.g., DOIT DevKit V1).

### Configuration
1. Open `src/main.cpp`.
2. Update the resistor values if using a voltage divider:
   ```cpp
   const float R1 = 29860.00; 
   const float R2 = 7450.00;
   ```
3. Update the ADC pins:
   ```cpp
   #define ADC_pin 34
   #define ADC2_pin 35
   ```

### Calibration Process
1. Use the `Unadjusted_ADC_Read_V` function to get raw readings.
2. Measure the actual voltage with a reliable Multimeter ($V_{true}$).
3. Plug these values into the desired calibration method in `loop()` to find your coefficients.

## Project Structure

- `src/main.cpp`: Main application logic containing calibration functions and measurement loops.
- `platformio.ini`: PlatformIO configuration for the ESP32 environment.
- `include/`: Folder for header files (if any).
- `lib/`: Folder for private libraries.

## Credits
- Code ADC Accuracy-Improvement techniques inspired by **G6EJD**.
- Documentation and reference: [Espressif ADC API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html).
