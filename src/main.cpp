#include <Arduino.h>
/* 
      This is for ESP32 ADC ONLY!!! 
      This code is from datasheets and example form espressif
      datasheet esp32 ----> https://docs.espressif.com/projects/esp-idf/en/v4.4.3/esp32/api-reference/peripherals/adc.html#adc-channels     
      tutorial        ----> https://embeddedexplorer.com/esp32-adc-esp-idf-tutorial/
      Code ADC Accuracy-Improvement from G6EJD

    +----------+-------------+-----------------+
    |          | attenuation | suggested range |
    |    SoC   |     (dB)    |      (mV)       |
    +==========+=============+=================+
    |          |       0     |    100 ~  950   |
    |          +-------------+-----------------+
    |          |       2.5   |    100 ~ 1250   |
    |   ESP32  +-------------+-----------------+
    |          |       6     |    150 ~ 1750   |
    |          +-------------+-----------------+
    |          |      11     |    150 ~ 2450   |
    +----------+-------------+-----------------+
    |          |       0     |      0 ~  750   |
    |          +-------------+-----------------+
    |          |       2.5   |      0 ~ 1050   |
    | ESP32-S2 +-------------+-----------------+
    |          |       6     |      0 ~ 1300   |
    |          +-------------+-----------------+
    |          |      11     |      0 ~ 2500   |
    +----------+-------------+-----------------+
*/

//--------------include lib from datasheet--------------------//
#include <driver/adc.h>
#include <esp_adc_cal.h>
//------------------------------------------------------------//

// define ADC pin
#define ADC_pin 35 

/* 
   find divider reduces voltage by voltage divider formula
   example in this case R1 = 29890.00 and R2 = 7485.00 
      Vo = 7485 /(29890 + 7485) * Vin
      Vo = 0.200267 * Vin
   So the divider reduces voltage to ~20.03% of input.
   and it have so many way to find voltage Offset

   normal with non - Calibration will get voltage by
   find Divider gain:
   Divider gain = (R1 + R2) / R2 ==> theoretical scaling

   calculate Vmeasure:
   Vmeasure = Vadc * Divider gain

  ? 1.Single-point Calibration  ==> good for only specific Voltage value
      this method is Works fine if you don’t need perfect accuracy across the whole range.

      find Calibration factor:
      Calibration factor = Vtrue / Vmeasure

      V_calibrated = Vmeasure * Calibration factor

  ? 2.Two-point Calibration     ==> good for specific voltage interval
      this method will remove Slope error and Offset error

      Vmeasure_1 = First measure Value 
      Vmeasure_2 = Second measure Value

      Vtrue_1 = First true Value
      Vtrue_2 = Second true Vlaue

             !slope = (Vtrue_2 - Vtrue_1) / (Vmeasure_2 - Vmeasure_1)

      it slope from 2 point 
      it can be calculate with linear formula y = ax + b
      from this equaltion we will need to find "b" coefficient and we know y and x in this equation
      y = Vtrue , x = Vmeasure , "a" = slope

      we get: 
             Vtrue = (slope * Vmeasure) + b
      and we calculate to find "b" substitute variable values from (Vmeasure_1,Vtrue_1):
             b = Vtrue - (slope * Vmeasure) 
      It will be said that:
             offset = b
      in final from we get:
             !V_calibrated = (slope * Vmeasure) + offset

  ? 3.2nd-order polynomial     ==> good for specific voltage interval
   
   Vmeasure = Voltage from Sensor
   Vtrue = Voltage from Meter



  ? all 
*/

// value Should be 2.000 
#define voltage_divider_offset 2.000 

float R1 = 29890.00;  // Resistor 1
float R2 = 7485.00;   // Resistor 2


void setup() {
  Serial.begin(9600);
  pinMode(ADC_pin, INPUT);
}
void loop() {


}

void Unadjusted_ADC_Read() {
  int rawValue = analogRead(ADC_pin);
  delay(1000);
  Serial.println("Raw ADC is " + String(rawValue));

  float VoltageADC = (rawValue * 3.3) / 4096.00;
  Serial.println("Voltage ADC is " + String(VoltageADC));

  float Voltage = VoltageADC * ((R1 + R2) / R2);
  Serial.println("Voltage is " + String(Voltage));

  Serial.println("-------------------------------");
  Serial.println(" ");
}

void adjusted_ADC_Read() {

}