#include <Arduino.h>
/*
      This is for ESP32 ADC ONLY!!! [successive-approximation-register]
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
#define ADC_pin 34

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

      V_calibrated = Vadc * Divider gain * Calibration factor
      and we can use another Calibration value to make value more accurate by
      adding Correction factor in to this calibration

      Correction factor is the internal voltage that the ADC
      (Analog-to-Digital Converter) uses as its "measurement yardstick".
      On the ESP32, it’s nominally 1100 mV (1.1 V).
      That means the ADC compares the input voltage to this reference to decide the digital code.
      Unlike some MCUs (like Arduino Uno with a very stable 5.0 V reference),
      the ESP32’s Vref is not fixed — it varies between 1000 mV and 1200 mV depending on the chip.
      so we can use { esp_adc_cal_characterize() } to read vref and we can make value more accurate

             !Correction factor =  1100 / Vref(esp_adc_cal_characterize)

      In final form is:

             !V_calibrated = Vadc * Divider gain * Calibration factor * Correction factor

  ? 2.Two-point Calibration [linearity]     ==> good for specific voltage interval
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
             b = Vtrue_1 - (slope * Vmeasure_1)
      It will be said that:
             offset = b
      in final from we get:
             !V_calibrated = (slope * Vmeasure) + offset

  ? 3.2nd-order polynomial --> quadratic calibration [non-linearity]     ==> best if you measure 3+ calibration voltages, accounts for ESP32 ADC non-linearity

   Vmeasure1,2,3 = Voltage from Sensor

   Vtrue1,2,3 = Voltage from Meter

  find V_calibrated by quadratic formula and you find coefficients "α" , "β" , "γ"
  by Plugging each point of Vmeasure and Vtrue into the quadratic formula
  so we will get system of equations:

  |     Vtrue1 = (α * Vmeasure1)^2 + (β * Vmeasure1) + γ ---1
  |     Vtrue2 = (α * Vmeasure2)^2 + (β * Vmeasure2) + γ ---2
  |     Vtrue3 = (α * Vmeasure3)^2 + (β * Vmeasure3) + γ ---3

  form this system of equations you can solve for coefficients "α" , "β" , "γ"
  and in final we will get calibration curve is:
             !V_calibrated = (α * Vmeasure)^2 + (β * Vmeasure) + γ
*/

bool DEBUG = true;

const float R1 = 29860.00000; // Resistor 1
const float R2 = 7450.00000;  // Resistor 2

esp_adc_cal_characteristics_t adc_chars;

float determinant3x3(float cof[3][3]){

  // For a matrix A = 
  // | a b c |
  // | d e f |
  // | g h i |
  // The determinant |A| = a(ei − fh) − b(di − fg) + c(dh − eg)

  return cof[0][0]*(cof[1][1]*cof[2][2]-cof[1][2]*cof[2][1])
       - cof[0][1]*(cof[1][0]*cof[2][2]-cof[1][2]*cof[2][0])
       + cof[0][2]*(cof[1][0]*cof[2][1]-cof[1][1]*cof[2][0]);
}

float Unadjusted_ADC_Read(adc1_channel_t Raw_AnalogPIN)
{  delay(100);

  const float VoltageADC = ((4096.000 - adc1_get_raw(Raw_AnalogPIN)) * 3.3) / 4096.00;
  const float Voltage = VoltageADC * ((R1 + R2) / R2);
  return Voltage;
}

float SingleP_Calibration_voltage_Read(
    adc1_channel_t Raw_AnalogPIN,
    float Vtrue,
    float Vmeter,
    float Manual_calibration)
{
  // find Average value
  long Sum = 0;
  const int Number_of_Sample = 100.000;
  for (int sample = 0; sample < Number_of_Sample; sample++)
  {
    Sum += adc1_get_raw(Raw_AnalogPIN);
  }
  const float AVG_Raw_AnalogValue = Sum / (float)Number_of_Sample;

  // Calculate voltage
  const float VoltageADC = (map(AVG_Raw_AnalogValue, 4095, 0, 0, 4095) * 3.300) / 4096.000;

  // Calculate Divider gain
  const float divider_gain = (R1 + R2) / R2;

  // Calculate Calibration factor
  const float Calibration_factor = Vtrue / Vmeter;

  // Calculate Correction_factor
  const float Correction_factor = 1100.0000 / adc_chars.vref;

  // Calculate manual Calibration
  const float Manual_calibrate = Manual_calibration; // Adjust for ultimate accuracy if reading too high then use e.g. 0.99, too low use 1.01

  // Calculate Calibration_Voltage
  const float Calibration_Voltage = VoltageADC * divider_gain * Calibration_factor * Manual_calibrate * Correction_factor;
  return Calibration_Voltage;
}
float Linear_Calibration_voltage_Read(
    adc1_channel_t Raw_AnalogPIN,
    float Vtrue_1,
    float Vtrue_2,
    float Vmeter_1,
    float Vmeter_2,
    float Manual_calibration)
{
  // find Average value
  long Sum = 0;
  int Number_of_Sample = 100.000;
  for (int sample = 0; sample < Number_of_Sample; sample++)
  {
    Sum += adc1_get_raw(Raw_AnalogPIN);
  }
  float AVG_Raw_AnalogValue = Sum / (float)Number_of_Sample;
 
  // Calculate voltage
  const float VoltageADC = (map(AVG_Raw_AnalogValue, 4095, 0, 0, 4095) * 3.300) / 4096.000;
  //uint32_t Vadc_pin_mv = esp_adc_cal_raw_to_voltage((4095.000 - AVG_Raw_AnalogValue), &adc_chars);
  //float VoltageADC = Vadc_pin_mv / 1000.0f;

  // Calculate Divider gain
  const float divider_gain = (R1 + R2) / R2;

  // Calculate slope and find offset coefficient
  float slope;
  float offset;
  if (fabs(Vmeter_2 - Vmeter_1) < 1e-6)
  {
    slope = 1.0f;
    offset = 0.0f;
  }
  else
  {
    slope = (Vtrue_2 - Vtrue_1) / (Vmeter_2 - Vmeter_1);
    offset = Vtrue_1 - (slope * Vmeter_1);
  }

  // Calculate Correction_factor
  const float Correction_factor = 1100.0000 / adc_chars.vref;

  // Calculate Calibration_Voltage
  // const float CAL_SLOPE  = 0.985139f;   // a
  // const float CAL_OFFSET = 0.054606f;   // b


  const float Calibration_Voltage = (slope * (VoltageADC * divider_gain)* Manual_calibration) + offset;
  return Calibration_Voltage;
}

float Non_Linear_Calibration_voltage_Read(
    adc1_channel_t Raw_AnalogPIN,
    float Vtrue_1,
    float Vtrue_2,
    float Vtrue_3,
    float Vmeter_1,
    float Vmeter_2,
    float Vmeter_3,
    float Manual_calibration)
{
  long Sum = 0;
  int Number_of_Sample = 100.000;
  for (int sample = 0; sample < Number_of_Sample; sample++)
  {
    Sum += adc1_get_raw(Raw_AnalogPIN);
  }
  const float AVG_Raw_AnalogValue = Sum / (float)Number_of_Sample;
 
  // Calculate voltage
  const float VoltageADC = (map(AVG_Raw_AnalogValue, 4095, 0, 0, 4095) * 3.300) / 4096.000;
  //uint32_t Vadc_pin_mv = esp_adc_cal_raw_to_voltage((4095.000 - AVG_Raw_AnalogValue), &adc_chars);
  //float VoltageADC = Vadc_pin_mv / 1000.0f;

  // Calculate Divider gain
  const float divider_gain = (R1 + R2) / R2;
  
  const float Vmeter_1_power = Vmeter_1*Vmeter_1;
  const float Vmeter_2_power = Vmeter_2*Vmeter_2;
  const float Vmeter_3_power = Vmeter_3*Vmeter_3;

  // |     Vtrue1 = (α * Vmeter_1)^2 + (β * Vmeter_1) + γ ---1
  // |     Vtrue2 = (α * Vmeter_2)^2 + (β * Vmeter_2) + γ ---2
  // |     Vtrue3 = (α * Vmeter_3)^2 + (β * Vmeter_3) + γ ---3
  //get value from system of equation

  float main_mat[3][3] = {{Vmeter_1_power, Vmeter_1, 1.000},
                          {Vmeter_2_power, Vmeter_2, 1.000},
                          {Vmeter_3_power, Vmeter_3, 1.000}};

  float MAT_A[3][3]    = {{Vtrue_1, Vmeter_1, 1.000},
                          {Vtrue_2, Vmeter_2, 1.000},
                          {Vtrue_3, Vmeter_3, 1.000}};

  float MAT_B[3][3]    = {{Vmeter_1_power, Vtrue_1, 1.000},
                          {Vmeter_2_power, Vtrue_2, 1.000},
                          {Vmeter_3_power, Vtrue_3, 1.000}};

  float MAT_C[3][3]    = {{Vmeter_1_power, Vmeter_1, Vtrue_1},
                          {Vmeter_2_power, Vmeter_2, Vtrue_2},
                          {Vmeter_3_power, Vmeter_3, Vtrue_3}};
                        
  //solve for coefficients "α" , "β" , "γ" by Cramer's rule
  
  float denominator = determinant3x3(main_mat);

  const float A = determinant3x3(MAT_A) / denominator;
  const float B = determinant3x3(MAT_B) / denominator;
  const float C = determinant3x3(MAT_C) / denominator;

  // Calculate Correction_factor
  const float Correction_factor = 1100.0000 / adc_chars.vref;

  //calculate Calibration Voltage
  const float V_Unadjust = VoltageADC * divider_gain * Correction_factor;
  const float Calibration_Voltage = ((A*V_Unadjust*V_Unadjust) + (B*V_Unadjust) + C ) * Manual_calibration;
  return Calibration_Voltage;
}
float ESP32_lib_Calibration(adc1_channel_t Raw_AnalogPIN){
  long Sum = 0;
  int Number_of_Sample = 100.000;
  for (int sample = 0; sample < Number_of_Sample; sample++)
  {
    Sum += adc1_get_raw(Raw_AnalogPIN);
  }
  const float AVG_Raw_AnalogValue = Sum / (float)Number_of_Sample;
  
  // Calculate voltage
  const uint32_t Vadc_pin_mv = esp_adc_cal_raw_to_voltage(map(AVG_Raw_AnalogValue, 4095, 0, 0, 4095), &adc_chars);
  const float Calibration_Voltage = Vadc_pin_mv / 1000.0f;
  return Calibration_Voltage;
}

float Map_value_Calibration(adc1_channel_t Raw_AnalogPIN,
                            long ADC_raw_min,
                            long ADC_raw_max)
{
  long Sum = 0;
  int Number_of_Sample = 100.000;
  for (int sample = 0; sample < Number_of_Sample; sample++)
  {
    Sum += adc1_get_raw(Raw_AnalogPIN);
  }
  const float AVG_Raw_AnalogValue = Sum / (float)Number_of_Sample;

  //map() = first linear approximation.
  //offset correction = fine-tuning to reduce error.

  const float mV = map(AVG_Raw_AnalogValue, ADC_raw_min, ADC_raw_max, 0, 16500);
  const float offset = map(mV, 16500, 0, -1000, 1000);

  const float Calibration_Voltage = (mV + offset) / 1000.0f;
  return Calibration_Voltage;
}


float data;

void setup()
{
  Serial.begin(9600);
  pinMode(ADC_pin, INPUT);
   
  // Setup analog pin
  esp_adc_cal_characterize(ADC_UNIT_1,
                           ADC_ATTEN_DB_12,
                           ADC_WIDTH_BIT_12,
                           0, &adc_chars);
}

void loop()
{ 
  //!---------------------------- ----> Normal Calculation <---- ----------------------------
  float raw = Unadjusted_ADC_Read(ADC1_CHANNEL_6);

  //!---------------------------- ----> METHOD 1 <---- ----------------------------
  // data = SingleP_Calibration_voltage_Read(ADC1_CHANNEL_6,//Pin 34
  //                                               3.3300,        //Vtrue
  //                                               2.6431,        //Vmeter
  //                                               1.0235);       //manual calibrate

  //!---------------------------- ----> METHOD 2 <---- ----------------------------
  // data = Linear_Calibration_voltage_Read(ADC1_CHANNEL_6, // Pin 34
  //                                              0.0550,  // Vtrue_1
  //                                              14.960,   // Vtrue_2
  //                                              0.004, // Vmeter_1
  //                                              15.1749, // Vmeter_2
  //                                              1.000 ); // manual calibrate

  //!---------------------------- ----> METHOD 3 <---- ----------------------------
  // data = Non_Linear_Calibration_voltage_Read(ADC1_CHANNEL_6, // Pin 34
  //                                            2,    // Vtrue_1
  //                                            9.99,      // Vtrue_2
  //                                            15.09,    // Vtrue_3
  //                                            1.3436,   // Vmeter_1
  //                                            9.28,    // Vmeter_2
  //                                            14.99, // Vmeter_3
  //                                            1.000 ); // manual calibrate

  //!---------------------------- ----> METHOD 4 <---- ----------------------------
  // data = Map_value_Calibration(ADC1_CHANNEL_6 , 142 ,3145);

  //!---------------------------- ----> METHOD 5 <---- ----------------------------
  // data = ESP32_lib_Calibration(ADC1_CHANNEL_6);

  if (DEBUG == true)
  {
    Serial.println("");
    delay(1000);
    Serial.println("Calibration_voltage = " + String(data, 4) + " | " + "Uncalibration_voltage = " + String(raw, 4));
    Serial.print("-------------------------------------------------------------");
    Serial.println("");
  }
}
