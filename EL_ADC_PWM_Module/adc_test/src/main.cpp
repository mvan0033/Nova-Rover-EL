#include <Arduino.h>
#include <Wire.h>
#include <MCP342x.h>

/* Demonstrate the use of convertAndRead().
 */

// 0x68 is the default address for all MCP342x devices
// 0x68
// 0x69
// 0x6B
// 0x6C

// 0x6B is current (shoud be 0x69)
// 0x69 is temp (should be 0x6B)

uint8_t adc_temp_addr = 0x69;
uint8_t adc_current_addr = 0x6B;

MCP342x adc_temp = MCP342x(adc_temp_addr);
MCP342x adc_current = MCP342x(adc_current_addr);

long ADC_MAX = 131071;
double ADC_MAX_V = 5;

void setup(void)
{
  Serial.begin(115200);
  Wire.begin();

  // Reset devices
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms

  // Check device present
  Wire.requestFrom(adc_current_addr, (uint8_t)1);
  if (!Wire.available())
  {
    Serial.print("No device found at address ");
    Serial.println(adc_current_addr, HEX);
    while (1)
      ;
  }

  // Check device present
  Wire.requestFrom(adc_temp_addr, (uint8_t)1);
  if (!Wire.available())
  {
    Serial.print("No device found at address ");
    Serial.println(adc_temp_addr, HEX);
    while (1)
      ;
  }
}

double util_voltage_to_temperature(double voltage)
{
  // temp divider
  double R_DIV_TOP = 1300;
  double R_0 = 10000;
  double B = 3992;
  double T_0 = 298.15; // 25degC in Kelvin

  // Calculate resistance of the NTC 
  // divTop/(x+divTop) = y
  // divTop = y*x + divtop*y
  // 
  double y = voltage/5;
  double NTC_R = (R_DIV_TOP - R_DIV_TOP*y)/y;

  // Serial.print("NTC_R = ");
  // Serial.println(NTC_R);

  // Now we can calculate temperature using the B equation.
  // T = B / ln(NTC_R / (R_0 * exp(-B/T0)))
  // Lets simplify so
  // E = exp(-B/T0)
  double E = exp(-B / T_0);
  double NTC_T = B / (log(NTC_R / (R_0 * E)));
  return NTC_T - 273.15; // Convert to C
}

double util_voltage_to_current(double voltage)
{
  // Voltage range is 2.5V +/- 0.8V.
  // so we can find a min and max value
  double V_CENTER = 2.4784; // Stable biased voltage at no current.

  double V_MAX = V_CENTER+0.8;
  double V_MIN = V_CENTER-0.8;
  // We know this to be factual, so we can clamp our result.
  double V_IN = voltage;
  
  if(V_IN > V_MAX)
  {
    V_IN = V_MAX;
  }
  if(V_IN < V_MIN)
  {
    V_IN = V_MIN;
  }

  // Theoretical sensitivity
  double G_TH = 25; //(25mV / A)
  double V_DIFF = V_IN - V_CENTER;
  double I_TH = ((V_DIFF)*1000)/G_TH;

  return I_TH;
}

void loop(void)
{
  long value_current_raw = 0;
  MCP342x::Config adc_current_status; // Return value 
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err1 = adc_current.convertAndRead(MCP342x::channel1, MCP342x::oneShot,MCP342x::resolution18, MCP342x::gain1,1000000, value_current_raw, adc_current_status);

  if (err1)
  {
    Serial.print("Convert error: ");
    Serial.println(err1);
  }
  else
  {
    Serial.print("CURRENT Raw: ");
    Serial.println(value_current_raw);
    double valueScaled = ADC_MAX_V * (double)value_current_raw / (double)ADC_MAX;
    Serial.print("CURRENT V: ");
    Serial.println(valueScaled,6);
    Serial.print("CURRENT I(A): ");
    Serial.println(util_voltage_to_current(valueScaled));
    Serial.println("---");
  }

  long value_temp_raw = 0;
  MCP342x::Config adc_temp_status; // Return value 
  
  uint8_t err2 = adc_temp.convertAndRead(MCP342x::channel1, MCP342x::oneShot,MCP342x::resolution18, MCP342x::gain1,1000000, value_temp_raw, adc_temp_status);

  if (err2)
  {
    Serial.print("Convert error: ");
    Serial.println(err2);
  }
  else
  {
    Serial.print("TEMP Raw: ");
    Serial.println(value_temp_raw);
    double valueScaled = ADC_MAX_V * (double)value_temp_raw / (double)ADC_MAX;
    Serial.print("TEMP V: ");
    Serial.println(valueScaled);

    Serial.print("TEMP Cdeg: ");
    Serial.println(util_voltage_to_temperature(valueScaled));
    Serial.println("---");
  }


  delay(1000);
}