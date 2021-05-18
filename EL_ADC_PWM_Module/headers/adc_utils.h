#include <Arduino.h>
#include <MCP342x.h>

long ADC_MAX_RAW = 131071;
double ADC_MAX_V = 5;

/* 
  Function to get a voltage reading from an ADC object, on a specified channel.
  Will return 0 on error.
*/
long adc_read_raw(MCP342x *adcObject,MCP342x::Channel channel)
{
  long value_raw = 0;

  // Return value 
  MCP342x::Config adc_status; 

  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t error = adcObject->convertAndRead(channel, MCP342x::oneShot,MCP342x::resolution18, MCP342x::gain1,1000000, value_raw, adc_status);

  if (error)
  {
    // Conversion error.
    Serial.print("ADC Read Error: ");
    Serial.println(error);
    return 0;
  }
  else
  {
    return value_raw;
  }
}

double adc_read_voltage(MCP342x *adcObject,MCP342x::Channel channel,double maxVoltage)
{
  // Get raw value
  long value_raw = adc_read_raw(adcObject,channel);

  // Scale by ADC_MAX raw val, and maxVoltage input.
  double valueScaled = maxVoltage * (double)value_raw / (double)ADC_MAX_RAW;
  return valueScaled;
}

double util_voltage_to_temperature(double voltage)
{
    /*
        Converts voltage reading from a thermistor into a temperature reading in Celsius.
    */

    // temp divider
    double R_DIV_TOP = 1300;
    
    double R_0 = 10000;
    double B = 3992;
    double T_0 = 298.15; // 25degC in Kelvin

    // Calculate resistance of the NTC
    // divTop/(x+divTop) = y
    // divTop = y*x + divtop*y
    //
    double y = voltage / 5;
    double NTC_R = (R_DIV_TOP - R_DIV_TOP * y) / y;

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

// void util_calibrate_zero_voltage()

double util_voltage_to_current(double voltage,double voltageRef)
{
    /* 
        Converts a voltage reading from a HLSR-32P current sensor, into a current estimate in Amps.
    */

    // Voltage range is 2.5V +/- 0.8V.
    // so we can find a min and max value
    double V_CENTER = voltageRef; // Stable biased voltage at no current.

    double V_MAX = V_CENTER + 0.8;
    double V_MIN = V_CENTER - 0.8;
    // We know this to be factual, so we can clamp our result.
    double V_IN = voltage;

    if (V_IN > V_MAX)
    {
        V_IN = V_MAX;
    }
    if (V_IN < V_MIN)
    {
        V_IN = V_MIN;
    }

    // Theoretical sensitivity
    double G_TH = 25; //(25mV / A)
    double V_DIFF = V_IN - V_CENTER;
    double I_TH = ((V_DIFF)*1000) / G_TH;

    return I_TH;
}

double adc_read_temperature(MCP342x *adcObject,MCP342x::Channel channel)
{
  /* Convenience function to read a temperature directly */
  double volt_result = adc_read_voltage(adcObject,channel,ADC_MAX_V);
  return util_voltage_to_temperature(volt_result);
}

double adc_read_current(MCP342x *adcObject,MCP342x::Channel channel)
{
  /* Convenience function to read a temperature directly */
  double volt_result = adc_read_voltage(adcObject,channel,ADC_MAX_V);
  return util_voltage_to_current(volt_result,2.5);
}