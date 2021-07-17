#include <Arduino.h>

double raw_to_voltage(long value_raw, double maxChannelVoltage, double maxRaw)
{
  // Scale by ADC_MAX raw val, and maxChannelVoltage input.
  double valueScaled = maxChannelVoltage * (double)value_raw / (double)maxRaw;
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

double raw_to_current(long value_raw, double maxChannelVoltage, double maxRaw)
{
  return util_voltage_to_current(raw_to_voltage(value_raw,maxChannelVoltage,maxRaw),2.5);
}

double raw_to_temperature(long value_raw, double maxChannelVoltage, double maxRaw)
{
  return util_voltage_to_temperature(raw_to_voltage(value_raw,maxChannelVoltage,maxRaw));
}