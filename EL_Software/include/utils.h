int util_check_i2c_device_exists(uint8_t address)
{
  /*
  Returns 1 if I2C address is responding to our request.
  else returns 0, usually indicating the device does not exist on the bus.
  */
  
  Wire.requestFrom(address, (uint8_t)1);
  if (!Wire.available())
  {
    return 0;
  }

  return 1;
}

double util_voltage_divider_find_resistance(double inputVoltage,double outputVoltage,double topResistorValue)
{
  // Maths
  // output = input * (x/(top+x))
  // output/input = x / (x+top)
  // let output/input = y
  // y*x + y*top = x
  // y*top = x - y*x
  // y*top = x*(1-y)
  // y*top/(1-y) = x

  double y = outputVoltage/inputVoltage;

  return (y*topResistorValue)/(1-y);
}