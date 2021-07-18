#include <Arduino.h>
#include <Wire.h>
#include <MCP342x.h>
#include "utils.h"

class ADCHandler
{

private:
  bool initSuccess = false;
  uint8_t i2cAddress = 0x6B;

  MCP342x chip; // Object for MCP3424 interface
  // To store temporary result
  long temp;

  // PER channel configuration, so we can have different gains per-channel etc.
  MCP342x::Config configCH1 = MCP342x::Config(MCP342x::channel1, MCP342x::continous, MCP342x::resolution18, MCP342x::gain1); // SET the current resolution, channel, gain, etc.
  MCP342x::Config configCH2 = MCP342x::Config(MCP342x::channel2, MCP342x::continous, MCP342x::resolution18, MCP342x::gain1);
  MCP342x::Config configCH3 = MCP342x::Config(MCP342x::channel3, MCP342x::continous, MCP342x::resolution18, MCP342x::gain1);
  MCP342x::Config configCH4 = MCP342x::Config(MCP342x::channel4, MCP342x::continous, MCP342x::resolution18, MCP342x::gain1);

  MCP342x::Config status; // GET the current result, ready-state, etc.

  // Current channel being monitored
  // From 0 to 3. (CH1 to 4)
  uint8_t chCurrent = 0;
  // Amount of average samples to take
  uint8_t avgSampleCount = 4;

  // Array of samples - per channel
  // chIndex is an array and each element always points to next slot to place a sample array!
  uint8_t chIndex[4] = {0, 0, 0, 0};
  long chSamples[4][8]; // We store samples here!
  long chAverages[4];    // Latest average values of the sample arrays
  long chLatest[4];      // Latest values of the sample arrays

  // Update average for current channel
  // May be incorrect upon first read, until sample buffer is full
  void updateChannelAverage()
  {
    // We will be converting many things into DOUBLES for calculation purposes.
    double temp = 0;
    // Calc avg
    for (int i = 0; i < this->avgSampleCount; i++)
    {
      temp += (double)this->chSamples[this->chCurrent][i] / (double)this->avgSampleCount;
    }

    // Store result as a double
    this->chAverages[this->chCurrent] = (long)temp;
  }

  // Once data is ready - this is called to update our sample arrays
  // and re-calculate the average result.
  void processSample()
  {
    // Operates on current _channel!
    // Update latest readings
    this->chLatest[this->chCurrent] = (long)this->temp;
    // First, store latestReading into chIndex in the sample array
    this->chSamples[this->chCurrent][this->chIndex[this->chCurrent]] = (long)this->temp;

    // Increment the channel index for next sample
    if (this->chIndex[this->chCurrent] + 1 == this->avgSampleCount)
    {
      this->chIndex[this->chCurrent] = 0;
    }
    else
    {
      this->chIndex[this->chCurrent]++;
    }

    // Calculate average
    this->updateChannelAverage();
  }

  // Sets CHIP configuration determined by current _channel.
  void updateDeviceConfig()
  {
    switch (this->chCurrent)
    {
    case 0:
      this->chip.configure(this->configCH1);
      break;
    case 1:
      this->chip.configure(this->configCH2);
      break;
    case 2:
      this->chip.configure(this->configCH3);
      break;
    case 3:
      this->chip.configure(this->configCH4);
      break;
    }
  }

  long resolutionToChannelMax(MCP342x::Resolution res)
  {
    if (res == MCP342x::resolution12)
    {
      return 2047;
    }
    else if (res == MCP342x::resolution14)
    {
      return 8191;
    }
    else if (res == MCP342x::resolution16)
    {
      return 32767;
    }
    else if (res == MCP342x::resolution18)
    {
      return 131071;
    }
    return 2047;
  }

public:
  ADCHandler(uint8_t address)
  {
    this->i2cAddress = address;
  }

  void init()
  {
    // Serial.println("Setting up ADC connection.");

    this->chip = MCP342x(this->i2cAddress);
    MCP342x::generalCallReset();
    delay(1); // MC342x needs 300us to settle, wait 1ms

    // Serial.println("Checking I2C exists...");
    if (!util_check_i2c_device_exists(this->i2cAddress))
    {
      // Serial.print("Fail");
      this->initSuccess = false;
      return;
    }

    // Set config
    this->updateDeviceConfig();

    this->initSuccess = true;
  }

  void setAverageSampleCount(uint8_t averageCount)
  {
    // Check averageCount
    if (averageCount < 1 || averageCount > 8)
    {
      this->avgSampleCount = 1; // Only take 1 sample
    }
    else
    {
      this->avgSampleCount = averageCount; // Take x many samples
    }
  }

  int update()
  {
    /*
      Called as many times per second as possible in higher scope.
      This is non-blocking, and manages the reading of channels and averaging behind-the-scenes.
      */
    if (!this->initSuccess)
    {
      // Wait until ready.
      return -1;
    }

    int readyChannel = -2; // Is set to the channel number when processed

    // Get chip ready status
    this->chip.read(this->temp, this->status);

    // Check chip ready status
    if (this->status.isReady())
    {
      // Set readyChannel
      readyChannel = this->chCurrent;

      // Chip is ready! Latest reading is inside of temp
      this->processSample();

      // Wrap channel
      if (this->chCurrent + 1 == 4)
      {
        this->chCurrent = 0;
      }
      else
      {
        this->chCurrent++;
      }

      // Update device config!
      this->updateDeviceConfig();
    }

    return readyChannel + 1;
  }

  long readAverage(uint8_t channel)
  {
    // Returns the latest averaged channel reading.
    if (!this->initSuccess)
    {
      return 0;
    }

    // Change channel
    if (channel < 1 || channel > 4)
    {
      return 0;
    }

    return this->chAverages[channel - 1];
  }

  long readLatest(uint8_t channel)
  {
    // Returns the latest NON-averaged channel reading.
    if (!this->initSuccess)
    {
      return 0;
    }

    // Change channel
    if (channel < 1 || channel > 4)
    {
      return 0;
    }

    return this->chLatest[channel - 1];
  }

  long getChannelMax(uint8_t channel)
  {
    // Returns the largest "RAW" value for this channel.
    switch (channel)
    {
    case 1:
      return resolutionToChannelMax(this->configCH1.getResolution());
      break;
    case 2:
      return resolutionToChannelMax(this->configCH2.getResolution());
      break;
    case 3:
      return resolutionToChannelMax(this->configCH3.getResolution());
      break;
    case 4:
      return resolutionToChannelMax(this->configCH4.getResolution());
      break;
    }
    return 0;
  }

  void setChannelConfig(uint8_t channel, MCP342x::Resolution resolution, MCP342x::Gain gain)
  {
    switch (channel)
    {
    case 1:
      this->configCH1 = MCP342x::Config(MCP342x::channel1, MCP342x::continous, resolution, gain);
      break;
    case 2:
      this->configCH2 = MCP342x::Config(MCP342x::channel2, MCP342x::continous, resolution, gain);
      break;
    case 3:
      this->configCH3 = MCP342x::Config(MCP342x::channel3, MCP342x::continous, resolution, gain);
      break;
    case 4:
      this->configCH4 = MCP342x::Config(MCP342x::channel4, MCP342x::continous, resolution, gain);
      break;
    }
  }

  void setResolution(MCP342x::Resolution resolution)
  {
    this->configCH1 = MCP342x::Config(MCP342x::channel1, MCP342x::continous, resolution, this->configCH1.getGain());
    this->configCH2 = MCP342x::Config(MCP342x::channel2, MCP342x::continous, resolution, this->configCH2.getGain());
    this->configCH3 = MCP342x::Config(MCP342x::channel3, MCP342x::continous, resolution, this->configCH3.getGain());
    this->configCH4 = MCP342x::Config(MCP342x::channel4, MCP342x::continous, resolution, this->configCH4.getGain());
  }

};