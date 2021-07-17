#include <Arduino.h>
#include <Wire.h>
#include <MCP342x.h>
#include <utils.h>

// TODO: Add ability to change Config per-channel

class ADCHandler {
  private:
    bool initSuccess = false;
    MCP342x chip; // Object for MCP3424 interface
    // To store temporary result
    long temp;

    // PER channel configuration, so we can have different gains per-channel etc.
    MCP342x::Config configCH1 = MCP342x::Config(MCP342x::channel1,MCP342x::continous,MCP342x::resolution18,MCP342x::gain1); // SET the current resolution, channel, gain, etc.
    MCP342x::Config configCH2 = MCP342x::Config(MCP342x::channel2,MCP342x::continous,MCP342x::resolution18,MCP342x::gain1);
    MCP342x::Config configCH3 = MCP342x::Config(MCP342x::channel3,MCP342x::continous,MCP342x::resolution18,MCP342x::gain1);
    MCP342x::Config configCH4 = MCP342x::Config(MCP342x::channel4,MCP342x::continous,MCP342x::resolution18,MCP342x::gain1);

    MCP342x::Config status; // GET the current result, ready-state, etc.

    // Current channel being monitored
    // From 0 to 3. (CH1 to 4)
    uint8_t chCurrent = 0;
    // Amount of average samples to take
    uint8_t avgSampleCount = 1;
    
    // Array of samples - per channel
    // chIndex is an array and each element always points to next slot to place a sample array!
    uint8_t chIndex[4] = {0,0,0,0};
    long chSamples[4][64]; // We store samples here!
    long chAverages[4]; // Latest average values of the sample arrays
    long chLatest[4]; // Latest values of the sample arrays

    // Update average for current channel
    // May be incorrect upon first read, until sample buffer is full
    void updateChannelAverage()
    {
      // We will be converting many things into DOUBLES for calculation purposes.
      double temp = 0;
      // Calc avg
      for(int i = 0; i<this->avgSampleCount; i++)
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
      if(this->chIndex[this->chCurrent] + 1 == this->avgSampleCount)
      {
        this->chIndex[this->chCurrent]=0;
      }else{
        this->chIndex[this->chCurrent]++;
      }

      // Calculate average
      this->updateChannelAverage();
    }

    // Sets CHIP configuration determined by current _channel.
    void updateDeviceConfig()
    {
        switch(this->chCurrent)
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

  public:
    void init(uint8_t i2cAddress, uint8_t averageCount)
    {
      // Serial.println("Setting up ADC connection.");
      
      this->chip = MCP342x(i2cAddress);

      // Check averageCount
      if(averageCount < 1 || averageCount > 64)
      {
        this->avgSampleCount = 1; // Only take 1 sample
      }else{
        this->avgSampleCount = averageCount; // Take x many samples
      }
      
      MCP342x::generalCallReset();
      delay(1); // MC342x needs 300us to settle, wait 1ms

      // Serial.println("Checking I2C exists...");
      if(!util_check_i2c_device_exists(i2cAddress))
      {
        // Serial.print("Fail");
        this->initSuccess = false;
        return;
      }

      // Set config
      this->updateDeviceConfig();

      this->initSuccess = true;
    }

    int update()
    {
      /*
      Called as many times per second as possible in higher scope.
      This is non-blocking, and manages the reading of channels and averaging behind-the-scenes.
      */
      if(!this->initSuccess)
      {
        // Wait until ready.
        return -1;
      }

      int readyChannel = -2; // Is set to the channel number when processed

      // Get chip ready status
      this->chip.read(this->temp,this->status);

      // Check chip ready status
      if(this->status.isReady())
      {
        // Set readyChannel 
        readyChannel = this->chCurrent;
        
        // Chip is ready! Latest reading is inside of temp
        this->processSample();

        // Wrap channel
        if(this->chCurrent + 1 == 4)
        {
          this->chCurrent = 0;
        }else{
          this->chCurrent ++;
        }

        // Update device config!
        this->updateDeviceConfig();
      }

      return readyChannel+1;
    }

    long readAverage(uint8_t channel)
    {
      // Returns the latest averaged channel reading.
      if(!this->initSuccess)
      {
        return 0;
      }

      // Change channel
      if(channel < 1 || channel > 4)
      {
        return 0;
      }

      return this->chAverages[channel-1];
    }

    long readLatest(uint8_t channel)
    {
      // Returns the latest NON-averaged channel reading.
      if(!this->initSuccess)
      {
        return 0;
      }

      // Change channel
      if(channel < 1 || channel > 4)
      {
        return 0;
      }

      return this->chLatest[channel-1];
    }
};