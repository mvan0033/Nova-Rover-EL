// Tasty control loops happen here!
// Handles SETUP of ADC/PWM
// Handles STATE of the controller feedback loop.
// Handles ERRORS in over-current or over-temperature etc.

// NO_HARDWARE_MODE is used to develop control loop dependant software,
// WITHOUT any of the PWM/ADC hardware modules.
// Set to 1 for no hardware
// Set to 0 for normal operation (expects I2C, SPI devices to be attached)
#define NO_HARDWARE_MODE 1

#include <Wire.h>
#include <SPI.h>

#include <MCP342x.h>
#include "Adafruit_TLC59711.h"

#include "utils.h"
#include "pwm_utils.h"
#include "adc_utils.h"

#define NUM_TLC59711 1
#define data 6
#define clock 5
#define PWM_MAX 65535

#if NO_HARDWARE_MODE == 0
// PWM MODULE OBJECT.
// Please only define this once.
Adafruit_TLC59711 pwm_module = Adafruit_TLC59711(NUM_TLC59711, clock, data);

// ADCs (0x68,0x69,0x6B,0x6C)
// uint8_t adc_addrs[4] = {0x68, 0x69, 0x6B, 0x6C};

// Define ADC function by addrress
uint8_t adc_temperature_addr = 0x6B; // MARKED AS 0X69 ON THE PCB
uint8_t adc_current_addr = 0x68; // MARKED AS 0X68 ON THE PCB
uint8_t adc_voltage_addr = 0x6C; // MARKED AS 0X6A ON THE PCB

// Define the ADC objects 
MCP342x adc_temperature = MCP342x(adc_temperature_addr);
MCP342x adc_current = MCP342x(adc_current_addr);
MCP342x adc_voltage = MCP342x(adc_voltage_addr);
#endif

int8_t load_voltage_channel = 3; // What channel of the adc_voltage chip, do we find out 0 to 75V LOAD voltage.

class ControlLoop
{
    public:

    void init()
    {
        #if NO_HARDWARE_MODE == 1
            Serial.println("NO HARDWARE MODE ENABLED.");
            Serial.println("(See control_loop.h to change this)");
        #else
            // Setup PWM module
            Serial.print("Setting up PWM module...");
            pwm_module.begin();
            pwm_module.write();
            pwm_module.simpleSetBrightness(127);
            pwm_module.write();
            pwm_set_duty_all(&pwm_module, 0);
            Serial.println("Done.");

            // Reset devices
            Serial.print("Calling reset of all MCP3424 chips...");
            MCP342x::generalCallReset();
            delay(1); // MC342x needs 300us to settle, wait 1ms
            Serial.println("Done.");

            Serial.println("Checking for ADC chips...");

            // Check for ADC modules SPECIFICALLY
            if(!util_check_i2c_device_exists(adc_temperature_addr))
            {
                Serial.print("Temperature Reading ADC, ");
                Serial.print(adc_temperature_addr,HEX);
                Serial.println(", NOT FOUND.");
            }

            if(!util_check_i2c_device_exists(adc_voltage_addr))
            {
                Serial.print("Voltage Reading ADC, ");
                Serial.print(adc_voltage_addr,HEX);
                Serial.println(", NOT FOUND.");
            }

            if(!util_check_i2c_device_exists(adc_current_addr))
            {
                Serial.print("Current Reading ADC, ");
                Serial.print(adc_current_addr,HEX);
                Serial.println(", NOT FOUND.");
            }
        #endif
    }

    void update()
    {
        /* Performs actions based on the current global state */
        /* Must be run as fast as possible */
        if(controlLoopError)
        {
            // SET PWM TO ZERO
            Serial.println("Control loop error. Setting PWMs to 0.");

            // Reset outputs
            reset_pwm_arrays_to_zero();

            #if NO_HARDWARE_MODE == 0
                pwm_set_duty_all(&pwm_module,0);
            #endif

            // RETURN
            return;
        }

        if(controlLoopRunning)
        {
            // Read currents
            read_currents();
            // Check current limits, if non-zero return and signal error state.
            int8_t result1 = check_current_limits();
            if(result1 != -1)
            {
                controlLoopError = true;
                errorType = 0; 
                errorMOSFET = result1;
                explain_limit_failure(result1,0);
                return;
            }

            // Read voltages
            read_voltages();
            // Check power limits, if non-zero return and signal error state.
            int8_t result2 = check_power_limits();
            if(result2 != -1)
            {
                controlLoopError = true;
                errorType = 1; 
                errorMOSFET = result2;
                explain_limit_failure(result2,1);
                return;
            }

            // Read temperatures
            read_temperatures();
            // Check temperature limits, if non-zero return and signal error state.
            int8_t result3 = check_temperature_limits();
            if(result3 != -1)
            {
                controlLoopError = true;
                errorType = 2; 
                errorMOSFET = result3;
                explain_limit_failure(result3,2);
                return;
            }

            // Print the readings
            Serial.println(" ");
            print_all_readings();

            // Update the target error
            update_target_error();

            // Update the PWM output per-channelm, based on this error.
            update_pwm_output_values();
            apply_pwm_output_values();

        }else{
            // Reset output PWMs
            reset_pwm_arrays_to_zero();

            // SET PWM TO ZERO
            #if NO_HARDWARE_MODE == 0
                // Sends zero to the PWM module
                pwm_set_duty_all(pwm_module,0);
            #else
                Serial.println("Setting PWM to 0.");
            #endif
        }
    }

    void set_enable(bool state)
    {
        /* Enable / Disable the control loop */
        /* Unless we are in an error state, then always be false. */
        if (controlLoopError)
        {
            controlLoopRunning = false;
        }
        else
        {
            controlLoopRunning = state;
        }
    }

    void reset_errors()
    {
        /* Resets an error state */
        controlLoopError = false;
    }

    void set_target_mode(int mode)
    {
        /* 
            Set the target mode: 
                0 = Current Target
                1 = Power Target 
            */
        targetMode = mode;
    }

    void set_target_value(int value)
    {
        /*
            Updates targetValue global variable.
            Respects Amp or Power limits based on targetMode.
            This is the value we are attempting to maintain with our control loop.
            */
        int tempValue = value;

        switch (targetMode)
        {
        case 0:
            if (tempValue > currentLimit)
            {
                Serial.println("Target value above current limit! Clamping...");
                tempValue = currentLimit;
            }
            if (tempValue < 0)
            {
                tempValue = 0;
            }
            // Apply
            targetValue = tempValue;
            break;

        case 1:
            if (tempValue > powerLimit)
            {
                Serial.println("Target value above power limit! Clamping...");
                tempValue = powerLimit;
            }
            if (tempValue < 0)
            {
                tempValue = 0;
            }
            // Apply
            targetValue = tempValue;
            break;
        }
    }

    void print_reading_array(double arr[])
    {
        for(int i =0; i<3; i++)
        {
            Serial.print(arr[i]);
            Serial.print(",");
        }
        Serial.println(arr[3]);
    }

    void print_output_array(uint16_t arr[])
    {
        for(int i =0; i<3; i++)
        {
            Serial.print(arr[i]);
            Serial.print(",");
        }
        Serial.println(arr[3]);
    }

    void print_all_readings()
    {
        /* Prints the Temperature, Current, and Voltage readings */
        Serial.println("CURRENTS");
        print_reading_array(readings_current);
        Serial.println("VOLTAGES");
        print_reading_array(readings_voltage);
        Serial.println("TEMPERATURES");
        print_reading_array(readings_temperature);
    }

    bool get_error_state()
    {
        return controlLoopError;
    }

    int8_t get_error_module()
    {
        return errorMOSFET;
    }

    int8_t get_error_type()
    {
        return errorType;
    }

    double get_total_current()
    {
        // Return combined current throughput
        double latestReadings[4] = {0,0,0,0};
        for(int i = 0; i<4; i++)
        {
            latestReadings[i] = readings_current[i];
        }
        return latestReadings[0] + latestReadings[1] + latestReadings[2] + latestReadings[3];
    }

    double get_total_power()
    {
        // Return combined power throughput
        double latestReadings[4] = {0,0,0,0};
        for(int i = 0; i<4; i++)
        {
            latestReadings[i] = readings_current[i] * readings_voltage[load_voltage_channel];
        }

        return latestReadings[0] + latestReadings[1] + latestReadings[2] + latestReadings[3];
    }

    double get_total_voltage()
    {
        // Return system load voltage
        return readings_voltage[load_voltage_channel];
    }

    double get_highest_temperature()
    {
        // Returns highest of the temperature readings
        double temp = readings_temperature[0];
        for(int i = 0; i<4; i++)
        {
            if(readings_temperature[i] > temp)
            {
                temp = readings_temperature[i];
            }
        }

        return temp;
    }

    bool get_pwm_active_state()
    {
        // Returns a boolean that signifies if PWM outputs are enabled or not.
        // This is another indication that the system is running.
        // Here, we just check if all outputs are zero.
        if(outputs_pwm[0] == 0 && outputs_pwm[1] == 0 && outputs_pwm[2] == 0 && outputs_pwm[3] == 0)
        {
            return true;
        }

        return false;
    }

private:
    /* HOW MANY MOSFET BOARDS DO WE HAVE */
    int8_t mosfetModuleCount = 3;

    /* Control loop targets */
    bool controlLoopRunning = false; // If false, set all outputs to zero. If true we enable control loop.
    bool controlLoopError = false;   // If true, we need to stop all activity until RESET is called.
    int8_t errorType = 0; // What kind of error? 0 for current, 1 for power, 2 for temperature
    int8_t errorMOSFET = 0; // What mosfet module experienced this error.

    int8_t targetMode = 0;              // 0 for current target. 1 for power target.
    int8_t targetValue = 0;             // Target value (could be Amps or Watts)
    
    /* Feedback variable */
    double targetErrors[4] = {0,0,0,0}; // Calculated error from our targetValue, identified PER-CHANNEL.

    /* GLOBAL SAFETY LIMITS */
    int8_t currentLimit = 100; // AMPS
    int8_t powerLimit = 1000;  // WATTS
    int8_t temperatureLimit = 600;  // DEGREES C

    /* Latest ADC readings per channel */
    double readings_temperature[4] = {0,0,0,0};
    double readings_current[4] = {0,0,0,0};
    double readings_voltage[4] = {0,0,0,0};

    /* PMW Output variables */
    int16_t outputs_pwm[4] = {0,0,0,0};
    double pwm_proportional_coeff = 10; // Multiply error from targetValue to get this.
    int16_t pwm_rate_limit = 100; // 1000 per update rate.

    /* Apply per-channel PWM */
    void apply_pwm_output_values()
    {
        #if NO_HARDWARE_MODE == 0
            pwm_set_duty(&pwm_module,0,outputs_pwm[0]);
            pwm_set_duty(&pwm_module,1,outputs_pwm[1]);
            pwm_set_duty(&pwm_module,2,outputs_pwm[2]);
            pwm_set_duty(&pwm_module,3,outputs_pwm[3]);
        #else
            Serial.println("APPLYING PWM OUTPUTS");
            Serial.print("CH0: ");
            Serial.println(outputs_pwm[0]);
            Serial.print("CH1: ");
            Serial.println(outputs_pwm[1]);
            Serial.print("CH2: ");
            Serial.println(outputs_pwm[2]);
            Serial.print("CH3: ");
            Serial.println(outputs_pwm[3]);
            
        #endif
    }

    /* From per-channel errors, update the PWM outputs */
    void update_pwm_output_values()
    {
        for(int i = 0; i<4; i++)
        {
            double pwmError = targetErrors[i] * pwm_proportional_coeff;

            // Clamp to pwm_proportional coeff
            if(pwmError > pwm_rate_limit)
            {
                pwmError = pwm_rate_limit;
            }
            if(pwmError < -pwm_rate_limit)
            {
                pwmError = -pwm_rate_limit;
            }

            // Apply this pwmError to the outputs_pwm array
            outputs_pwm[i] += pwmError;
            // Clamp outputs_PWM to 0 and PWM_MAX
            if(outputs_pwm[i] < 0)
            {
                outputs_pwm[i] = 0;
            }
            if(outputs_pwm[i] > PWM_MAX)
            {
                outputs_pwm[i] = PWM_MAX;
            }
        }

        // Print the output efforts
        Serial.print("PWM Outputs: ");
        print_output_array(outputs_pwm);
    }

    /* Update the targetErrors per channel */
    void update_target_error()
    {
        // Depending on mode we calculate target error.
        double latestReadings[4] = {0,0,0,0};
        double perChannelTarget = (double)targetValue / mosfetModuleCount;

        if(targetMode == 0)
        {
            // Reading is just the currents per channel
            for(int i = 0; i<4; i++)
            {
                latestReadings[i] = readings_current[i];
            }
        }

        if(targetMode == 1)
        {
            // Reading is the current per channel * system voltage
            for(int i = 0; i<4; i++)
            {
                latestReadings[i] = readings_current[i] * readings_voltage[load_voltage_channel];
            }
        }

        // Calcualte error value per channel
        for(int i = 0; i<4; i++)
        {
            double error = perChannelTarget - latestReadings[i];
            targetErrors[i] = error;
        }
        
        Serial.print("Target Errors: ");
        print_reading_array(targetErrors);
    }

    /* Print messages as a result of limit exceed */
    void explain_limit_failure(int8_t result_code,int8_t failureMode)
    {
        // Result code determines which MOSFET failed
        // Failure mode 0 means current, 1 means power, 2 means temperature
        switch(failureMode)
        {
            case 0:
                Serial.print("CURRENT LIMIT ON MOSFET #");
                Serial.println(result_code);
                break;
            case 1:
                Serial.print("POWER LIMIT ON MOSFET #");
                Serial.println(result_code);
                break;
            case 2:
                Serial.print("TEMPERATURE LIMIT ON MOSFET #");
                Serial.println(result_code);
                break;
        }
    }

    /* Check current, power, temperature limits */
    int8_t check_current_limits()
    {
        double perChannelLimit = currentLimit / mosfetModuleCount;
        for(int i = 0; i<4; i++)
        {
            if(readings_current[i] > perChannelLimit)
            {
                // Return index of the failing mosfet.
                return i;
            }            
        }

        // No error
        return -1;
    }

    int8_t check_power_limits()
    {
        double perChannelLimit = currentLimit / mosfetModuleCount;
        perChannelLimit = perChannelLimit * readings_voltage[load_voltage_channel]; // TODO: CHANGE TO PROPER VOLTAGE READING CHANNEL

        for(int i = 0; i<4; i++)
        {
            if(readings_current[i]* readings_voltage[load_voltage_channel] > perChannelLimit)
            {
                // Return index of the failing mosfet.
                return i;
            }            
        }

        // No error
        return -1;
    }

    int8_t check_temperature_limits()
    {
        double perChannelLimit = temperatureLimit;

        for(int i = 0; i<4; i++)
        {
            if(readings_temperature[i] > perChannelLimit)
            {
                // Return index of the failing mosfet.
                return i;
            }            
        }

        // No error
        return -1;
    }    


    /* Functions for reading ADCs */
    void read_temperatures() 
    {
        /* Read all the temperature ADC channels and store into 
        readings_temperature array. (Degrees Celcius)
        */
        #if NO_HARDWARE_MODE == 0
            readings_temperature[0] = adc_read_temperature(&adc_temperature,MCP342x::channel1);
            readings_temperature[1] = adc_read_temperature(&adc_temperature,MCP342x::channel2);
            readings_temperature[2] = adc_read_temperature(&adc_temperature,MCP342x::channel3);
            readings_temperature[3] = adc_read_temperature(&adc_temperature,MCP342x::channel4);
        #else
            // In no hardware mode, we set all temperatures to 30 deg, plus some random noise.
            // We also simulate the time it takes to perform a reading by delaying here.
            readings_temperature[0] = 30 + (float)random(-20,20)/10;
            readings_temperature[1] = 30 + (float)random(-20,20)/10;
            readings_temperature[2] = 30 + (float)random(-20,20)/10;
            readings_temperature[3] = 30 + (float)random(-20,20)/10;
            
            // At 14-bit precision, we can expect ~66 ms delay for a 4-IC reading.
            // See MCP3424 for samples-per-second figures, to determine this.
            delay(66);
        #endif

        // Override if nan
        for(int i = 0; i<4; i++){
            if(isnan(readings_temperature[i]))
            {
                readings_temperature[i] = 0;
            }
        }
    }
    
    double fake_mosfet_model(double vgs)
    {
        /* Returns a fake current throughput, assuming V_DS == 10V,
            given a base voltage.

            This was done from our real-world data, and equations derived from trendlines.
        */
        // The equation is piece-wise, with two different exponential regions.
        // They intersection about the ~4V mark.
        // Equation 1 (<4V GS, very low current)
        double eq1_current = (double)(2.5923060142355E-15) * exp(8.1407449923113 * vgs);
        double eq2_current = 734.04065811719 * pow(vgs,2) - 5824.45578150388 * vgs + 11554.22728209;

        // Intersection calculated with wolfram alpha.
        if(vgs < 3.96739)
        {
            return eq1_current;
        }else{
            return eq2_current;
        }
    }

    void read_currents()
    {
        /* Read all the Current ADC channels and store into 
        readings_current array. (AMPS)
        */
        #if NO_HARDWARE_MODE == 0
            readings_current[0] = adc_read_current(&adc_current,MCP342x::channel1);
            readings_current[1] = adc_read_current(&adc_current,MCP342x::channel2);
            readings_current[2] = adc_read_current(&adc_current,MCP342x::channel3);
            readings_current[3] = adc_read_current(&adc_current,MCP342x::channel4);
        #else
            // In no hardware mode, we use the PWM settings 
            // and 'pretend' it's letting more current through.
            // This gives us a 'fake' current response :)
            
            // We will also make this non-linear, according to our tests at 10V_DS.
            // (see DATA/MOSFET_MODULE_10VDS_Variable_VGS)
            // I plotted some points of V_GS vs I_DS, and fit a curve to that.
            // Then used the curve equation here to make it 'semi-realistic'
            // Also added noise! (+/- 0.2A)    
        
            readings_current[0] = fake_mosfet_model(5*(outputs_pwm[0]/65535)) + (float)random(-20,20)/10;
            readings_current[1] = fake_mosfet_model(5*(outputs_pwm[1]/65535)) + (float)random(-20,20)/10;
            readings_current[2] = fake_mosfet_model(5*(outputs_pwm[2]/65535)) + (float)random(-20,20)/10;
            readings_current[3] = fake_mosfet_model(5*(outputs_pwm[3]/65535)) + (float)random(-20,20)/10;
            
            // At 14-bit precision, we can expect ~66 ms delay for a 4-IC reading.
            // See MCP3424 for samples-per-second figures, to determine this.
            delay(66);
        #endif

        // Override if nan, or <0
        for(int i = 0; i<4; i++){
            if(isnan(readings_current[i]) || readings_current[i] < 0)
            {
                readings_current[i] = 0;
            }
        }
    }

    void read_voltages() 
    {
        /* Read all the Voltage ADC channels and store into 
        readings_voltages array. (Volts)
        */
        #if NO_HARDWARE_MODE == 0
            readings_voltage[0] = adc_read_voltage(&adc_voltage,MCP342x::channel1,5);
            readings_voltage[1] = adc_read_voltage(&adc_voltage,MCP342x::channel2,5);
            readings_voltage[2] = adc_read_voltage(&adc_voltage,MCP342x::channel3,75);
            readings_voltage[3] = adc_read_voltage(&adc_voltage,MCP342x::channel4,5);
        #else
            // Non-connected noise
            readings_voltage[0] = (float)random(-200,200)/10;
            readings_voltage[1] = (float)random(-200,200)/10;
            readings_voltage[2] = (float)random(-200,200)/10;
            readings_voltage[3] = (float)random(-200,200)/10;

            // System load. 10th volt noise
            readings_voltage[load_voltage_channel] = 10 + (float)random(-10,10)/10;
        #endif

        // Override if nan, or <0
        for(int i = 0; i<4; i++){
            if(isnan(readings_voltage[i]) || readings_voltage[i] < 0)
            {
                readings_voltage[i] = 0;
            }
        }
    }

    void reset_pwm_arrays_to_zero()
    {
        // Resets outputs_pwm arrays to zero
        outputs_pwm[0] = 0;
        outputs_pwm[1] = 0;
        outputs_pwm[2] = 0;
        outputs_pwm[3] = 0;
    }
};