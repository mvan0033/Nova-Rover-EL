// Tasty control loops happen here!
// Handles SETUP of ADC/PWM
// Handles STATE of the controller feedback loop.
// Handles ERRORS in over-current or over-temperature etc.

// NO_HARDWARE_MODE is used to develop control loop dependant software,
// WITHOUT any of the PWM/ADC hardware modules.
// Set to 1 for no hardware
// Set to 0 for normal operation (expects I2C, SPI devices to be attached)
#define NO_HARDWARE_MODE 1

#include <Adafruit_TLC59711.h>
#include <adc_nonblocking.h>
#include "pwm_utils.h"
#include "adc_utils.h"
#include <PID_v1.h>

class ControlLoop
{
public:
    Adafruit_TLC59711 *pwm_module;
    ADCHandler *adc_temperature;
    ADCHandler *adc_current;
    ADCHandler *adc_voltage;

    /* HOW MANY MOSFET BOARDS DO WE HAVE */
    // We will split the current across these modules,
    // and according to the target values.
    // Note that the system hardware will always assume a 4-module installation,
    // so the unused 'modules' will have their PWM set to zero.

    /*
    Modules Diagram
        M4  |   M3
        ----------
        M1  |   M2
    */

    int8_t mosfetModuleCount = 4;
    

    /* Control loop targets */
    bool controlLoopRunning = false; // If false, set all outputs to zero. If true we enable control loop.
    bool controlLoopError = false;   // If true, we need to stop all activity until RESET is called.
    int8_t errorType = 0;            // What kind of error? 0 for current, 1 for power, 2 for temperature
    int8_t errorMOSFET = 0;          // What mosfet module experienced this error.

    uint16_t targetMode = 0;  // 0 for current target. 1 for power target.
    uint16_t targetValue = 0; // Target value (could be Amps or Watts)

    /* GLOBAL SAFETY LIMITS */
    uint16_t currentLimit = 100;     // AMPS
    uint16_t powerLimit = 1000;      // WATTS
    uint16_t temperatureLimit = 600; // DEGREES C

    /* Latest ADC readings per channel */
    // These are updated only when the MCP3424 chip has a fresh set of readings for us.
    // This is so our update() loop can execute as fast as possible!
    double readings_temperature[4] = {0, 0, 0, 0};
    double readings_current[4] = {0, 0, 0, 0};
    double readings_current_avg[4] = {0, 0, 0, 0};
    double readings_voltage[4] = {0, 0, 0, 0};
    uint8_t load_voltage_channel = 3; // Which channel of this->readings_voltage is the primary load voltage.

    /* Current per-mosftet target (could be watts or amps!). Reflects this->targetValue.*/
    double target_values[4] = {0,0,0,0};

    /* The current per-module measurement value (this could be Amps or Watts!). Reflects this->targetValue. */
    double current_values[4] = {0,0,0,0};
    int current_sample_status = 0;

    /* PMW Output variables */
    double outputs_pid[4] = {0, 0, 0, 0};
    uint16_t outputs_pwm[4] = {0, 0, 0, 0};

    /* PIDs, attached to relevant data points */
    double fastKp=100, fastKi=0.0, fastKd=0.0;
    double Kp=1, Ki=0.0, Kd=0.0;
    PID module1PID = PID(&current_values[0], &outputs_pid[0], &target_values[0], fastKp, fastKi, fastKd, DIRECT);
    PID module2PID = PID(&current_values[1], &outputs_pid[1], &target_values[1], fastKp, fastKi, fastKd, DIRECT);
    PID module3PID = PID(&current_values[2], &outputs_pid[2], &target_values[2], fastKp, fastKi, fastKd, DIRECT);
    PID module4PID = PID(&current_values[3], &outputs_pid[3], &target_values[3], fastKp, fastKi, fastKd, DIRECT);

    // Debug output info
    unsigned long debugTime = millis();


    ControlLoop(Adafruit_TLC59711 *pwm_module, ADCHandler *adc_temperature, ADCHandler *adc_current, ADCHandler *adc_voltage)
    {
        // Attach modules
        this->pwm_module = pwm_module;

        this->adc_temperature = adc_temperature;
        this->adc_current = adc_current;
        this->adc_voltage = adc_voltage;
    }

    void init()
    {
        // Attach things
        this->module1PID.SetMode(MANUAL);
        this->module2PID.SetMode(MANUAL);
        this->module3PID.SetMode(MANUAL);
        this->module4PID.SetMode(MANUAL);

        this->module1PID.SetOutputLimits(-32768,32767);
        this->module2PID.SetOutputLimits(-32768,32767);
        this->module3PID.SetOutputLimits(-32768,32767);
        this->module4PID.SetOutputLimits(-32768,32767);

#if NO_HARDWARE_MODE == 1
        Serial.println("NO HARDWARE MODE ENABLED.");
        Serial.println("(See control_loop.h to change this)");
#else
        Serial.println("HARDWARE MODE ENABLED.");
        Serial.println("(See control_loop.h to change this)");

        // Setup PWM module
        Serial.print("Setting up PWM module...");
        this->pwm_module->begin();
        this->pwm_module->write();
        this->pwm_module->simpleSetBrightness(0); // Maximises voltage range upward

        this->pwm_module->write();
        pwm_set_duty_all(this->pwm_module, 0);
        Serial.println("Done.");

        // Reset devices
        this->adc_temperature->setResolution(MCP342x::resolution18);
        this->adc_current->setResolution(MCP342x::resolution16);
        this->adc_voltage->setResolution(MCP342x::resolution16);

        // Start ADcs
        this->adc_temperature->init();
        this->adc_current->init();
        this->adc_voltage->init();
#endif
    }

    void update()
    {
/* Performs actions based on the current global state */
/* Must be run as fast as possible */

/* Regardless of control loop state, always update the ADC chips! */
#if NO_HARDWARE_MODE == 0
        this->current_sample_status = this->adc_current->update();
        this->adc_temperature->update();
        this->adc_voltage->update();
#else
        this->current_sample_status++;
        if(this->current_sample_status == 5)
        {
            this->current_sample_status = 1;
        }
        delay(500);
#endif

        if (this->controlLoopError)
        {
            // SET PWM TO ZERO
            Serial.println("Control loop error. Setting PWMs to 0.");

            // Reset outputs
            this->reset_pwm_arrays_to_zero();

#if NO_HARDWARE_MODE == 0
            pwm_set_duty_all(pwm_module, 0);
#endif

            // RETURN
            return;
        }

        if (this->controlLoopRunning)
        {
            this->module1PID.SetMode(AUTOMATIC);
            this->module2PID.SetMode(AUTOMATIC);
            this->module3PID.SetMode(AUTOMATIC);
            this->module4PID.SetMode(AUTOMATIC);
    

            // Read currents
            this->read_currents();
            // Check current limits, if non-zero return and signal error state.
            int8_t result1 = this->check_current_limits();
            if (result1 != -1)
            {
                this->controlLoopError = true;
                this->errorType = 0;
                this->errorMOSFET = result1;
                this->explain_limit_failure(result1, 0);
                return;
            }

            // Read voltages
            this->read_voltages();
            // Check power limits, if non-zero return and signal error state.
            int8_t result2 = this->check_power_limits();
            if (result2 != -1)
            {
                this->controlLoopError = true;
                this->errorType = 1;
                this->errorMOSFET = result2;
                this->explain_limit_failure(result2, 1);
                return;
            }

            // Read temperatures
            this->read_temperatures();
            // Check temperature limits, if non-zero return and signal error state.
            int8_t result3 = this->check_temperature_limits();
            if (result3 != -1)
            {
                this->controlLoopError = true;
                this->errorType = 2;
                this->errorMOSFET = result3;
                this->explain_limit_failure(result3, 2);
                return;
            }

            // Print the readings
            Serial.println(" ");
            print_all_readings();

            // Update the PWM output per-channel, based on this error.
            this->compute_pids();
            this->apply_pwm_output_values();
        }
        else
        {
            // Stop PIDs
            this->module1PID.SetMode(MANUAL);
            this->module2PID.SetMode(MANUAL);
            this->module3PID.SetMode(MANUAL);
            this->module4PID.SetMode(MANUAL);
            
            // Reset output PWMs
            this->reset_pwm_arrays_to_zero();

// SET PWM TO ZERO
#if NO_HARDWARE_MODE == 0
            // Sends zero to the PWM module
            pwm_set_duty_all(pwm_module, 0);
#else
            Serial.println("Setting PWM to 0.");
#endif
        }
    }

    void reset_errors()
    {
        this->reset_pwm_arrays_to_zero();
        /* Resets an error state */
        this->controlLoopError = false;
    }

    void set_enable(bool state)
    {
        /* Enable / Disable the control loop */
        /* Unless we are in an error state, then always be false. */
        if (this->controlLoopError)
        {
            this->controlLoopRunning = false;
        }
        else
        {
            this->controlLoopRunning = state;
        }
    }

    void set_target_mode(int mode)
    {
        /* 
            Set the target mode: 
                0 = Current Target
                1 = Power Target 
            */
        if (mode == 0 || mode == 1)
        {
            this->targetMode = mode;
        }
        else
        {
            Serial.println("Invalid this->targetMode!");
        }
    }

    void set_target_value(int value)
    {
        /*
            Updates this->targetValue global variable.
            Respects Amp or Power limits based on this->targetMode.
            This is the value we are attempting to maintain with our control loop.
            */
        uint16_t tempValue = abs(value);

        switch (this->targetMode)
        {
        case 0:
            if (tempValue > this->currentLimit)
            {
                Serial.println("Target value above current limit! Clamping...");
                tempValue = this->currentLimit;
            }
            if (tempValue < 0)
            {
                tempValue = 0;
            }
            // Apply
            this->targetValue = tempValue;
            break;

        case 1:
            if (tempValue > this->powerLimit)
            {
                Serial.println("Target value above power limit! Clamping...");
                tempValue = this->powerLimit;
            }
            if (tempValue < 0)
            {
                tempValue = 0;
            }
            // Apply
            this->targetValue = tempValue;
            break;
        }
    }

    void print_double_array(double arr[])
    {
        for (int i = 0; i < 3; i++)
        {
            Serial.print(arr[i]);
            Serial.print(",");
        }
        Serial.println(arr[3]);
    }

    void print_uint16t_array(uint16_t arr[])
    {
        for (int i = 0; i < 3; i++)
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
        this->print_double_array(this->readings_current);
        Serial.println("CURRENTS AVG");
        this->print_double_array(this->readings_current_avg);
        Serial.println("VOLTAGES");
        this->print_double_array(this->readings_voltage);
        Serial.println("TEMPERATURES");
        this->print_double_array(this->readings_temperature);
        Serial.println("PWM OUTPUTS");
        this->print_uint16t_array(this->outputs_pwm);
    }

    bool get_error_state()
    {
        return this->controlLoopError;
    }

    int8_t get_error_module()
    {
        return this->errorMOSFET;
    }

    int8_t get_error_type()
    {
        return this->errorType;
    }

    double get_total_current()
    {
        // Return combined current throughput
        double _temp[4] = {0, 0, 0, 0};
        for (int i = 0; i < 4; i++)
        {
            _temp[i] = this->readings_current[i];
        }
        return _temp[0] + _temp[1] + _temp[2] + _temp[3];
    }

    double get_total_power()
    {
        // Return combined power throughput
        double _temp[4] = {0, 0, 0, 0};
        for (int i = 0; i < 4; i++)
        {
            _temp[i] = this->readings_current[i] * this->readings_voltage[this->load_voltage_channel-1];
        }

        return _temp[0] + _temp[1] + _temp[2] + _temp[3];
    }

    double get_total_voltage()
    {
        // Return system load voltage
        return this->readings_voltage[this->load_voltage_channel-1];
    }

    double get_highest_temperature()
    {
        // Returns highest of the temperature readings
        double temp = this->readings_temperature[0];
        for (int i = 0; i < 4; i++)
        {
            if (this->readings_temperature[i] > temp)
            {
                temp = this->readings_temperature[i];
            }
        }

        return temp;
    }

    bool get_pwm_active_state()
    {
        // Returns a boolean that signifies if PWM outputs are enabled or not.
        // This is another indication that the system is running.
        // Here, we just check if all outputs are zero.
        if (this->outputs_pwm[0] == 0 && this->outputs_pwm[1] == 0 && this->outputs_pwm[2] == 0 && this->outputs_pwm[3] == 0)
        {
            return false;
        }

        return true;
    }

    uint16_t get_global_limit_current()
    {
        return this->currentLimit;
    }

    uint16_t get_global_limit_power()
    {
        return this->powerLimit;
    }

    uint16_t get_global_limit_temperature()
    {
        return this->temperatureLimit;
    }

    void set_global_limit_current(uint16_t val)
    {
        this->currentLimit = val;
    }

    void set_global_limit_power(uint16_t val)
    {
        this->powerLimit = val;
    }

    void set_global_limit_temperature(uint16_t val)
    {
        this->temperatureLimit = val;
    }

private:
    /* Apply per-channel PWM */
    void apply_pwm_output_values()
    {
#if NO_HARDWARE_MODE == 0
        pwm_set_duty(this->pwm_module, 0, this->outputs_pwm[0]);
        pwm_set_duty(this->pwm_module, 1, this->outputs_pwm[1]);
        pwm_set_duty(this->pwm_module, 2, this->outputs_pwm[2]);
        pwm_set_duty(this->pwm_module, 3, this->outputs_pwm[3]);
#endif
    }

    uint16_t nonoverflowAddition(uint16_t a, int16_t b)
    {
        uint16_t result = 0;

        // If a + b both positive result in < a, we overflowed.
        if(b >= 0)
        {
            result = a + b;

            if(result < a)
            {
                // Overflow occured.
                result = 65535;
            }
        }else{
            result = a + b;
            if(result > a)
            {
                // Overflow occured
                result = 0;
            }
        }

        return result;
    } 

    /* From per-channel errors, update the PWM outputs */
    void compute_pids()
    {
        // 1. Update target per-mosfet values
        double perModuleTarget = this->targetValue / this->mosfetModuleCount;
        this->target_values[0] = perModuleTarget;
        this->target_values[1] = perModuleTarget;
        this->target_values[2] = perModuleTarget;
        this->target_values[3] = perModuleTarget;

        // 2. Update current_mosfet_values
        if(targetMode == 0)
        {
            this->current_values[0] = this->readings_current[0];
            this->current_values[1] = this->readings_current[1];
            this->current_values[2] = this->readings_current[2];
            this->current_values[3] = this->readings_current[3];
        }else{
            this->current_values[0] = this->readings_current[0] * this->readings_voltage[this->load_voltage_channel-1];
            this->current_values[1] = this->readings_current[1] * this->readings_voltage[this->load_voltage_channel-1];
            this->current_values[2] = this->readings_current[2] * this->readings_voltage[this->load_voltage_channel-1];
            this->current_values[3] = this->readings_current[3] * this->readings_voltage[this->load_voltage_channel-1];
        }

        // Depending on latest sample state
        // 3. Re-compute PIDs
        // 4. Apply efforts
        switch(this->current_sample_status)
        {
            case 1:
                // Check close-ness to target
                if(this->target_values[0] - this->current_values[0] < 2.5)
                {
                    this->module1PID.SetTunings(fastKp,Ki,Kd);
                }else{
                    this->module1PID.SetTunings(fastKp,fastKi,fastKd);
                }
                this->module1PID.Compute();
                this->outputs_pwm[0] = nonoverflowAddition(this->outputs_pwm[0],(int16_t)this->outputs_pid[0]);
                break;
            case 2:
                if(this->target_values[1] - this->current_values[1] < 2.5)
                {
                    this->module2PID.SetTunings(fastKp,Ki,Kd);
                }else{
                    this->module2PID.SetTunings(fastKp,fastKi,fastKd);
                }
                this->module2PID.Compute();
                this->outputs_pwm[1] = nonoverflowAddition(this->outputs_pwm[1],(int16_t)this->outputs_pid[1]);
                break;
            case 3:
                if(this->target_values[2] - this->current_values[2] < 2.5)
                {
                    this->module3PID.SetTunings(fastKp,Ki,Kd);
                }else{
                    this->module3PID.SetTunings(fastKp,fastKi,fastKd);
                }
                this->module3PID.Compute();
                this->outputs_pwm[2] = nonoverflowAddition(this->outputs_pwm[2],(int16_t)this->outputs_pid[2]);
                break;
            case 4:
                if(this->target_values[3] - this->current_values[3] < 2.5)
                {
                    this->module4PID.SetTunings(fastKp,Ki,Kd);
                }else{
                    this->module4PID.SetTunings(fastKp,fastKi,fastKd);
                }
                this->module4PID.Compute();
                this->outputs_pwm[3] = nonoverflowAddition(this->outputs_pwm[3],(int16_t)this->outputs_pid[3]);
                break;
            case -1:
                break;
        }
    }

    /* Print messages as a result of limit exceed */
    void explain_limit_failure(int8_t result_code, int8_t failureMode)
    {
        // Result code determines which MOSFET failed
        // Failure mode 0 means current, 1 means power, 2 means temperature
        switch (failureMode)
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
        double perChannelLimit = (double)this->currentLimit / (double)this->mosfetModuleCount;
        for (int i = 0; i < 4; i++)
        {
            if (this->readings_current[i] > perChannelLimit)
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
        double perChannelLimit = (double)this->currentLimit / (double)this->mosfetModuleCount;
        perChannelLimit = perChannelLimit * this->readings_voltage[this->load_voltage_channel-1]; // TODO: CHANGE TO PROPER VOLTAGE READING CHANNEL

        for (int i = 0; i < 4; i++)
        {
            if (this->readings_current[i] * this->readings_voltage[this->load_voltage_channel-1] > perChannelLimit)
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
        double perChannelLimit = this->temperatureLimit;

        for (int i = 0; i < 4; i++)
        {
            if (this->readings_temperature[i] > perChannelLimit)
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
        this->readings_temperature array. (Degrees Celcius)
        */
#if NO_HARDWARE_MODE == 0
        this->readings_temperature[0] = raw_to_temperature(this->adc_temperature->readLatest(1), 5.0, this->adc_temperature->getChannelMax(1));
        this->readings_temperature[1] = raw_to_temperature(this->adc_temperature->readLatest(2), 5.0, this->adc_temperature->getChannelMax(2));
        this->readings_temperature[2] = raw_to_temperature(this->adc_temperature->readLatest(3), 5.0, this->adc_temperature->getChannelMax(3));
        this->readings_temperature[3] = raw_to_temperature(this->adc_temperature->readLatest(4), 5.0, this->adc_temperature->getChannelMax(4));
#else
        // In no hardware mode, we set all temperatures to 30 deg, plus some random noise.
        // We also simulate the time it takes to perform a reading by delaying here.
        this->readings_temperature[0] = 30 + (float)(random(0, 40) - 20) / 100;
        this->readings_temperature[1] = 30 + (float)(random(0, 40) - 20) / 100;
        this->readings_temperature[2] = 30 + (float)(random(0, 40) - 20) / 100;
        this->readings_temperature[3] = 30 + (float)(random(0, 40) - 20) / 100;

        // At 14-bit precision, we can expect ~66 ms delay for a 4-IC reading.
        // See MCP3424 for samples-per-second figures, to determine this.
        delay(66);
#endif

        // Override if nan
        for (int i = 0; i < 4; i++)
        {
            if (isnan(this->readings_temperature[i]))
            {
                this->readings_temperature[i] = 0;
            }
        }
    }

    double fake_mosfet_model(double vgs)
    {
        /* Returns a fake current throughput, assuming V_DS == 10V,
            given a base voltage.
            This was done from our real-world data, and equations derived from trendlines.
        */
        // Trend 1, 3.4>3.95
        double trend1 = 0.4629629629629629 * vgs - 1.574074074074074;
        // Trend 2, 3.95>4.0
        double trend2 = 12.49999999999999 * vgs - 48.99999999999996;
        // Trend 3, 4.0 > 5.0
        double trend3 = 749.0 * vgs - 2995.0;

        if (trend1 < 0)
        {
            trend1 = 0;
        }
        if (trend2 < 0)
        {
            trend2 = 0;
        }
        if (trend3 < 0)
        {
            trend3 = 0;
        }

        if (vgs <= 3.4)
        {
            return 0;
        }
        else if (vgs <= 3.94)
        {
            return trend1;
        }
        else if (vgs <= 4.0)
        {
            return trend2;
        }
        else
        {
            return trend3;
        }
    }

    void read_currents()
    {
/* Read all the Current ADC channels and store into 
        this->readings_current array. (AMPS)
        */
#if NO_HARDWARE_MODE == 0
        this->readings_current[0] = raw_to_current(this->adc_current->readLatest(1), 5.0, this->adc_current->getChannelMax(1));
        this->readings_current[1] = raw_to_current(this->adc_current->readLatest(2), 5.0, this->adc_current->getChannelMax(2));
        this->readings_current[2] = raw_to_current(this->adc_current->readLatest(3), 5.0, this->adc_current->getChannelMax(3));
        this->readings_current[3] = raw_to_current(this->adc_current->readLatest(4), 5.0, this->adc_current->getChannelMax(4));

        this->readings_current_avg[0] = raw_to_current(this->adc_current->readAverage(1), 5.0, this->adc_current->getChannelMax(1));
        this->readings_current_avg[1] = raw_to_current(this->adc_current->readAverage(2), 5.0, this->adc_current->getChannelMax(2));
        this->readings_current_avg[2] = raw_to_current(this->adc_current->readAverage(3), 5.0, this->adc_current->getChannelMax(3));
        this->readings_current_avg[3] = raw_to_current(this->adc_current->readAverage(4), 5.0, this->adc_current->getChannelMax(4));
#else
        // In no hardware mode, we use the PWM settings
        // and 'pretend' it's letting more current through.
        // This gives us a 'fake' current response :)

        // We will also make this non-linear, according to our tests at 10V_DS.
        // (see DATA/MOSFET_MODULE_10VDS_Variable_VGS)
        // I plotted some points of V_GS vs I_DS, and fit a curve to that.
        // Then used the curve equation here to make it 'semi-realistic'
        // Also added noise! (+/- 2mV on the output_pwm line)

        double baseClean1 = (double)5 * (double)this->outputs_pwm[0] / (double)65535;
        double baseClean2 = (double)5 * (double)this->outputs_pwm[1] / (double)65535;
        double baseClean3 = (double)5 * (double)this->outputs_pwm[2] / (double)65535;
        double baseClean4 = (double)5 * (double)this->outputs_pwm[3] / (double)65535;

        double baseVoltage1 = baseClean1;
        double baseVoltage2 = baseClean2;
        double baseVoltage3 = baseClean3;
        double baseVoltage4 = baseClean4;

        Serial.println("BASE VOLTAGES");
        Serial.print(baseVoltage1, 6);
        Serial.print(",");
        Serial.print(baseVoltage2, 6);
        Serial.print(",");
        Serial.print(baseVoltage3, 6);
        Serial.print(",");
        Serial.println(baseVoltage4, 6);

        this->readings_current[0] = this->fake_mosfet_model(baseVoltage1);
        this->readings_current[1] = this->fake_mosfet_model(baseVoltage2);
        this->readings_current[2] = this->fake_mosfet_model(baseVoltage3);
        this->readings_current[3] = this->fake_mosfet_model(baseVoltage4);

        // At 14-bit precision, we can expect ~66 ms delay for a 4-IC reading.
        // See MCP3424 for samples-per-second figures, to determine this.
        delay(66);
#endif

        // Override if nan, or <0
        for (int i = 0; i < 4; i++)
        {
            if (isnan(this->readings_current[i]) || this->readings_current[i] < 0)
            {
                this->readings_current[i] = 0;
            }
        }
    }

    void read_voltages()
    {
/* Read all the Voltage ADC channels and store into 
        this->readings_voltages array. (Volts)
        */
#if NO_HARDWARE_MODE == 0
        this->readings_voltage[0] = raw_to_voltage(this->adc_voltage->readLatest(1), 5.0, this->adc_voltage->getChannelMax(1));
        this->readings_voltage[1] = raw_to_voltage(this->adc_voltage->readLatest(2), 5.0, this->adc_voltage->getChannelMax(2));
        this->readings_voltage[2] = raw_to_voltage(this->adc_voltage->readLatest(3), 75.0, this->adc_voltage->getChannelMax(3));
        this->readings_voltage[3] = raw_to_voltage(this->adc_voltage->readLatest(4), 5.0, this->adc_voltage->getChannelMax(4));
#else
        // Non-connected noise
        this->readings_voltage[0] = (float)random(-200, 200) / 1000;
        this->readings_voltage[1] = (float)random(-200, 200) / 1000;
        this->readings_voltage[2] = 30 + (float)random(-200, 200) / 1000;
        this->readings_voltage[3] = (float)random(-200, 200) / 1000;

        // System load. 10th volt noise
        this->readings_voltage[this->load_voltage_channel-1] = 10 + (float)random(-10, 10) / (float)100;
#endif

        // Override if nan, or <0
        for (int i = 0; i < 4; i++)
        {
            if (isnan(this->readings_voltage[i]) || this->readings_voltage[i] < 0)
            {
                this->readings_voltage[i] = 0;
            }
        }
    }

    uint16_t get_highest_pwm_output()
    {
        uint16_t maxOut = 0;
        if (this->outputs_pwm[0] > maxOut)
        {
            maxOut = this->outputs_pwm[0];
        }
        if (this->outputs_pwm[1] > maxOut)
        {
            maxOut = this->outputs_pwm[1];
        }
        if (this->outputs_pwm[2] > maxOut)
        {
            maxOut = this->outputs_pwm[2];
        }
        if (this->outputs_pwm[3] > maxOut)
        {
            maxOut = this->outputs_pwm[3];
        }
        return maxOut;
    }

    void reset_pwm_arrays_to_zero()
    {
        // Resets this->outputs_pwm arrays to zero
        this->outputs_pwm[0] = 0;
        this->outputs_pwm[1] = 0;
        this->outputs_pwm[2] = 0;
        this->outputs_pwm[3] = 0;
    }
};