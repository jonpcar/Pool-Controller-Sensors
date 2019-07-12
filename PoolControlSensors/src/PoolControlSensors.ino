/*
 * Project PoolControlSensors
 * Description:
 * Author:
 * Date:
 */

#include "OneWire.h"
#include <tgmath.h>       // Only needed for the fabs() function...thinking about getting rid of this


STARTUP(WiFi.selectAntenna(ANT_AUTO)); // continually switches at high speed between antennas
//SYSTEM_THREAD(ENABLED);

// Pump Status Section  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
/* 
This "sketch" is dedicated to monitoring the status replies of a Hayward EcoStar Pool Pump on Hayward's implementation of the RS485 bus.
In a real system with a Hayward controller, there are many, many communication packets flowing on this bus, even if there is
only a single pump attached to the wire (as in my system).  This program filters out ONLY the pump status packets so that
the pump wattage and RPM can be extracted from it.  

Thanks to the work by many people but especially on this site for figuring what is going on with the Hayward RS485 bus.
http://www.desert-home.com/2014/07/controlling-hayward-ecostar-pump.html

This is a stop-gap program to monitor the pump status and will be integrated into my pool system monitoring as part of my 1st stage in 
my pool controller software/hardware.   Eventually,  full pump control well be substituted so that the pool controller can 
implement all pump rpm changes and scheduling.

The Photon needs a RS485 Adapter Module connected to its serial port to "talk" to the RS485 bus.   Here is an example:
https://www.amazon.com/gp/product/B07B1WMZM8/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1
On the RS485 Adapter Module, connect VCC to Photon VIN (Pin1), could be another VCC source
On the RS485 Adapter Module, connect VSS to Photon VSS, TXD to Photon TX (Pin3), RXD to Photon RX (Pin4)
On the RS485 Adapter Module, connect the A+ side to EcoStar Pin 7 (same as Ecommand4 Pin2)
On the RS485 Adapter Module, connect the B- side to Ecostar Pin 8 (same as Ecommand4 Pin 3)
On the RS485 Adapter Module, connect the GND side to earth ground (not VSS)
*/

#define currentPumpRPMPercent_tol  5        // if Pump RPM% changes by this number or more, publish the change immediately
#define currentPumpWatts_tol  25            // if Pump Watts changes by more than this, publish the change immmediately
// END Pump Status Section ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



// ETAPE SECTION  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Must calibrate ETape by measuring at least two values from each to get the equation for the resistance line
#define VREF_ADC 3.35F        // Voltage Reference of the PHOTON ADC Converter, PIN 3V3 value
#define VREF_ETAPE 5.14F      // Voltage Reference across the ETape Resistors  (VSS @Red wire, VREF @Black wire)
#define ACID_RR 2.151F        // Reference Resistor of Acid ETAPE (kOhms between White/Black wires, a constant)
#define ACID_R1 1.996F        // Resistance Read from Acid Point 1 (sensor kOhms between Red/White wires)
#define ACID_G1 1.0F          // Gallons of water when ACID_R1 was recorded
#define ACID_R2 .642F         // Resistance Read from Acid Point 2 (sensor kOhms between Red/White wires)
#define ACID_G2 5.0F          // Gallons of water when ACID_R2 was recorded
#define CHLORINE_RR 4.15F     // ....as above execept for cholorine ETape
#define CHLORINE_R1 4.06F
#define CHLORINE_G1 2.0F
#define CHLORINE_R2 0.993F
#define CHLORINE_G2 15.0F

#define CHEM_TANK_SAMPLES_PER_INTERVAL  10  // defines the number of chem tank samples averaged during each CHEM_TANK_SAMPLE_INTERVAL
#define CHEM_TANK_SAMPLE_TIME          400  // time between chem tank analog reads
#define CHEM_TANK_SAMPLE_INTERVAL     5000  // defines the chem tank sampling interval, which will repeatedly be rescheduled, every 5 seconds


#define ONEWIRE_SEARCH 0  // OneWire option: ignore the search code for devices/device addresses
#define ONEWIRE_CRC 1     // OneWire option: enable the CRC code
#define ONEWIRE_CRC16 0   // OneWire option: ignore 16-bit CRC code (redundant since CRC is eliminated on prior line)
// END ETAPE SECTION ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


// DS18B20 Temperature Section vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// ds18b20 resolution is determined by the byte written to it's configuration register
enum DS18B20_RESOLUTION   : uint8_t {
  DS18B20_9BIT  = 0x1f,         //   9 bit   93.75 ms conversion time
  DS18B20_10BIT = 0x3f,         //  10 bit  187.50 ms conversion time
  DS18B20_11BIT = 0x5F,         //  11 bit  375.00 ms conversion time
  DS18B20_12BIT = 0x7F,         //  12 bit  750.00 ms conversion time
};

//if ds18b20 resolution is less than full 12-bit, the low bits of the data should be masked...
enum DS18B20_RES_MASK   :   uint8_t {
  DS18B20_9BIT_MASK  = 0xf8,        
  DS18B20_10BIT_MASK = 0xfc,      
  DS18B20_11BIT_MASK = 0xfe,        
  DS18B20_12BIT_MASK = 0xff,       
};

//ds18b20 conversion time is ALSO determined by the byte written to it's configuration register
enum DS18B20_CONVERSION_TIME   : uint16_t {
  DS18B20_9BIT_TIME  = 94,          //   9 bit   93.75 ms conversion time w/pad
  DS18B20_10BIT_TIME = 188,         //  10 bit  187.50 ms conversion time w/pad
  DS18B20_11BIT_TIME = 375,         //  11 bit  375.00 ms conversion time w/pad
  DS18B20_12BIT_TIME = 750,         //  12 bit  750.00 ms conversion time w/pad
};

#define DS18B20_PIN_ONEWIRE D2                       //  my system implements OneWire on Photon pin D2
#define NUM_DS18B20_DEVICES 5                        //  my system has FIVE DS18B20 devices attached to OneWire (on pin D2)
#define DS18B20_CONVERSION_TIME DS18B20_12BIT_TIME   //  match desired enumerated conversion time above
#define DS18B20_RESOLUTION  DS18B20_12BIT            //  match desired enumerated resolution above
#define DS18B20_RES_MASK DS18B20_12BIT_MASK          //  match desired enumerated resolution mask above (low bits at lower resolutions mean nothing)
#define DS18B20_CRC_RETRIES 2                        //  define how many DS18B20 CRC failure retries are done before moving on
#define DS18B20_FAIL_CRC_VALUE 0x07ff                //  returned when a CRC Fail Condition occurs: =2047 decimal...177 degree celsius...way outside of spec 
#define DS18B20_TEMP_HI_REG 0x55                     //  set to a known value, checkerboard pattern (could be used to abort a "going to fail" crc check)
#define DS18B20_TEMP_LO_REG 0xAA                     //  set to a known value, checkerboard pattern (ditto)

#define DS18B20_SAMPLE_INTERVAL  1000       //  defines project specific DS18B20 Sampling Interval, which determines how often to sample the 
                                            //  DS18B20 devices...in this code, interval reschedules automatically, but could be changed or
                                            //          implemented as a one-shot. 
                                            //  .....for periodic sampling, should be set to: DS18B20_CONVERSION_TIME + pad....but it doesn't matter
                                            //  IF SET to 0, temperature conversions are started and re-started as quickly as possible
OneWire ds18b20_onewire(DS18B20_PIN_ONEWIRE);   // instantiate the OneWire bus                                          

// OneWire DS18B20 8-byte unique addresses that must be obtained and entered into the table below
// Use the OneWire example code called Address_Scanner to gather these...
// ...then input them into the array below.  
// Trying to to discover the addresses on the fly has proven to be troublesome for many who use multiple DS18B20 devices on a single OneWire
// ...it's more efficient to determine them and save them once forever, unless sensors in your system are constantly being swapped 
// ...for new ones...NOTE: finding addresses might be easy now with the FIX to the OneWire code that has been floated out there
// ONLY_ONE: addresses are not needed if there is ONLY_ONE DS18B20 on a OneWire bus, it is faster to not use them, but you still can use them
// If you don't want to find this unique addess for ONLY_ONE device, see the top of this file regarding ONLY_ONE device.
//
// ********* REPLACE THESE ADDRESSES WITH THE UNIQUE ADDRESSES OF YOUR DS18B20 DEVICES **************
//
const uint8_t DS18B20_OneWire_ADDRESSES[NUM_DS18B20_DEVICES][8] = 
    {0x28, 0xAA, 0x8D, 0x68, 0x3F, 0x14, 0x01, 0x2E,    // address of first DS18B20 device
     0x28, 0xAA, 0x49, 0x88, 0x3F, 0x14, 0x01, 0x5A,    // address of 2nd DS18B20 device
     0x28, 0xAA, 0x49, 0x67, 0x3F, 0x14, 0x01, 0x89,    // ..
     0x28, 0xAA, 0xB3, 0x6E, 0x3F, 0x14, 0x01, 0x01,    // ..
     0x28, 0xAA, 0xF6, 0x6F, 0x3C, 0x14, 0x01, 0x51};   // address of last DS18B20 device       

// END DS18B20 Temperature SECTION ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^




// PSI ADS1115 SECTION vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
/* PSI CODE: most variables and constants contain one of these keywords psi/PSI/ADS1115
This "sketch" is dedicated to record FOUR analog sensors attached to an ADS1115 chip.  Ive attached PSI transducers. I've chosen to not use the ADS1115 
library and instead flattened the functions I needed into this code.  I only needed two functions provided by the library:  
[1] start/configure an A2D conversion on one of the four channels (write_PSIadcConfigRegister) AND 
[2] read the A2D result from a previous A2D conversion (read_PSIadcConversionRegister).

In this code, multiple samples (PSI_SAMPLES_PER_INTERVAL) from each of the ADS1115 channels are taken and averaged before recording an interval's value 

A variation of this code will be incorporated into my pool automation project.   My pool controller hardware is controlled by a Particle.io Photon.
Early on I realized that to implement ALL the functionality that I eventually want...I was going to be I/O limited on the Photon.   This is solved
by adding a couple chips to my solution.  In this case, I have added one ADS1115 chip which handles FOUR analog signals and attaches to the i2c bus of 
the Photon.   I am using that chip to read FOUR PSI measurements from various places in my pool plumbing.

This actually solved 2 problems for me: (1) A2D I/O limitation described earlier, this doesn't use up any of the Photons A2D channels or any additional
pins because it is hooked up to the i2c bus of the Photon.  (2) The PSI transducers I chose for this project output a 5V operating range (0.5-5.5V),
the Photon is a 3.3V product, although it is 5V tolerant.  However, the
analog VREF voltage for the Photon is 3.3V and would have limited my ability to read the full PSI range from my chosen transducers.   So,
I now feed 5V directly into the ADS1115 as its VDD (which also is its A2D VREF).   The numbers I get from the
PSI transducers look pretty good and now I can accurately sample the full output voltage range that the PSI transducers produce in my system. 
*/

#define PSI_ADS1115_I2C_ADDRESS   0x48    // i2c base address of the PSI ADS1115 chip in my pool controller, chip must be "hardwired" to this value
// Following is the command for PSI ADS1115 configuration.  It is identical for all four PSI samplings with the exception of the 2-bits in the 
// multiplexer select field which selects one of the four analog inputs.
// See the "ADS1115 Data Sheet" for a more complete description of these bit values...
// bit 15 = start conversion(1), bit 14:12 = input multiplexer field...compare to GND (base 100,101,110,111) , bit 11:9 = Programmable Gain Amplifier (2/3 Mode=000),
// bit 8 = conversion mode(Single Shot=1),  bit 7:5 = Data Rate (860 samples per second=111), bit 4 = compare mode (traditional mode = 0),
// bit 3 = comparator polarity (don't Care for this project =0), bit 2 = comparator latch (don't care for this project = 0),
// bit 1:0 = comparator queue and disable (Disable Comparator=11)
// Resulting PSI_ADS1115_START_COMMAND value: 1+1xx/000+1/111+0/0+0+11 = 0xc1e3 (AD0), 0xd1e3 (AD1), 0xe1e3 (AD2), 0xf1e3 (AD3)
#define PSI_ADS1115_START_COMMAND   0xc1e3  // see explanation above, only bits 13:12 will change depending on which A2D input will be started/read
#define ADS1115_REG_CONVERSION      0x00    // conversion register address for all ADS1115 devices
#define ADS1115_REG_CONFIG          0x01    // configuration register address for all ADS1115 devices
#define PSI_SAMPLE_INTERVAL         1000    // defines the PSI Sampling Interval, which will repeatedly be rescheduled
                                            // .....should be greater than: PSI_SAMPLES_PER_INTERVAL * PSI_CONVERSION_TIME * 4 (# of Sensors) + pad(tbd)
                                            // .....but an "interval check" in the rescheduling should handle this issue if this is "violated"
#define PSI_SAMPLES_PER_INTERVAL      30    // defines the number of PSI samples from each sensor and then averaged during each PSI_SAMPLE_INTERVAL
#define PSI_CONVERSION_TIME	           2    // allowed time for PSI A2D conversion (and to start the next), at 860 samples per second, must be a minimum 1ms
                                            // .....can be increased to "space out" samples within the PSI_SAMPLE_INTERVAL
#define PSI_ADS1115_VOLT_RESOLUTION  0.1875F  // in my system, this is the corresponding voltage (mv) for each bit of ADS1115 A2D resolution 

#define PSI_PUMP_VACUUM_OFFSET    -0.2F     // offset-callibration for sensor connected to ADS1115 AD0, Pool Pump: Vacuum Side
#define PSI_PUMP_PRESSURE_OFFSET  -0.02F    // offset-callibration for sensor connected to ADS1115 AD1, Pool Pump: Pressure Side
#define PSI_FILTER_OFFSET          1.26F    // offset-callibration for sensor connected to ADS1115 AD2, Filter
#define PSI_IFCS_OFFSET            0.82F    // offset-callibration for sensor connected to ADS1115 AD3, In Floor Cleaning System manifold


const float PSI_SENSOR_OFFSETS[4] = 
    {PSI_PUMP_VACUUM_OFFSET,      // experimentally measured offsets to correct/calibrate the phsical readings of my PSI transducers
     PSI_PUMP_PRESSURE_OFFSET,    // ...these offsets are simply a value to make 0 psi readings accurate (they read ~0)
     PSI_FILTER_OFFSET,           // 
     PSI_IFCS_OFFSET};            // 

// END PSI ADS1115 SECTION ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



//Publishing Definitions and variables
#define PUBLISH_MAX_INTERVAL       30000    // every 30 seconds  ...(these values change continuously as I am testing my system...this is 60 seconds)
#define PUBLISH_MIN_INTERVAL       5000     // every 1 second (currently 5 seconds)
unsigned long currentMillis;
bool publishNOW;                            // a particular function may request an immediate status publish by making this "true"
  // publishing temperature differential, publish the data immediately (subject to PUBLISH_MIN_INTERVAL) if its 
  // temperature differential from the previous PUBLISHED value is greater than this number...used in a floating point comparison
  // ...An easy way to get quick publishes during testing...grab a probe with your hand and raise its temperature
#define PUBLISH_TEMPERATURE_DIFF      1     


// Pump Status Variables
uint8_t currentPumpRPMPercent;           // current pump RPM% 0-100
uint8_t currentPumpRPMPercent_pub;       // current published pump RPM% 0-100
uint16_t currentPumpWatts; 
uint16_t currentPumpWatts_pub; 

//Publishing Parameters for the PSI
float f_current_psi_pub[4];       // last published value of the PSIs
float f_current_psi[4];           // PSIs from most recently finished PSI sampling interval
float f_psi_min[4];               // used to store min and max values of PSIs that occur between PUBLISHED values (hi-lo values during that period)
float f_psi_max[4];               // ....not sure exactly what I will use these for yet, just testing it
int current_psis_raw[4];          // contains the current (latest) completed psi samples in raw format


int16_t current_temps_raw[NUM_DS18B20_DEVICES];    // current raw readings from temperature sensors
float f_current_temps[NUM_DS18B20_DEVICES];        // current temperature readings from sensors
float f_current_temps_pub[NUM_DS18B20_DEVICES];    // last published temperatures readings from sensors


// Pump Status Variables
uint16_t current_acid_tank_raw, current_chlorine_tank_raw;         
float current_acid_tank, current_acid_tank_pub, current_chlorine_tank, current_chlorine_tank_pub;        

// Function declarations
void write_PSIadcConfigRegister(uint16_t value);
int16_t read_PSIadcConversionRegister();
bool publishAllStatus();
bool publishNonBlocking(const char sheet_name, const char message);
bool PsiSamplingComplete();
void doPsiCalculations();
void publishData();
bool timeToPublish();

void start_DS18B20_Conversions();   
int16_t read_DS18B20_Conversion(const uint8_t addr[8], uint8_t ptr);
void doTemperatureCalculations();
bool TemperatureSamplingComplete();

bool ChemTankSamplingComplete();
void doChemTankCalculations();

void grabRS485PumpStatus();



void setup() 
{
  Serial.begin(9600);
  Wire.begin();   // initialize the i2c bus
  set_DS18B20_Resolutions(DS18B20_RESOLUTION); 

    //Setup for the RS485 bus
  Serial1.blockOnOverrun(false);
	Serial1.begin(19200, SERIAL_8N2);  // 1 stop bit and 2 stop bits both seemed to work, I settled on 1

}



void loop() 
{
  currentMillis = millis();


    // Publish status if conditions are met
  if (timeToPublish()) publishData();

    // Pump Status
  grabRS485PumpStatus();

    // PSI checks
  if (PsiSamplingComplete()) doPsiCalculations();

    // Etapes for chemical tanks
  if (ChemTankSamplingComplete()) doChemTankCalculations();

    // temperatures
  if (TemperatureSamplingComplete()) doTemperatureCalculations();
}



void publishData(){
// function that publishes selected data...this will be expanded
  if (publishAllStatus()) {     // function attempts to publish the status
    publishNOW = false;         // ...if successful then get ready for next publish
    for (uint8_t i=0; i<4; i++) {
      f_current_psi_pub[i] = f_current_psi[i];  // update the published values
      f_psi_max[i] = -14.7;                     // reset the min/max's for the publish interval
      f_psi_min[i] = 50;  
    }
    
    for (uint8_t i = 0; i < NUM_DS18B20_DEVICES; i++) {
      f_current_temps_pub[i] = f_current_temps[i];  // update the published temperaturre data
    }

    current_acid_tank_pub = current_acid_tank;
    current_chlorine_tank_pub = current_chlorine_tank;

    currentPumpRPMPercent_pub = currentPumpRPMPercent;
    currentPumpWatts_pub = currentPumpWatts;  
  }
}



bool timeToPublish() {
// function to check if it is time to Publish: either forced (publishNOW) or a timeout of the PUBLISH_MAX_INTERVAL
// and then setup for the next publish event
  static long prior_publish_time;    

  if (((currentMillis - prior_publish_time >= PUBLISH_MIN_INTERVAL) && publishNOW) ||
      (currentMillis - prior_publish_time >= PUBLISH_MAX_INTERVAL)) {
    prior_publish_time = currentMillis;                    // setup for the next publish time
    return(true);
  }
  return(false);
}



bool PsiSamplingComplete() {
// The following code does analog sampling of the PSI transducers attached to the ADS1115 chip.  Currently, one ADS1115 chip is used in this
// project.  All four channels are dedicated to PSI measurements throughout the system.   This makes it easy to
// combine the code for all readings (which are similar) into one routine.  
//
// When an ADS1115 A2D conversion is started, the start time is recorded as a marker to know when the next can be started (PSI_CONVERSION_TIME)
// The four analog conversions (corresponding to my
// pool's four pressure sensors) are started and then read one by one and stored.   Based on the desired PSI_SAMPLES_PER_INTERVAL, multiple
// readings are repeated and accumulated in the psi_accumulators.  At the end of an interval, the accumulated PSI readings are divided by 
// the number of samples in each accumulator (PSI_SAMPLES_PER_INTERVAL) to obtain an average reading for the sampling interval
//
// The PSI A2D readings are controlled using the same command (with the exception of the channel selection bits).  
// One-shot, single ended A2D readings are taken.  Each conversion are started individually by code, and the results are
// read individually by the code.  Conversions are done at the fastest sampling speed (860 samples per second).  
  static unsigned long prior_psi_a2d_start = 0, prior_psi_interval_start = 0, current_psi_interval_start = 0;
  static int psi_accumulator[4];  // accumulators for the PSI raw samples, divide by PSI_SAMPLES_PER_INTERVAL to get an average reading for each interval
  static uint8_t psi_sample_count, psi_pntr;
  static bool psi_conversion_started;

  // Enter this code body if within a valid PSI sampling interval window AND any prior PSI a2d conversion has been completed
  if (((currentMillis - prior_psi_a2d_start) >= PSI_CONVERSION_TIME)  && ((currentMillis - prior_psi_interval_start) >= PSI_SAMPLE_INTERVAL)) {
        // 1) start a PSI sampling conversion 2) read a sampled PSI conversion 3) sampling for the interval is complete
    if ((!psi_conversion_started) && (psi_sample_count < PSI_SAMPLES_PER_INTERVAL)) {      
        // starts a PSI a2d conversion on the appropriate channel by bitwise ORing in channel (AD3-AD0) from current psi_pntr           
      write_PSIadcConfigRegister(PSI_ADS1115_START_COMMAND | (psi_pntr << 12)); 
      prior_psi_a2d_start = millis();                       // capture the start time for completion check reference
      psi_conversion_started = true;
      if ((psi_sample_count == 0) && (psi_pntr == 0 ))      // checks if this is the VERY FIRST conversion for the PSI sampling interval
        current_psi_interval_start = prior_psi_a2d_start;   // ...if so, record interval start time so that the next interval can be scheduled later
    }
    else if (psi_conversion_started) {
      psi_accumulator[psi_pntr] += read_PSIadcConversionRegister();    // accumulates the result of the previously started PSI a2d conversion
      psi_conversion_started = false;
      if (++psi_pntr >= 4) {  //  advance pointer to next analog channel of ADS1115
        psi_pntr = 0;         //  ...reset to 0 if 4+, there are only four channels 0:3
        psi_sample_count++;   //  increment sample count for this PSI interval, all four channels have been sampled and values read/accumulated
      }
    }
    else {   // once IP gets here, all samples have been completed, so setup for the next PSI sample interval and return "true" for FINISHED
      for (uint8_t i = 0; i < 4; i++) {
        current_psis_raw[i] = psi_accumulator[i] / PSI_SAMPLES_PER_INTERVAL;   // simple integer truncate divide, no real need for floating point
        //Serial.printlnf("%01d PSI RAW %05d", i, current_psis_raw[i]);
        psi_accumulator[i] = 0;
      }                        
      psi_sample_count = 0;   
      prior_psi_interval_start =    // just in case the sampling during the sample interval was held up or exceeded the PSI_SAMPLE_INTERVAL period
        ((currentMillis - PSI_SAMPLE_INTERVAL) > current_psi_interval_start) ? currentMillis : current_psi_interval_start;
      return(true);
    }
  }
  return(false);
}


void doPsiCalculations() {
  // All conversion for the PSI interval have been completed... 
  //
  // For my sensors:
  // Vs (from sensor to ADS1115 pin) = analogRead (of the ADS1115 sensor) * 
  //                          PSI_ADS1115_VOLT_RESOLUTION (in mv) / 1000
  //  
  // ...my sensors measure from -14.7PSI to 50PSI (total PSI range of 64.7)
  // ...my sensors' voltage output is .5V - 5.5V (total Voltage range of 5.0V)
  // ...my sensors' have an offset voltage of 0.5 (the lowest voltage output for -14.7 PSI)
  // PSI = (PSI range of sensor) / (voltage range of sensor) * [Vs - (Voffset of sensor)]
  //     = (64.7) / (5) * [Vs - 0.5]
  //
  // To normalize for ambient air pressure (in Gilbert, AZ where my pool is), 14.05 must be subtracted 
  //        (Gilbert is not at sea level)
  // Finally, an offset is needed to calibrate physical readings from the 
  //        imperfect sensor (calibration measurements were taken at 0psi)   
  // TODO: possibly a dynamic calibration at startup or other 'pump off' conditons when PSI should read 0

  for (uint8_t i = 0; i < 4; i++) {       // actual calculations to determine PSI
    f_current_psi[i] = ((current_psis_raw[i] * PSI_ADS1115_VOLT_RESOLUTION / 1000-.5) * 64.7/5) 
                          - 14.05 + PSI_SENSOR_OFFSETS[i]; 
    if (f_current_psi[i] < f_psi_min[i]) f_psi_min[i] = f_current_psi[i];       // keep a min and max per "publish period"
    else if (f_current_psi[i] > f_psi_max[i]) f_psi_max[i] = f_current_psi[i];  // .....don't know exactly what I will use this for yet
  }
     
/*
  // the following code is for testing to determine if it is worthwhile to publish more often during times when  data is changing rapidly
  // ...I'll continue to experiment with variations of this and "clean it up" if I go ahead with it (would be in prior loop)
  if (fabs(f_current_psi_pub[1] - f_current_psi[1]) > .2) {  //TODO: is it worthwhile to include <tgmath.h> library for this one command?
  //if (((f_current_psi_pub[1] - f_current_psi[1] > .2)) || ((f_current_psi_pub[1] - f_current_psi[1]) < -.2)) {
//    publishNOW = true;
    //Serial.println("Large PSI difference from published value, force a new publish"); 
    //Serial.print("PV0: "); Serial.print(f_current_psi[0],1); Serial.print(" "); Serial.println(f_current_psi_pub[0],1);
    //Serial.print("PV1: "); Serial.print(f_current_psi[1],1); Serial.print(" "); Serial.println(f_current_psi_pub[1],1);
    //Serial.print("PV2: "); Serial.print(f_current_psi[2],1); Serial.print(" "); Serial.println(f_current_psi_pub[2],1);
    //Serial.print("PV3: "); Serial.print(f_current_psi[3],1); Serial.print(" "); Serial.println(f_current_psi_pub[3],1);    
    //Serial.println(" ");
  }
*/

}


void write_PSIadcConfigRegister(uint16_t value) {
// Writes the PSI ADS1115 Configuration Register via the i2c bus
  Wire.beginTransmission((uint8_t)PSI_ADS1115_I2C_ADDRESS);
  Wire.write((uint8_t)ADS1115_REG_CONFIG);
  Wire.write((uint8_t)(value>>8));
  Wire.write((uint8_t)(value & 0xFF));
  Wire.endTransmission();
}

int16_t read_PSIadcConversionRegister() {
// Reads from the PSI ADS1115 Conversion Register via the i2c bus
  Wire.beginTransmission((uint8_t)PSI_ADS1115_I2C_ADDRESS);
  Wire.write((uint8_t)ADS1115_REG_CONVERSION);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)PSI_ADS1115_I2C_ADDRESS, (uint8_t)2);
  return ((Wire.read() << 8) | Wire.read());  
}



bool TemperatureSamplingComplete() {
// This code starts a conversion on all DS18B20s simultaneously, and then, later when the conversions are finished, reads the results
// Only one sampled conversion is taken for each DS18B20..if the sampled conversion fails the CRC checks, a previous sampled conversion is kept 
// Since there is no rush to get these conversions recorded, this function is designed so 
// ...that only one conversion read happens on any given pass through it.  This avoids cramming 
// ...a bunch of execution time into one particular pass of the user code.
  static long prior_DS18B20_interval_start = 10000; 
  static long prior_DS18B20_conversion_start = 10000;
  static long current_DS18B20_interval_start = 20000;
  static int16_t temperature_read_raw;
  static uint8_t DS18B20_ptr = 0;
  static bool DS18B20_conversion_reads_in_progress = false;

  // Enter the code body ONLY if within a valid DS18B20 sampling interval window AND prior DS18B20 temperature conversions have had time to complete
  if (((currentMillis - prior_DS18B20_conversion_start) >= DS18B20_CONVERSION_TIME)  && 
      ((currentMillis - prior_DS18B20_interval_start) >= DS18B20_SAMPLE_INTERVAL)) {

    if (!DS18B20_conversion_reads_in_progress && (DS18B20_ptr == 0)) {      
        // starts temperature conversions on all DS18B20 devices attached to the OneWire bus           
      start_DS18B20_Conversions(); 
      prior_DS18B20_conversion_start = millis();                         // capture conversion start so the "reads" can be scheduled
      current_DS18B20_interval_start = prior_DS18B20_conversion_start;   // capture the start time so next interval can be scheduled
      DS18B20_conversion_reads_in_progress = true;
    }
    else if (DS18B20_conversion_reads_in_progress) {
        // reads one of the DS18B20 temperature conversions
      temperature_read_raw = read_DS18B20_Conversion(DS18B20_OneWire_ADDRESSES[DS18B20_ptr], DS18B20_ptr); //if ONLY_ONE DS18B20, take out the address reference
      
      if (temperature_read_raw != DS18B20_FAIL_CRC_VALUE)
        current_temps_raw[DS18B20_ptr] = temperature_read_raw; 

      if (++DS18B20_ptr >= NUM_DS18B20_DEVICES)   
        DS18B20_conversion_reads_in_progress = false;  // all DS18B20 conversions have been read
    }
    else {              // all sampled conversion have been recorded, so setup for the next DS18B20 sample interval 
      DS18B20_ptr = 0;                  //  setup to read the sensors again
      prior_DS18B20_interval_start =    // check if (for any reason) it took longer than DS18B20_SAMPLE_INTERVAL to get the conversions
        ((currentMillis - current_DS18B20_interval_start) > DS18B20_SAMPLE_INTERVAL) ? millis() : current_DS18B20_interval_start;
      return(true);
    }
  }
  return(false);
}


void doTemperatureCalculations() {
// This does the temperature calculations (from the RAW values) and stores them,
// the latest results are updated and always available within these global arrays: 
// RAW values: current_temps_raw[NUM_DS18B20_DEVICES], these are the integer values read from the sensors
// current temperatures:  f_current_temps[NUM_DS18B20_DEVICES]
  float temperature;

  for (uint8_t i = 0; i < NUM_DS18B20_DEVICES; i++) {
    //temperature = current_temps_raw[i] / 16.0;  // this is the Celsius calculation read from the ds18b20
    temperature = current_temps_raw[i] / 16.0 * 1.8 + 32;  // this is the Farenheit calculation read from the ds18b20
         // force a publish if temperature has changed by more than 1 degree since last published
    if (fabs(f_current_temps_pub[i] - temperature) > PUBLISH_TEMPERATURE_DIFF) publishNOW = true;  
    f_current_temps[i] = temperature; 
  }
}



void set_DS18B20_Resolutions(uint8_t resolution) {
// this function sets the resolution for ALL ds18b20s on an instantiated OneWire
  ds18b20_onewire.reset();        // onewire intialization sequence, to be followed by other commands
  ds18b20_onewire.write(0xcc);    // onewire "SKIP ROM" command, selects ALL ds18b20s on bus
  ds18b20_onewire.write(0x4e);    // onewire "WRITE SCRATCHPAD" command (requires write to 3 registers: 2 hi-lo regs, 1 config reg)
  ds18b20_onewire.write(DS18B20_TEMP_HI_REG);   // 1) write known value to temp hi register 
  ds18b20_onewire.write(DS18B20_TEMP_LO_REG);   // 2) write known value to temp lo register
  ds18b20_onewire.write(resolution);            // 3) write selected resolution to configuration registers of all ds18b20s
}


void start_DS18B20_Conversions() {
// this function intitalizes simultaneous temperature conversions for ALL ds18b20s on an instantiated OneWire
  ds18b20_onewire.reset();          // onewire intitialization sequence, to be followed by other commands
  ds18b20_onewire.write(0xcc);      // onewire "SKIP ROM" command, addresses ALL (or one if there is only one) ds18b20s on bus
  ds18b20_onewire.write(0x44);      // onewire wire "CONVERT T" command, starts temperature conversion on ALL ds18b20s
}



int16_t read_DS18B20_Conversion(const uint8_t addr[8], uint8_t ptr) {
// this function returns the RAW temperature conversion result of a SINGLE selected DS18B20 device (via it's address)
// If there is a CRC failure in the process, the previously converted result is just re-read...a new conversion is not started.
// It is reattempted up to DS18B20_CRC_RETRIES times
// The pointer to a particular DS18B20 was addeed as a parameter for testing purposes  to check if a particular DS18B20 device
// was having issues with the OneWire Protocol.   I'm leaving it for now
  byte  data[9];
  bool crc_error;
  int crc_retries = 0;

  do {
    ds18b20_onewire.reset();          // onewire intitialization sequence, to be followed by other commands
    ds18b20_onewire.select(addr);     // issues onewire "MATCH ROM" address which selects a SPECIFIC (only one) ds18b20 device
      //if ONLY_ONE DS18B20, replace the line above  "ds18b20_onewire.select(addr);" with the one directly below
      //  ds18b20_onewire.write(0xcc);      // onewire "SKIP ROM" command, selects the ONLY_ONE ds18b20 on bus without needing address
      //
    ds18b20_onewire.write(0xBE);      // onewire "READ SCRATCHPAD" command, to access selected ds18b20's scratchpad
      // reading the bytes (9 available) of the selected ds18b20's scratchpad 
    for (int i = 0; i < 9; i++) data[i] = ds18b20_onewire.read();
      // check the crc
    crc_error = (data[8] != OneWire::crc8(data, 8));  

  } while ((crc_error && (crc_retries++ < DS18B20_CRC_RETRIES)));

    // if the temperature conversion was successfully read, pass it back...else return the CRC FAIL value 
  return (int16_t) (crc_error ?  DS18B20_FAIL_CRC_VALUE : ((data[1] << 8) | (data[0] & DS18B20_RES_MASK)));
}



bool ChemTankSamplingComplete() {
  static unsigned long prior_chem_tank_sample = 0, prior_chem_tank_interval_start = 0, current_chem_tank_interval_start = 0;
  static int acid_tank_accumulator, chlorine_tank_accumulator;  // accumulators for tank read, will be averaged
  static uint8_t chem_tank_sample_count;

  // Enter this code body if within a valid chem tank sampling interval window AND another chem tank sample is scheduled
  if (((currentMillis - prior_chem_tank_sample) >= CHEM_TANK_SAMPLE_TIME)  && 
      ((currentMillis - prior_chem_tank_interval_start) >= CHEM_TANK_SAMPLE_INTERVAL)) {
    if (chem_tank_sample_count < CHEM_TANK_SAMPLES_PER_INTERVAL) { 
      acid_tank_accumulator += analogRead(A0);
      chlorine_tank_accumulator += analogRead(A1);  
      prior_chem_tank_sample = currentMillis;                      
      if (chem_tank_sample_count == 0)      // checks if this is the VERY FIRST read of the chem tanks for this sampling interval
        current_chem_tank_interval_start = currentMillis;   // ...if so, record interval start time so that the next interval can be scheduled later
      chem_tank_sample_count++;
    }
    else {   // once IP gets here, all samples have been completed, so setup for the next chem tank sample interval and return "true" for FINISHED
      current_acid_tank_raw = acid_tank_accumulator / CHEM_TANK_SAMPLES_PER_INTERVAL;  // simple integer truncate divide
      current_chlorine_tank_raw = chlorine_tank_accumulator / CHEM_TANK_SAMPLES_PER_INTERVAL;  // simple integer truncate divide
      acid_tank_accumulator = 0;
      chlorine_tank_accumulator = 0;
      chem_tank_sample_count = 0;
      prior_chem_tank_interval_start =    // just in case the sampling during the sample interval was held up or exceeded the CHEM_TANK_SAMPLE_INTERVAL period
        ((currentMillis - CHEM_TANK_SAMPLE_INTERVAL) > current_chem_tank_interval_start) ? currentMillis : current_chem_tank_interval_start;
      return(true);
    }
  }
  return(false);
}


void doChemTankCalculations() {
// There are probably easier ways to do this but here is how I figured out the gallons from the ETape measurements
// First find the actual resistance of the sensor reading by using the fact that the sensor resistance is in series with the
// reference resistance and a voltage reading was done betweend those two resistors.  That calculation is as follows:
// Vout = (Rsensor/(Rsensor + Rref)) * VREF_ETAPE  where  Vout = VREF_ADC * (raw reading/4095)
// solving for Rsensor    Rsensor = (Vout * Rref) / (VREF_ETAPE - Vout)
// Next plug the Rsensor value into the equation for a line derived from the 2 data points taken from each sensor
// Equation of a line solving for gallons    G = (g2-g1)/(r2-r1) * (Rsensor - r1) + g1
  float Rsensor;

  Rsensor = ACID_RR * (current_acid_tank_raw * VREF_ADC/4095) / (VREF_ETAPE - current_acid_tank_raw * VREF_ADC/4095);
  current_acid_tank =  (ACID_G2 - ACID_G1)/(ACID_R2 - ACID_R1) * (Rsensor - ACID_R1) + ACID_G1;
  Rsensor = CHLORINE_RR * (current_chlorine_tank_raw * VREF_ADC/4095) / (VREF_ETAPE - current_chlorine_tank_raw * VREF_ADC/4095);
  current_chlorine_tank =  (CHLORINE_G2 - CHLORINE_G1)/(CHLORINE_R2 - CHLORINE_R1) * (Rsensor - CHLORINE_R1) + CHLORINE_G1;

  // Serial.printlnf("Resistance : %0.3f    Gallons: %0.2f", , gallons);
}


void grabRS485PumpStatus() {
// Hayward RS485 SNIFFING for pump status responses, extract RPM and Watts, does not check the checksum or
//    any other bytes after the values needed: RPM, Watts
// All traffic on the bus that is not a pump status is simply discarded
  uint8_t wattsHi, wattsLo;

    //clear all useless bytes from the serial port, wait for a packet start byte (0x10)
  while ((Serial1.available() >= 1) && (Serial1.peek() != 0x10)) Serial1.read();

    // try to read pump status only if it starts correctly & has 13 bytes, if not, return and wait for all bytes to get here
	if (Serial1.available() >= 13 && Serial1.peek() == 0x10) { 
    Serial1.read(); //discard the first byte (0x10)
    if (Serial1.read() == 0x02)       // we get status only if all the early btes correspond to "pump status packet"
      if (Serial1.read() == 0x00)
        if (Serial1.read() == 0x0c)
          if (Serial1.read() == 0x00)
            if (Serial1.read() == 0x00) {
              currentPumpRPMPercent = Serial1.read();  // finally...the RPM
              wattsHi = Serial1.read();                // and the watts
              wattsLo = Serial1.read();
              currentPumpWatts = ((wattsHi & 0xF0) >> 4) * 1000 + ((wattsHi & 0x0F ) * 100) +
				                         ((wattsLo & 0xF0) >> 4) * 10   +  (wattsLo & 0x0F );
              if (abs(currentPumpRPMPercent-currentPumpRPMPercent_pub) >= currentPumpRPMPercent_tol) publishNOW = true;
              if (abs(currentPumpWatts - currentPumpWatts_pub) >= currentPumpWatts_tol) publishNOW = true;
            }
  }
}



bool publishAllStatus() {
// Publishes the status, in my case: specifically sends it to a Google spreadsheet and the PoolController Android app
// Formatting (using snprintf) changes as per Scruff recommendation
  const char gsSheet[] = "AllStatus";  // google sheet page
  char stats[622];  // place holder for now

snprintf(stats, sizeof(stats),
      "{\"Pv\":%.1f"      // PSI Pump Vacuum Side
        ",\"Pp\":%.1f"    // PSI Pump Pressure Side
        ",\"Pf\":%.1f"    // PSI Filter
        ",\"Pi\":%.1f"    // PSI On-Floor-Cleaning-System Manifold
        ",\"Tp\":%.1f"    // Temperature Pool
        ",\"Tc\":%.1f"    // Temperature Chlorine
        ",\"Tm\":%.1f"    // Temperature Muriatic Acid
        ",\"Ta\":%.1f"    // Temperature Ambient
        ",\"Te\":%.1f"    // Temperature Electronics (pool controller electronics)
        ",\"La\":%0.2f"   // Level Acid Tank
        ",\"Lc\":%0.2f"   // Level Chrlorine Tank
        ",\"Rp\":%d"      // RPM Pump
        ",\"Wp\":%d"      // Watts Pump
     "}",
      f_current_psi[0],
      f_current_psi[1],
      f_current_psi[2],
      f_current_psi[3],
      f_current_temps[0],
      f_current_temps[1],
      f_current_temps[2],
      f_current_temps[3],
      f_current_temps[4],
      current_acid_tank,
      current_chlorine_tank,
      currentPumpRPMPercent,
      currentPumpWatts
  );
  return publishNonBlocking(gsSheet, stats);
}


bool publishNonBlocking(const char* sheet_name, const char* message) {
// A wrapper around Partical.publish() to check connection first to prevent
// blocking. The prefix "pool-" is added to all names to make subscription easy.
// "name" is the "sheet" name within the google spreadsheet that this is being sent to
    const char evtPrefix[] = "pool-";
    char evtName[sizeof(evtPrefix) + strlen(sheet_name)];

    snprintf(evtName, sizeof(evtName), "%s%s", evtPrefix, sheet_name);
    // TODO replace with a failure queue?
    if (Particle.connected()) {
        bool success = Particle.publish(evtName, message, PUBLIC); // TODO, need to understand ramifications of making this PRIVATE
//        Serial.printlnf("Published \"%s\" : \"%s\" with success=%d",
//                        name.c_str(), message.c_str(), success);
        return success;
    } else {
//        Serial.printlnf("Published \"%s\" : \"%s\" with success=0 no internet",
//                        name.c_str(), message.c_str());
    }
    return false;
}


