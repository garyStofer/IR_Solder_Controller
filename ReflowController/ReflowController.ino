// ----------------------------------------------------------------------------
// Reflow Oven Controller
//  2016/2017 Gary Stofer gary@stofer.name
// (c) 2014 Karl Pitrich <karl@pitrich.com>
// (c) 2012-2013 Ed Simmons
// ----------------------------------------------------------------------------

#include <avr/eeprom.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <SPI.h>
#include <PDQ_GFX.h>             // PDQ: Core graphics library
#include "PDQ_ST7735_config.h"   // PDQ: ST7735 pins and other setup for this sketch
#include <PDQ_ST7735.h>          // PDQ: Hardware-specific driver library
#include <Menu.h>
#include <ClickEncoder.h>
#include <TimerOne.h>
#include "max6675.h"

#include "portMacros.h"
#include "temperature.h"
#include "helpers.h"
#include "UI.h"
#include "globalDefs.h"


// ----------------------------------------------------------------------------
static volatile uint16_t    timerTicks = 0;  // this counts 0.1ms needs to be large enough to hold the longest wavepacked interval, 6.5second is way long enough 
static const uint8_t TIMER1_PERIOD_US = 100;
// ----------------------------------------------------------------------------
uint32_t lastUpdate        = 0;
State    previousState     = Idle;
bool     stateChanged      = false;
uint32_t stateChangedTicks = 0;
// ----------------------------------------------------------------------------
// PID

PID PID(&Input, &Output, &Setpoint, heaterPID.Kp, heaterPID.Ki, heaterPID.Kd, DIRECT);


struct {
  double temp;
  uint16_t ticks;
} airTemp[NUM_TEMP_READINGS];

double readingsT1[NUM_TEMP_READINGS]; // temp averageing
double runningTotalRampRate;
double rateOfRise = 0;          // the result that is displayed
double totalT1 = 0;             // the running total
double averageT1 = 0;           // the average
uint8_t index = 0;              // the index of the current reading
/*
// data structure for wave packet control of heater SCR 
struct  {
	volatile uint8_t target; // the desired percentage of on-time
	uint8_t state;           // current state counter
	int16_t next;            // time in 0.1us when the next change in output shall occur , type must match timerTicks var
	bool On;                // off/On action for next cycle On == flase means the solid state switch is going to be commanded off
	uint8_t pin;             // io pin of solid state relais, a high on this pin turns on the SolidState switch
} Heater = { 0, 0, 0, false, PIN_HEATER };
*/
// delay in us to align SCR activation with the actual zero crossing of the AC sine wave
uint16_t volatile zxDelay = 1;		// skip initial interrupt upon reset

// Initialize the ports and set pins to their appropriate state for save startup
void setupPins(void) 
{
  	pinAsOutput(PIN_HEATER);
	digitalLow(PIN_HEATER); // off

	pinAsInputPullUp(PIN_ZX);		// This pin needs an external 3.9K pull up -- Internal pull up is between 100K and 500K and is not enough

	pinAsOutput(PIN_TC_CS);
	digitalHigh(PIN_TC_CS); // off

	pinAsOutput(PIN_LCD_CS);
	digitalHigh(PIN_LCD_CS); // off

#ifdef PIN_BEEPER
	pinAsOutput(PIN_BEEPER);
#endif
	
#ifdef PIN_LED
	pinMode(PIN_LED, OUTPUT);    // Left hand LED
#endif

}
// ----------------------------------------------------------------------------
void abort(void) 
{
	Timer1.stop();
	detachInterrupt(INT_ZX);
	digitalLow(PIN_HEATER);

}
// ----------------------------------------------------------------------------
void abortWithError(int error) {
  abort();
  displayError(error);
}
/*  WavePacket Control

	With wave packet control the SS switch of the load is always energized for complete half cycles, therfore eliminating harmfull harmonics from 
	sharp edges of phase-cut control and providing a soft start for the halogen lamp (heater). A zero cross detector circuit provides 
	the timing means to start the SS switch at around 0V. After the SS switch is turned on near the 0-xing its control signal can be turned off
	midway into the cycle, or can be left on until the middle of the last ON cycle in a sequence.
	
	Wave packet control, unlike a simple fixed timeslot control intersperses ON and OFF cycles at the fastest possible rate to eliminate 
	flickering.
	
	For example, in a fixed timeslot system with a 1% control resolution and a chosen duty cycle of 50% the load would be switched  ON for 50 cycles,
	followed by the next 50 cycles being OFF.
	
	In a WavePacket system the same 50% dutycycle is achieved by having one cycle ON followed by the next cycle OFF. Or for a 25% 
	dutycycle, 3 cycles OFF followed by 1 cycle ON, 90% = 1OFF 9ON, etc..

	The structure WavePack_t below is used to figure the distribution of the ON v.s. the OFF. The math assumes a 1% resolution, so the 
	dutycycle can be chosen in 1% steps between 0 and 100%. The variables target and state are involved in finding the distribution.

*/

// NOTE: this ISR is specific to INT1 , PC3 external interrupt, 
void zeroCrossingIsr(void) {
	// measure how long the zero crossing signal is. This depends on the value of the drive resistors of the 0-xing opto
	// and should be in the range of 1 - 4ms. The zero crossing is the half way point of this signal.  
	// Change the interupt edge to falling to measure the time between the rising and falling edge of the signal	
	
	if (zxDelay == 1 ) // skipp over the first interrupt, it can happen async to the signal during turn on
	{	
		zxDelay=0;
		return;
	}
	
	if (zxDelay == 0 ) // do once upon startup -- Should not change much for the duration
	{	
		if ( (EICRA & 0xc)  == 0xc) // if triggering on rising edge of INT1
		{	
			TCNT1 = 0;
			EICRA ^= 0x4; 		// switch to falling edge 
			return;
		}
		
		zxDelay = TCNT1;		// The delay to the 0 crossing is half of the pulse width of the 0-xing device, then divided by 2 for the 0.5us counters step to make it in us 
		zxDelay /= 4;			// must read TCNT1 into variable first before applying math, TCNT1 is a macro that reads two 8 bit registers
		EICRA |= 0x4;			// switch back to raising edge for rest of operation	
		zeroCrossTicks++;
	}
	else
	{
		// Sync the 100us timer to the line frequency, this triggers the 100us interrupt right away 
		TCNT1 = 0; 
		
	  	// calculate wave packet parameters
		Heater.state += Heater.target;

		// every half cycle we evaluate if the heater needs to be on or off in the next half cycle
		if (Heater.state >= 100) // 100 == 100% 
		{
			Heater.state -= 100;
			Heater.On = true;
		}
		else 
			Heater.On = false;
		
		Heater.next = timerTicks + (( zxDelay+50) /100); // round and divide by 100 to make 100us units for the delay
		zeroCrossTicks++;		// this counts AC 1/2 cycles , ie. 10ms at 50HZ, 8.33ms at 60HZ
	}

}

// ----------------------------------------------------------------------------
//                                    TIMER ISR
// ----------------------------------------------------------------------------
// timer interrupt handling -- timer gets reset on evry 0-xing (half cycle) 

void timerIsr(void) // ticks at 100µS
{ 

	// turn on the heater control at the 0 xing  using a SCR 
	if ( timerTicks == Heater.next && Heater.On) 
		digitalHigh(Heater.pin);
	
	// we can take the trigger signal off after it fired the SCR it is self holding
	if ( timerTicks == Heater.next+20)  // after 2ms shut off 
		digitalLow(Heater.pin); 

	// handle encoder + button evry 1ms
	if (!(timerTicks % 10)) 
	{
		Encoder.service();
	}
	timerTicks++;

}

// ----------------------------------------------------------------------------
// The Arduino Environment setup function -- called upon reset
void setup() 
{
	setupPins();
	Serial.begin(115200);
	Serial.println("Reflow controller started");
    setupTFT();

	displaySplash();
	
	//Timer1.initialize(10000);	//Setup TC1 to run for 10ms with a 4 us clock to count the duration of the zero scrossing signal
	
	TCCR1A =0;
	TCCR1B =2; // clk /8 == 2mhz, == 0.5us 
	TCNT1 =0;
	TIMSK1 =0;
	TIFR1 =0;
	
	attachInterrupt(INT_ZX, zeroCrossingIsr, RISING);	 //triggers on every rising edge of the zero crossing interrupt.
	do
	{
		zeroCrossTicks =0;
		delay(1000);  // waiting 1 second to measure line frequency in HZ 
		LineFreq = (zeroCrossTicks+1)/2; // rounding and dividing by two for HZ readout. 
		if (LineFreq < 40 )
		{
			Serial.println("No AC frequency detected -- Not plugged in ?");
			  tft.setTextSize(1);
			  tft.setTextColor(ST7735_WHITE, ST7735_RED);
			  tft.setCursor(20, 10);
			  tft.println("No AC detected");
#ifdef PIN_BEEPER
			digitalHigh(PIN_BEEPER); // beep
			delay(100);
			digitalLow(PIN_BEEPER); // off
#endif
		}
		
	} while (LineFreq < 40 );

	Serial.print( "ZX delay us:");
	Serial.println( zxDelay);

	Serial.print( "Line Freq  HZ:");
	Serial.println( LineFreq );
	
  // need to wait at least 250ms from when reset was released because the CS of the TC device was active during reset and it takes ~200ms for a conversion to be finished
  // the above delay for neasuring the line frequency does satisfy this requirement

	temperature = readThermocouple();
	Serial.print("temp reading: ");
	Serial.println(temperature);

	// initialize moving average filter
	for(int i = 0, runningTotalRampRate =0; i < NUM_TEMP_READINGS; i++) 
	{
		airTemp[i].temp = temperature;
		runningTotalRampRate += temperature;
	}

	// initializer the 0.1ms timer and attach the interrupt that deals with the scr control
	Timer1.initialize(TIMER1_PERIOD_US);	// clicks at 0.1ms
	Timer1.attachInterrupt(timerIsr);


	if (firstRun()) 
	{
		factoryReset();
		loadParameters(0);
	} 
	else
		loadLastUsedProfile();
	

	loadFanSpeed();
	loadPID();

	PID.SetOutputLimits(0, 100); // max output 100%
	PID.SetSampleTime(PID_SAMPLE_TIME);
	PID.SetMode(AUTOMATIC);

	setupMenu();

}

uint32_t lastRampTicks;
uint32_t lastSoakTicks;

void updateRampSetpoint(bool down = false) 
{
  if (zeroCrossTicks > lastRampTicks + ZX_TICKS_PER_UPDATE) 
  {
    double rate = (down) ? activeProfile.rampDownRate : activeProfile.rampUpRate;
    Setpoint += (rate / (float)ZX_TICKS_PER_SEC * (zeroCrossTicks - lastRampTicks)) * ((down) ? -1 : 1);
    lastRampTicks = zeroCrossTicks;
  }
}

void updateSoakSetpoint(bool down = false) 
{
  if (zeroCrossTicks > lastSoakTicks + ZX_TICKS_PER_UPDATE) 
  {
    double rate = (activeProfile.soakTempB-activeProfile.soakTempA)/(float)activeProfile.soakDuration;
    Setpoint += (rate / (float)ZX_TICKS_PER_SEC * (zeroCrossTicks - lastSoakTicks)) * ((down) ? -1 : 1);
    lastSoakTicks = zeroCrossTicks;
  }
}

// The Arduino environment loop function -- Contineously beeing called
void loop(void) 
{
	// handle encoder
	encMovement = Encoder.getValue();
	if (encMovement) 
	{
		encAbsolute += encMovement;
		if (currentState == Settings) 
		{
			MenuEngine.navigate((encMovement > 0) ? MenuEngine.getNext() : MenuEngine.getPrev());
			menuUpdateRequest = true;
		}
	}

	// handle button
	switch (Encoder.getButton()) 
	{
	case ClickEncoder::Clicked:
		
		if (currentState < UIMenuEnd) 
		{
			menuUpdateRequest = true;
			MenuEngine.invoke();
		}
		else if (currentState > UIMenuEnd)  // if button pressed during solder cycle then abort the process
		{
			currentState = CoolDown;
		}
		break;

	case ClickEncoder::DoubleClicked:
		if (currentState < UIMenuEnd) {
			if (MenuEngine.getParent() != &miExit) {
				MenuEngine.navigate(MenuEngine.getParent());
				menuUpdateRequest = true;
			}
		}
		break;
	}

	// --------------------------------------------------------------------------
	// update current menu item while in edit mode
	//
	if (currentState == Edit) {
		if (MenuEngine.currentItem != &Menu::NullItem) {
			MenuEngine.executeCallbackAction(Menu::actionDisplay);      
		}
	}

	// --------------------------------------------------------------------------
	// handle menu update
	//
	if (menuUpdateRequest) 
	{
		menuUpdateRequest = false;
		if (currentState < UIMenuEnd && !encMovement && currentState != Edit && previousState != Edit) { // clear menu on child/parent navigation
			tft.fillScreen(ST7735_WHITE);
		}  
		MenuEngine.render(renderMenuItem, menuItemsVisible);
	}

	// --------------------------------------------------------------------------
	// track state changes
	//
	if (previousState != currentState ) {
		stateChangedTicks = zeroCrossTicks;
		stateChanged = true;
		previousState = currentState;
	}


	if (zeroCrossTicks - lastUpdate >= ZX_TICKS_PER_UPDATE) 
	{
		uint32_t deltaT = zeroCrossTicks - lastUpdate;
		lastUpdate = zeroCrossTicks;
		
		temperature = readThermocouple(); 	// should be sufficient to read it every 250ms or 500ms 
											// Can't read it faster than 250ms since device conversion time is ~220ms

		// rolling average of the temp T1 and T2
		totalT1 -= readingsT1[index];       // subtract the last reading
		readingsT1[index] = temperature;
		totalT1 += readingsT1[index];       // add the reading to the total
		index = (index + 1) % NUM_TEMP_READINGS;  // next position
		averageT1 = totalT1 / (float)NUM_TEMP_READINGS;  // calculate the average temp

		// need to keep track of a few past readings in order to work out rate of rise
		for (int i = 1; i < NUM_TEMP_READINGS; i++) { // iterate over all previous entries, moving them backwards one index
			airTemp[i - 1].temp = airTemp[i].temp;
			airTemp[i - 1].ticks = airTemp[i].ticks;     
		}

		airTemp[NUM_TEMP_READINGS - 1].temp = averageT1; // update the last index with the newest average
		airTemp[NUM_TEMP_READINGS - 1].ticks = (uint16_t)deltaT;

		// calculate rate of temperature change
		uint32_t collectTicks = 0;
		for (int i = 0; i < NUM_TEMP_READINGS; i++) {
			collectTicks += airTemp[i].ticks;
		}
		float tempDiff = (airTemp[NUM_TEMP_READINGS - 1].temp - airTemp[0].temp);
		float timeDiff = collectTicks / (float)(ZX_TICKS_PER_SEC);
		
		rampRate = tempDiff / timeDiff;

		Input = airTemp[NUM_TEMP_READINGS - 1].temp; // update the variable the PID reads
		
#ifdef SERIAL_VERBOSE
		Serial.write((uint8_t)Input);
#endif

		// display update 
		if (currentState > UIMenuEnd) 
			updateProcessDisplay();  // the graph
		else 
		{
			displayThermocoupleData(1, tft.height()-16); // the current temp on the bottom of the menu
			displayBatteryVoltage( -1,-1);				 // display the input voltage following the temperatur
		}


		switch (currentState) 
		{

		case RampToSoak:
			if (stateChanged) 
			{
				lastRampTicks = zeroCrossTicks;
				stateChanged = false;
				Output = 50;
				PID.SetMode(AUTOMATIC);
				PID.SetControllerDirection(DIRECT);
				PID.SetTunings(heaterPID.Kp, heaterPID.Ki, heaterPID.Kd);
				Setpoint = Input;

			}

			updateRampSetpoint();

			if (Setpoint >= activeProfile.soakTempA - 1) 
			{
				currentState = Soak;
			}
			break;

		case Soak:
			if (stateChanged) 
			{
				lastSoakTicks = zeroCrossTicks;
				stateChanged = false;
				Setpoint = activeProfile.soakTempA;
			}

			updateSoakSetpoint();

			if (zeroCrossTicks - stateChangedTicks >= (uint32_t)activeProfile.soakDuration * ZX_TICKS_PER_SEC) {
				currentState = RampUp;
			}
			break;

		case RampUp:
			if (stateChanged) {
				stateChanged = false;
				lastRampTicks = zeroCrossTicks;
			}

			updateRampSetpoint();

			if (Setpoint >= activeProfile.peakTemp - 1) {
				Setpoint = activeProfile.peakTemp;
				currentState = Peak;
			}
			break;

		case Peak:
			if (stateChanged) {
				stateChanged = false;
				Setpoint = activeProfile.peakTemp;
			}

			if (zeroCrossTicks - stateChangedTicks >= (uint32_t)activeProfile.peakDuration * ZX_TICKS_PER_SEC) {
				currentState = RampDown;
			}
			break;

			// GS : RampDown and Cooldown rely on a fan to drive the temp down. The heater is turned off as soon as we 
			//      enter Rampdown state.  If we don't have a means to actively cool, then all this PID setting of the 
			//      cool off phases is for nothing.
		case RampDown:
			if (stateChanged) 
			{
				stateChanged = false;
				lastRampTicks = zeroCrossTicks;
				PID.SetControllerDirection(REVERSE);
				PID.SetTunings(fanPID.Kp, fanPID.Ki, fanPID.Kd);
				Setpoint = activeProfile.peakTemp - 15; // get it all going with a bit of a kick! v sluggish here otherwise, too hot too long
#ifdef PIN_BEEPER
				// Beep as a reminder that CoolDown starts (and maybe open up the oven door for fast enough cooldown)
				digitalHigh(PIN_BEEPER); // beep
				delay(50);
				digitalLow(PIN_BEEPER); // off
#endif
				
 
#ifdef WITH_SERVO       
				// TODO: implement servo operated lid
#endif   
			}

			updateRampSetpoint(true);

			if (Setpoint <= idleTemp) 
				currentState = CoolDown;
		
			break;
  
		case CoolDown:
			if (stateChanged) 
			{
				stateChanged = false;
				PID.SetControllerDirection(REVERSE);
				PID.SetTunings(fanPID.Kp, fanPID.Ki, fanPID.Kd);
				Setpoint = idleTemp;
			}

			if (Input < (idleTemp + 5)) 
			{
				currentState = Complete;
				PID.SetMode(MANUAL);
				Output = 0;
			}
			break;
			
		case Complete:
#ifdef PIN_BEEPER
			digitalHigh(PIN_BEEPER); // beep
			delay(500);
			digitalLow(PIN_BEEPER); // off
#endif	

			delay(2000);			// let the screen display for a bit before switching to start again
			menuExit(Menu::actionDisplay); // reset to start Cycle state 
			MenuEngine.navigate(&miCycleStart); 
			currentState = Settings;
			menuUpdateRequest = true;
			break;
			
		default:
				break;
		}
		
	}

// GS: what about the PID cycle time. 
// With having the statemachine and screen update happening every ZX_TICKS_PER_UPDATE time the 
// PID compute call is not executed at a fixed and repeated interval.
// This should be a problem for the derivateive term unless the PID.Compute function is savy to the fact.
	PID.Compute();

	// decides which control signal is fed to the output for this cycle
	if ( currentState == RampToSoak ||
		 currentState == Soak ||
		 currentState == RampUp||
		 currentState == Peak )		
	{
		Heater.target = Output;
		fanValue = fanAssistSpeed;
	} 
	else 
	{
		Heater.target = 0;
		fanValue = Output;
	}


	// TODO:  If FAN control is needed use a PWM pin connected to a transistor to switch 12V for a 12V fan.
} // end loop


void saveProfile(unsigned int targetProfile, bool quiet) {

  activeProfileId = targetProfile;

  if (!quiet) 
    memoryFeedbackScreen(activeProfileId, false);
  
  saveParameters(activeProfileId); // activeProfileId is modified by the menu code directly, this method is called by a menu action

  if (!quiet) 
	delay(500);

}

#define WITH_CHECKSUM 1

bool firstRun() { 

#ifndef ALWAYS_FIRST_RUN

	// if all bytes of a profile in the middle of the eeprom space are 255, we assume it's a first run
	unsigned int offset = 15 * sizeof(Profile_t);

	for (uint16_t i = offset; i < offset + sizeof(Profile_t); i++) 
	{
		if (EEPROM.read(i) != 255) 
		{
			return false;
		}
	}
#endif

	return true;
}

