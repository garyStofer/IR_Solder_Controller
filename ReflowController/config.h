#ifndef CONFIG_H
#define CONFIG_H
#define VERSION "GS.1.0"

//#define ALWAYS_FIRST_RUN
//#define WITH_SERVO // Enables Lid opening Servo (not yet implemented)
//#define SERIAL_VERBOSE

#define PIN_BEEPER 5  // aka PD5, digital pin5 , 
#define PIN_LED 17	  // aka PC3, digital pin 17

#define VBUS_ADC 7			// ADC7
#define VBUS_ADC_BW  (5.0*(14+6.8)/(1024*6.8))		//adc bit weight for voltage divider 14.0K and 6.8k to gnd.


#define PIN_LCD_CS  10	//PB2
#define PIN_LCD_DC  8	//PB0
#define PIN_LCD_RST 7	//PD7
// the commented out pins are assumed by the fact that the SPI inteface is used -- only here for clarity
//#define PIN_LCD_SCL	13 // PB5,SPI_CLK
//#define PIN_LCD_SDA  11 // PB3,SPI_MOSI
#define LCD_ROTATION  1 // 0/2-> portrait, 1/3-> landscape

#define PIN_TC_CS   4	// D4
#define PIN_TC_DO   12	// PB4, SPI_MISO, D12, PB4
#define PIN_TC_CLK  13  // PB5,SPI_CLK

#define PIN_HEATER  A1 	// PC1,D15

// --- encoder
#define PIN_ENC_A          2 	//PD2,INT0
#define PIN_ENC_B          A0	//PC0,D14
#define PIN_ENC_BTN        A2  	//PC2,D16
#define ENC_STEPS_PER_NOTCH  4
const boolean IS_ENC_ACTIVE       = false; // encoder module actively fed with VCC ( seems to works bad if set to true )


#define  PIN_ZX  3 	//PD3,INT1	 pin for zero crossing detector  **** Must be D3 ****, INT1  ISR has INT1 specific code
								 // Pin must have a 3.3Ko pull-up to VCC for the optocouple to work properlyrk
#define INT_ZX  digitalPinToInterrupt(PIN_ZX) // interrupt for zero crossing detector

#define NUM_TEMP_READINGS   5
#define TC_ERROR_TOLERANCE  5 // allow for n consecutive errors due to noisy power supply before bailing out
#define TEMP_COMPENSATION   1.0 // correction factor to match temperature measured with other device



// see: https://www.compuphase.com/electronics/reflowsolderprofiles.htm  
#define DEFAULT_SOAK_TEPM_A       110
#define DEFAULT_SOAK_TEPM_B       160 
#define DEFAULT_SOAK_DURATION     180 
#define DEFAULT_PEAK_TEPM         240
#define DEFAULT_PEAK_DURATION     35
#define DEFAULT_RAMP_UP_RATE      1.2 // degrees / second (keep it about 1/2 of maximum to prevent PID overshooting)
#define DEFAULT_RAMP_DOWN_RATE    2.0 // degrees / second
#define FACTORY_FAN_ASSIST_SPEED  33


/*
Kp: Determines how aggressively the PID reacts to the current amount of error (Proportional) (double >=0)
Ki: Determines how aggressively the PID reacts to error over time (Integral) (double>=0)
Kd: Determines how aggressively the PID reacts to the change in error (Derivative) (double>=0)

Experimental method to tune PID:

> Set all gains to 0.
> Increase Kd until the system oscillates.
> Reduce Kd by a factor of 2-4.
> Set Kp to about 1% of Kd.
> Increase Kp until oscillations start.
> Decrease Kp by a factor of 2-4.
> Set Ki to about 1% of Kp.
> Increase Ki until oscillations start.
> Decrease Ki by a factor of 2-4.

*/
#define PID_SAMPLE_TIME 200
#define FACTORY_KP  1.75 // 1.75 //4.0 
#define FACTORY_KI 0.03 // 0.03 // 0.05 
#define FACTORY_KD 3.0 //3.0//2.0 





#endif // CONFIG_H
