#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <Arduino.h>
#include <SPI.h>
#include "config.h"
#include "globalDefs.h"

MAX6675 thermocouple(PIN_TC_CLK, PIN_TC_CS, PIN_TC_DO);


double readThermocouple() 
{
  
	  // TODO: this should throw  an exception, but since this part is C we just lock the execution with a while(1) loop for now.
	  // there is no save recovering from a bus contention --  The SPI module should really handle this with an aquire/release mechanism
		
	  if ( digitalState(PIN_LCD_CS) == 0 )
	  {
		  Serial.println("ERROR: -- SPI BUS contention -- LCD_CS was low !! Fix the code so that this cant happen : stopping ");
		  while(1)
		  ;
	  }

	  double reading = thermocouple.readCelsius();
	   
	  // GS: When is it getting a NAN ? What would cause that.  Calling it to fast  maybe? 
	  
	  if (reading == NAN) 
	  {
		  Serial.println("ERROR: TC-Reading failed : stopping ");
		  while(1)
		  ;
	  }
	  else 
	  {
		tcStat = 0;
	  }

	return reading;

 }


#endif
