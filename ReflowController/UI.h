#ifndef UI_H
#define UI_H


#include <PDQ_GFX.h>        // PDQ: Core graphics library
#include "PDQ_ST7735_config.h"      // PDQ: ST7735 pins and other setup for this sketch
#include <PDQ_ST7735.h>     // PDQ: Hardware-specific driver library
                                         
#include <Menu.h>
#include <ClickEncoder.h>
#include "globalDefs.h"
#include "eepromHelpers.h"
#include "config.h"

#define MENU_TEXT_XPOS 5
#define MENU_BAR_XPOS 3


// ----------------------------------------------------------------------------
// Hardware Configuration 

// 1.44" or 1.8" TFT via SPI -> breadboard

#define TEMPERATURE_WINDOW 1.2 // n times the profile's maximum temperature


// ----------------------------------------------------------------------------


PDQ_ST7735 tft;     // PDQ: create LCD object (using pins in "PDQ_ST7735_config.h")


// ------------ menu

Menu::Engine MenuEngine;

const uint8_t menuItemHeight = 12;
const uint8_t menuItemsVisible = 8;
bool menuUpdateRequest = true;
bool initialProcessDisplay = false;

// track menu item state to improve render preformance
typedef struct {
  const Menu::Item_t *mi;
  uint8_t pos;
  bool current;
} LastItemState_t;

LastItemState_t currentlyRenderedItems[menuItemsVisible];

// ------------ encoder
ClickEncoder Encoder(PIN_ENC_A, PIN_ENC_B, PIN_ENC_BTN, ENC_STEPS_PER_NOTCH, IS_ENC_ACTIVE);
int16_t encMovement;
int16_t encAbsolute;
int16_t encLastAbsolute = -1;

// ------------ 

float pxPerSec;
float pxPerC;
uint16_t xOffset; // used for wraparound on x axis


void setupTFT() {

  
  FastPin<ST7735_RST_PIN>::setOutput();
  FastPin<ST7735_RST_PIN>::hi();
  FastPin<ST7735_RST_PIN>::lo();
  delay(1);
  FastPin<ST7735_RST_PIN>::hi();
  
   tft.begin();
  
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setRotation(LCD_ROTATION);
  tft.fillScreen(ST7735_WHITE);
  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
}
void displaySplash() 
{
  tft.fillScreen(ST7735_WHITE);
  tft.setTextColor(ST7735_BLUE);
  tft.setTextSize(2);	// 12pt font
  tft.setCursor(2, 30);
  tft.print("Solder");
  tft.setCursor(20, 48);
  tft.print("Reflow");
  tft.setCursor(tft.width()-120, 66);
  tft.print("Controller");
  tft.setTextSize(1);
    tft.setTextColor(ST7735_BLACK);
  tft.setCursor(10, 110);
  tft.print("ver:"); tft.print(VERSION);
  tft.print(", ");
  tft.print(__DATE__);
  delay(1000);
}


void displayError(int error) {

  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_RED);
  tft.fillScreen(ST7735_RED);

  tft.setCursor(10, 10);
  
  if (error < 9) {
    tft.println("Thermocouple Error");
    tft.setCursor(10, 30);
    switch (error) {
      case 0b001:
        tft.println("Open Circuit");
        break;
      case 0b010:
        tft.println("GND Short");
        break;
      case 0b100:
        tft.println("VCC Short");
        break;
    }
    tft.setCursor(10, 60);
    tft.println("Power off,");
    tft.setCursor(10, 75);
    tft.println("check connections");
  }
  else {
    tft.println("Temperature"); 
    tft.setCursor(10, 30);
    tft.println("following error");
    tft.setCursor(10, 45);
    tft.print("during ");
    tft.println((error == 10) ? "heating" : "cooling");
  }
  
#ifdef PIN_BEEPER
	digitalHigh(PIN_BEEPER); // beep
	delay(2000);
	digitalLow(PIN_BEEPER); // off
#endif
  while (1) { //  stop
    ;
  }
}

// ----------------------------------------------------------------------------

void alignRightPrefix(uint16_t v) {
  if (v < 1e2) tft.print(' '); 
  if (v < 1e1) tft.print(' ');
}

// ----------------------------------------------------------------------------

void displayThermocoupleData(uint8_t xpos, uint8_t ypos) 
{
  tft.setCursor(xpos, ypos);
  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  // temperature
  tft.setTextSize(2);
  alignRightPrefix((int)temperature);
  switch (tcStat) {
    case 0:
      tft.print((uint8_t)temperature);
      tft.print("\367C");
      break;
    case 1:
      tft.print("---");
      break;
  }

}

void displayBatteryVoltage(int8_t xpos, int8_t ypos) 
{
	if (xpos >= 0 )
		tft.setCursor(xpos, ypos);
	
	short adc_val = analogRead(VBUS_ADC);	// read the battery voltage 
	float Vbus_Volt = adc_val * VBUS_ADC_BW;

	if (Vbus_Volt < 6.7 )
		tft.setTextColor(ST7735_RED, ST7735_WHITE);
	else
		tft.setTextColor(ST7735_GREEN, ST7735_WHITE);
	tft.print(" ");
    tft.print(Vbus_Volt);
    tft.print(" V");
}
// ----------------------------------------------------------------------------


void clearLastMenuItemRenderState() {
  // memset(&currentlyRenderedItems, 0xff, sizeof(LastItemState_t) * menuItemsVisible);
  for (uint8_t i = 0; i < menuItemsVisible; i++) {
    currentlyRenderedItems[i].mi = NULL;
    currentlyRenderedItems[i].pos = 0xff;
    currentlyRenderedItems[i].current = false;
  }
}

// ----------------------------------------------------------------------------

extern const Menu::Item_t miRampUpRate, miRampDnRate, miSoakTime, 
                          miSoakTempA, miSoakTempB, miPeakTime, miPeakTemp,
                          miLoadProfile, miSaveProfile,
                          miPidSettingP, miPidSettingI, miPidSettingD,
                          miFanSettings;


// ----------------------------------------------------------------------------

bool menuExit(const Menu::Action_t a) {
  clearLastMenuItemRenderState();
  MenuEngine.lastInvokedItem = &Menu::NullItem;
  menuUpdateRequest = false;
  return false;
}


// ----------------------------------------------------------------------------

bool menuDummy(const Menu::Action_t a) {
  return true;
}

// ----------------------------------------------------------------------------

void printDouble(double val, uint8_t precision = 1) {
  ftoa(buf, val, precision);
  tft.print(buf);
}


// ----------------------------------------------------------------------------

void getItemValuePointer(const Menu::Item_t *mi, double **d, int16_t **i) {
  if (mi == &miRampUpRate)  *d = &activeProfile.rampUpRate;
  if (mi == &miRampDnRate)  *d = &activeProfile.rampDownRate;
  if (mi == &miSoakTime)    *i = &activeProfile.soakDuration;
  if (mi == &miSoakTempA)    *i = &activeProfile.soakTempA;
  if (mi == &miSoakTempB)    *i = &activeProfile.soakTempB;
  if (mi == &miPeakTime)    *i = &activeProfile.peakDuration;
  if (mi == &miPeakTemp)    *i = &activeProfile.peakTemp;
  if (mi == &miPidSettingP) *d = &heaterPID.Kp;
  if (mi == &miPidSettingI) *d = &heaterPID.Ki;
  if (mi == &miPidSettingD) *d = &heaterPID.Kd; 
  if (mi == &miFanSettings) *i = &fanAssistSpeed;
}

// ----------------------------------------------------------------------------

bool isPidSetting(const Menu::Item_t *mi) {
  return mi == &miPidSettingP || mi == &miPidSettingI || mi == &miPidSettingD;
}

bool isRampSetting(const Menu::Item_t *mi) {
  return mi == &miRampUpRate || mi == &miRampDnRate;
}

// ----------------------------------------------------------------------------

bool getItemValueLabel(const Menu::Item_t *mi, char *label) {
  int16_t *iValue = NULL;
  double  *dValue = NULL;
  char *p;
  
  getItemValuePointer(mi, &dValue, &iValue);

  if (isRampSetting(mi) || isPidSetting(mi)) {
    p = label;
    ftoa(p, *dValue, (isPidSetting(mi)) ? 2 : 1); // need greater precision with pid values
    p = label;
    
    if (isRampSetting(mi)) {
      while(*p != '\0') p++;
      *p++ = 0xf7; *p++ = 'C'; *p++ = '/'; *p++ = 's';
      *p = '\0';
    }
  }
  else {
    if (mi == &miPeakTemp || mi == &miSoakTempA || mi == &miSoakTempB ) {
      itostr(label, *iValue, "\367C");
    }
    if (mi == &miPeakTime || mi == &miSoakTime) {
      itostr(label, *iValue, "s");
    }
    if (mi == &miFanSettings) {
      itostr(label, *iValue, "%");
    }
  }

  return dValue || iValue;
}

// ----------------------------------------------------------------------------

bool menu_editNumericalValue(const Menu::Action_t action) { 
  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    tft.setTextSize(1);
    if (initial) {
      tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
      tft.setCursor(MENU_TEXT_XPOS, 80);
      tft.print("Edit & click to save.");
      Encoder.setAccelerationEnabled(true);
    }

    for (uint8_t i = 0; i < menuItemsVisible; i++) {
      if (currentlyRenderedItems[i].mi == MenuEngine.currentItem) {
        uint8_t y = currentlyRenderedItems[i].pos * menuItemHeight + 2;

        if (initial) {
          tft.fillRect(59+MENU_TEXT_XPOS, y - 1, 60, menuItemHeight - 2, ST7735_RED);
        }

        tft.setCursor(60+MENU_TEXT_XPOS, y);
        break;
      }
    }

    tft.setTextColor(ST7735_WHITE, ST7735_RED);

    int16_t *iValue = NULL;
    double  *dValue = NULL;
    getItemValuePointer(MenuEngine.currentItem, &dValue, &iValue);

    if (isRampSetting(MenuEngine.currentItem) || isPidSetting(MenuEngine.currentItem)) {
      double tmp;
      double factor = (isPidSetting(MenuEngine.currentItem)) ? 100 : 10;
      
      if (initial) {
        tmp = *dValue;
        tmp *= factor;
        encAbsolute = (int16_t)tmp;
      }
      else {
        tmp = encAbsolute;
        tmp /= factor;
        *dValue = tmp;
      }      
    }
    else {
      if (initial) encAbsolute = *iValue;
      else *iValue = encAbsolute;
    }

    getItemValueLabel(MenuEngine.currentItem, buf);
    tft.print(buf);
    tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
  }

  if (action == Menu::actionParent || action == Menu::actionTrigger) {
    clearLastMenuItemRenderState();
    menuUpdateRequest = true;
    MenuEngine.lastInvokedItem = &Menu::NullItem;


    if (currentState == Edit) { // leave edit mode, return to menu
      if (isPidSetting(MenuEngine.currentItem)) {
        savePID();
      }
      else if (MenuEngine.currentItem == &miFanSettings) {
        saveFanSpeed();
      }
      // don't autosave profile, so that one can do "save as" without overwriting the current profile

      currentState = Settings;
      Encoder.setAccelerationEnabled(false);
      return false;
    }

    return true;
  }
}


// ----------------------------------------------------------------------------

void factoryReset() {

  makeDefaultProfile();
  
  tft.fillScreen(ST7735_BLUE);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(10, 50);
  tft.print("Resetting...");

  // then save the same profile settings into all slots
  for (uint8_t i = 0; i < maxProfiles; i++) {
    saveParameters(i);
  }

  fanAssistSpeed = FACTORY_FAN_ASSIST_SPEED;
  saveFanSpeed();

  heaterPID.Kp =  FACTORY_KP;// 0.60; 
  heaterPID.Ki =  FACTORY_KI; //0.01;
  heaterPID.Kd = FACTORY_KD; //19.70;
  savePID();

  activeProfileId = 0;
  saveLastUsedProfile();

  delay(500);

}

// ----------------------------------------------------------------------------

bool menu_factoryReset(const Menu::Action_t action) {

  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    if (initial) { // TODO: add eyecandy: colors or icons
      tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
      tft.setTextSize(1);
      tft.setCursor(10, tft.height()-38);
      tft.print("Click to confirm");
      tft.setCursor(10, tft.height()-28);
      tft.print("Doubleclick to exit");
    }
  }

  if (action == Menu::actionTrigger) { // do it
    factoryReset();
    tft.fillScreen(ST7735_WHITE);
    MenuEngine.navigate(MenuEngine.getParent());
    return false;
  }

  if (action == Menu::actionParent) {
    if (currentState == Edit) { // leave edit mode only, returning to menu
      currentState = Settings;
      clearLastMenuItemRenderState();
      return false;
    }
  }

}


void memoryFeedbackScreen(uint8_t profileId, bool loading) {
  tft.fillScreen(ST7735_GREEN);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_BLACK);
  tft.setCursor(10, 50);
  tft.print(loading ? "Loading" : "Saving");
  tft.print(" profile ");
  tft.print(profileId);  
}

// ----------------------------------------------------------------------------

void saveProfile(unsigned int targetProfile, bool quiet = false);


void loadProfile(unsigned int targetProfile) {
  memoryFeedbackScreen(activeProfileId, true);
  bool ok = loadParameters(targetProfile);

#if 0
  if (!ok) {
    lcd.setCursor(0, 2);
    lcd.print("Checksum error!");
    lcd.setCursor(0, 3);
    lcd.print("Review profile.");
    delay(2500);
  }
#endif

  // save in any way, as we have no undo
  activeProfileId = targetProfile;
  saveLastUsedProfile();

  delay(500);
}


// ----------------------------------------------------------------------------

bool menu_saveLoadProfile(const Menu::Action_t action) {

  bool isLoad = MenuEngine.currentItem == &miLoadProfile;

  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
    tft.setTextSize(1);

    if (initial) {
      encAbsolute = activeProfileId;      
      tft.setCursor(10, 90);
      tft.print("Doubleclick to exit");
    }

    if (encAbsolute > maxProfiles) encAbsolute = maxProfiles;
    if (encAbsolute <  0) encAbsolute =  0;

    tft.setCursor(10, 80);
    tft.print("Click to ");
    tft.print((isLoad) ? "load " : "save ");
    tft.setTextColor(ST7735_WHITE, ST7735_RED);
    tft.print(encAbsolute);
  }

  if (action == Menu::actionTrigger) {
    (isLoad) ? loadProfile(encAbsolute) : saveProfile(encAbsolute);
    tft.fillScreen(ST7735_WHITE);
    MenuEngine.navigate(MenuEngine.getParent());
    return false;
  }

  if (action == Menu::actionParent) {    
    if (currentState == Edit) { // leave edit mode only, returning to menu
      currentState = Settings;
      clearLastMenuItemRenderState();
      return false;
    }
  }

}

// ----------------------------------------------------------------------------


bool menu_cycleStart(const Menu::Action_t action) {
  if (action == Menu::actionDisplay) {
    startCycleZeroCrossTicks = zeroCrossTicks;
    menuExit(action);

    currentState = RampToSoak;

    initialProcessDisplay = false;
    menuUpdateRequest = false;
  }

  return true;
}

// ----------------------------------------------------------------------------

void renderMenuItem(const Menu::Item_t *mi, uint8_t pos) 
{
  //ScopedTimer tm("  render menuitem");
  bool isCurrent = MenuEngine.currentItem == mi;
  uint8_t y = pos * menuItemHeight + 2;

  if (currentlyRenderedItems[pos].mi == mi 
      && currentlyRenderedItems[pos].pos == pos 
      && currentlyRenderedItems[pos].current == isCurrent) 
  {
    return; // don't render the same item in the same state twice
  }

  tft.setCursor(MENU_TEXT_XPOS, y);
  tft.setTextSize(1);

  // menu cursor bar
  tft.fillRect(MENU_BAR_XPOS, y - 2, tft.width() - 16, menuItemHeight, isCurrent ? ST7735_BLUE : ST7735_WHITE);
  if (isCurrent) tft.setTextColor(ST7735_WHITE, ST7735_BLUE);
  else tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  tft.print(MenuEngine.getLabel(mi));

  // show values if in-place editable items
  if (getItemValueLabel(mi, buf)) {
    tft.print(' '); tft.print(buf); tft.print("   ");
  }

  // mark items that have children
  if (MenuEngine.getChild(mi) != &Menu::NullItem) {
    tft.print(" \x10   "); // 0x10 -> filled right arrow
  }

  currentlyRenderedItems[pos].mi = mi;
  currentlyRenderedItems[pos].pos = pos;
  currentlyRenderedItems[pos].current = isCurrent;
}

// ----------------------------------------------------------------------------
// Name, Label, Next, Previous, Parent, Child, Callback
// MenuItem(...) is #defined in menu.h
	MenuItem(miExit, "", Menu::NullItem, Menu::NullItem, Menu::NullItem, miCycleStart, menuExit);
	MenuItem(miCycleStart,  "Start Cycle",  miEditProfile, Menu::NullItem, miExit, Menu::NullItem, menu_cycleStart);
	MenuItem(miEditProfile, "Edit Profile", miLoadProfile, miCycleStart,   miExit, miRampUpRate, menuDummy);
	MenuItem(miRampUpRate, "Ramp up  ",   miSoakTempA,      Menu::NullItem, miEditProfile, Menu::NullItem, menu_editNumericalValue);
	MenuItem(miSoakTempA,   "Soak temp A", miSoakTempB,      miRampUpRate,   miEditProfile, Menu::NullItem, menu_editNumericalValue);
	MenuItem(miSoakTempB,   "Soak temp B", miSoakTime,      miSoakTempA,   miEditProfile, Menu::NullItem, menu_editNumericalValue);
	MenuItem(miSoakTime,   "Soak time", miPeakTemp,      miSoakTempB,     miEditProfile, Menu::NullItem, menu_editNumericalValue);
	MenuItem(miPeakTemp,   "Peak temp", miPeakTime,      miSoakTime,     miEditProfile, Menu::NullItem, menu_editNumericalValue);
	MenuItem(miPeakTime,   "Peak time", miRampDnRate,    miPeakTemp,     miEditProfile, Menu::NullItem, menu_editNumericalValue);
	MenuItem(miRampDnRate, "Ramp down", Menu::NullItem,  miPeakTime,     miEditProfile, Menu::NullItem, menu_editNumericalValue);
	MenuItem(miLoadProfile,  "Load Profile",  miSaveProfile,  miEditProfile, miExit, Menu::NullItem, menu_saveLoadProfile);
	MenuItem(miSaveProfile,  "Save Profile",  miFanSettings,  miLoadProfile, miExit, Menu::NullItem, menu_saveLoadProfile);
	MenuItem(miFanSettings,  "Fan Speed",  miPidSettings,  miSaveProfile, miExit, Menu::NullItem, menu_editNumericalValue);
	MenuItem(miPidSettings,  "PID Settings",  miFactoryReset, miFanSettings, miExit, miPidSettingP,  menuDummy);
	MenuItem(miPidSettingP,  "Heater Kp",  miPidSettingI, Menu::NullItem, miPidSettings, Menu::NullItem, menu_editNumericalValue);
	MenuItem(miPidSettingI,  "Heater Ki",  miPidSettingD, miPidSettingP,  miPidSettings, Menu::NullItem, menu_editNumericalValue);
	MenuItem(miPidSettingD,  "Heater Kd",  Menu::NullItem, miPidSettingI, miPidSettings, Menu::NullItem, menu_editNumericalValue);
	MenuItem(miFactoryReset, "Factory Reset", Menu::NullItem, miPidSettings, miExit, Menu::NullItem, menu_factoryReset);

// ----------------------------------------------------------------------------
void   setupMenu() 
{
	menuExit(Menu::actionDisplay); // reset to initial state
	MenuEngine.navigate(&miCycleStart);
	currentState = Settings;
	menuUpdateRequest = true; 
}	
	
void drawInitialProcessDisplay()
{
    const uint8_t h =  tft.height()-42;
  const uint8_t w = tft.width()-24;
  const uint8_t yOffset =  30; // space not available for graph  
    double tmp;
 initialProcessDisplay = true;

    tft.fillScreen(ST7735_WHITE);
    tft.fillRect(0, 0, tft.width(), menuItemHeight, ST7735_BLUE);
    tft.setCursor(1, 2);

    tft.print("Profile ");
    tft.print(activeProfileId);


    tmp = h / (activeProfile.peakTemp * TEMPERATURE_WINDOW) * 100.0;
    pxPerC = tmp;
    
    double estimatedTotalTime = 0;//60 * 12;
    // estimate total run time for current profile
    estimatedTotalTime = activeProfile.soakDuration + activeProfile.peakDuration;
    estimatedTotalTime += (activeProfile.peakTemp - temperature)/(float)activeProfile.rampUpRate;
    estimatedTotalTime += (activeProfile.peakTemp - temperature)/(float)activeProfile.rampDownRate;
    estimatedTotalTime *= 1.1; // add some spare
    
    tmp = w / estimatedTotalTime ; 
    pxPerSec = (float)tmp;
   
#ifdef SERIAL_VERBOSE
 Serial.print("estimatedTotalTime: ");
    Serial.println(estimatedTotalTime);
    Serial.print("pxPerSec: ");
    Serial.println(pxPerSec);
    Serial.print("Calc pxPerC/S: ");
    Serial.println(pxPerC);
    Serial.print("/");
    Serial.println(pxPerSec);
#endif   
    // 50°C grid
    int16_t t = (uint16_t)(activeProfile.peakTemp * TEMPERATURE_WINDOW);
    tft.setTextColor(tft.Color565(0xa0, 0xa0, 0xa0));
    tft.setTextSize(1);
    for (uint16_t tg = 0; tg < t; tg += 50) {
      uint16_t l = h - (tg * pxPerC / 100) + yOffset;
      tft.drawFastHLine(0, l, tft.width(), tft.Color565(0xe0, 0xe0, 0xe0));
      tft.setCursor(tft.width()-24, l-7);
      alignRightPrefix((int)tg); 
      tft.print((int)tg);
      tft.print("\367");
    }
}
// ----------------------------------------------------------------------------
void updateProcessDisplay() {
  const uint8_t h =  tft.height()-42;
  const uint8_t w = tft.width()-24;
  const uint8_t yOffset =  30; // space not available for graph  

  uint16_t dx, dy;
  uint8_t y = 2;
  double tmp;

  // header & initial view
  tft.setTextColor(ST7735_WHITE, ST7735_BLUE);
  tft.setTextSize(1);

  if (!initialProcessDisplay) {
    drawInitialProcessDisplay();
  }

  // elapsed time
  uint16_t elapsed = (zeroCrossTicks - startCycleZeroCrossTicks) / (float)(ZX_TICKS_PER_SEC);
  tft.setCursor(tft.width()-35, y);
  alignRightPrefix(elapsed); 
  tft.print(elapsed);
  tft.print("s");

  y += menuItemHeight ;
  displayThermocoupleData(0, y+2);
  tft.setTextSize(1);
 	
  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
  tft.setCursor(tft.width()-45, y);
  tft.print("SP:"); 
  tft.print( (int) Setpoint);
  tft.print("\367  ");
 
  // current state
  
  tft.setCursor(65, y+8);
  tft.setTextColor(ST7735_BLACK, ST7735_GREEN);
  switch (currentState) 
  {
	  case RampToSoak:	tft.print("RampToSoak"); break;  
	  case Soak: 		tft.print("   Soak   "); break;
	  case RampUp:		tft.print("  Ramp Up "); break;
	  case Peak:		tft.print("   Peak   "); break;
	  case RampDown:	tft.print("Ramp Down "); break;
	  case CoolDown:	tft.print(" Cool Off "); break;
	  case Complete:	tft.print("   Done   "); break;
     default: tft.print((uint8_t)currentState); break;
  }

  // draw temperature curves

  if (xOffset >= elapsed) {
    xOffset = 0;
  }

  do { // x with wrap around
    
    dx = (uint16_t)((elapsed - xOffset) * pxPerSec);
    if (dx > w) {
      xOffset = elapsed;
    }
  } while(dx > w);

  // temperature setpoint
  dy = h - ((uint16_t)Setpoint * pxPerC / 100) + yOffset;
  tft.drawPixel(dx, dy, ST7735_BLUE);

  // actual temperature
  dy = h - ((uint16_t)temperature * pxPerC / 100) + yOffset;
  tft.drawPixel(dx, dy, ST7735_RED);

  // bottom line
  y = 119;

  // set values
  tft.setTextColor(ST7735_BLACK, ST7735_GREEN);
  tft.setCursor(1, y);
  tft.print("\xef");
  alignRightPrefix((int)Heater.target); 
  tft.print((int)Heater.target);
  tft.print('%');

  tft.print(" \x2a");
  alignRightPrefix((int)fanValue); 
  tft.print((int)fanValue);
  tft.print('%');

  tft.print(" \x12 "); // alternative: \x7f
  printDouble(rampRate);
  tft.print("\367C/s    ");
  
}
// ----------------------------------------------------------------------------



#endif // UI_H
