

// https://pmdway.com/products/0-96-80-x-160-full-color-lcd-module
#include <UTFT.h>
#include <EEPROM.h>
#include <PID_v1.h>


// First we include the libraries
#include <OneWire.h>
#include <DallasTemperature.h>
/********************************************************************/
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2
/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);


// Define buttons
//
#define BUTTON_A  3
#define UNITS 4


#define HEATER_DRIVE 6
#define SETPOINT 32.0




// Library only supports software SPI at this time
//NOTE: support  DUE , MEGA , UNO
//SDI=11  SCL=13  /CS =10  /RST=8  D/C=9
UTFT myGLCD(ST7735S_4L_80160, 11, 13, 10, 8, 9); //LCD:  4Line  serial interface      SDI  SCL  /CS  /RST  D/C    NOTE:Only support  DUE   MEGA  UNO
extern uint8_t SmallFont[];
extern uint8_t BigFont[];




int tick;
double emissivity = 33;
const double E_MIN = 16;
const double E_MAX = 40;
const double E_STEP = 0.1;
double ambient, ambientC;
double temper, temperC;
char s[32];
double dt;
char *units;


#define TEMP_TOP 63
#define GRAPH_TOP 16
#define GRAPH_LEFT 0
#define GRAPH_X 160
#define GRAPH_BOTTOM (TEMP_TOP-2)
int _graph[GRAPH_X];


#define GRAPH_BACK_COLOR VGA_WHITE
#define GRAPH_POINT_COLOR VGA_RED
void graph(uint16_t p, bool init = false)
{
  int i;
  uint16_t miny = 0xffff;;
  uint16_t maxy = 0;
  int y1, y2;

  if (init) {
    for (i = 0; i < GRAPH_X; i++) {
      _graph[i] = p;
    }
    return;
  }

  myGLCD.setColor(GRAPH_BACK_COLOR);
  myGLCD.fillRoundRect(0, GRAPH_TOP, GRAPH_X, GRAPH_BOTTOM);
  myGLCD.setColor(GRAPH_POINT_COLOR);

  // skootch the old data and add the new point
  //
  for (i = 0; i < (GRAPH_X - 1); i++) {
    _graph[i] = _graph[i + 1];
  }
  _graph[i] = p;

  // get the min/max
  for (i = 0; i < GRAPH_X; i++) {
    if (_graph[i] < miny) {
      miny = _graph[i];
    }
    if (_graph[i] > maxy) {
      maxy = _graph[i];
    }
  }
  if ((maxy - miny) < 10) {
    if (miny > 10) {
      miny -= 10;
    }
    else {
      maxy += 10;
    }
  }


  for (i = 0; i < (GRAPH_X - 1); i++) {
    y1 = map(_graph[i], miny, maxy, GRAPH_BOTTOM, GRAPH_TOP);
    y2 = map(_graph[i + 1], miny, maxy, GRAPH_BOTTOM, GRAPH_TOP);

    //  sprintf(s, "miny=%d maxy=%d y1=%d y2=%d\n", miny, maxy, y1, y2);
    //  Serial.print(s);
    myGLCD.drawLine(i, y1, i + 1, y2);
  }

}




void setup()
{
  uint16_t ereg;
  double ee;
  unsigned int setup = 0;
  float de;

  Serial.begin(115200);

  pinMode(HEATER_DRIVE, OUTPUT);

  // Setup switches and blinky
  //
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(UNITS, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  sensors.begin();


  // Setup the LCD
  //
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);


  // SETUP Mode
  // Hold the button down on powerup to enter SETUP mode.
  // Units switch determines increment or decrement on each button push.
  // 20 second idle time to get out of setup mode
  //
#define SETUP_DONE 2000
  if (!digitalRead(BUTTON_A)) {

    // Clear the screen and draw the frame
    myGLCD.clrScr();
    myGLCD.setColor(VGA_BLUE);
    myGLCD.fillRect(0, 0, 159, 13);

    myGLCD.setColor(VGA_YELLOW);
    myGLCD.setBackColor(VGA_BLUE);
    myGLCD.print("SETUP Emissitivity", CENTER, 1);
    myGLCD.setFont(SmallFont);
    myGLCD.setColor(VGA_WHITE);
    myGLCD.setBackColor(VGA_BLACK);

    while (setup < SETUP_DONE) {

      de = emissivity + 0.005;
      sprintf(s, "e=%d.%02d ", int(de), int(de * 100) % 100);
      myGLCD.print(s, 18, 26);
      Serial.println(s);

      // wait for button up
      //
      while (!digitalRead(BUTTON_A)) {
        delay(10);
      }
      delay(10); // debounce

      // wait for the button to go down or timeout of out setup mode
      //
      setup = 0;
      while (digitalRead(BUTTON_A)) {
        if (++setup > SETUP_DONE) {
          break;
        }
        delay(10);
      }
      delay(10); // debounce

      // if the button is down then increment/decrement
      //
      if (!digitalRead(BUTTON_A)) {
        if (digitalRead(UNITS)) {
          emissivity += E_STEP;
          if (emissivity > E_MAX) {
            emissivity = E_MAX;
          }
        }
        else {
          emissivity -= E_STEP;
          if (emissivity < E_MIN) {
            emissivity = E_MIN;
          }
        }
      }
    }
    // WRITE E HERE
    //
    // eeprom;
  }


  // setup the screen for action
  //
  // Clear the screen and draw the frame
  //
  myGLCD.clrScr();
  myGLCD.setColor(VGA_BLUE);
  myGLCD.fillRoundRect(0, 0, 159, 13);

  myGLCD.setColor(VGA_YELLOW);
  myGLCD.setBackColor(VGA_BLUE);
  myGLCD.print("MakeIt Labs", CENTER, 1);

  temperC = sensors.getTempCByIndex(0);
  graph(temperC * 100, true);
  delay(2000);
}


double CtoF(double c)
{
  return (c * 9) / 5 + 32;
}


  

double mapd(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void loop()
{
  double error;
  double pwm;

  sensors.requestTemperatures();
  temperC = sensors.getTempCByIndex(0);

  error = SETPOINT - temperC;
  if (error < 0) {
    error = 0;
    pwm = 0;
  }
  else {
    pwm = mapd(temperC, SETPOINT-2.0, SETPOINT,   255, 0);
    if (pwm > 255) {
      pwm = 255;
    }
    else if (pwm < 0) {
      pwm = 0;
    }
  }
  analogWrite(HEATER_DRIVE, (int) pwm);


  if (!digitalRead(UNITS)) {
    temper = CtoF(temperC);
    units = "F";
  }
  else {
    temper = temperC;
    units = "C";
  }


  myGLCD.setFont(SmallFont);
  myGLCD.setColor(VGA_WHITE);
  myGLCD.setBackColor(VGA_BLACK);


  myGLCD.setFont(BigFont);
  if (temperC > SETPOINT) {
    myGLCD.setColor(VGA_RED);
  }
  else if (temperC > (SETPOINT-2.0)) {
    myGLCD.setColor(VGA_YELLOW);
  }
  else {
    myGLCD.setColor(VGA_GREEN);
  }

  dt = temper + 0.005;
  sprintf(s, " %d.%02d %s %d", int(dt), int(dt * 100) % 100, units, (int) pwm);
  myGLCD.print(s, 16, TEMP_TOP);
  Serial.println(s);


  if (tick & 4) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }


  graph(temperC * 100);


  delay(500);
}
