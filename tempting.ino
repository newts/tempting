

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


double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID heaterPID(&Input, &Output, &Setpoint, 250, 2, 1.2, DIRECT);



// Define buttons
//
#define BUTTON_A  3
#define UNITS 4


#define HEATER_DRIVE 6
#define SETPOINT 35.0

const int graph_setpoint = SETPOINT * 100;




// Library only supports software SPI at this time
//NOTE: support  DUE , MEGA , UNO
//SDI=11  SCL=13  /CS =10  /RST=8  D/C=9
UTFT myGLCD(ST7735S_4L_80160, 11, 13, 10, 8, 9); //LCD:  4Line  serial interface      SDI  SCL  /CS  /RST  D/C    NOTE:Only support  DUE   MEGA  UNO
extern uint8_t SmallFont[];
extern uint8_t BigFont[];




int tick;

double ambient, ambientC;
double temper, temperC, error;
char s[32];
double dt;
char *units;


#define TEMP_TOP 63
#define GRAPH_TOP 16
#define GRAPH_LEFT 0
#define GRAPH_X 160
#define GRAPH_BOTTOM (TEMP_TOP-2)
int _graph[GRAPH_X];


void SetupTimer2() {
  noInterrupts();
  // Clear registers
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  // 100.16025641025641 Hz (16000000/((155+1)*1024))
  OCR2A = 155;
  // CTC
  TCCR2A |= (1 << WGM21);
  // Prescaler 1024
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
  // Output Compare Match A Interrupt Enable
  TIMSK2 |= (1 << OCIE2A);
  interrupts();
}


int isr_ticker = 20;

ISR(TIMER2_COMPA_vect) {
  if (isr_ticker--) return;
  isr_ticker = 20;
  DoPID();
}


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

  // draw the setpoint if it's on the screen
  //
  if ((graph_setpoint > miny) && (graph_setpoint < maxy)) {
    myGLCD.setColor(VGA_GREEN);
    y1 = map(graph_setpoint, miny, maxy, GRAPH_BOTTOM, GRAPH_TOP);

    myGLCD.drawLine(0, y1, (GRAPH_X - 1), y1);
  }


  for (i = 0; i < (GRAPH_X - 1); i++) {
    y1 = map(_graph[i], miny, maxy, GRAPH_BOTTOM, GRAPH_TOP);
    y2 = map(_graph[i + 1], miny, maxy, GRAPH_BOTTOM, GRAPH_TOP);

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
  // This line disables delay in the library
  sensors.setWaitForConversion(false);

  // Setup the LCD
  //
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);


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

  //initialize the variables we're linked to
  //
  Input = temperC;
  Setpoint = SETPOINT;

  //turn the PID on
  heaterPID.SetMode(AUTOMATIC);
  heaterPID.SetSampleTime(200);

  SetupTimer2();


  delay(20);
}


double CtoF(double c)
{
  return (c * 9) / 5 + 32;
}




double mapd(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void DoPID()
{

  sensors.requestTemperatures();
  temperC = sensors.getTempCByIndex(0);

  Input = temperC;
  heaterPID.Compute();

  analogWrite(HEATER_DRIVE, (int) Output);
}


void loop()
{

  // assume the timer interrupt will update temperC in the background

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

  error = abs(temperC - SETPOINT);

  myGLCD.setFont(BigFont);
  if (error < 0.2) {
    myGLCD.setColor(VGA_GREEN);
  }
  else {
    myGLCD.setColor(VGA_YELLOW);
  }


  dt = temper + 0.005;
  sprintf(s, " %d.%02d %s %d", int(dt), int(dt * 100) % 100, units, (int) Output);
  myGLCD.print(s, 16, TEMP_TOP);
  Serial.println(s);


  if (tick & 4) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  graph(temperC * 100);

  delay(1000);
}
