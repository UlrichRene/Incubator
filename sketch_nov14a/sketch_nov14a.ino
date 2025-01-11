//|------------------------------------------------------------------------------------------------------|//
//|******************************************************************************************************|//
//|*                                     EGG INCUBATOR CONTROLLER                                       *|//
//|*       with TEMPERATURE, HUMIDITY CONTROL, EGG TURNING SYSTEM AND HATCHING TIME MANAGEMENT          *|//
//|*                       version 1.2 by: ULRICH SEUJIP RENE AKA @EEE_wOLF                             *|//
//|*                                        January, 2025                                               *|//
//|******************************************************************************************************|//
//|------------------------------------------------------------------------------------------------------|//
//|         find us at https://www.youtube.com/@eee_wolf4812                                             |//

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include "RTClib.h"
#include <TimerOne.h>
#include <TimerOne.h>

#define leftButtonPinNumber 2
#define rightButtonPinNumber 3
#define menuButtonPinNumber 4
#define DHTPIN 5
#define heaterPinNumber 6
#define humidifierPinNumber 7
#define fanPinNumber 8
#define motorLeftPinNumber 9
#define motorRightPinNumb 10
#define buzzerPinNumber 11
#define DHTTYPE DHT22 // DHT 22 sensor (AM2302)

#define maxSetableTemperature 99
#define minSetableTemperature 0
#define maxSetableTemperatureTolarance 5
#define minSetableTemperatureTolerance 0

#define maxSetableHumidity 99
#define minSetableHumidity 0
#define maxSetableHumidityTolerance 5
#define minSetableHumidityTolerance 0

#define maxSetableTemperatureTolerenceTreshold 5
#define minSetableTemperatureTolerenceTreshold 5

#define maxSetableHumidityTolerenceTreshold 5
#define minSetableHumidityTolerenceTreshold 0

#define maxSetableNumberOfIncubationDays 100
#define minSetableNumberOfIncubationDays 0

#define maxSetableTimeDuringRotations 4
#define minSetableTimeDuringRotations 0

#define maxSetableTimeBetweenRotations 100
#define minSetableTimeBetweenRotations 0

class Button
{
private:
  uint8_t buttonPin;

  void setPinMode(void)
  {
    pinMode(buttonPin, INPUT_PULLUP);
  }

public:
  Button(int pinNumber)
  {
    buttonPin = pinNumber;
    setPinMode();
  }

  bool isPressed(void)
  {
    if (digitalRead(buttonPin) == 0)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool isLongPressed(void)
  {

    if (digitalRead(buttonPin) == 0)
    {
      delay(100);
      if (digitalRead(buttonPin) == 0)
      {
        do
        {
          return true;
          delay(20);
        } while (digitalRead(buttonPin) == 0);
      }
    }
    else
      return false;
  }

  bool isNotPressed()
  {
    if (digitalRead(buttonPin) == 0)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
};

class Buzzer
{
private:
  uint8_t buzzerPin;

  void setPinMode(void)
  {
    pinMode(buzzerPin, OUTPUT);
  };

public:
  Buzzer(uint8_t pinNumber)
  {
    buzzerPin = pinNumber;
    setPinMode();
  }

  void beep(void)
  {
    digitalWrite(buzzerPin, HIGH);
    delay(100);
    digitalWrite(buzzerPin, LOW);
    delay(100);
  };
};

class Fan
{
private:
  uint8_t fanPin;
  void setPinMode(void)
  {
    pinMode(fanPin, OUTPUT);
  }

public:
  Fan(uint8_t pinNumber)
  {
    fanPin = pinNumber;
    setPinMode();
  }
  void turnOn(void)
  {
    digitalWrite(fanPin, HIGH);
  }

  void turnOff(void)
  {
    digitalWrite(fanPin, LOW);
  }
};

class Heater
{
private:
  uint8_t heaterPin;
  void setPinMode(void)
  {
    pinMode(heaterPin, OUTPUT);
  }

public:
  Heater(uint8_t pinNumber)
  {
    heaterPin = pinNumber;
    setPinMode();
  }
  void turnOn(void)
  {
    digitalWrite(heaterPin, HIGH);
  }

  void turnOff(void)
  {
    digitalWrite(heaterPin, LOW);
  }
};

class Humidifier
{
private:
  uint8_t humidifierPin;
  void setPinMode(void)
  {
    pinMode(humidifierPin, OUTPUT);
  }

public:
  Humidifier(uint8_t pinNumber)
  {
    humidifierPin = pinNumber;
    setPinMode();
  }

  void turnOn(void)
  {
    digitalWrite(humidifierPin, HIGH);
  }

  void turnOff(void)
  {
    digitalWrite(humidifierPin, LOW);
  }
};

class Motor
{
  uint8_t motorLeftTurnPin;
  uint8_t motorRightTurnPin;

  void setPinMode(void)
  {
    pinMode(motorLeftTurnPin, OUTPUT);
    pinMode(motorRightTurnPin, OUTPUT);
    digitalWrite(motorLeftTurnPin, LOW);
    digitalWrite(motorRightTurnPin, LOW);
  }

  void rotateLeft(int timeDuringRotation)
  {
    digitalWrite(motorRightTurnPin, LOW);
    digitalWrite(motorLeftTurnPin, HIGH);
    delay(timeDuringRotation * 1000);
    stop();
    lastRotationDirection = left;
  }

  void rotateRight(int timeDuringRotation)
  {
    digitalWrite(motorLeftTurnPin, LOW);
    digitalWrite(motorRightTurnPin, HIGH);
    delay(timeDuringRotation * 1000);
    stop();
    lastRotationDirection = right;
    Serial.println(timeDuringRotation);
  }

  void stop()
  {
    digitalWrite(motorLeftTurnPin, LOW);
    digitalWrite(motorRightTurnPin, LOW);
  }

public:
  enum RotationDirection
  {
    left,
    right
  };

  RotationDirection lastRotationDirection;
  Motor(uint8_t motorLeftpinNumber, uint8_t motorRightPinNumber)
  {
    motorLeftTurnPin = motorLeftpinNumber;
    motorRightTurnPin = motorRightPinNumber;
    setPinMode();
  }

  void rotateMotor(int timeDuringRotation)
  {
    if (lastRotationDirection == right)
    {
      rotateLeft(timeDuringRotation);
    }
    else
    {
      rotateRight(timeDuringRotation);
    }
  }
};

struct TimeDecount
{
  uint8_t hourComponent;
  uint8_t minuteComponent;
  uint8_t secondsComponent;
};

byte arrow[] = {
    0b01000,
    0b01100,
    0b01110,
    0b01111,
    0b01110,
    0b01100,
    0b01000,
    0b00000};

struct valueAndSafeAddress
{
public:
  float value;
  int saveAddress;
};
struct referenceParameter
{
  valueAndSafeAddress temperature;
  valueAndSafeAddress humidity;
  valueAndSafeAddress temperatureTolerance;
  valueAndSafeAddress humidityTolerance;
  valueAndSafeAddress temperatureTolerenceTreshold; // tolerence above which warning alarm is turned on
  valueAndSafeAddress humidityTolerenceTreshold;
};

struct motorRotationTime
{
  uint8_t hour = 2;
  uint8_t minute = 00;
  uint8_t seconds = 00;
};

TimeDecount RotationTimeCountDown;
Motor eggRotationMotor(motorLeftPinNumber, motorRightPinNumb);
Buzzer buzzer(buzzerPinNumber);
Fan coolingfan(fanPinNumber);
Humidifier humidifier(humidifierPinNumber);
Heater heater(heaterPinNumber);
DHT dht(DHTPIN, DHTTYPE);
RTC_DS1307 rtc;
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display
motorRotationTime rotationTime;
TimeDecount motorRotationTimeInterval;

referenceParameter ref;
valueAndSafeAddress timeDuringRotation;
valueAndSafeAddress timeBetweenRotations;
valueAndSafeAddress numberOfIncubationDays;
valueAndSafeAddress numberofIncubationDaysLeft;

Button menu_slash_Okay_Button(menuButtonPinNumber);
Button rightButton(rightButtonPinNumber);
Button leftButton(leftButtonPinNumber);

DateTime startDate;
DateTime endDate;

bool isRotationTimeReached;
float actualHumidity;
float actualTemperature;
int startDateSaveAddress;


void attributeMemorySafeAddressToParameters()
{
  ref.temperature.saveAddress = 5;
  ref.humidity.saveAddress = 10;
  ref.temperatureTolerance.saveAddress = 20;
  ref.humidityTolerance.saveAddress = 30;
  ref.temperatureTolerenceTreshold.saveAddress = 40;
  ref.humidityTolerenceTreshold.saveAddress = 50;

  timeDuringRotation.saveAddress = 60;
  timeBetweenRotations.saveAddress = 70;
  numberOfIncubationDays.saveAddress = 80;
  numberofIncubationDaysLeft.saveAddress = 90;
  startDateSaveAddress = 100;
}

void PrintWelcomeMessage()
{
  lcd.setCursor(1, 0);
  lcd.print("*****EEE_WOLF*****");
  lcd.setCursor(4, 1);
  lcd.print("EGG INCUBATOR");
  lcd.setCursor(0, 2);
  lcd.print("by ULRICH SEUJIP R.");
  delay(500);
  lcd.setCursor(1, 3);

  String slogan = "Make life easier!";
  for (uint8_t i = 0; i < 17; i++)
  {
    lcd.setCursor(i + 1, 3);
    lcd.print(slogan[i]);
    delay(50);
    lcd.blink();
  }
  lcd.blink_off();
  delay(1000);
  lcd.clear();
}

void initialisation(void)
{
  rtc.begin();
  dht.begin();
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  PrintWelcomeMessage();
  lcd.createChar(10, arrow); // create arrow character for menu loading display from the arrow array
}

// decounting the time for motor rotation every second and updating the value on the LCD
void runEverySecond(void)
{
  isRotationTimeReached = false;
  if (RotationTimeCountDown.secondsComponent > 0)
  {
    RotationTimeCountDown.secondsComponent--;
  }
  else if ((RotationTimeCountDown.secondsComponent == 0) && (RotationTimeCountDown.minuteComponent != 0))
  {
    RotationTimeCountDown.minuteComponent--;
    RotationTimeCountDown.secondsComponent = 59;
  }
  else if ((RotationTimeCountDown.secondsComponent == 0) && (RotationTimeCountDown.minuteComponent == 0) && (RotationTimeCountDown.hourComponent != 0))
  {
    RotationTimeCountDown.hourComponent--;
    RotationTimeCountDown.minuteComponent = 59;
    RotationTimeCountDown.secondsComponent = 59;
  }
  else if ((RotationTimeCountDown.secondsComponent == 0) && (RotationTimeCountDown.minuteComponent == 0) && (RotationTimeCountDown.hourComponent == 0))
  {
    isRotationTimeReached = true;
  }
}

void decountIncubationDays()
{
  for (unsigned short n = 1; n <= numberOfIncubationDays.value; n++)
  {
    if (startDate.isValid() && rtc.now() == (startDate + TimeSpan(0, 0, 0, n)) && numberofIncubationDaysLeft.value > 0)
    {
      numberofIncubationDaysLeft.value = numberOfIncubationDays.value - n;
      if (numberofIncubationDaysLeft.value < 0)
      {
        numberofIncubationDaysLeft.value = 0;
      }
      EEPROM.write(numberofIncubationDaysLeft.saveAddress, static_cast<int>(numberofIncubationDaysLeft.value));
      delay(10);
    }
  }
  if (numberofIncubationDaysLeft.value == 0)
  {
    // alarm();
    lcd.setCursor(0, 3);
    lcd.print("                   ");
    delay(10);
    lcd.setCursor(0, 3);
    lcd.print("INCUB. TIME REACHED");
    delay(500);
    lcd.setCursor(0, 3);
    lcd.print("                    ");
    delay(10);
  }
}

void readtempandhumidity(void)
{
  actualTemperature = dht.readTemperature();
  actualHumidity = dht.readHumidity();
}

void TriggerAlarm()
{
}
void saveChangesToMemory(void)
{
  RotationTimeCountDown.hourComponent = timeBetweenRotations.value;
  RotationTimeCountDown.minuteComponent = 0;
  RotationTimeCountDown.secondsComponent = 0;

  startDate = rtc.now();
  delay(20);
  EEPROM.put(startDateSaveAddress, startDate);
  delay(20);
  EEPROM.put(ref.temperature.saveAddress, ref.temperature.value);
  delay(20);
  EEPROM.put(ref.temperatureTolerance.saveAddress, ref.temperatureTolerance.value);
  delay(20);
  EEPROM.put(ref.temperatureTolerenceTreshold.saveAddress, ref.temperatureTolerenceTreshold.value);
  delay(20);
  EEPROM.put(ref.humidity.saveAddress, ref.humidity.value);
  delay(20);
  EEPROM.put(ref.humidityTolerance.saveAddress, ref.humidityTolerance.value);
  delay(20);
  EEPROM.put(ref.humidityTolerenceTreshold.saveAddress, ref.humidityTolerenceTreshold.value);
  delay(20);
  EEPROM.put(timeDuringRotation.saveAddress, timeDuringRotation.value);
  delay(20);
  EEPROM.put(timeBetweenRotations.saveAddress, timeBetweenRotations.value);
  delay(20);
  EEPROM.put(numberOfIncubationDays.saveAddress, numberOfIncubationDays.value);
  delay(20);
  numberofIncubationDaysLeft.value = numberOfIncubationDays.value;
  delay(20);
  EEPROM.write(numberofIncubationDaysLeft.saveAddress, static_cast<int>(numberofIncubationDaysLeft.value));
  delay(20);
}

void discardChanges()
{
  delay(20);
  EEPROM.get(startDateSaveAddress, startDate);
  delay(20);
  EEPROM.get(ref.temperature.saveAddress, ref.temperature.value);
  delay(20);
  EEPROM.get(ref.temperatureTolerance.saveAddress, ref.temperatureTolerance.value);
  delay(20);
  EEPROM.get(ref.temperatureTolerenceTreshold.saveAddress, ref.temperatureTolerenceTreshold.value);
  delay(20);
  EEPROM.get(ref.humidity.saveAddress, ref.humidity.value);
  delay(20);
  EEPROM.get(ref.humidityTolerance.saveAddress, ref.humidityTolerance.value);
  delay(20);
  EEPROM.get(ref.humidityTolerenceTreshold.saveAddress, ref.humidityTolerenceTreshold.value);
  delay(20);
  EEPROM.get(timeDuringRotation.saveAddress, timeDuringRotation.value);
  delay(20);
  EEPROM.get(timeBetweenRotations.saveAddress, timeBetweenRotations.value);
  delay(20);
  EEPROM.get(numberOfIncubationDays.saveAddress, numberOfIncubationDays.value);
  delay(20);
  numberofIncubationDaysLeft.value = EEPROM.read(numberofIncubationDaysLeft.saveAddress);
  delay(20);
  RotationTimeCountDown.hourComponent = timeBetweenRotations.value;
  RotationTimeCountDown.minuteComponent = 0;
  RotationTimeCountDown.secondsComponent = 0;
}

void displayDashboard(void)
{
  lcd.setCursor(0, 0);
  lcd.print("TEMP= ");
  lcd.setCursor(6, 0);
  lcd.print(actualTemperature);
  lcd.setCursor(12, 0);
  lcd.print((char)223);
  lcd.setCursor(13, 0);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("HUMID= ");
  lcd.setCursor(7, 1);
  lcd.print(actualHumidity);
  lcd.setCursor(13, 1);
  lcd.print("%");

  lcd.setCursor(0, 2);
  lcd.print("T_Spin =");
  lcd.setCursor(9, 2);
  lcd.print(RotationTimeCountDown.hourComponent); // the rotation time is counted down by the timer interrupt function which runs every second
  // lcd.print("00");
  lcd.setCursor(11, 2);
  lcd.print(":");
  lcd.setCursor(12, 2);
  lcd.print(RotationTimeCountDown.minuteComponent);
  // lcd.print("00");
  lcd.setCursor(14, 2);
  lcd.print(":");
  lcd.setCursor(15, 2);
  lcd.print(RotationTimeCountDown.secondsComponent);
  // lcd.print("00");

  lcd.setCursor(0, 3);
  lcd.print("HATCH IN = ");
  lcd.setCursor(11, 3);
  lcd.print(static_cast<int>(numberofIncubationDaysLeft.value));
  lcd.setCursor(14, 3);
  if (numberofIncubationDaysLeft.value == 1)
  {
    lcd.print("Day");
  }
  else
  {
    lcd.print("Days");
  }

  if (numberofIncubationDaysLeft.value < 99)
  {
    lcd.setCursor(13, 3);
    lcd.print(" ");
  }
  if (numberofIncubationDaysLeft.value < 10)
  {
    lcd.setCursor(12, 3);
    lcd.print(" ");
  }
}

void temperatureregulation(void)
{
  readtempandhumidity();
  if ((actualTemperature >= (ref.temperature.value - ref.temperatureTolerance.value)) && (actualTemperature <= (ref.temperature.value + ref.temperatureTolerance.value)))
  {
    coolingfan.turnOff();
    heater.turnOff();
  }
  else if (actualTemperature > (ref.temperature.value + ref.temperatureTolerance.value))
  {
    coolingfan.turnOn();
    heater.turnOff();
    if (actualTemperature >= (ref.temperature.value + ref.temperatureTolerance.value + ref.temperatureTolerenceTreshold.value))
    {
      buzzer.beep();
      lcd.setCursor(16, 0);
      lcd.print(" ");
      delay(150);
    }
    lcd.setCursor(16, 0);
    lcd.write(1);
  }

  else if (actualTemperature < (ref.temperature.value - ref.temperatureTolerance.value))
  {
    coolingfan.turnOff();
    heater.turnOn();

    if (actualTemperature <= (ref.temperature.value - ref.temperatureTolerance.value - ref.temperatureTolerenceTreshold.value))
    {
      buzzer.beep();
      lcd.setCursor(16, 0);
      lcd.write(1);
      delay(100);
      lcd.setCursor(16, 0);
      lcd.print(" ");
      delay(100);
    }

    lcd.setCursor(16, 0);
    lcd.write(1);
  }
}

void humidityregulation()
{
  readtempandhumidity();
  if ((actualHumidity >= (ref.humidity.value - ref.humidityTolerance.value)) && (actualHumidity <= (ref.humidity.value + ref.humidityTolerance.value)))
  {
    humidifier.turnOff();
    lcd.setCursor(16, 1);
    lcd.write(0);
  }

  else if (actualHumidity > (ref.humidity.value + ref.humidityTolerance.value))
  {

    humidifier.turnOff();
    if (actualHumidity >= (ref.humidity.value + ref.humidityTolerance.value + ref.humidityTolerenceTreshold.value))
    {
      buzzer.beep();
      lcd.setCursor(16, 1);
      lcd.write(1);
      delay(100);
      lcd.setCursor(16, 1);
      lcd.print(" ");
      delay(100);
    }
    lcd.setCursor(16, 1);
    lcd.write(1);
  }
  readtempandhumidity();
  if (actualHumidity < (ref.humidity.value - ref.humidityTolerance.value))
  {
    humidifier.turnOn();
    if (actualHumidity <= (ref.humidity.value - ref.humidityTolerance.value - ref.humidityTolerenceTreshold.value))
    {
      lcd.setCursor(16, 1);
      lcd.write(1);
      delay(100);
      buzzer.beep();
      lcd.setCursor(16, 1);
      lcd.print(" ");
      delay(100);
    }
  }
}
void motorRotation()
{
  // rotate the motor in the direction that has to be rotated.
  lcd.setCursor(0, 2);
  lcd.print("                   ");
  delay(10);
  lcd.setCursor(3, 2);
  lcd.print("Motor Rotation");
  delay(200);
  lcd.setCursor(0, 2);
  lcd.print("                    ");
  delay(10);
  eggRotationMotor.rotateMotor(timeDuringRotation.value);
  RotationTimeCountDown.hourComponent = timeBetweenRotations.value;
}

void temperatureAndHumidityRegulation(void)
{
  temperatureregulation();
  humidityregulation();
}

void blinkIncubatorTimeReachedMessageOnLCD(void)
{
  lcd.setCursor(0, 3);
  lcd.print("                   ");
  delay(10);
  lcd.setCursor(0, 3);
  lcd.print("INCUB. TIME REACHED");
  delay(500);
  lcd.setCursor(0, 3);
  lcd.print("                    ");
  delay(10);
}

void increment(float *varToIncrement, float incrementatioinFactor)
{
  *varToIncrement = *varToIncrement + incrementatioinFactor;
}

void menu(void)
{
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Entering MENU");
  lcd.setCursor(4, 1);
  lcd.print("Please Wait...");
  delay(20);
  lcd.clear();

  do
  {
    lcd.setCursor(4, 1);
    lcd.print("=>"); // special select item character
    lcd.setCursor(1, 0);
    lcd.print("Set Temperature:");
    lcd.setCursor(6, 1);
    lcd.print(ref.temperature.value);
    lcd.setCursor(11, 1);
    lcd.print((char)223);
    lcd.setCursor(12, 1);
    lcd.print("C");

    lcd.setCursor(2, 2);
    lcd.print("Temp Tolerance:");
    lcd.setCursor(6, 3);
    lcd.print(ref.temperatureTolerance.value);
    lcd.setCursor(10, 3);
    lcd.print((char)223);
    lcd.setCursor(11, 3);
    lcd.print("C");

    if ((rightButton.isPressed()) && (ref.temperature.value < maxSetableTemperature))
    {
      increment(&ref.temperature.value, 0.1);
      lcd.setCursor(6, 1);
      lcd.print(ref.temperature.value);
      delay(100);
    }

    while ((rightButton.isLongPressed()) && (ref.temperature.value < maxSetableTemperature))
    {
      increment(&ref.temperature.value, 0.1);
      lcd.setCursor(6, 1);
      lcd.print(ref.temperature.value);
      delay(1);
    }

    if ((leftButton.isPressed()) && (ref.temperature.value > minSetableTemperature))
    {
      increment(&ref.temperature.value, -0.1);
      if (ref.temperature.value < minSetableTemperature)
      {
        ref.temperature.value = minSetableTemperature;
      }
      lcd.setCursor(6, 1);
      lcd.print(ref.temperature.value);
      delay(100);
    }
    while ((leftButton.isLongPressed()) && (ref.temperature.value > minSetableTemperature))
    {
      increment(&ref.temperature.value, -0.1);
      if (ref.temperature.value < minSetableTemperature)
      {
        ref.temperature.value = minSetableTemperature;
      }
      lcd.setCursor(6, 1);
      lcd.print(ref.temperature.value);
      delay(1);
    }
  } while (menu_slash_Okay_Button.isNotPressed());

  lcd.setCursor(4, 1);
  lcd.print("  "); // special select item character removal
  delay(100);

  do
  {
    lcd.setCursor(4, 3);
    lcd.print("=>"); // special select item character

    if ((rightButton.isPressed()) && (ref.temperatureTolerance.value < maxSetableTemperatureTolarance))
    {
      increment(&ref.temperatureTolerance.value, 0.1);
      lcd.setCursor(6, 3);
      lcd.print(ref.temperatureTolerance.value);
      delay(100);
    }
    while ((rightButton.isLongPressed()) && (ref.temperatureTolerance.value < maxSetableTemperatureTolarance))
    {
      increment(&ref.temperatureTolerance.value, 0.1);
      lcd.setCursor(6, 3);
      lcd.print(ref.temperatureTolerance.value);
      delay(1);
    }

    if (leftButton.isPressed() && (ref.temperatureTolerance.value > minSetableTemperatureTolerance))
    {
      increment(&ref.temperatureTolerance.value, -0.1);
      if (ref.temperatureTolerance.value < minSetableTemperatureTolerance)
      {
        ref.temperatureTolerance.value = minSetableTemperatureTolerance;
      }
      lcd.setCursor(6, 3);
      lcd.print(ref.temperatureTolerance.value);
      delay(100);
    }
    while ((leftButton.isLongPressed()) && (ref.temperatureTolerance.value > minSetableTemperatureTolerance))
    {
      increment(&ref.temperatureTolerance.value, -0.1);
      if (ref.temperatureTolerance.value < minSetableTemperatureTolerance)
      {
        ref.temperatureTolerance.value = minSetableTemperatureTolerance;
      }
      lcd.setCursor(6, 3);
      lcd.print(ref.temperatureTolerance.value);
      delay(1);
    }

  } while (menu_slash_Okay_Button.isNotPressed());
  lcd.clear();
  delay(100);

  do
  {
    lcd.setCursor(4, 1);
    lcd.print("=>"); // special select item character
    lcd.setCursor(3, 0);
    lcd.print("Set Humidity:");
    lcd.setCursor(6, 1);
    lcd.print(ref.humidity.value);
    lcd.setCursor(11, 1);
    lcd.print("% RH");
    lcd.setCursor(1, 2);
    lcd.print("Humidity Tolerance:");
    lcd.setCursor(6, 3);
    lcd.print(ref.humidityTolerance.value);
    lcd.setCursor(10, 3);
    lcd.print("% RH");

    if ((rightButton.isPressed()) && (ref.humidity.value < maxSetableHumidity))
    {
      increment(&ref.humidity.value, 0.1);
      lcd.setCursor(6, 1);
      lcd.print(ref.humidity.value);
      delay(100);
    }
    while ((rightButton.isLongPressed()) && (ref.humidity.value < maxSetableHumidity))
    {
      increment(&ref.humidity.value, 0.1);
      lcd.setCursor(6, 1);
      lcd.print(ref.humidity.value);
      delay(1);
    }

    if ((leftButton.isPressed()) && (ref.humidity.value > minSetableHumidity))
    {
      increment(&ref.humidity.value, -0.1);
      if (ref.humidity.value < minSetableHumidity)
      {
        ref.humidity.value = minSetableHumidity;
      }
      lcd.setCursor(6, 1);
      lcd.print(ref.humidity.value);
      delay(100);
    }
    while ((leftButton.isLongPressed()) && (ref.humidity.value > minSetableHumidity))
    {
      increment(&ref.humidity.value, -0.1);
      if (ref.humidity.value < minSetableHumidity)
      {
        ref.humidity.value = minSetableHumidity;
      }
      lcd.setCursor(6, 1);
      lcd.print(ref.humidity.value);
      delay(1);
    }

  } while (menu_slash_Okay_Button.isNotPressed());

  lcd.setCursor(4, 1);
  lcd.print("  "); // special select item character removal

  delay(100); // debouncing the menu button to prevent jump to next when just entering this do itteration
  do
  {
    lcd.setCursor(4, 3);
    lcd.print("=>"); // special select item character

    if (rightButton.isPressed() && (ref.humidityTolerance.value < maxSetableHumidityTolerance))
    {
      increment(&ref.humidityTolerance.value, 0.1);
      if (ref.humidityTolerance.value > maxSetableHumidityTolerance)
      {
        ref.humidityTolerance.value = maxSetableHumidityTolerance;
      }
      lcd.setCursor(6, 3);
      lcd.print(ref.humidityTolerance.value);
      delay(100);
    }
    while (rightButton.isLongPressed() && (ref.humidityTolerance.value < maxSetableHumidityTolerance))

    {
      increment(&ref.humidityTolerance.value, 0.1);
      if (ref.humidityTolerance.value > maxSetableHumidityTolerance)
      {
        ref.humidityTolerance.value = maxSetableHumidityTolerance;
      }
      lcd.setCursor(6, 3);
      lcd.print(ref.humidityTolerance.value);
      delay(1);
    }

    if (leftButton.isPressed() && (ref.humidityTolerance.value > minSetableHumidityTolerance))
    {
      increment(&ref.humidityTolerance.value, -0.1);
      if (ref.humidityTolerance.value < minSetableHumidityTolerance)
      {
        ref.humidityTolerance.value = minSetableHumidityTolerance;
      }
      delay(100);
      lcd.setCursor(6, 3);
      lcd.print(ref.humidityTolerance.value);
    }
    while (leftButton.isLongPressed() && (ref.humidityTolerance.value > minSetableHumidityTolerance))
    {

      increment(&ref.humidityTolerance.value, -0.1);
      if (ref.humidityTolerance.value < minSetableHumidityTolerance)
      {
        ref.humidityTolerance.value = minSetableHumidityTolerance;
      }
      lcd.setCursor(6, 3);
      lcd.print(ref.humidityTolerance.value);
      delay(1);
    }

  } while (menu_slash_Okay_Button.isNotPressed());
  delay(100);
  lcd.clear();
  do
  {
    lcd.setCursor(1, 0);
    lcd.print("Temp_Tol Treshold:");
    lcd.setCursor(4, 1);
    lcd.print("=>"); // special select item character
    lcd.setCursor(6, 1);
    lcd.print(ref.temperatureTolerenceTreshold.value);
    lcd.setCursor(10, 1);
    lcd.print((char)223);
    lcd.setCursor(11, 1);
    lcd.print("C");

    lcd.setCursor(0, 2);
    lcd.print("Humid_Tol Treshold:");
    lcd.setCursor(6, 3);
    lcd.print(ref.humidityTolerenceTreshold.value);
    lcd.setCursor(10, 3);
    lcd.print("% RH");

    if ((rightButton.isPressed()) && (ref.temperatureTolerenceTreshold.value < maxSetableTemperatureTolerenceTreshold))
    {
      increment(&ref.temperatureTolerenceTreshold.value, 0.1);
      lcd.setCursor(6, 1);
      lcd.print(ref.temperatureTolerenceTreshold.value);
      delay(100);
    }
    while ((rightButton.isLongPressed()) && (ref.temperatureTolerenceTreshold.value < maxSetableTemperatureTolerenceTreshold))
    {
      increment(&ref.temperatureTolerenceTreshold.value, 0.1);
      lcd.setCursor(6, 1);
      lcd.print(ref.temperatureTolerenceTreshold.value);
      delay(1);
    }

    if (leftButton.isPressed() && (ref.temperatureTolerenceTreshold.value > minSetableTemperatureTolerenceTreshold))
    {
      increment(&ref.temperatureTolerenceTreshold.value, -0.1);
      if (ref.temperatureTolerenceTreshold.value < minSetableTemperatureTolerenceTreshold)
      {
        ref.temperatureTolerenceTreshold.value = minSetableTemperatureTolerenceTreshold;
      }
      lcd.setCursor(6, 1);
      lcd.print(ref.temperatureTolerenceTreshold.value);
      delay(100);
    }
    while ((leftButton.isLongPressed()) && (ref.temperatureTolerenceTreshold.value > minSetableTemperatureTolerenceTreshold))
    {
      increment(&ref.temperatureTolerenceTreshold.value, -0.1);
      if (ref.temperatureTolerenceTreshold.value < minSetableTemperatureTolerenceTreshold)
      {
        ref.temperatureTolerenceTreshold.value = minSetableTemperatureTolerenceTreshold;
      }
      lcd.setCursor(6, 1);
      lcd.print(ref.temperatureTolerenceTreshold.value);
      delay(1);
    }

  } while (menu_slash_Okay_Button.isNotPressed());

  lcd.setCursor(4, 1);
  lcd.print("  "); // special select item character removal
  delay(100);

  do
  {
    lcd.setCursor(4, 3);
    lcd.print("=>"); // special select item character

    if ((rightButton.isPressed()) && (ref.humidityTolerenceTreshold.value < maxSetableHumidityTolerenceTreshold))
    {
      increment(&ref.humidityTolerenceTreshold.value, 0.1);
      lcd.setCursor(6, 3);
      lcd.print(ref.humidityTolerenceTreshold.value);
      delay(100);
    }
    while ((rightButton.isLongPressed()) && (ref.humidityTolerenceTreshold.value < maxSetableHumidityTolerenceTreshold))
    {
      increment(&ref.humidityTolerenceTreshold.value, 0.1);
      lcd.setCursor(6, 3);
      lcd.print(ref.humidityTolerenceTreshold.value);
      delay(1);
    }

    if ((leftButton.isPressed()) && (ref.humidityTolerenceTreshold.value > minSetableHumidityTolerenceTreshold))
    {
      increment(&ref.humidityTolerenceTreshold.value, -0.1);
      if (ref.humidityTolerenceTreshold.value < minSetableHumidityTolerenceTreshold)
      {
        ref.humidityTolerenceTreshold.value = minSetableHumidityTolerenceTreshold;
      }
      lcd.setCursor(6, 3);
      lcd.print(ref.humidityTolerenceTreshold.value);
      delay(100);
    }
    while ((leftButton.isLongPressed()) && (ref.humidityTolerenceTreshold.value > minSetableHumidityTolerenceTreshold))
    {
      increment(&ref.humidityTolerenceTreshold.value, -0.1);
      if (ref.humidityTolerenceTreshold.value < minSetableHumidityTolerenceTreshold)
      {
        ref.humidityTolerenceTreshold.value = minSetableHumidityTolerenceTreshold;
      }
      lcd.setCursor(6, 3);
      lcd.print(ref.humidityTolerenceTreshold.value);
      delay(1);
    }

  } while (menu_slash_Okay_Button.isNotPressed());

  delay(100);
  lcd.clear();

  do
  {
    // display of incubation time in days
    lcd.setCursor(4, 2);
    lcd.print("=>");
    lcd.setCursor(0, 0);
    lcd.print("Set Incubation_Time:");
    lcd.setCursor(6, 2);
    lcd.print(static_cast<int>(numberOfIncubationDays.value));
    if (numberOfIncubationDays.value < 100)
    {
      lcd.setCursor(8, 2);
      lcd.print(" ");
    }
    if (numberOfIncubationDays.value < 10)
    {
      lcd.setCursor(7, 2);
      lcd.print(" ");
    }
    lcd.setCursor(10, 2);
    lcd.print("Days");
    if ((rightButton.isPressed()) && (numberOfIncubationDays.value < maxSetableNumberOfIncubationDays))
    {
      increment(&numberOfIncubationDays.value, 1);
      lcd.setCursor(6, 2);
      lcd.print(static_cast<int>(numberOfIncubationDays.value));
      delay(100);
    }
    delay(20);
    while ((rightButton.isLongPressed()) && (numberOfIncubationDays.value < maxSetableNumberOfIncubationDays))
    {
      increment(&numberOfIncubationDays.value, 1);
      lcd.setCursor(6, 2);
      lcd.print(static_cast<int>(numberOfIncubationDays.value));
      delay(1);
    }

    if ((leftButton.isPressed()) && (numberOfIncubationDays.value > minSetableNumberOfIncubationDays))
    {
      increment(&numberOfIncubationDays.value, -1);

      if (numberOfIncubationDays.value < minSetableNumberOfIncubationDays)
      {
        numberOfIncubationDays.value = minSetableNumberOfIncubationDays;
      }
      lcd.setCursor(6, 2);
      lcd.print(static_cast<int>(numberOfIncubationDays.value));
      delay(10);
    }

    while ((leftButton.isLongPressed()) && (numberOfIncubationDays.value > minSetableNumberOfIncubationDays))
    {
      increment(&numberOfIncubationDays.value, -1);
      if (numberOfIncubationDays.value < minSetableNumberOfIncubationDays)
      {
        numberOfIncubationDays.value = minSetableNumberOfIncubationDays;
      }
      lcd.setCursor(6, 2);
      lcd.print(static_cast<int>(numberOfIncubationDays.value));
      delay(1);
    }
  } while (menu_slash_Okay_Button.isNotPressed());
  delay(10);
  lcd.clear();

  do
  {
    lcd.setCursor(0, 0);
    lcd.print("Time During Rot.");
    lcd.setCursor(4, 1);
    lcd.print("=>");
    lcd.setCursor(6, 1);
    lcd.print(static_cast<int>(timeDuringRotation.value));
    if (timeDuringRotation.value < 10)
    {
      lcd.setCursor(7, 1);
      lcd.print(" ");
    }
    lcd.setCursor(8, 1);
    lcd.print("s");

    lcd.setCursor(0, 2);
    lcd.print("Time Between Rot.");
    lcd.setCursor(6, 3);
    lcd.print(static_cast<int>(timeBetweenRotations.value));
    if (timeBetweenRotations.value < 10)
    {
      lcd.setCursor(7, 3);
      lcd.print(" ");
    }
    lcd.setCursor(8, 3);
    lcd.print("Hrs");

    if (rightButton.isPressed() && (timeDuringRotation.value < maxSetableTimeDuringRotations))
    {
      increment(&timeDuringRotation.value, 1);
      lcd.setCursor(6, 1);
      lcd.print(static_cast<int>(timeDuringRotation.value));
      delay(100);
    }

    while (rightButton.isLongPressed() && (timeDuringRotation.value < maxSetableTimeDuringRotations))
    {
      increment(&timeDuringRotation.value, 1);
      lcd.setCursor(6, 1);
      lcd.print(static_cast<int>(timeDuringRotation.value));
      delay(1);
    }

    if ((leftButton.isPressed()) && (timeDuringRotation.value > minSetableTimeDuringRotations))
    {
      increment(&timeDuringRotation.value, -1);
      if (timeDuringRotation.value < minSetableTimeDuringRotations)
      {
        timeDuringRotation.value = minSetableTimeDuringRotations;
      }
      lcd.setCursor(6, 1);
      lcd.print(static_cast<int>(timeDuringRotation.value));
      delay(100);
    }
    while (leftButton.isLongPressed() && (timeDuringRotation.value > minSetableTimeDuringRotations))
    {
      increment(&timeDuringRotation.value, -1);
      lcd.setCursor(6, 1);
      lcd.print(static_cast<int>(timeDuringRotation.value));
      delay(1);
    }
  } while (menu_slash_Okay_Button.isNotPressed());
  lcd.setCursor(4, 1);
  lcd.print("  ");
  delay(100);
  do
  {
    lcd.setCursor(4, 3);
    lcd.print("=>"); // special select item character

    if ((rightButton.isPressed()) && (timeBetweenRotations.value < maxSetableTimeBetweenRotations))
    {
      increment(&timeBetweenRotations.value, 1);
      if (timeBetweenRotations.value > maxSetableTimeBetweenRotations)
      {
        ref.temperatureTolerance.value = maxSetableTimeBetweenRotations;
      }
      delay(100);
      lcd.setCursor(6, 3);
      lcd.print(static_cast<int>(timeBetweenRotations.value));
    }
    while ((rightButton.isLongPressed()) && (timeBetweenRotations.value < maxSetableTimeBetweenRotations))
    {
      increment(&timeBetweenRotations.value, 1);
      lcd.setCursor(6, 3);
      lcd.print(static_cast<int>(timeBetweenRotations.value));
      delay(1);
    }

    if (leftButton.isPressed() && (timeBetweenRotations.value > minSetableTimeBetweenRotations))
    {
      increment(&timeBetweenRotations.value, -1);
      if (timeBetweenRotations.value < minSetableTimeBetweenRotations)
      {
        ref.temperatureTolerance.value = minSetableTimeBetweenRotations;
      }
      lcd.setCursor(6, 3);
      lcd.print(static_cast<int>(timeBetweenRotations.value));
      delay(100);
    }
    while ((leftButton.isLongPressed()) && (timeBetweenRotations.value > minSetableTimeBetweenRotations))
    {
      increment(&timeBetweenRotations.value, -1);
      if (timeBetweenRotations.value < minSetableTimeBetweenRotations)
      {
        timeBetweenRotations.value = minSetableTimeBetweenRotations;
      }
      lcd.setCursor(6, 3);
      lcd.print(static_cast<int>(timeBetweenRotations.value));
      delay(1);
    }

  } while (menu_slash_Okay_Button.isNotPressed());
  lcd.clear();
  delay(20);

  delay(100);
  lcd.clear();
  do
  {
    lcd.setCursor(0, 0);
    lcd.print(" Press 'Ok' to save");
    lcd.setCursor(2, 1);
    lcd.print(" Changes or any");
    lcd.setCursor(0, 2);
    lcd.print("Other key to discard");

  } while (menu_slash_Okay_Button.isNotPressed() && leftButton.isNotPressed() && rightButton.isNotPressed());

  if (menu_slash_Okay_Button.isPressed())
  {
    saveChangesToMemory();
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("SAVING...");
    lcd.setCursor(3, 3);
    for (int i = 0; i < 17; i++)
    {
      lcd.setCursor(i + 1, 1);
      lcd.write(byte(10)); // display of the progressively loading arrow, only for esthetics. In reality the saving had been done before this step
      lcd.setCursor(i + 1, 2);
      lcd.write(byte(10));
      delay(60);
    }
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("SAVED SUCCESSFULLY!!");
    delay(1000);
  }
  else
  {
    lcd.clear();
    delay(10);
    discardChanges();
    lcd.setCursor(2, 1);
    lcd.print("Changes Discarded");
    delay(300);
  }

  lcd.clear();
}

void setup()
{
  initialisation();
  attributeMemorySafeAddressToParameters();
  discardChanges(); // the Discardchanges function loads parameter data setpoints from the memory.
  decountIncubationDays();

  // Initialize Timer1 to trigger every 1 second
  Timer1.initialize(1000000);
  Timer1.attachInterrupt(runEverySecond); // make the runEverySecond function to run every 1 second
}

void loop()
{
  readtempandhumidity();
  displayDashboard();
  temperatureAndHumidityRegulation();
  decountIncubationDays();
  if (isRotationTimeReached)
  {
    motorRotation();
  }
  if (menu_slash_Okay_Button.isPressed())
  {
    menu();
  }
}