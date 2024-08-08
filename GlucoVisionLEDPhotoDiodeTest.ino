#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

const int irLedPin = 9;
const int photodiodePin = A0;
const float referenceVoltage = 5.0;

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

LiquidCrystal_I2C lcd(0x27, 16, 2);


void setup() {
  pinMode(irLedPin, OUTPUT);
  Serial.begin(9600);

  lcd.init();                   
  lcd.backlight();              


  if (!tsl.begin()) {
    Serial.print("No TSL2561 sensor found ... check your connections");
    while (1);
  }

  displaySensorDetails();
  configureSensor();

  delay(1000);

  digitalWrite(irLedPin, HIGH);
}

void loop() {
  delay(1000);

  sensors_event_t event;
  tsl.getEvent(&event);

  if (event.light) {
    uint16_t broadband, infrared;
    tsl.getLuminosity(&broadband, &infrared);

    Serial.print("Broadband: ");
    Serial.print(broadband);
    Serial.print(" | Infrared: ");
    Serial.println(infrared);

    lcd.setCursor(0, 1);
    lcd.print("BB: ");
    lcd.print(broadband);
    lcd.print(" IR: ");
    lcd.print(infrared);
  } else {
    Serial.println("No sensor data.");
  }

  int adcValueOn = analogRead(photodiodePin);
  float voltageOn = (adcValueOn / 1023.0) * referenceVoltage;
  Serial.print("LED On - ADC Value: ");
  Serial.print(adcValueOn);
  Serial.print(" | Voltage: ");
  Serial.print(voltageOn);
  Serial.println(" V");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LED On - ADC: ");
  lcd.setCursor(0, 1);
  lcd.print(adcValueOn);
  lcd.print(" V:");
  lcd.print(voltageOn, 2); 
}

void displaySensorDetails(void) {
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void configureSensor(void) {
  tsl.enableAutoRange(true);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);  
}
