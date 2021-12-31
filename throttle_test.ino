/*
  Analog Input

  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().

  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13 through 220 ohm resistor
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.

  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogInput
*/

#define CHANNEL1_INPUT      A3
#define CHANNEL2_INPUT      A4
#define CHANNEL3_INPUT      A5

#define SIM_CHANNEL1_INPUT  A0
#define SIM_CHANNEL2_INPUT  A1
#define SIM_CHANNEL3_INPUT  A2

#define CHANNEL1_OUTPUT     9   // green
#define CHANNEL2_OUTPUT     10  // blue
#define CHANNEL3_OUTPUT     11  // orange

#define SENSOR_PIN CHANNEL3_INPUT

#define BAUDRATE            115200

#define CHANNEL_1_INIT_mV  4080
#define CHANNEL_2_INIT_mV  1410
#define CHANNEL_3_INIT_mV  780

#define CHANNEL_1_END_mV   1270
#define CHANNEL_2_END_mV   3560
#define CHANNEL_3_END_mV   3030

#define MIN_INPUT_SIGNAL   160
#define MAX_INPUT_SIGNAL   625

#define MAX_OUTPUT_SIGNAL  255
#define MAX_OUTPUT_VOLTAGE 4500

#define ADC_TO_VOLTAGE_FACTOR 4.88f

#define TROTLE_VARIATION_THRESHOLD 50 //In percentage points   

void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin(BAUDRATE);
  pinMode(CHANNEL1_OUTPUT, OUTPUT);
  pinMode(CHANNEL2_OUTPUT, OUTPUT);
  pinMode(CHANNEL3_OUTPUT, OUTPUT);

  Serial.println("Throttle test");
  Serial.println("Send value range from 0 to 100%");
  //Serial.print("Sim1,Sim2,Sim3,Ch1,Ch2,Ch3,");
  delay(100);
}

void loop() {
  // read the value from the sensor:
  static int   sensorValue = 0;
  static float sensorLevel = 0;
  static int computedVoltageCH1 = 0;
  static int computedVoltageCH2 = 0;
  static int computedVoltageCH3 = 0;

  static int lastAnalogSensorValue = 0;
  
  static int serialSensorValue = 0;
  static int analogSensorValue = 0;
  static bool useSerialWrittenValue = false;
  static bool isFloatingInput = false;

  sensorValue = analogRead(SENSOR_PIN);

  if (Serial.available()){
    delay(20);
    serialSensorValue = Serial.parseInt();
    if (serialSensorValue == 200){
      Serial.println("Toggling floating mode");
      isFloatingInput = !isFloatingInput;

      if(isFloatingInput){
        Serial.println("Floating mode ENABLED");
      } else {
        Serial.println("Floating mode DISABLE");
      }
      
    }
    else if (serialSensorValue > 0 || serialSensorValue <= 100){
      
      useSerialWrittenValue = true;
    }
    //Flushing for the Serial Input because I HAVE TO DO EVERYTHING
    for (int i = 0; i < 50; i++){
        Serial.read();
      }
  }

  //If floating input is enable then will no longer read from ADC due to high rate fluctuating values
  if (isFloatingInput){
    analogSensorValue = 0;
  } else {
    analogSensorValue = computeInputStatus(sensorValue, MIN_INPUT_SIGNAL, MAX_INPUT_SIGNAL);
  }
  
  if (useSerialWrittenValue){
    if (percentualVariation(analogSensorValue, lastAnalogSensorValue) > TROTLE_VARIATION_THRESHOLD){
      Serial.println("recuperamos el valor analogico con una variacion del: " + String(percentualVariation(sensorValue, lastAnalogSensorValue)));
      useSerialWrittenValue = false;
    }
  }

  if (useSerialWrittenValue){
    sensorLevel = serialSensorValue;
  } else {
    sensorLevel = analogSensorValue;
    lastAnalogSensorValue = analogSensorValue;
  }

  Serial.println("ADC: " + String(sensorValue));
  Serial.println("Sensor: " + String(sensorLevel));
  
  computedVoltageCH1 = computeVoltage(sensorLevel, CHANNEL_1_INIT_mV, CHANNEL_1_END_mV);
  computedVoltageCH2 = computeVoltage(sensorLevel, CHANNEL_2_INIT_mV, CHANNEL_2_END_mV);
  computedVoltageCH3 = computeVoltage(sensorLevel, CHANNEL_3_INIT_mV, CHANNEL_3_END_mV);

  //
  analogWrite(CHANNEL1_OUTPUT, computePWM(computedVoltageCH1, MAX_OUTPUT_VOLTAGE, MAX_OUTPUT_SIGNAL));
  analogWrite(CHANNEL2_OUTPUT, computePWM(computedVoltageCH2, MAX_OUTPUT_VOLTAGE, MAX_OUTPUT_SIGNAL));
  analogWrite(CHANNEL3_OUTPUT, computePWM(computedVoltageCH3, MAX_OUTPUT_VOLTAGE, MAX_OUTPUT_SIGNAL));
  analogWrite(CHANNEL3_OUTPUT, 255);
  
  Serial.print(String(ADC_TO_VOLTAGE_FACTOR * sensorValue) + " ");
  Serial.print(String(computedVoltageCH2) + " ");
  Serial.print(String(computePWM(computedVoltageCH2, MAX_OUTPUT_VOLTAGE, MAX_OUTPUT_SIGNAL)) + " ");
  Serial.println("");
  
  delay(500);
}

int computeOutput (int voltage){
  return 0;
}

//Esto va como un tiro si los valores estan bien ajusados
int computePWM (int voltaje, int maxVoltajeOut, int maxPwmValue){
  return voltaje / (float)(maxVoltajeOut / (float) maxPwmValue);
}

int computeVoltage(float controlLevel, int initPoint, int endPoint){
  //Serial.print("computeVoltage: " + String(controlLevel) + " Init: " + String(initPoint) + " End: " + String(endPoint) + " Calculated:");
  if(initPoint < endPoint){
    //Serial.print(String( (int)(((endPoint - initPoint) * controlLevel/100) + initPoint)) + "\n");
    return ((endPoint - initPoint) * controlLevel/100) + initPoint;
  }
  else {
    return (initPoint - ((initPoint - endPoint) * controlLevel/100));
  }
}

float computeInputStatus(int inputValue, int minValue, int maxValue){
  //Serial.println(value);
  float result = 100*((inputValue - minValue)/(float)(maxValue - minValue));
    if (result < 2.5){
      return 0;
    }
    if (result > 97.5){
      return 100;
    }
    return result;
}

float percentualVariation(int sensorValue, int lastSensorValue){
  Serial.println ("sensorValue: " + String(sensorValue));
  Serial.println ("lastSensorValue" + String(lastSensorValue));
  if (lastSensorValue == 0 && sensorValue == 0){
    return 0;
  }
  if (lastSensorValue == 0 || sensorValue == 0){
    return 100;
  }
  if (sensorValue > lastSensorValue){
    return ((sensorValue / (float)lastSensorValue) * 100);
  }
  else {
    return ((lastSensorValue / (float)sensorValue) * 100);
  }
}
