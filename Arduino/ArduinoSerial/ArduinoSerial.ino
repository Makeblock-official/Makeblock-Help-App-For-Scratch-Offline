#include "Makeblock.h"
#include <SoftwareSerial.h>
#include <Wire.h>
MeSerial serial;
MeDCMotor motor;
MeServo servo;
MeUltrasonicSensor ultrasonic;
MeLineFinder linefinder;
MeLimitSwitch limitswitch;
MeJoystick joystick;
MeSoundSensor soundsensor;
MeLightSensor lightsensor;
Me4Button button;
MeTemperature temperature;
char *device;
char *method;
int port;
char *pin;
int value;
int slot;
int ultrasonic_port=-1;
int switch_port=-1;
int switch_slot=1;
typedef struct{
  int type;
  int mode;
  int slot[2];
}Module;
Module ports[10];
Module pins[18];
int analogPins[6] = {A0,A1,A2,A3,A4,A5};
void addDevice(){
  if(port>0){
    int index = port-1;
    if(strcmp(device,"motor")==0){
      ports[index].type=1;
    }else if(strcmp(device,"servo")==0){
      ports[index].type=2;
      ports[index].slot[slot-1]=1;
    }else if(strcmp(device,"Ultrasonic")==0){
      ports[index].type=3;
    }else if(strcmp(device,"Line%20Finder")==0){
      ports[index].type=4;
    }else if(strcmp(device,"LimitSwitch")==0){
      ports[index].type=5;
      ports[index].slot[slot-1]=1;
    }else if(strcmp(device,"Light%20Sensor")==0){
      ports[index].type=6;
    }else if(strcmp(device,"Sound%20Sensor")==0){
      ports[index].type=7;
    }else if(strcmp(device,"Temperature")==0){
      ports[index].type=8;
      ports[index].slot[slot-1]=1;
    }else if(strcmp(device,"Joystick")==0){
      ports[index].type=9;
    }else if(strcmp(device,"Button")==0){
      ports[index].type=10;
    }
  }else if(pin>0){
    
  }
}
void runDevice(){
  if(port>0){
    int index = port-1;
    Module portModule = ports[index];
    if(strcmp(device,"motor")==0){
      motor.reset(port);
      motor.run(value);
    }else if(strcmp(device,"servo")==0){
      servo.reset(port,slot);
      servo.begin();
      servo.write(value);
      delay(200);
      servo.detach();
    }
  }else if(pin>0){
    
  }
}
void checkDevice(){
  int i,portNum;
  for(i=0;i<10;i++){
    portNum = i+1;
    if(ports[i].type>0){
      switch(ports[i].type){
        case 3:{
          ultrasonic.reset(portNum);
          int dist = ultrasonic.distanceCm();
          serial.print("Ultrasonic/Port");
          serial.print(portNum);
          serial.print(" ");
          serial.println(dist);
        }
        break;
        case 4:{
          linefinder.reset(portNum);
          int state = linefinder.readSensors();
          serial.print("Line%20Finder/Port");
          serial.print(portNum);
          serial.print(" ");
          serial.println(state);
        }
        break;
        case 5:{
          limitswitch.reset(portNum,ports[i].slot[0]==1?1:2);
          int touched = limitswitch.touched();
          serial.print("LimitSwitch/Port");
          serial.print(portNum);
          serial.print("/Slot");
          serial.print(ports[i].slot[0]==1?1:2);
          serial.print(" ");
          serial.println(touched==1?"true":"false");
        }
        break;
        case 6:{
          lightsensor.reset(portNum);
          int light = lightsensor.strength();
          serial.print("LightSensor/Port");
          serial.print(portNum);
          serial.print(" ");
          serial.println(light);
        }
        break;
        case 7:{
          soundsensor.reset(portNum);
          int sound = soundsensor.strength();
          serial.print("SoundSensor/Port");
          serial.print(portNum);
          serial.print(" ");
          serial.println(sound);
        }
        break;
        case 8:{
          temperature.reset(portNum,ports[i].slot[0]==1?1:2);
          float t = temperature.temperature();
          if(t==-100){
            t = temperature.temperature();
          }
          if(t>-100){
            serial.print("Temperature/Port");
            serial.print(portNum);
            serial.print("/Slot");
            serial.print(ports[i].slot[0]==1?1:2);
            serial.print(" ");
            serial.println(t);
          }
        }
        break;
        case 9:{
          joystick.reset(portNum);
          int dist = joystick.readX();
          serial.print("Joystick/Port");
          serial.print(portNum);
          serial.print(ports[i].slot[0]==1?"/X-Axis":"/Y-Axis");
          serial.print(" ");
          serial.println(dist);
        }
        break;
        case 10:{
          button.reset(portNum);
          int state = button.pressed();
          serial.print("Button/Port");
          serial.print(portNum);
          serial.print(" ");
          serial.println(state);
        }
        break;
      }
    }
  }
  for(i=0;i<18;i++){
    if(pins[i].type==1){
      serial.print("digital/D");
      serial.print(i+2);
      serial.print(" ");
      serial.println(digitalRead(i+2)?"true":"false");
    }else if(pins[i].type==2){
      serial.print("analog/A");
      serial.print(i-12);
      serial.print(" ");
      serial.println(analogRead(analogPins[i-12]));
    }
  }
}
void setPinMode(){
  if(value==0){
    return;
  }
  if(pin[0]=='D'){
    pin[0]='0';
    int p = atoi(pin);
    pinMode(p,value==2?OUTPUT:INPUT);
    if(value==1){
      pins[p-2].type = 1;
    }else{
      pins[p-2].type = 0;
    }
  }else if(pin[0]=='A'){
    pin[0]='0';
    int p = atoi(pin);
    pinMode(analogPins[p],value==2?OUTPUT:INPUT);
    if(value==1){
      pins[p+12].type = 2;
    }else{
      pins[p+12].type = 0;
    }
  }
}
void pinWrite(){
  if(pin[0]=='D'){
    pin[0]='0';
    digitalWrite(atoi(pin),value==2?HIGH:LOW);
  }else if(pin[0]=='A'){
    pin[0]='0';
    digitalWrite(analogPins[atoi(pin)],value==2?HIGH:LOW);
  }
}

long currentTime = 0;
long sampleTime = 100;
void setup() {
  serial.begin(115200);
  while (!Serial) ;
  serial.println("Application Start");
}
void loop() {
  long time = millis()-currentTime;
  if(time>sampleTime||time<0){
    currentTime = millis();
    checkDevice();
  }
  if(serial.paramAvailable()){
    device = serial.getParamCode("device");
    port = serial.getParamValue("port");
    pin = serial.getParamCode("pin");
    value = serial.getParamValue("value");
    slot = serial.getParamValue("slot");
    method = serial.getParamCode("method");
    if(strcmp(device,"reset")==0){
      int i;
      for(i=0;i<10;i++){
         ports[i].type = 0;
         ports[i].slot[0] = 0;
         ports[i].slot[1] = 0;
      }
      for(i=0;i<18;i++){
         pins[i].type = 0;
         pins[i].slot[0] = 0;
         pins[i].slot[1] = 0;
      }
      motor.stop();
      return;
    }
    if(strcmp(method,"add")==0){
     addDevice(); 
     return;
    }else if(strcmp(method,"run")==0){
     runDevice(); 
     return;
    }else if(strcmp(method,"mode")==0){
     setPinMode(); 
     return;
    }else if(strcmp(method,"write")==0){
     pinWrite(); 
     return;
    }
  }
}
