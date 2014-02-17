#include "Makeblock.h"
#include <SoftwareSerial.h>
#include <Wire.h>
MeSerial serial;
//MeDCMotor motor1(M1);
//MeDCMotor motor2(M2);
//MeDCMotor motor3(PORT_1);
//MeDCMotor motor4(PORT_2);
MeUltrasonicSensor us;
MeLimitSwitch ls;
char device[20];
char *method;
int port;
int value;
int slot;
int ultrasonic_port=-1;
int switch_port=-1;
int switch_slot=1;

void setup() {
  serial.begin(9600);
  serial.println("application start");
  us = MeUltrasonicSensor(3);
  ls = MeLimitSwitch(6,1);
}
void loop() {
  if(serial.paramAvailable()){
    
    strcpy(device,serial.getParamCode("device"));
    port = serial.getParamValue("port");
    value = serial.getParamValue("value");
    slot = serial.getParamValue("slot");
    method = serial.getParamCode("method");
//    if(strcmp(device,"motor")==0){
//      switch(port){
//        case M1:{
//         motor1.run(value); 
//        }
//        break;
//        case M2:{
//         motor2.run(value); 
//        }
//        break;
//        case PORT_1:{
//         motor3.run(value); 
//        }
//        break;
//        case PORT_2:{
//         motor4.run(value); 
//        }
//        break;
//      }
//    }
    if(strcmp(device,"Ultrasonic")==0){
      if(strcmp(method,"add")==0){
        
        ultrasonic_port = port;
        us.reset(port);
      }
    }
    if(strcmp(device,"LimitSwitch")==0){
      if(strcmp(method,"add")==0){
          switch_port = port;
          switch_slot = slot;
          ls.reset(port,slot);
      }
    }
    if(strcmp(device,"poll")==0){
      if(ultrasonic_port>-1){
        serial.print("Ultrasonic/Port");
        serial.print(ultrasonic_port);
        serial.print(" ");
        serial.println(us.distanceCm());
      } 
      if(switch_port>-1){
        serial.print("LimitSwitch/Port");
        serial.print(switch_port);
        serial.print("/Slot");
        serial.print(switch_slot);
        serial.print(" ");
        serial.println(ls.touched()?"false":"true");
      }
    }
  }
}
