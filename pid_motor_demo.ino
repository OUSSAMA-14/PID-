

      //semple PID motor controller demo  
      // by Ayad oussama and bendjfel ilyes

      
const int analogInPin = A0; // Analog input pin that the potentiometer sensor is attached to
const int analogInPin1 = A1;  // Analog input pin that the potentiometer for control is attached to


long outputValue,sensorValue,control;        // value read from the pot
long m,error;   



int motorInput1 = 2;
int motorInput2 = 3;

long d,i,p;
long Kp,Ki,Kd;
int sensor,setPoint =80;

long pid,lastError;

  void cw()
  {
    
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2,HIGH );

  }
  void acw(){

  
  digitalWrite(motorInput1,HIGH );
  digitalWrite(motorInput2,LOW );


  }


void setup() {
   Serial.begin(9600);
   pinMode(motorInput1, OUTPUT);
   pinMode(motorInput2, OUTPUT);

  p = 0;
  i = 0;
  d = 0;

  }

void loop() {
  
 Kp = 500;           //the bes 500,10,550 -------/4 some test value
 Ki =  5;            //10  -------- 
 Kd =   550 ;        //3500000    /512
 
control= analogRead(analogInPin1) - 512;           //get actual servo position, range -512 to +511
long  setvalue = map(control, 0, 1023, 0, 255);    //map it to the range of the analog out: 

sensor= analogRead(analogInPin) - 512;             //get actual servo position, range -512 to +511
long  outputValue = map(sensor, 0, 1023, 0, 255);  //map it to the range of the analog out: 
 
error=outputValue-setvalue;
p = error ;                     // Proportional is just the error
i = i + error;                  // Integral
d = error  - lastError ;        // error differential 
lastError = error;              // Save last error for next loop

pid = ((Kp * p)/4) + (Ki * i) + ((Kd * d)/1) ;  // Do PID 


if (pid >=0 ){  
  cw();
  //delay(1);
  }
 else  {
 acw();
// delay(1);
}





//   sensorValue = analogRead(analogInPin);

 
 

//   change the analog out value:

  analogWrite(analogOutPin, outputValue);

// print the results to the serial monitor:
Serial.print("setvalue = ");
Serial.print(setvalue);

Serial.print("    outputValue = ");
Serial.print(outputValue);

Serial.print("\t error = ");
Serial.println(error);

//Serial.println(analogRead(A0));

// wait 2 milliseconds before the next loop
// for the analog-to-digital converter to settle
// after the last reading:
//delay(1);

//Here is a simple software loop that implements a PID algorithm directly from wikipidia 
//
//Kp - proportional gain
//Ki - integral gain
//Kd - derivative gain
//dt - loop interval time
//previous_error := 0
//integral := 0
//
//loop:
//    error := setpoint − measured_value
//    proportional := error;
//    integral := integral + error × dt
//    derivative := (error − previous_error) / dt
//    output := Kp × proportional + Ki × integral + Kd × derivative
//    previous_error := error
//    wait(dt)
//    goto loop
 
}
