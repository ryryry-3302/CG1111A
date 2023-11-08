#include <MeMCore.h>
#include <PID_v1.h>
MeBuzzer buzzer;
MeLineFollower lineFinder(PORT_1);

#define RGBwait 300  //time taken for LDR to stabilise
unsigned long current_time = 0;


int e = 329;
int f = 349;
int g = 370;
int d = 277;
int c = 247;
int status = 0;
//colour indexes according to list
#define red 0
#define green 1
#define blue 2
#define orange 3
#define purple 4
#define white 5

#define TURNING_TIME_MS 400.5 // The time duration (ms) for turning

#define TIMEOUT 1200 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define ULTRASONIC 10 // Ultrasonic Sensor Pin

#define BIT_A_ORANGE A2 // S1
#define BIT_B_YELLOW A3 // S2

#define LDR A0 //Pin of LDR
#define ir_receiver A1 // Pin of IR Receiver

int rcompensate = 0;
int lcompensate = 0;



//Define Variables we'll be connecting to
double ambient, LeftInput, Output, ambient_without_ir;

//Specify the links and initial tuning parameters
double Kp=1, Ki=0.2, Kd=0;

PID leftPID(&LeftInput, &Output, &ambient, Kp, Ki, Kd, DIRECT);


double right_distance = 1; //distance from right wall
double left_distance = 0; //distance from left wall

MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
uint8_t motorSpeed = 255;
uint8_t lowSpeed  = motorSpeed - 150;


//list of rgb values of colours, update more accurate values aft calibration
float colourList[6][3] = 
{
  { 182, 110, 121 },
  { 60, 121, 85 },
  { 97, 166, 145 },
  { 182,133, 109 },
  { 109, 110 ,121 },
  { 225, 225, 223}
};

String colourName[6] = 
{
    "red",
    "green",
    "blue",
    "orange",
    "purple",
    "white"
};

//floats to hold colour arrays
float colourArray[] = { 0, 0, 0 };
float whiteArray[] = { 22, 25, 22 };
float blackArray[] = { 1, 2, 2 };
float greyDiff[] = { 21, 23, 20 };


double gen_ultrasonic() {
    pinMode(ULTRASONIC, OUTPUT);
    digitalWrite(ULTRASONIC, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC, LOW);
    pinMode(ULTRASONIC, INPUT);
    long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
    if (duration > 0) {
    double distance = duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100 - 3.40;
   // Serial.print("distance(cm) = ");
   // Serial.println(distance);
    return distance;
    }
    else {
   //  Serial.println("out of range");
    return 0;
    }
}

void celebrate() {
    // Code for playing celebratory tune
    int e = 329;
    int f = 349;
    int g = 370;
    int d = 277;
    int c = 247;
    buzzer.tone(e, 600);
   
    
}
void stopMotor() {
    // Code for stopping motor
    leftMotor.stop(); // Stop left motor
    rightMotor.stop();
}
void moveForward() {
    // Code for moving forward for some short interval
    leftMotor.run(motorSpeed); // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed);
}
void turnRight() {
    // Code for turning right 90 deg
    leftMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS); // Keep turning left for this time duration
    leftMotor.stop(); // Stop left motor
    rightMotor.stop();
}
void turnLeft() {
    // Code for turning left 90 deg
    leftMotor.run(motorSpeed); // Positive: wheel turns clockwise
    rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS); // Keep turning left for this time duration
    leftMotor.stop(); // Stop left motor
    rightMotor.stop(); // Stop right motor
    }
void uTurn() {
    // Code for u-turn
    turnLeft();
    delay(1500);
    turnLeft();
    delay(1500);
    leftMotor.stop(); // Stop left motor
    rightMotor.stop(); 
}
void doubleLeftTurn() {
    }
void doubleRightTurn() {
    }
void nudgeRight() {
    leftMotor.run(lowSpeed); 
    rightMotor.run(-motorSpeed);
    }
void nudgeLeft() {

    leftMotor.run(motorSpeed); 
    rightMotor.run(-lowSpeed); 

}
void shineIR() {
    // Code for turning on the IR emitter only
    digitalWrite(BIT_A_ORANGE, HIGH);
    digitalWrite(BIT_B_YELLOW, HIGH);
}
void shineRed() {
    // Code for turning on the red LED only
    digitalWrite(BIT_A_ORANGE, LOW); 
    digitalWrite(BIT_B_YELLOW, LOW); 
}
void shineGreen() {
    // Code for turning on the green LED only
    digitalWrite(BIT_A_ORANGE, LOW); 
    digitalWrite(BIT_B_YELLOW, HIGH);
}
void shineBlue() {
    // Code for turning on the blue LED only
    digitalWrite(BIT_A_ORANGE, HIGH); 
    digitalWrite(BIT_B_YELLOW, LOW);
}

void shine(int c) {
  if (c == red) {
    shineRed();
  } else if (c == green) {
    shineGreen();
  } else {
    shineBlue();
  }
}

int getAvgReading(int times) {
  //find the average reading for the requested number of times of scanning LDR
  int reading;
  int total = 0;
  //take the reading as many times as requested and add them up
  for (int i = 0; i < times; i++) {
    reading = analogRead(LDR);
    total = reading + total;
    delay(10);  //LDRwait delay before another ldr reading
  }
  //calculate the average and return it
  return total / times;
}

int detectColour() {
  for (int c = 0; c < 3; c++) {  //red then green then blue
    shine(c);                    //turn on current led
    delay(RGBwait);
    colourArray[c] = getAvgReading(10);                                        // read ldr avg values for current led
    colourArray[c] = (colourArray[c] - blackArray[c]) / (greyDiff[c]) * 255;  //calc rgb value
    delay(RGBwait);
  }

  // colour identification
  int r = colourArray[red];
  int g = colourArray[green];
  int b = colourArray[blue];
  if (r > 145) 
  {
    if (g >200) 
    {
      return white;
    } 
    
    if (g >60 ) 
    {
      return orange;
    }
    return red;
  } 
  if(g > 110) 
  {
    return blue;
  }
  if (b < 95) 
  {
    return green;
  }
  return purple;
}

void setBalance() {
  //set white balance
  //Serial.println("Put White Sample For Calibration ...");
  delay(3000);  //delay for five seconds for getting sample ready
  //scan the white sample.
  //go through one colour at a time, set the maximum reading for each colour to the white array
  for (int i = 0; i <= 2; i++) {
    shine(i);
    delay(RGBwait);
    whiteArray[i] = getAvgReading(5);  //scan 5 times and return the average,
    delay(RGBwait);
  }

  //done scanning white, time for the black sample.
  //set black balance
  buzzer.tone(e, 600);
  
  //Serial.println("Put Black Sample For Calibration ...");
  delay(3000);  //delay for five seconds for getting sample ready
  //go through one colour at a time, set the minimum reading for red, green and blue to the black array
  for (int i = 0; i <= 2; i++) {
    shine(i);
    delay(RGBwait);
    blackArray[i] = getAvgReading(5);
    delay(RGBwait);

    //the differnce between the maximum and the minimum gives the range
    greyDiff[i] = whiteArray[i] - blackArray[i];
  }

  //delay another 5 seconds for getting ready colour objects
  //Serial.println("Colour Sensor Is Ready.");
  buzzer.tone(e, 600);
  buzzer.tone(e, 600);
  delay(2000);
}


double ambient_ir(){
  double voltage = 0;

  if(millis() > current_time + 30)
  { 
    ambient_without_ir = analogRead(ir_receiver);
    shineIR();
  }
  
  if (millis() > current_time + 60){
    current_time=millis();
    double temp = 0;
    for (int i=0; i<5; i++){
    temp += analogRead(ir_receiver);
    voltage = ambient_without_ir - temp / 5;
    shineRed();
  }
  
  }
   
  return voltage;
}


void challenge(int color){
  if (color == 0)
  {
    turnLeft();  
  }
  if (color == 1)
  {
    turnRight();
  }
  if (color == 2) {
    turnRight();
    stopMotor();
    moveForward();
    delay(750);
    turnRight();    

  }

  if (color == 3){
    turnLeft();
    stopMotor();
    delay(1000);
    turnLeft();
  }
  if (color == 4) {
    turnLeft();
    stopMotor();
    moveForward();
    delay(750);
    turnLeft();    

  }
  if (color == 5){
    celebrate();
  }
}

void setup()
{
// Configure pinMode for A0, A1, A2, A3
Serial.begin(9600); // to initialize the serial monitor
pinMode(BIT_A_ORANGE, OUTPUT);
pinMode(BIT_B_YELLOW, OUTPUT);
pinMode(LDR, INPUT);



setBalance(); //calibrate colour sensor with white and black
  
  //print calibrated values
  for (int i = 0; i < 3; i ++) {
    Serial.println(whiteArray[i]);
  }
 for (int i = 0; i < 3; i ++) {
    Serial.println(blackArray[i]);
  }
  for (int i = 0; i < 3; i ++) {
    Serial.println(greyDiff[i]);
  } 
 
  Serial.print("ambient");
  Serial.println(ambient);
}

void loop()
{
    if (analogRead(A7) < 100) { // If push button is pushed, the value will be very low
    status += 1; // Toggle status
    delay(500);
    buzzer.tone(e, 600);

   
    }
    if (status == 1){
      shineRed();
      delay(20);
      ambient = ambient_ir();
      buzzer.tone(e, 600);
      status += 1;
      
    }
    int sensorState = lineFinder.readSensors();
    if(status == 3){
    if (sensorState == S1_IN_S2_IN) //check if on black line
        { 
           
            stopMotor();
            delay(100);
            int colour = detectColour();
            for (int i = 0; i < 3; i ++) {
              Serial.println(colourArray[i]);
            }
            int orangeness = 0;
            if (colour == 3){
              orangeness += 1;
              while (orangeness <4 && orangeness != 0){
                colour = detectColour();
                if (orangeness == 3){
                  challenge(3);
                }
                if (colour == 3){
                  orangeness +=1;
                }
                else
                {
                  orangeness = 0;
                }
              }
            }
            if (colour != 3){
            challenge(colour);
            }
        }
    else {

        if (lcompensate >= 100){
          
          for (int i = 0; i <= 20; i++){
            nudgeLeft();
            delay(20);
            if (sensorState == S1_IN_S2_IN){
              stopMotor();
              lcompensate = 0;
              break;
            }
          }
          lcompensate = 0;
          moveForward();
        }
         if (rcompensate >= 100){
          for (int i = 0; i<=20; i++){
            nudgeRight();
            delay(20);

            if (sensorState == S1_IN_S2_IN){
              stopMotor();
              rcompensate = 0;
              break;
            }
          }
          rcompensate = 0;
          moveForward();
        }
        

       
        LeftInput = ambient_ir();
        right_distance = gen_ultrasonic();
        if (right_distance != 0)
        {   
            Serial.println(right_distance);
            if (right_distance <5.1)
            {
                nudgeLeft();
                //Serial.println("nudging left");
                lcompensate =0;
                rcompensate +=1;
                delay(20);
                moveForward();
            }
            else if (right_distance >5.9)
            {
                nudgeRight();
              //  Serial.println("nudging right");

                rcompensate = 0;
                lcompensate +=1;
                delay(20);
                moveForward();
            }
            else
            {   
                rcompensate = 0;
                lcompensate = 0;
                moveForward();
                delay(20);
                
            }    
        }
        

        if ((ambient - LeftInput > 120) && (LeftInput != 0) && right_distance == 0)
        {
          
          if(ambient - LeftInput > 140){
          Serial.println("nudgeRight");
          nudgeRight();
          delay(20);
          }
          else if (ambient - LeftInput < 135){
          nudgeLeft();
          delay(20);
          }
       
        else{
          leftMotor.run(lowSpeed); // Positive: wheel turns clockwise
          rightMotor.run(-lowSpeed);
        }
        }
        else 
        {
          leftMotor.run(lowSpeed); // Positive: wheel turns clockwise
          rightMotor.run(-lowSpeed);
        }
        
    }
  }
       

   
    
    


// Read IR sensing distance (turn off IR, read IR detector, turn on IR, read IR detector, estimate distance)
// if within black line, stop motor, detect colour, and take corresponding action
// else if too near to left wall, nudge right
// else if too near to right wall, nudge left
// else move forward
}