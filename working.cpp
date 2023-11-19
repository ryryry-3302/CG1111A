#include <MeMCore.h>

MeBuzzer buzzer;
MeLineFollower lineFinder(PORT_1);

#define RGBwait 100  //time taken for LDR to stabilise
unsigned long current_time = 0;

//music notes
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

//defining timings for the various challenge commands
#define TURNING_TIME_MS 340 
#define forward_blue 800
#define forward_purple 950
#define TIMEOUT 1100 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment


#define ULTRASONIC 10 // Ultrasonic Sensor Pin
#define BIT_A_ORANGE A2 // S1
#define BIT_B_YELLOW A3 // S2
#define LDR A0 //Pin of LDR
#define ir_receiver A1 // Pin of IR Receiver
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2


//initialise right and left distance compensation to prevent overturns on the bollards
int rcompensate = 0;
int lcompensate = 0;



//variables for IR sensor
double ambient, LeftInput, Output, ambient_without_ir;





double right_distance = 1; //distance from right wall
double left_distance = 0; //distance from left wall

//Standardise Motorspeeds
uint8_t motorSpeed = 255;
uint8_t lowSpeed  = motorSpeed - 150;




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
float whiteArray[] = { 32, 76, 49 };
float blackArray[] = { 19, 25, 19 };
float greyDiff[] = { 13, 51, 30 };

//function to geneate a pulse and measure distance from the right wall, returns 0 if timedout
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

    return distance;
    }
    else {

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
    buzzer.tone(e, 300);
    buzzer.tone(e, 300);
    buzzer.tone(f, 300);
    buzzer.tone(g, 300);
    buzzer.tone(g, 300);
    buzzer.tone(f, 300);
    buzzer.tone(e, 300);
    buzzer.tone(d, 300);
    buzzer.tone(c, 300);
    buzzer.tone(c, 300);
    buzzer.tone(d, 300);
    buzzer.tone(e, 300);
    buzzer.tone(e, 300);
    buzzer.tone(d, 300);
    buzzer.tone(d, 300);
    buzzer.noTone();
    delay(500);
    buzzer.tone(e, 300);
    buzzer.tone(e, 300);
    buzzer.tone(f, 300);
    buzzer.tone(g, 300);
    buzzer.tone(g, 300);
    buzzer.tone(f, 300);
    buzzer.tone(e, 300);
    buzzer.tone(d, 300);
    buzzer.tone(c, 300);
    buzzer.tone(c, 300);
    buzzer.tone(d, 300);
    buzzer.tone(e, 300);
    buzzer.tone(d, 300);
    buzzer.tone(c, 300);
    buzzer.tone(c, 300);
   
    
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

    }
void nudgeRight() {
    leftMotor.run(lowSpeed+30); 
    rightMotor.run(-motorSpeed);
    }
void nudgeLeft() {

    leftMotor.run(motorSpeed); 
    rightMotor.run(-lowSpeed-30); 

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

//function to turn on led based on called colour
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
     for (int i = 0; i < 3; i ++) {
              Serial.println(colourArray[i]);
              }
  //check for white
  if (r > 190 && g >190 && b >190){
    return white;
  }
  //check for green
  if (g > r && g > b){
    return green;
  }
  //check blue
  if (b > g && b > r){
    if (b - r > 85 ){
    return blue;
    }
    else
    return purple;
  }

  //check for purple
  if ( r > g && r > b) {
    if (r - b < 50) {
      return purple;
    }
  }

  //check for orange
  if (g -b> 25){
    return orange;
  }

  // check for red
  return red;
}

//function for callibrating the whiteArray, blackArray, greyDiff
void setBalance() {
  
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
  
}


//returns the differemce in analog reading of ir receiver of ambient light and reflected ir ray
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

//function to do challenege based on the colour detected
void challenge(int color){
  if (color == 0)
  {
    turnLeft();  
  }
  if (color == 1)
  {
    leftMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    delay(415); // Keep turning left for this time duration
    leftMotor.stop(); // Stop left motor
    rightMotor.stop();
  }
  if (color == 2) {
    leftMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    delay(415); // Keep turning left for this time duration
    leftMotor.stop(); // Stop left motor
    rightMotor.stop();
    stopMotor();
    delay(100);
    moveForward();
    delay(forward_blue-40);
    stopMotor();
    delay(100);
    leftMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    delay(415); // Keep turning left for this time duration
    leftMotor.stop(); // Stop left motor
    rightMotor.stop();
    stopMotor();
    delay(200);

  }

  if (color == 3){
    turnLeft();
    stopMotor();
    delay(500);
    turnLeft();
  }
  if (color == 4) {
  turnLeft();
    delay(100);
    moveForward();
    
    delay(forward_purple);
    stopMotor();
    delay(100);

    turnLeft();
    stopMotor();
    delay(200);   

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
shineRed();
delay(20);

//sets the target differential ir reading from the wall
ambient = ambient_ir();




}

void loop()
{
    //allows changing of status by pushing button
    if (analogRead(A7) < 100) { 
    status += 1; // Toggle status
    delay(500);
    buzzer.tone(e, 600);
    }

    if (status == 1){
      shineRed();
      delay(20);

      buzzer.tone(e, 600);
      status += 1;
      
    }
    int sensorState = lineFinder.readSensors();
    if(status >= 3){
    if (sensorState == S1_IN_S2_IN) //check if on black line
    
        { 
           
            stopMotor();
            
                rcompensate = 0;
                lcompensate = 0;
            delay(20);
            int colour = detectColour();
            Serial.println(colour);
            
            int orangeness = 0;
            if (colour == 3){
              orangeness += 1;
              while (orangeness <2 && orangeness != 0){
                colour = detectColour();
             
                if (orangeness == 1){
                challenge(colour);
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

    //executes moving algorithm if not on black line
    else {
        //accounts for over turning
        if (lcompensate >= 3){
          
          for (int i = 0; i <= 3; i++){
            nudgeLeft();
            delay(2);
            if (sensorState == S1_IN_S2_IN){
              stopMotor();
              lcompensate = 0;
              break;
            }
          }
          nudgeRight();
          delay(5);
          lcompensate = 0;
          moveForward();
        }
         if (rcompensate >= 3){
          for (int i = 0; i<=3; i++){
            nudgeRight();
            delay(2);

            if (sensorState == S1_IN_S2_IN){
              stopMotor();
              rcompensate = 0;
              break;
            }
          }
          nudgeLeft();
          delay(5);
          rcompensate = 0;
          moveForward();
        }
        

       // generates left and right distances
        LeftInput = ambient_ir();
        right_distance = gen_ultrasonic();

        //movement algorithm adjusts based on right wall if detected
        if (right_distance != 0)
        {   
            delay(20);

             right_distance = gen_ultrasonic();

            if (right_distance < 4.5)
            {
                nudgeLeft();
              
                lcompensate =0;
                rcompensate +=1;
                delay(20);
                moveForward();
            }
            else if (right_distance >5.9)
            {
                nudgeRight();
             

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

        //if ultrasonic times out and a Left Wall is readable and varies by more than 100 use IR to nudge
        if ((abs(ambient - LeftInput) > 100) && (LeftInput > 20) && right_distance == 0)
        {
          
          if(ambient > LeftInput){
          nudgeRight();
          //delay(20);
          
                rcompensate = 0;
                lcompensate = 0;
          }

       
        else{
          moveForward();
          
                rcompensate = 0;
                lcompensate = 0;
        }
        }
        // if both sensors do not detect a wall
        else 
        {
            moveForward();
          
                rcompensate = 0;
                lcompensate = 0;
        }
        
    }
  }
       

   
    
    


// Read IR sensing distance (turn off IR, read IR detector, turn on IR, read IR detector, estimate distance)
// if within black line, stop motor, detect colour, and take corresponding action
// else if too near to left wall, nudge right
// else if too near to right wall, nudge left
// else move forward
}