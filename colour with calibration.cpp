#define RGBwait 200  //time taken for LDR to stabilise

//colour indexes according to list
#define red 0
#define green 1
#define blue 2
#define orange 3
#define purple 4
#define white 5

//list of rgb values of colours, update more accurate values aft calibration
float colourList[5][3] = {
  { 182, 110, 121 },
  { 60, 121, 85 },
  { 97, 166, 145 },
  { 182,133, 109 },
  { 109, 110 ,121 },
  { 225, 225, 223}
};

//floats to hold colour arrays
float colourArray[] = { 0, 0, 0 };
float whiteArray[] = { 22, 25, 22 };
float blackArray[] = { 1, 2, 2 };
float greyDiff[] = { 21, 23, 20 };

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
    colourArray[c] = getAvgReading(5);                                        // read ldr avg values for current led
    colourArray[c] = (colourArray[c] - blackArray[c]) / (greyDiff[c]) * 255;  //calc rgb value
    delay(RGBwait);
  }

  // colour identification
  int r = colourArray[red];
  int g = colourArray[green];
  int b = colourArray[blue];
  if (r > 145) {
    if (g < 160) {
      return red;
    } else if (b < 150) {
      return orange;
    }
    return white;
  } else if (g > 200) {
    return blue;
  } else if (g > 100) {
    return green;
  }
  return purple;
}

void setBalance() {
  //set white balance
  Serial.println("Put White Sample For Calibration ...");
  delay(5000);  //delay for five seconds for getting sample ready
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
  Serial.println("Put Black Sample For Calibration ...");
  delay(5000);  //delay for five seconds for getting sample ready
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
  Serial.println("Colour Sensor Is Ready.");
  delay(5000);
}

void setup() {
  // Configure pinMode for A0, A1, A2, A3
  Serial.begin(9600);  // to initialize the serial monitor
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
}

void loop() {
  Serial.println(detectColour());
  for (int i = 0; i < 3; i ++) {
    Serial.println(colourArray[i]);
  }
}
