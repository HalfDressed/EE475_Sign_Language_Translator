const int FLEX_PIN1 = A0; // Pin connected to voltage divider output
const int FLEX_PIN2 = A1; // Pin connected to voltage divider output
const int FLEX_PIN3 = A2; // Pin connected to voltage divider output
const int FLEX_PIN4 = A3; // Pin connected to voltage divider output
const int FLEX_PIN5 = A4; // Pin connected to voltage divider output
const int SIDE_TILT_PIN = 2;
const int UP_TILT_PIN = 3;

// Global constants
int SAMPLES = 3;
int FINGERS = 5;
int LETTERS = 26;
int MIN_ERROR_DELTA = 10;

// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 47500.0; // Measured resistance of 3.3k resistor

// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
const float STRAIGHT_RESISTANCE [5] = {53523.38, 48154.52, 53945.73, 47407.23, 53314.31}; // Resistance when straight
const float BEND_RESISTANCE [5] = {99305.13, 139394.22, 123000.0, 126666.67, 136562.5}; // Resistance at 90 deg

// Contains all 5 sensor value (each finger) for every letter of the alphabet
// NOTES: 
//  G = Q
//  H = U = V
//  I = J 

int letterMatrix[26][5] = { 
  {19.00, 97.00, 89.00, 90.00, 95.00},    //A
  {52.00, 2.00, 0.00, 0.00, 3.00},        //B
  {22.00, 17.00, 26.00, 14.00, 4.00},     //C
  {15.00, 6.00, 44.00, 44.00, 39.00},     //D
  {75.00, 75.00, 86.00, 79.00, 64.00},    //E
  {13.00, 51.00, 6.00, 2.00, 5.00},       //F
  {3.00, 23.00, 99.00, 100.00, 104.00},   //G TODO: same as Q
  {26.00, 12.00, 6.00, 64.00, 57.00},     //H TODO: same as U and V
  {39.00, 87.00, 82.00, 79.00, 21.00},    //I TODO: same as J
  {39.00, 87.00, 82.00, 79.00, 21.00},    //J TODO: same as I
  {2.00, 6.00, 34.00, 78.00, 84.00},      //K
  {-7.00, 5.00, 100.00, 79.00, 90.00},    //L
  {59.00, 93.00, 60.00, 59.00, 66.00},    //M
  {36.00, 92.00, 58.00, 64.00, 67.00},    //N
  {14.00, 52.00, 39.00, 33.00, 24.00},    //O
  {0.00, 9.00, 24.00, 65.00, 72.00},      //P
  {3.00, 23.00, 99.00, 100.00, 104.00},   //Q TODO: same as G
  {32.00, 7.00, 4.00, 87.00, 84.00},      //R
  {43.00, 113.00, 114.00, 104.00, 114.00},//S
  {21.00, 73.00, 76.00, 70.00, 88.00},    //T
  {30.00, 4.00, 1.00, 71.00, 64.00},      //U TODO: same as H and V
  {34.00, 7.00, 0.00, 62.00, 64.00},      //V TODO: same as H and U
  {39.00, 8.00, 4.00, 2.00, 49.00},       //W
  {33.00, 42.00, 104.00, 92.00, 86.00},   //X
  {-2.00, 97.00, 61.00, 66.00, 16.00},    //Y
  {33.00, 12.00, 91.00, 85.00, 102.00},   //Z
};

int error[26][5] = {
  {15, 15,  15, 15, 15}, //A
  {15, 15,  15, 15, 15}, //B
  {15, 15,  15, 15, 15}, //C
  {15, 15,  15, 15, 15}, //D
  {15, 15,  15, 15, 15}, //E
  {15, 15,  15, 15, 15}, //F
  {15, 15,  15, 15, 15}, //G
  {15, 15,  15, 15, 15}, //H
  {15, 15,  15, 15, 15}, //I
  {15, 15,  15, 15, 15}, //J
  {15, 15,  15, 15, 15}, //K
  {15, 15,  15, 15, 15}, //L
  {15, 15,  15, 15, 15}, //M
  {15, 15,  15, 15, 15}, //N
  {15, 15,  15, 15, 15}, //O
  {15, 15,  15, 15, 15}, //P
  {15, 15,  15, 15, 15}, //Q
  {15, 15,  15, 15, 15}, //R
  {15, 15,  15, 15, 15}, //S
  {15, 15,  15, 15, 15}, //T
  {15, 15,  15, 15, 15}, //U
  {15, 15,  15, 15, 15}, //V
  {15, 15,  15, 15, 15}, //W
  {15, 15,  15, 15, 15}, //X
  {15, 15,  15, 15, 15}, //Y
  {15, 15,  15, 15, 15}, //Z
};

// Side and up tilt
// {LOW, HIGH} = up, {LOW, LOW} = down, {HIGH, LOW} = sideways
int letterTilt[26][2] = { 
  {LOW, HIGH}, //A
  {LOW, HIGH}, //B
  {LOW, HIGH}, //C
  {LOW, HIGH}, //D
  {LOW, HIGH}, //E
  {LOW, HIGH}, //F
  {HIGH, LOW}, //G
  {HIGH, LOW}, //H
  {LOW, HIGH}, //I
  {LOW, HIGH}, //J TODO: not needed?
  {LOW, HIGH}, //K
  {LOW, HIGH}, //L
  {LOW, HIGH}, //M
  {LOW, HIGH}, //N
  {LOW, HIGH}, //O
  {LOW, LOW}, //P
  {LOW, LOW}, //Q
  {LOW, HIGH}, //R
  {LOW, HIGH}, //S
  {LOW, HIGH}, //T
  {LOW, HIGH}, //U
  {LOW, HIGH}, //V
  {LOW, HIGH}, //W
  {LOW, HIGH}, //X
  {LOW, HIGH}, //Y
  {LOW, HIGH}, //Z TODO: not needed?
};


String sound = "ON";

void setup() 
{
  // Setup sensors
  Serial.begin(9600);
  pinMode(FLEX_PIN1, INPUT);
  Serial.begin(9600);
  pinMode(FLEX_PIN2, INPUT);
  Serial.begin(9600);
  pinMode(FLEX_PIN3, INPUT);
  Serial.begin(9600);
  pinMode(FLEX_PIN4, INPUT);
  Serial.begin(9600);
  pinMode(FLEX_PIN5, INPUT);

  // Greeting and determine mode. 
  Serial.println("Welcome to G.L.O.V.E ASL translator. Please select one of the following options:");
  Serial.print("Calibrate glove (c) or use default values (d): ");
  while (!Serial.available());    // is a character available? 
  char rx_byte = Serial.read();
  Serial.println(rx_byte);
  if(rx_byte == 'c'){ // Callibrate mode
    performCalibration();
    Serial.println();
    Serial.println("Calibrated LetterMatrix with sensor data: ");
    printMatrix(letterMatrix);
    
    Serial.println();
    Serial.println("Calibrated errorMatrix with sensor sample tolerance: ");
    printMatrix(error);
    Serial.println();
  }
  Serial.print("Sound is ON. Toggle sound with s");
}

void loop() 
{
  // Get commands
  char command_recieved = Serial.read();
  if(command_recieved == 's'){
    // Toggle the audio on and off
    if(sound == "ON"){
      sound = "OFF";
    }
    else {
      sound = "ON";
    }
    Serial.println("Sound is " + sound);
  }
  //Serial.println("OLD METHOD BELOW: ");
  //determineLetterOLD();

  Serial.println("NEW METTHOD BELOW: ");
  determineLetterNew();
}

void determineLetterOLD() {
   
  int angle1 = readFinger(FLEX_PIN1,1);
  int angle2 = readFinger(FLEX_PIN2,2);
  int angle3 = readFinger(FLEX_PIN3,3);
  int angle4 = readFinger(FLEX_PIN4,4);
  int angle5 = readFinger(FLEX_PIN5,5);
  Serial.println ("{" + String(angle1) + ", " + String(angle2) + ", " + String(angle3) + ", " + String(angle4) + ", " + String(angle5) + "}");

  delay(500);
  String message = "not found";
  for(int i = 0; i < 26; i++){
    int currDiff = 0;
    int lastDiff = 999999;
    int currAngle1 = letterMatrix[i][0];
    int currAngle2 = letterMatrix[i][1];
    int currAngle3 = letterMatrix[i][2];
    int currAngle4 = letterMatrix[i][3];
    int currAngle5 = letterMatrix[i][4];    //5 is the range. So value can be + or - 5 from actual to be correct, 
    //                                        we can change this value to have a higher or lower tolerance
    if(angle1<=currAngle1+error[i][0] && angle1>= currAngle1-error[i][0]){
       currDiff += abs(currAngle1-angle1);
       if(angle2<=currAngle2+error[i][1] && angle2>= currAngle2-error[i][1]){
          currDiff += abs(currAngle2-angle2);
          if(angle3<=currAngle3+error[i][2] && angle3>= currAngle3-error[i][2]){
              currDiff += abs(currAngle3-angle3);
              if(angle4<=currAngle4+error[i][3]&& angle4>= currAngle4-error[i][3]){
                  currDiff += abs(currAngle4-angle4);
                  if(angle5<=currAngle5+error[i][4] && angle5>= currAngle5-error[i][4]){
                      currDiff += abs(currAngle5-angle5);
                      char currLetter;
                        if(currDiff < lastDiff){
                          currLetter = i + 65;
                          lastDiff = currDiff;
                          Serial.println("\t changed letter");
                        }
                         message = "letter = ";
                         Serial.print(message);
                         Serial.println(currLetter);
                          
                  }
             }
          }
       }
    }
  }

  Serial.println(message);
}

void determineLetterNew() {
  // Tilt switches
  int handTiltSide = digitalRead(SIDE_TILT_PIN);
  int handTiltUp = digitalRead(UP_TILT_PIN);

  // Collect samples
  int sensorReadings [FINGERS];
  for (int finger = 0; finger < FINGERS; finger++) {
    sensorReadings[finger] = readFingerByIndex(finger);
  }

  // Find best match
  int bestLetter = -1;
  int bestConfidence = -1;
  for (int letter = 0; letter < LETTERS; letter++) {
    // Check tilt
    if(handTiltSide != letterTilt[letter][0] || handTiltUp != letterTilt[letter][1]){
      continue;
    }
    
    int confidence = compareLetterHand(letter, sensorReadings);
    Serial.println("\t confidence for " + String((char) (65 + letter)) + " is " + String(confidence));
    if (confidence >= bestConfidence) {
      bestLetter = letter;
      bestConfidence = confidence;
    }
  }

  Serial.println(String(bestConfidence) + "% confident that the letter is " + String((char) bestLetter + 65));
}

// 76% confident the letter is A

void performCalibration() {  
  Serial.println("We are beginning the calibration process for the ASL alphabet.");
  Serial.println();
  
  for(int letter = 0; letter < LETTERS; letter++){
    char curr = letter + 65; // unicode A - Z;    
    int finger_samples [FINGERS][SAMPLES];
    int finger_error_delta [FINGERS];

    Serial.print("Calibrating letter: " + String(curr));
    for (int j = 0; j < SAMPLES; j++) {
      Serial.println("\t Waiting until hand is flat...");
      waitUntilHandFlat();
      Serial.print("\t Sampling letter in ");
      for (int k = 3; k > 0; k--) {
        Serial.print(String(k));
        Serial.print("...");
        delay(750); // ms delay 1000ms = 1s
      }
      Serial.print("0... ");
      for (int k = 0; k < FINGERS; k++) {
        finger_samples[k][j] = readFingerByIndex(k);
      }
      Serial.println("Completed!");
      Serial.println();
    }
    
    // Average all samples and store
    for (int finger = 0; finger < FINGERS; finger++) {
      int sum = 0;
      for (int sample = 0; sample < SAMPLES; sample++) {
        sum += finger_samples[finger][sample];
      }
      int average = sum / SAMPLES;
      letterMatrix[letter][finger] = average;
    }
    
    // Calculate error delta for each finger of every letter.
    for (int finger = 0; finger < FINGERS; finger++) {
      int minVal, maxVal;
      for (int sample = 0; sample < SAMPLES; sample++) {
        int currentSample = finger_samples[finger][sample];
        if (sample == 0) { // First sample is automatically max and min
          minVal = currentSample;
          maxVal = currentSample;
        } else { // Update as necessary
          if (currentSample < minVal) { // Set new min
            minVal = currentSample;
          }
          if (currentSample > maxVal) { // Set new max
            maxVal = currentSample;
          }
        }
      } // End samples
      // Store error with atleast MIN_ERROR_DELTA value. 
      int curError = abs(maxVal - minVal);
      if (curError < MIN_ERROR_DELTA) { // Ensures a minimum error tolerance
        curError = MIN_ERROR_DELTA;
      }
      error[letter][finger] = curError;
    }

    // DEBUG ************************
    Serial.println("letterMatrix:");
    printMatrixRow(letter, letterMatrix); // Print stored values for each letter avg
    Serial.println("errorMatrix:");
    printMatrixRow(letter, error); // Print stored values for each letter error
  }
}

// Prints matrix to be easily copy and pasted into code. 
void printMatrix (int data[26][5]) {
  Serial.println("{");
  for (int row = 0; row < 26; row++) {
    Serial.print("{");
    for (int column = 0; column < 5; column++) {
      Serial.print(String(data[row][column]));
      if (column != 4) {
        Serial.print(", ");
      }
    }
    Serial.println("},\t\t //" + String((char) (65 + row)));
  }
  Serial.println("};");
}

// Print row of matrix
void printMatrixRow (int row, int data[26][5]) {
  Serial.print("{");
  for (int column = 0; column < 5; column++) {
    Serial.print(String(data[row][column]));
    if (column != 4) {
      Serial.print(", ");
    }
  }
}

// 0 based indexing from right most finger to left most finger. 
// Finger 0 = thumb .. finger 4 = pinky.
int readFingerByIndex (int finger) {
  if (finger == 0) {
    return readFinger(FLEX_PIN1, 1);
  } else if (finger == 1) {
    return readFinger(FLEX_PIN2, 2);
  } else if (finger == 2) {
    return readFinger(FLEX_PIN3, 3);
  } else if (finger == 3) {
    return readFinger(FLEX_PIN4, 4);
  } else if (finger == 4) {
    return readFinger(FLEX_PIN5, 5);
  }
}

void waitUntilHandFlat() {
  int flatFingers = 0;
  while (flatFingers != 4) {
    if (readFingerByIndex(flatFingers) <= 20) {
      flatFingers += 1;
    } else {
      flatFingers = 0; // finger wasn't flat so restart.
    }
  }
}

// Returns confidence 0-100 of reading matching letter based on LetterMatrix and error values
int compareLetterHand (int letter, int reading[5]) {
  float confidence = 0;
  for (int finger = 0; finger < FINGERS; finger++) {
    float curError    = (float) error[letter][finger];
    float actual      = (float) reading[finger];
    float expected    = (float) letterMatrix[letter][finger];
    float delta       = (float) abs(actual - expected);
    
    if (curError < delta) {
      confidence += 20; // Max confidence
    } else if (curError >= delta && curError <= (delta*2)) {
      confidence += (int) ((20.0 - 20 * (curError / (delta*2) - 0.5)));
    } // 0 confidence in match
  }
  return (int) confidence;
}


int readFinger(const int FLEX_PIN, int fingerNumber){
  //Serial.print("Value read from: " + String(fingerNumber));
  float flexADC1 = analogRead(FLEX_PIN);
  float flexV1 = flexADC1 * VCC / 1023.0;
  float flexR1 = R_DIV * (VCC / flexV1 - 1.0);
  //Serial.print(" Resistance: " + String(flexR1) + " ohms");

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  float angle1 = map(flexR1, STRAIGHT_RESISTANCE[fingerNumber - 1], BEND_RESISTANCE[fingerNumber - 1],
                   0, 90.0);
return (int) angle1;
}
