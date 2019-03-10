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
int MIN_ERROR_DELTA = 5; // Ensures some range of motion. Enforced when determining confidence. 
int TILT_CONFIDENCE = 25; // +/- amount that is added if BOTH tilt sensors are correct


// Measured voltage of arduino 5V
const float VCC = 4.98; // Measured voltage of Ardunio 5V line

// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
const float STRAIGHT_RESISTANCE [5] = {53523.38, 48154.52, 53945.73, 47407.23, 53314.31}; // Resistance when straight
const float BEND_RESISTANCE [5]     = {99305.13, 139394.22, 123000.0, 126666.67, 136562.5}; // Resistance at 90 deg
const float R_DIV [5]                  = {47500.0, 47500.0, 47500.0, 47500.0, 47500.0}; // Measured resistance of each fingers resistor
// Contains all 5 sensor value (each finger) for every letter of the alphabet
// NOTES: 
//  G = Q
//  H = U = V
//  I = J 
// 0 == Thumb, 4== pink
int letterMatrix[26][5] = {
  {25, 126, 116, 116, 128},    //A
  {53, 4, 2, 3, 4},    //B
  {42, 51, 52, 55, 35},    //C
  {35, 7, 61, 67, 62},     //D
  {72, 89, 87, 73, 69},    //E
  {34, 67, 9, 5, 1},     //F
  {4, 38, 117, 124, 127},    //G
  {63, 15, 7, 78, 86},     //H
  {28, 89, 59, 60, 7},     //I
  {27, 90, 64, 62, 15},    //J
  {6, 21, 38, 95, 112},    //K
  {-5, 5, 128, 111, 128},    //L
  {83, 123, 99, 94, 83},     //M
  {52, 117, 83, 101, 103},     //N
  {24, 65, 54, 53, 45},    //O
  {6, 18, 36, 87, 108},    //P
  {-1, 30, 110, 106, 103},     //Q
  {43, 10, 5, 90, 80},     //R
  {52, 132, 122, 116, 111},    //S
  {27, 102, 97, 93, 114},    //T
  {44, 6, 2, 86, 92},    //U
  {43, 5, 1, 70, 82},    //V
  {61, 8, 5, 8, 64},     //W
  {60, 64, 109, 107, 114},     //X
  {-7, 114, 77, 72, 5},    //Y
  {53, 14, 99, 88, 101},     //Z
};


int error[26][5] = {
  {6, 7, 5, 5, 5},     //A
  {12, 5, 5, 5, 5},    //B
  {15, 5, 5, 5, 7},    //C
  {5, 5, 5, 7, 12},    //D
  {17, 10, 8, 10, 5},    //E
  {6, 7, 5, 5, 5},     //F
  {5, 5, 6, 7, 5},     //G
  {5, 7, 5, 7, 5},     //H
  {5, 7, 12, 14, 5},     //I
  {7, 5, 7, 9, 5},     //J
  {5, 5, 5, 17, 14},     //K
  {5, 5, 7, 8, 12},    //L
  {5, 10, 5, 5, 7},    //M
  {5, 8, 5, 5, 13},    //N
  {5, 5, 5, 5, 9},     //O
  {5, 5, 5, 5, 7},     //P
  {5, 5, 6, 6, 14},    //Q
  {10, 5, 5, 6, 5},    //R
  {5, 10, 5, 5, 5},    //S
  {5, 5, 5, 5, 11},    //T
  {5, 5, 5, 5, 5},     //U
  {5, 5, 5, 12, 19},     //V
  {5, 5, 5, 5, 6},     //W
  {5, 5, 14, 10, 11},    //X
  {5, 8, 6, 16, 5},    //Y
  {9, 8, 9, 5, 25},    //Z
};

// Side and up tilt
// {LOW, HIGH} = up, {LOW, LOW} = down, {HIGH, LOW} = sideways
boolean letterTilt[26][2] = { 
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


String sound = "OFF";

void setup() {
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
  Serial.println("Sound is " + sound + ". Toggle sound with s");
}

void loop() 
{
  delay(500);
  getCommands();

  // Collect samples
  int sampleSize = 5;
  int sensorReadings [FINGERS] = {0, 0, 0, 0, 0};
  for (int samples = 0; samples < sampleSize; samples++) {
    for (int finger = 0; finger < FINGERS; finger++) {
      sensorReadings[finger] += readFingerByIndex(finger);
    }
  }
  
  // Average value
  for (int finger = 0; finger < FINGERS; finger++) {
    sensorReadings[finger] = sensorReadings[finger] / sampleSize;
  }
  
  determineLetter(sensorReadings);
}

// Processes commands from python script. 
char getCommands() {
  char command_recieved = 'x';
  if (Serial.available()) {
    command_recieved = Serial.read();
    Serial.println("Recieved command: " + String(command_recieved));
    executeCommands(command_recieved);
    delay(1000);
  }
  return command_recieved;
}


void executeCommands(char command){
  switch (command) {
    case 's': // toggle sound
      if(sound == "ON"){
        sound = "OFF";
      } else {
        sound = "ON";
      }
      Serial.println("Sound is " + sound);
      break;
      
    case 't': // set tilt confidence values
      TILT_CONFIDENCE = userInputInt(2); // 0-99 value
      Serial.println("TILT_CONFIDENCE is " + String(TILT_CONFIDENCE));

      break;
      
    case 'm': // change minium error delta
      MIN_ERROR_DELTA = userInputInt(2); // 0-99 value
      Serial.println("MIN_ERROR_DELTA is " + String(MIN_ERROR_DELTA));
      break;

    case 'a': // change amount of data samples
      SAMPLES = userInputInt(2); // 0-99 value
      Serial.println("SAMPLES is " + String(SAMPLES));

      break;

    case 'v': // print internal arrays
      Serial.println();
      Serial.println("Calibrated LetterMatrix with sensor data: ");
      printMatrix(letterMatrix);
      Serial.println();
      Serial.println("Calibrated errorMatrix with sensor sample tolerance: ");
      printMatrix(error);
      Serial.println();
      break;
    case 'e':
      int letter = userInputInt(2);
      for (int finger = 0; finger < 5; finger++) {
        error[letter][finger] = userInputInt(2);
      }
      break;
      
    default:
      break; // do nothing
  }
}

int userInputInt(int digits) {
  int n = 0;
  for (int numbers = digits - 1; numbers >= 0; numbers--) {
    while (!Serial.available()); // wait for user input
    int num = Serial.parseInt();
    n += ((int) pow(10, numbers)) * num;
  }
  return n;
}

// Returns confidence rating for letter based on LetterMatrix and error values
int getLetterConfidence (int letter, int data[5]) {
  float confidence = 0;
  for (int finger = 0; finger < FINGERS; finger++) {
    float actual      = (float) data[finger];
    float expected    = (float) letterMatrix[letter][finger];
    float delta       = (float) abs(actual - expected);
    float tolerance   = (float) error[letter][finger];
    
    if (tolerance < MIN_ERROR_DELTA) { // Ensures a minimum error tolerance
        tolerance = MIN_ERROR_DELTA;
    }
      
    int rating = (int) ((20.0/tolerance)*(tolerance - delta));
    confidence += rating; // Max confidence
  }
  return (int) confidence;
}

// Displays best letter match to sensorReadings
void determineLetter(int sensorReadings [5]) {
  // Tilt switches
  int handTiltSide = digitalRead(SIDE_TILT_PIN);
  int handTiltUp = digitalRead(UP_TILT_PIN);

  // Find best letter match
  int bestLetter = -1;
  int bestConfidence = -1;
  for (int letter = 0; letter < LETTERS; letter++) {
    // Take into consideration tilt sensor readings
    int tiltSensor = -1;
    if(handTiltSide != letterTilt[letter][0] || handTiltUp != letterTilt[letter][1]){
      tiltSensor = TILT_CONFIDENCE;
    } else {
      tiltSensor = -TILT_CONFIDENCE;
    }

    // Determine confidence for this letter and remember best
    int letterConfidence = getLetterConfidence(letter, sensorReadings) + tiltSensor;
    Serial.println("\t confidence for " + String(getLetter(letter)) + " is " + String(letterConfidence));
    if (letter == 0 || letterConfidence >= bestConfidence) {
      bestLetter = letter;
      bestConfidence = letterConfidence;
    }
  }

  // Convert to 0-100 scale
  int displayConfidence = 0;
  if (bestConfidence >= 0) {
    displayConfidence = 100;
  } else {
    displayConfidence = (int) abs(100.0 * abs(((float) bestConfidence) / 1000.0));
  }
  
  Serial.println("Display confidence: " + String(displayConfidence));
  Serial.println(String(bestConfidence) + "% confident that the letter is " + String(getLetter(bestLetter)));
  delay(500); // Repurpose
}

// Calibrates expected sensor readings (letterMatrix) and their tolerances (error) for every alphabetic letter.
// Prints matrixs at the end for easy copy and pasting into code. 
void performCalibration() {  
  Serial.println("We are beginning the calibration process for the ASL alphabet.");
  Serial.println();
  
  for(int letter = 0; letter < LETTERS; letter++){
    char curr = getLetter(letter); // unicode A - Z;    
    int finger_samples [FINGERS][SAMPLES];
    int finger_error_delta [FINGERS];

    Serial.println("Calibrating letter: " + String(curr));
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
      if(getCommands() == '-') {
        letter -=1;
        continue;
      }
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
    // TODO: move into data sampling for-loop for efficency. 
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
      error[letter][finger] = abs(maxVal - minVal);
    } // End fingers

    // DEBUG ************************
    Serial.print("letterMatrix:\n");
    printMatrixRow(letter, letterMatrix); // Print stored values for each letter avg
    Serial.println();
    Serial.print("errorMatrix:\n");
    printMatrixRow(letter, error); // Print stored values for each letter error
    Serial.println();
  }
}

// 0 based indexing from right most finger to left most finger. 
// Finger 0 = thumb .. finger 4 = pinky.
int readFingerByIndex (int finger) {
  if (finger == 0) {
    return readFinger(FLEX_PIN1, finger);
  } else if (finger == 1) {
    return readFinger(FLEX_PIN2, finger);
  } else if (finger == 2) {
    return readFinger(FLEX_PIN3, finger);
  } else if (finger == 3) {
    return readFinger(FLEX_PIN4, finger);
  } else if (finger == 4) {
    return readFinger(FLEX_PIN5, finger);
  }
}

// Returns angle of finger using FLEX_PIN reading and experimentally calibrated values 
// (R_DIV, STRAIGHT_RESISTANCE, BEND_RESISTANCE) for flex resistor of fingerNumber
int readFinger(const int FLEX_PIN, int fingerNumber){
  //Serial.print("Value read from: " + String(fingerNumber));
  float flexADC = analogRead(FLEX_PIN);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV[fingerNumber] * (VCC / flexV - 1.0);
  //Serial.print(" Resistance: " + String(flexR1) + " ohms");

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  float angle = map(flexR, STRAIGHT_RESISTANCE[fingerNumber], BEND_RESISTANCE[fingerNumber],
                   0, 90.0);
  return (int) angle;
}

// Pauses until all fingers are within 20 degrees of being straight
void waitUntilHandFlat() {
  int flatFingers = 0;
  while (flatFingers != 4) {
    if (readFingerByIndex(flatFingers) <= 20) {
      flatFingers += 1;
    } else {
      flatFingers = 0; // A finger wasn't flat so restart.
    }
  }
  delay (300);
}

// Below are useful print functions for calibration (can easily copy/paste into code) and debugging

// Converts index to corresponding capital alphabetical letter.
// 0 = A, 25 = Z
char getLetter (int index) {
  return ((char)(65 + index));
}

// Prints entire matrix in code copy/paste format
void printMatrix (int data[26][5]) {
  Serial.println("{");
  for (int row = 0; row < 26; row++) {
    Serial.print("\t{");
    for (int column = 0; column < 5; column++) {
      Serial.print(String(data[row][column]));
      if (column != 4) {
        Serial.print(", ");
      }
    }
    Serial.println("},\t\t //" + String(getLetter(row)));
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
  Serial.print("},\t //" + String (getLetter(row)));
  Serial.println();
}
