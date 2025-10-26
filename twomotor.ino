#include <Dynamixel2Arduino.h>
#include <SPI.h>
/*
#include <nRF24L01.h>
#include <RF24.h>
*/

/*
// REMOTE CONFIG
#define USE_RF24
// CE = 7, CSN = 8
RF24 radio(7, 8);
*/



// USER SETTINGS
// Define which hardware serial port is used 
#define DXL_SERIAL Serial1

// Direction control pin for hald-duplex communication
const uint8_t DXL_DIR_PIN = 2;

// Define motor IDs
const int DXL_ID_1 = 1;
const int DXL_ID_2 = 2;



// LIBRARY INIT 
// Create a Dynamixel2Arduino object using Serial1 and direcion pin
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);



// PARAMETERS 
// Struct stores amplitudes and phase for predifined motion pattern
struct MotionParams {

  // Amplitude in degrees
  float amplitude_deg;

  // Phase offset in degrees
  float phase_deg;
};

// Array of predefined motion patterns
// Each pattern specifies how the motor moves ==> 
// amplitude sets how far it swings and phase sets the timing offset
MotionParams motions[] = {

  // f ensures the numbers are floating and not doubles

  // Section 0 = Stop
  {0.0f, 0.0f},

  // Section 1
  {30.0f, 0.0f},

  // Section 2
  {45.0f, 90.0f},

  // Section 3
  {60.0f, 120.0f},
};



// MOTOR STATE STRUCT

// Stores the current control paramters for each motors
struct MotorState {

  // Which motion pattern is being used
  int section = 0;

  // Motion frequency in Hz
  float freq = 0.0f;

  // User-input offset angle
  float refInput = 0.0f;

  // Target baseline angle in degrees
  float refAngle = 180;

  // Motor's actual starting angle
  float baseline = 180;
};

// Create instances for both motors
MotorState m1, m2;



// HELPERS 

// Convert degress (0-360) to Dynamixel internal position units (0-4095)
int angleToValue_deg(float deg) {
  if (deg < 0.0f) deg = 0.0f;
  if (deg > 360.0f) deg = 360.0f;
  return (int)round((deg / 360.0f) * 4095.0f);
}

// Convert Dynamixel internal position units (0-4095) to degree (0-360)
float valueToAngle_deg(int val) {
  if (val < 0) val = 0;
  if (val > 4095) val = 4095;
  return (float)val / 4095.0f * 360.0f;
}



// GLOBAL STATE 

// t0: starting time reference (milliseconds since program start)
unsigned long t0 = 0;

// inputStage: which step of serial input that user is currently on
int inputStage = 0;

// currentMotor: which motor(1 or 2) the user if configuring
int currentMotor = 0;

// running: whether the motors are currently moving
bool running = false;

// Clears the serial console visually by printing blank lines
void clearSerialScreen() {
  for (int i = 0; i < 50; ++i) Serial.println();
}



// SETUP FUNCTION

void setup() {

  // Start USB serial for use communication
  Serial.begin(115200);

  // Wait until serial monitor is connected 
  while (!Serial);

  // Clear terminal screen
  clearSerialScreen();

  // Begin communication wtih Dynamixel. motors
  dxl.begin(57600);

  //Use Dynamixel protovol version 2.o 
  dxl.setPortProtocolVersion(2.0);

  // Check connection
  dxl.ping(DXL_ID_1);

  //Disabel torque to change mode
  dxl.torqueOff(DXL_ID_1);

  // Set to position control
  dxl.setOperatingMode(DXL_ID_1, OP_POSITION);

  // Re-enable torque
  dxl.torqueOn(DXL_ID_1);

  // Initialize Motor 2
  dxl.ping(DXL_ID_2);
  dxl.torqueOff(DXL_ID_2);
  dxl.setOperatingMode(DXL_ID_2, OP_POSITION);
  dxl.torqueOn(DXL_ID_2);

  // Read current motor positions and set baseline angles
  int p1 = dxl.getPresentPosition(DXL_ID_1);
  int p2 = dxl.getPresentPosition(DXL_ID_2);

  // Convert to degree if valid, otherwise set to default 180 degrees
  if (p1 >= 0 && p1 <= 4095) m1.baseline = valueToAngle_deg(p1);
  else m1.baseline = 180.0f;
  if (p2 >= 0 && p2 <= 4095) m2.baseline = valueToAngle_deg(p2);
  else m2.baseline = 180.0f;

  // Initialize reference angles to baseline 
  m1.refAngle = m1.baseline;
  m2.refAngle = m2.baseline;

  // Move both motors to their baseline positions 
  dxl.setGoalPosition(DXL_ID_1, angleToValue_deg(m1.baseline));
  dxl.setGoalPosition(DXL_ID_2, angleToValue_deg(m2.baseline));

  // Record start time
  t0 = millis();

  /*
  #ifdef USE_RF24
    radio.begin();
    radio.setChannel(76);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    radio.startListening();
    Serial.println("RF24 enabled (compiled with USE_RF24).");
  #endif
  */

  // Print setup instructions to use
  Serial.println("Choose section, frequency, and angle for each motor");
  Serial.println("Start with Motor 1 then Motor 2.");
  Serial.println("Sections: 0=Stop, 1..N = motion patterns (see code)");
  Serial.println("Enter section (0-" + String((int)((sizeof(motions)/sizeof(MotionParams))-1)) + ") for Motor 1: ");

  // Begin user input squence
  inputStage = 0;
}



// MAIN LOOP 
void loop() {
  /*
  #ifdef USE_RF24
    if (radio.available()) {
      float remoteData[4] = {0,0,0,0};
      radio.read(&remoteData, sizeof(remoteData));
    }
  #endif
  */



// SERIAL USER INPUT SECTION

  if (Serial.available()) {

    // Read user input line
    String input = Serial.readStringUntil('\n');

    // Remote whitespace
    input.trim();

    // Check if proceed is empty
    if (input.length() == 0) { }



// MOTOR 1: select secetion

    // Contiune on if input is not empty
    else {

      switch (inputStage) {

        case 0: {

          int sec = input.toInt();

          int maxSec = (int)(sizeof(motions)/sizeof(MotionParams)) - 1;

          if (sec >= 0 && sec <= maxSec) {

            m1.section = sec;

            Serial.print("Motor 1 section = "); Serial.println(sec);

            if (sec == 0) {

              // Stop selected
              Serial.println("Motor 1 stopped.");
              Serial.println("Enter section (0-" + String(maxSec) + ") for Motor 2: ");

              inputStage = 3;
            } else {

              Serial.println("Enter frequency in Hz for Motor 1: ");

              inputStage = 1;
            }
          } 
          
          else {
            
            Serial.print("Invalid! Enter section (0-"); Serial.print(maxSec); Serial.println(") for Motor 1: ");
          }
        } 
        
        break;



// MOTOR 1: enter frequency

        case 1: {

          m1.freq = input.toFloat();

          if (m1.freq <= 0.0f) m1.freq = 0.5f;

          Serial.print("Motor 1 frequency = "); Serial.print(m1.freq); Serial.println(" Hz");
          Serial.println("Enter reference angle (deg) for Motor 1:");

          inputStage = 2;
        } 
        
        break;



// MOTION 1: enter a reference angle

        case 2: {

          m1.refInput = input.toFloat();

          m1.refAngle = m1.baseline - m1.refInput;

          if (m1.refAngle < 0.0f) m1.refAngle = 0.0f;

          if (m1.refAngle > 360.0f) m1.refAngle = 360.0f;

          Serial.print("Motor 1 setup complete -> section="); Serial.print(m1.section);

          Serial.print(", freq="); Serial.print(m1.freq); Serial.print(", angle="); Serial.println(m1.refAngle);
          
          Serial.println("Enter section for Motor 2: ");

          inputStage = 3;
        } 
        
        break;



// MOTOR 2: select secetion

        case 3: {

          int sec = input.toInt();

          int maxSec = (int)(sizeof(motions)/sizeof(MotionParams)) - 1;

          if (sec >= 0 && sec <= maxSec) {

            m2.section = sec;

            Serial.print("Motor 2 section = "); Serial.println(sec);

            if (sec == 0) {

              Serial.println("Motor 2 stopped. Setup complete.");

              running = true;

              inputStage = 6;

              Serial.println("To change or stop, enter motor number (1 or 2): ");

            } 
            
            else {

              Serial.println("Enter frequency in Hz for Motor 2: ");

              inputStage = 4;
            }
          } 
          
          else {

            Serial.print("Invalid! Enter section (0-"); Serial.print(maxSec); Serial.println(") for Motor 2: ");
          }
        } 
        
        break;



// MOTOR 2: enter frequency

        case 4: {

          m2.freq = input.toFloat();

          if (m2.freq <= 0.0f) m2.freq = 0.5f;

          Serial.print("Motor 2 frequency = "); Serial.print(m2.freq); Serial.println(" Hz");

          Serial.println("Enter reference angle (deg) for Motor 2:");

          inputStage = 5;
        } 

        break;



// MOTION 2: enter regerence angle

        case 5: {

          m2.refInput = input.toFloat();

          m2.refAngle = m2.baseline - m2.refInput;

          if (m2.refAngle < 0.0f) m2.refAngle = 0.0f;

          if (m2.refAngle > 360.0f) m2.refAngle = 360.0f;

          Serial.println("Setup complete. Motors running.");

          running = true;

          inputStage = 6;

          Serial.println("To change or stop, enter motor number (1 or 2): ");
        } 
        
        break;



// RUNTIME CONTROL
//User selects which motor to modify while running
 
        case 6: {

          int which = input.toInt();

          if (which == 1 || which == 2) {

            currentMotor = which;

            Serial.print("Enter section for Motor "); Serial.println(currentMotor);

            inputStage = 7;
          } 
          
          else {

            Serial.println("Invalid! Enter motor number (1 or 2): ");
          }
        } 
        
        break;

        // User sets new sectiosn for selected motor 
        case 7: {

          int sec = input.toInt();

          int maxSec = (int)(sizeof(motions)/sizeof(MotionParams)) - 1;

          if (sec >= 0 && sec <= maxSec) {

            if (currentMotor == 1) m1.section = sec; else m2.section = sec;

            Serial.print("Motor "); Serial.print(currentMotor);

            Serial.print(" section = "); Serial.println(sec);

            if (sec == 0) {

              Serial.print("Motor "); Serial.print(currentMotor); Serial.println(" stopped.");

              inputStage = 6;
            } 
            
            else {
              Serial.print("Enter frequency for Motor "); Serial.println(currentMotor);

              inputStage = 8;
            }
          }
        } 
        
        break;

        // User sets frequency for selected motor
        case 8: {

          float f = input.toFloat();

          if (f <= 0.0f) f = 0.5f;

          if (currentMotor == 1) m1.freq = f; else m2.freq = f;

          Serial.print("Motor "); Serial.print(currentMotor); Serial.print(" frequency = "); Serial.println(f);

          Serial.print("Enter reference angle for Motor "); Serial.println(currentMotor);

          inputStage = 9;
        } 
        
        break;

        // User sets reference angle for selected motor
        case 9: {

          float ang = input.toFloat();

          if (currentMotor == 1) {

            m1.refInput = ang;

            m1.refAngle = m1.baseline - m1.refInput;

            if (m1.refAngle < 0.0f) m1.refAngle = 0.0f;

            if (m1.refAngle > 360.0f) m1.refAngle = 360.0f;

          } else {

            m2.refInput = ang;

            m2.refAngle = m2.baseline - m2.refInput;

            if (m2.refAngle < 0.0f) m2.refAngle = 0.0f;

            if (m2.refAngle > 360.0f) m2.refAngle = 360.0f;
          }

          // Return runtime mode
          inputStage = 6;
        } 
        
        break;
      }
    }
  }



// MOTOR MOTION CONTROL SECTION

  if (running) {

    // Current time(ms)
    unsigned long now = millis();

    // Elapsed time in seconds 
    float t = (now - t0) / 1000.0f;

    // Motor 1
    if (m1.section == 0)

      // Hold Position
      dxl.setGoalPosition(DXL_ID_1, angleToValue_deg(m1.refAngle));

    else {

      // Convert phase to radians
      MotionParams mp = motions[m1.section];

      float ph_rad = mp.phase_deg * (PI / 180.0f);

      float target = m1.refAngle + mp.amplitude_deg * sin(2.0f * PI * m1.freq * t + ph_rad);

      dxl.setGoalPosition(DXL_ID_1, angleToValue_deg(target));
    }

    // Motor 2
    if (m2.section == 0)

      dxl.setGoalPosition(DXL_ID_2, angleToValue_deg(m2.refAngle));
      
    else {

      MotionParams mp = motions[m2.section];
      
      float ph_rad = mp.phase_deg * (PI / 180.0f);

      float target = m2.refAngle - mp.amplitude_deg * sin(2.0f * PI * m2.freq * t + ph_rad);

      dxl.setGoalPosition(DXL_ID_2, angleToValue_deg(target));
    }
  }

  // Short delay to prevent overwhelming communtion 
  delay(10);
}
