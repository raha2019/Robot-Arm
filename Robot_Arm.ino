/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-joystick
 */

#include <ezButton.h>
#include <Stepper.h>
#include <Servo.h>

#define VRX_PIN A0  // Arduino pin connected to VRX pin
#define VRY_PIN A1  // Arduino pin connected to VRY pin
#define SW_PIN 3    // Arduino pin connected to SW  pin

#define CLAW_PIN 12
#define BOTTOM_PIN 11
#define MIDDLE_PIN 10
#define TOP_PIN 9

int xValue = 0;  // To store value of the X axis
int yValue = 0;  // To store value of the Y axis
int bValue = 0;  // To store value of the button

int openClaw = 130;
int closedClaw = 170;
int clawPos = 0;

int rotateServoPos = 0;

int bottomServo = 90;
int middleServo = 90;
int topServo = 90;
int servoIncrement = 1;

const int bottomButtonPin = 2;
const int middleButtonPin = 8;
const int topButtonPin = 13;

int bottomButtonState = 0;
int middleButtonState = 0;
int topButtonState = 0;

const int stepsPerRevolution = 2038;

ezButton button(SW_PIN);

// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, 4, 6, 5, 7);

Servo BottomServo;
Servo MiddleServo;
Servo TopServo;
Servo claw;

void setup() {
  BottomServo.attach(BOTTOM_PIN);
  MiddleServo.attach(MIDDLE_PIN);
  TopServo.attach(TOP_PIN);
  claw.attach(CLAW_PIN);

  pinMode(bottomButtonPin, INPUT);
  pinMode(middleButtonPin, INPUT);
  pinMode(topButtonPin, INPUT);


  Serial.begin(9600);
  button.setDebounceTime(50);  // set debounce time to 50 milliseconds

  // Servo Setup

  BottomServo.write(bottomServo);
  MiddleServo.write(middleServo);
  TopServo.write(topServo);
  clawPos = openClaw;
  claw.write(clawPos);

  myStepper.setSpeed(5);
  myStepper.step(rotateServoPos);
  delay(1000);
}

void loop() {
  button.loop();  // MUST call the loop() function first
  // read analog X and Y analog values
  xValue = analogRead(VRX_PIN);
  yValue = analogRead(VRY_PIN);

  // Read the button value
  bValue = button.getState();

  bottomButtonState = digitalRead(bottomButtonPin);
  middleButtonState = digitalRead(middleButtonPin);
  topButtonState = digitalRead(topButtonPin);

  if (button.isPressed()) {
    Serial.println("The button is pressed");
    Serial.println(clawPos);
    if (clawPos == openClaw) {
      clawPos = closedClaw;
      // claw.write(clawPos);
    } else if (clawPos == closedClaw) {
      clawPos = openClaw;
      // claw.write(clawPos);
    }
    claw.write(clawPos);
    // TODO do something here
  }

  // if (button.isReleased()) {
  //   Serial.println("The button is released");
  //   claw.write(openClaw);
  //   // TODO do something here
  // }

  if (yValue < 200) {
    myStepper.step(rotateServoPos - 100);
    delay(50);
  }

  if (yValue > 800) {
    myStepper.step(rotateServoPos + 100);
  }

  if (xValue < 200) {
    if (bottomButtonState == HIGH) {
      bottomServo = bottomServo + servoIncrement;
      BottomServo.write(bottomServo);
    }
    if (middleButtonState == HIGH) {
      middleServo = middleServo + servoIncrement;
      MiddleServo.write(middleServo);
    }
    if (topButtonState == HIGH) {
      topServo = topServo + servoIncrement;
      TopServo.write(topServo);
    }
  }

  if (xValue > 800) {
    if (bottomButtonState == HIGH) {
      bottomServo = bottomServo - servoIncrement;
      BottomServo.write(bottomServo);
    }
    if (middleButtonState == HIGH) {
      middleServo = middleServo - servoIncrement;
      MiddleServo.write(middleServo);
    }
    if (topButtonState == HIGH) {
      topServo = topServo - servoIncrement;
      TopServo.write(topServo);
    }
  }

  // print data to Serial Monitor on Arduino IDE
  Serial.print("x = ");
  Serial.print(xValue);
  Serial.print(", y = ");
  Serial.print(yValue);
  Serial.print(" : button = ");
  Serial.println(bValue);
}
