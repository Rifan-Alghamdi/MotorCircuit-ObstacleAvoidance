# MotorCircuit-ObstacleAvoidance

This project presents a beginner-friendly yet powerful example of intelligent robotics using simple electronic components. The design focuses on combining motion control and environmental awareness to simulate autonomous decision-making in a small-scale robot.

By integrating four DC motors with an L293D motor driver and enhancing the system with an ultrasonic distance sensor mounted on a servo motor, the robot gains the ability to move freely while detecting and responding to obstacles in real time. Unlike basic circuits that follow a fixed path, this system reacts dynamically to its environment, offering a glimpse into how real-world autonomous robots function.

The key value of this circuit lies in how it connects different concepts—motion control, sensor data reading, signal processing, and conditional logic—into one cohesive, interactive system. It provides a strong foundation for learning how multiple hardware components can cooperate under one program to achieve smart behavior, making it ideal for educational and prototyping purposes in robotics and embedded systems.

---
## 📷 Screenshot

<img src="‪MotorCircuit-ObstacleAvoidance.png‬" alt="Motor Circuit Screenshot" width="800"/>

---
## 📂 Included Files

| File Name                        | Description                                 |
|----------------------------------|---------------------------------------------|
| ‪MotorCircuit-ObstacleAvoidance.png‬    | Screenshot of the circuit in Tinkercad      |
| motorcircuit_obstacleavoidance.ino | Full Arduino code                         |

---

## 🧩 Components Used

| Component               | Quantity |
|-------------------------|----------|
| Arduino UNO             | 1        |
| L293D Motor Driver      | 2        |
| DC Motors               | 4        |
| Servo Motor             | 1        |
| Ultrasonic Sensor (HC-SR04) | 1    |
| Power Supply (2 Batteries) | 2    |
| Jumper Wires            | as needed |

---

## 💡 Key Features
 • Autonomous obstacle avoidance
 • Dynamic direction selection using servo scan
 • Multi-motor coordination
 • Real-time distance detection
 • Arduino-based control logic

---

## 🧠 How It Works

The robot continuously moves forward until it detects an obstacle closer than 10 cm. It then stops, moves backward, scans left and right with the servo-mounted ultrasonic sensor, and turns toward the path with more space. This cycle allows continuous navigation in cluttered environments.

---


## ⚙️ Arduino Code

```cpp
#include <Servo.h>

Servo myservo;

int servopin = A5;
int trigpin = A1;
int echopin = A0;

int speedpin1 = 3;
int dir1 = 5;
int dir2 = 7;

int speedpin2 = 6;
int dir3 = 8;
int dir4 = 10;

int speedpin3 = 9;
int dir5 = 2;
int dir6 = 4;

int speedpin4 = 11;
int dir7 = 12;
int dir8 = 13;

void setup() {
  Serial.begin(9600);
  myservo.attach(servopin);
  myservo.write(90);
  delay(3000);

  pinMode(trigpin, OUTPUT);
  pinMode(echopin, INPUT);

  pinMode(speedpin1, OUTPUT);
  pinMode(speedpin2, OUTPUT);
  pinMode(speedpin3, OUTPUT);
  pinMode(speedpin4, OUTPUT);

  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(dir3, OUTPUT);
  pinMode(dir4, OUTPUT);
  pinMode(dir5, OUTPUT);
  pinMode(dir6, OUTPUT);
  pinMode(dir7, OUTPUT);
  pinMode(dir8, OUTPUT);
}

void loop() {
  int distance = getDistance();

  if (distance <= 10) {
    stopMotors();
    delay(500);

    back();
    delay(1500);

    stopMotors();
    delay(500);

    int rightDist = scanRight();
    int leftDist = scanLeft();

    if (rightDist > leftDist) {
      turnRight();
    } else {
      turnLeft();
    }

    forward();
  } else {
    forward();
  }

  delay(100);
}

int getDistance() {
  digitalWrite(trigpin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);

  long duration = pulseIn(echopin, HIGH);
  return duration * 0.034 / 2;
}

void forward() {
  digitalWrite(dir1, HIGH); digitalWrite(dir2, LOW); analogWrite(speedpin1, 255);
  digitalWrite(dir3, LOW);  digitalWrite(dir4, HIGH); analogWrite(speedpin2, 255);
  digitalWrite(dir5, HIGH); digitalWrite(dir6, LOW); analogWrite(speedpin3, 255);
  digitalWrite(dir7, LOW);  digitalWrite(dir8, HIGH); analogWrite(speedpin4, 255);
}

void back() {
  digitalWrite(dir1, LOW); digitalWrite(dir2, HIGH); analogWrite(speedpin1, 255);
  digitalWrite(dir3, HIGH); digitalWrite(dir4, LOW); analogWrite(speedpin2, 255);
  digitalWrite(dir5, LOW); digitalWrite(dir6, HIGH); analogWrite(speedpin3, 255);
  digitalWrite(dir7, HIGH); digitalWrite(dir8, LOW); analogWrite(speedpin4, 255);
}

void stopMotors() {
  digitalWrite(dir1, LOW); digitalWrite(dir2, LOW);
  digitalWrite(dir3, LOW); digitalWrite(dir4, LOW);
  digitalWrite(dir5, LOW); digitalWrite(dir6, LOW);

digitalWrite(dir7, LOW); digitalWrite(dir8, LOW);

  analogWrite(speedpin1, 0);
  analogWrite(speedpin2, 0);
  analogWrite(speedpin3, 0);
  analogWrite(speedpin4, 0);
}

int scanRight() {
  myservo.write(15);
  delay(700);
  int d = getDistance();
  myservo.write(90);
  delay(300);
  return d;
}

int scanLeft() {
  myservo.write(165);
  delay(700);
  int d = getDistance();
  myservo.write(90);
  delay(300);
  return d;
}

void turnRight() {
  digitalWrite(dir1, LOW); digitalWrite(dir2, HIGH); analogWrite(speedpin1, 255);
  digitalWrite(dir3, HIGH); digitalWrite(dir4, LOW); analogWrite(speedpin2, 255);
  digitalWrite(dir5, HIGH); digitalWrite(dir6, LOW); analogWrite(speedpin3, 255);
  digitalWrite(dir7, LOW); digitalWrite(dir8, HIGH); analogWrite(speedpin4, 255);
  delay(1000);
  stopMotors();
}

void turnLeft() {
  digitalWrite(dir1, HIGH); digitalWrite(dir2, LOW); analogWrite(speedpin1, 255);
  digitalWrite(dir3, LOW); digitalWrite(dir4, HIGH); analogWrite(speedpin2, 255);
  digitalWrite(dir5, LOW); digitalWrite(dir6, HIGH); analogWrite(speedpin3, 255);
  digitalWrite(dir7, HIGH); digitalWrite(dir8, LOW); analogWrite(speedpin4, 255);
  delay(1000);
  stopMotors();
}
