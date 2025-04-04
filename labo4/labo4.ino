#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Stepper.h>

// Configuration matérielle
#define TRIG_PIN 5
#define ECHO_PIN 4
#define STEPS_PER_REV 2048
#define MOTOR_PIN1 8
#define MOTOR_PIN2 9
#define MOTOR_PIN3 10
#define MOTOR_PIN4 11

// Paramètres système
#define OPEN_ANGLE 170
#define CLOSED_ANGLE 10
#define TRANSITION_TIME 2000 // 2 secondes
#define OPEN_DISTANCE 30    // 30 cm
#define CLOSE_DISTANCE 60   // 60 cm

LiquidCrystal_I2C lcd(0x27, 16, 2);
Stepper stepper(STEPS_PER_REV, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4); // Séquence modifiée

// États du système
enum DoorState {
  INIT,
  CLOSED,
  OPENING,
  OPEN,
  CLOSING
};
DoorState doorState = INIT;

// Variables globales
unsigned long currentMillis = 0;
int currentAngle = CLOSED_ANGLE;
unsigned long moveStartTime = 0;
int previousAngle = CLOSED_ANGLE;
const float STEPS_PER_DEGREE = STEPS_PER_REV / 360.0;

void setup() {
  Serial.begin(115200);
  
  // Configuration des broches
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Initialisation LCD
  lcd.init();
  lcd.backlight();
  
  // Configuration moteur
  stepper.setSpeed(15); // Augmentation à 15 RPM
  
  // Affichage initial
  showStartScreen();
  
  doorState = CLOSED;
}

void loop() {
  currentMillis = millis();
  
  manageDoorState();
  updateDisplay();
  sendSerialData();
}

void showStartScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("2384980"); // Numéro étudiant
  lcd.setCursor(0, 1);
  lcd.print("Labo 4A"); // Énoncé A
  delay(2000);
  lcd.clear();
}

void manageDoorState() {
  int distance = getDistance();
  
  switch(doorState) {
    case CLOSED:
      if (distance < OPEN_DISTANCE) {
        doorState = OPENING;
        moveStartTime = currentMillis;
      }
      break;
      
    case OPENING:
      currentAngle = map(currentMillis - moveStartTime, 0, TRANSITION_TIME, CLOSED_ANGLE, OPEN_ANGLE);
      moveStepper();
      
      if (currentAngle >= OPEN_ANGLE) {
        currentAngle = OPEN_ANGLE;
        doorState = OPEN;
      }
      break;
      
    case OPEN:
      if (distance >= CLOSE_DISTANCE) {
        doorState = CLOSING;
        moveStartTime = currentMillis;
      }
      break;
      
    case CLOSING:
      currentAngle = map(currentMillis - moveStartTime, 0, TRANSITION_TIME, OPEN_ANGLE, CLOSED_ANGLE);
      moveStepper();
      
      if (currentAngle <= CLOSED_ANGLE) {
        currentAngle = CLOSED_ANGLE;
        doorState = CLOSED;
      }
      break;
  }
}

void moveStepper() {
  int stepsToMove = (currentAngle - previousAngle) * STEPS_PER_DEGREE;
  
  if (stepsToMove != 0) {
    stepper.step(stepsToMove);
    previousAngle = currentAngle;
    
    // Détacher le moteur après mouvement
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, LOW);
    digitalWrite(MOTOR_PIN3, LOW);
    digitalWrite(MOTOR_PIN4, LOW);
  }
}

int getDistance() {
  static unsigned long lastMeasure = 0;
  static int lastDistance = 0;
  
  if (currentMillis - lastMeasure < 50) return lastDistance;
  lastMeasure = currentMillis;
  
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  lastDistance = duration * 0.034 / 2;
  
  return lastDistance;
}

void updateDisplay() {
  static unsigned long lastUpdate = 0;
  
  if (currentMillis - lastUpdate < 100) return;
  lastUpdate = currentMillis;
  
  lcd.clear();
  
  // Ligne 1: Distance
  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
  lcd.print(getDistance());
  lcd.print("cm");
  
  // Ligne 2: État
  lcd.setCursor(0, 1);
  switch(doorState) {
    case CLOSED:
      lcd.print("Porte :Fermee");
      break;
    case OPEN:
      lcd.print("Porte : Ouverte");
      break;
    case OPENING:
      lcd.print("Porte ");
      lcd.print(currentAngle);
      lcd.print("°");
      break;
    case CLOSING:
      lcd.print("Porte ");
      lcd.print(currentAngle);
      lcd.print("°");
      break;
  }
}

void sendSerialData() {
  static unsigned long lastSend = 0;
  
  if (currentMillis - lastSend < 100) return;
  lastSend = currentMillis;
  
  Serial.print("etd:2384980,dist:");
  Serial.print(getDistance());
  Serial.print(",deg:");
  Serial.println(currentAngle);
}