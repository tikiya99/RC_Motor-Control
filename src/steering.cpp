#include <Arduino.h>

#define RECEIVER_PIN 15 
#define RPWM_PIN 22       
#define LPWM_PIN 23   

#define ENCODER_A 12    
#define ENCODER_B 14     

volatile uint32_t channel4Pulse = 0;  
volatile uint32_t lastPulseTime = 0; 
volatile bool channel4Updated = false; 
volatile uint8_t currentChannel = 0; 
volatile int encoderCount = 0;     

#define DEADZONE 50
#define CENTER_PULSE 1500
#define MIN_PULSE 1000
#define MAX_PULSE 2000

void IRAM_ATTR readPPM() {
  uint32_t currentTime = micros();           
  uint32_t pulseDuration = currentTime - lastPulseTime; 
  lastPulseTime = currentTime;                 


  if (pulseDuration > 3000) {
    currentChannel = 0; 
  } else if (currentChannel == 3) {  
    if (pulseDuration >= 1000 && pulseDuration <= 2000) {
      channel4Pulse = pulseDuration; 
      channel4Updated = true;      
    }
    currentChannel++;
  } else {
    currentChannel++; 
  }
}

void IRAM_ATTR readEncoder() {
  if (digitalRead(ENCODER_B) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  pinMode(RECEIVER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RECEIVER_PIN), readPPM, RISING);

  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, RISING);

  Serial.begin(115200);
  Serial.println("System Initialized");
}

void loop() {
  static uint32_t lastStablePulse = CENTER_PULSE;  
  static uint32_t lastUpdateTime = 0;            
  uint32_t currentTime = millis();

  if (currentTime - lastUpdateTime >= 50) {
    lastUpdateTime = currentTime;

    if (channel4Updated) {
      lastStablePulse = channel4Pulse;  
      channel4Updated = false;          
    }

    if (lastStablePulse >= CENTER_PULSE - DEADZONE && lastStablePulse <= CENTER_PULSE + DEADZONE) {
      analogWrite(RPWM_PIN, 0); 
      analogWrite(LPWM_PIN, 0);
      Serial.println("Motor Stopped (Deadzone)");
    } else if (lastStablePulse > CENTER_PULSE + DEADZONE) {

      int speed = map(lastStablePulse, CENTER_PULSE + DEADZONE, MAX_PULSE, 0, 255);
      analogWrite(RPWM_PIN, speed);
      analogWrite(LPWM_PIN, 0);
      Serial.print("Forward Speed: ");
      Serial.println(speed);
    } else if (lastStablePulse < CENTER_PULSE - DEADZONE) {
 
      int speed = map(lastStablePulse, CENTER_PULSE - DEADZONE, MIN_PULSE, 0, 255);
      analogWrite(RPWM_PIN, 0);
      analogWrite(LPWM_PIN, speed);
      Serial.print("Reverse Speed: ");
      Serial.println(speed);
    }


    Serial.print("Encoder Count: ");
    Serial.println(encoderCount);
  }
}