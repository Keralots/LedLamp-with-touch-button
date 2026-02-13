/*
 * Touch LED Lamp Controller (Final Version)
 * 
 * Hardware Configuration:
 *  1. TTP223 Jumper A: OPEN (Active HIGH mode)
 *  2. TTP223 Jumper B: Open (Momentary mode)
 * 
 * Wiring:
 *   - TTP223 OUT -> Pin 2 (Interrupt 0)
 *   - PWM Output -> Pin 9
 */

#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>

// Pin definitions
const int TOUCH_PIN = 2;      
const int PWM_PIN = 9;        

// EEPROM addresses
const int EEPROM_BRIGHTNESS = 0;
const int EEPROM_MAGIC = 1;
const byte EEPROM_MAGIC_VALUE = 0xAB;

// Timing constants
const unsigned long HOLD_THRESHOLD = 300;    // Time to distinguish Tap vs Hold
const unsigned long DIM_STEP_INTERVAL = 20; // Speed of dimming
const unsigned long SAVE_DELAY = 2000;      // Delay before writing to EEPROM
const unsigned long DEBOUNCE_DELAY = 100;    // Button debounce time

// Brightness limits
const byte MIN_BRIGHTNESS = 10;
const byte MAX_BRIGHTNESS = 255;
const byte DEFAULT_BRIGHTNESS = 128;
const byte DIM_STEP = 3;

// State variables
volatile bool lampOn = false;
byte brightness = DEFAULT_BRIGHTNESS;
bool dimDirection = true; // true = Increase, false = Decrease

// Touch handling variables
bool lastTouchState = LOW; // Active High: LOW = Idle
bool touchState = LOW;
unsigned long touchStartTime = 0;
bool touchHandled = false;
bool isHolding = false;

// Dimming timing
unsigned long lastDimTime = 0;

// EEPROM save timing
bool needsSave = false;
unsigned long lastChangeTime = 0;

// Wake flag
volatile bool wakeFlag = false;

void setup() {
  // =====================================================
  // FIX: Set PWM frequency to ~31kHz (ultrasonic)
  // This eliminates coil whine from boost converter
  // Affects Timer1: Pin 9 and Pin 10
  // =====================================================
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  
  pinMode(TOUCH_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  
  disableUnusedPeripherals();
  loadBrightness();
  
  analogWrite(PWM_PIN, 0); // Start OFF
  
  // Interrupt for waking up (Rising edge = Touch start)
  attachInterrupt(digitalPinToInterrupt(TOUCH_PIN), wakeUp, RISING);
  
  delay(100); // Initial stability delay
}

void loop() {
  // ============================================
  // WAKE-UP HANDLER
  // ============================================
  if (wakeFlag) {
    wakeFlag = false; // Clear flag
    
    // CRITICAL LOGIC: Only run the "Special Wake Up" routine if the lamp is OFF.
    // If the lamp is ON, the interrupt fired because the user is touching to DIM.
    // We skip this block and let the standard logic handle the touch.
    if (!lampOn) {
      
      detachInterrupt(digitalPinToInterrupt(TOUCH_PIN)); // Detach to handle manually
      
      delay(100); // Stabilization
      
      // Wait for user to release the button after waking
      while(digitalRead(TOUCH_PIN) == HIGH) {
        delay(10); 
      }
      
      toggleLamp(); // Turn light ON
      
      lastTouchState = LOW; // Reset states
      touchState = LOW;
      
      attachInterrupt(digitalPinToInterrupt(TOUCH_PIN), wakeUp, RISING); // Re-attach
      
      return; // Skip standard loop for this cycle
    }
  }
  
  // ============================================
  // STANDARD OPERATING LOGIC (Handles Dimming & Toggling)
  // ============================================
  
  int reading = digitalRead(TOUCH_PIN);
  
  // Simple debounce
  if (reading != lastTouchState) {
    touchState = reading;
  }
  
  unsigned long currentTime = millis();
  
  // 1. Detect Touch Start
  if (touchState == HIGH && lastTouchState == LOW) {
    touchStartTime = currentTime;
    touchHandled = false;
    isHolding = false;
  }
  
  // 2. Handle Holding (Dimming)
  if (touchState == HIGH && !touchHandled) {
    unsigned long holdDuration = currentTime - touchStartTime;
    
    if (holdDuration >= HOLD_THRESHOLD) {
      if (!isHolding) {
        isHolding = true;
      }
      
      if (lampOn) {
        if (currentTime - lastDimTime >= DIM_STEP_INTERVAL) {
          lastDimTime = currentTime;
          adjustBrightness();
        }
      }
    }
  }
  
  // 3. Detect Release
  if (touchState == LOW && lastTouchState == HIGH) {
    unsigned long holdDuration = currentTime - touchStartTime;
    
    if (holdDuration < HOLD_THRESHOLD) {
      // It was a Tap -> Toggle
      toggleLamp();
    } 
    else if (isHolding) {
      // It was a Hold -> Toggle Direction for next time
      dimDirection = !dimDirection;
      if (lampOn) {
        needsSave = true;
        lastChangeTime = currentTime;
      }
    }
    
    touchHandled = true;
    isHolding = false;
  }
  
  // 4. Save to EEPROM
  if (needsSave && (currentTime - lastChangeTime >= SAVE_DELAY)) {
    saveBrightness();
    needsSave = false;
  }
  
  lastTouchState = touchState;

  // ============================================
  // SLEEP LOGIC
  // ============================================
  if (!lampOn && digitalRead(TOUCH_PIN) == LOW) {
    delay(DEBOUNCE_DELAY);  
    if (digitalRead(TOUCH_PIN) == LOW) {
      enterSleep();
    }
  }
}

// ============================================
// HELPER FUNCTIONS
// ============================================

void toggleLamp() {
  lampOn = !lampOn;
  if (lampOn) {
    analogWrite(PWM_PIN, brightness);
  } else {
    analogWrite(PWM_PIN, 0);
  }
}

void adjustBrightness() {
  if (dimDirection) {
    if (brightness < MAX_BRIGHTNESS) brightness += DIM_STEP;
  } else {
    if (brightness > MIN_BRIGHTNESS) brightness -= DIM_STEP;
  }
  analogWrite(PWM_PIN, brightness);
}

void loadBrightness() {
  if (EEPROM.read(EEPROM_MAGIC) == EEPROM_MAGIC_VALUE) {
    brightness = EEPROM.read(EEPROM_BRIGHTNESS);
    if (brightness < MIN_BRIGHTNESS || brightness > MAX_BRIGHTNESS) {
      brightness = DEFAULT_BRIGHTNESS;
    }
  } else {
    brightness = DEFAULT_BRIGHTNESS;
    EEPROM.write(EEPROM_BRIGHTNESS, brightness);
    EEPROM.write(EEPROM_MAGIC, EEPROM_MAGIC_VALUE);
  }
}

void saveBrightness() {
  if (EEPROM.read(EEPROM_BRIGHTNESS) != brightness) {
    EEPROM.write(EEPROM_BRIGHTNESS, brightness);
  }
}

void disableUnusedPeripherals() {
  ADCSRA &= ~(1 << ADEN);
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();
  power_timer2_disable();
  power_usart0_disable();
}

void enterSleep() {
  if (needsSave) {
    saveBrightness();
    needsSave = false;
  }
  analogWrite(PWM_PIN, 0);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  attachInterrupt(digitalPinToInterrupt(TOUCH_PIN), wakeUp, RISING);
  sleep_mode(); 
  sleep_disable();
  power_timer0_enable(); 
  power_timer1_enable(); 
}

void wakeUp() {
  wakeFlag = true;
}
