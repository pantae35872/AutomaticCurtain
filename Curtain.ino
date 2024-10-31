#include <IRremote.hpp>
#include <Wire.h>
#include <SPI.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <Vector.h>
#include <Streaming.h>

// ---- CLASSES ----

class Button {
  public:
    int prev_value = 2;
    int pin;
    Button(int provided_pin) : pin(provided_pin) {}
    void update(void (*callback)(int)) {
      int value = digitalRead(pin);
      
      if (value != prev_value) {
        callback(value);
      }

      prev_value = value;
    }
};


class ToggleButton : public Button {
  private:
    bool is_toggled = false;

  public:
    ToggleButton(int provided_pin) : Button(provided_pin) {}

    void update(void (*callback)(bool)) {
      int value = digitalRead(pin);

      if (value != prev_value && value == HIGH) {
        is_toggled = !is_toggled;
        callback(is_toggled);
      }

      prev_value = value;
    }
};

// ---- ENUMS ----

enum class CurtainState {
  Open,
  Close
};

enum class MotorDirection {
  Right,
  Left,
};

// ---- MACRO ----
#define DEFINE_VECTOR(Type, Name)\
    Type Name##_array[ELEMENT_COUNT_MAX];  \
    Vector<Type> Name;           \
    Name.setStorage(Name##_array);

// ---- PINS ----

#define CLOCK_INTERRUPT_PIN 2
#define MOTOR_R_PWM_PIN 5 
#define MOTOR_L_PWM_PIN 6 
#define BUTTON_OPEN_CLOSE_PIN 3
#define BUTTON_BUTTON2 4
#define BUTTON_3_PIN A0
#define BUTTON_4_PIN A1
#define BUTTON_5_PIN 7
#define BUTTON_6_PIN 8
#define BUTTON_7_PIN 9
#define BUTTON_8_PIN 10
#define CHANGE_TIME_BUTTON_PIN 11

// ---- GLOBAL VARIABLES ----

CurtainState curtainState = CurtainState::Open;
CurtainState lastCurtainState = CurtainState::Close;
ToggleButton openCloseButton(BUTTON_OPEN_CLOSE_PIN);
Button button2(BUTTON_BUTTON2);
Button button3(BUTTON_3_PIN);
Button button4(BUTTON_4_PIN);
Button button5(BUTTON_5_PIN);
Button button6(BUTTON_6_PIN);
Button button7(BUTTON_7_PIN);
Button button8(BUTTON_8_PIN);
Button setCurrentTimeButton(CHANGE_TIME_BUTTON_PIN);

LiquidCrystal_I2C lcd(0x27, 20, 4);
RTC_DS3231 rtc;

bool alarm = false;

bool motor_busy = false;
uint32_t motor_issued_time = 0;
int motor_wait_time = 0;
MotorDirection motor_direction = MotorDirection::Right;


// ---- UTILS ----
void delay_loop() {
  while (1) delay(10);
}

void clear_line(uint8_t line) {
  lcd.setCursor(0, line);
  lcd << "                    ";
  lcd.setCursor(0, line);
}

// ---- SETUP CODE ----
void setup() {
  Serial.begin(9600);
  setup_lcd();
  setup_rtc();
  setup_motor();
  setup_button();
  curtain_reset();

  // lcd.setCursor(0, 0);
  // lcd.print("Input: ");
  // lcd.setCursor(0, 1);
  // lcd.print("(5, 8, 3, 6) -> 50");

  // DEFINE_VECTOR(double, numbers);
  // numbers.push_back(5);
  // numbers.push_back(8);
  // numbers.push_back(3);
  // numbers.push_back(6);
  // solve24Game(numbers, 50);
}

void setup_motor() {
  pinMode(MOTOR_R_PWM_PIN, OUTPUT);
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
}

void setup_button() {
  pinMode(BUTTON_OPEN_CLOSE_PIN
, INPUT);
  pinMode(BUTTON_BUTTON2

, INPUT);
  pinMode(BUTTON_3_PIN, INPUT);
  pinMode(BUTTON_4_PIN, INPUT);
  pinMode(BUTTON_5_PIN, INPUT);
  pinMode(BUTTON_6_PIN, INPUT);
  pinMode(BUTTON_7_PIN, INPUT);
  pinMode(BUTTON_8_PIN, INPUT);
  pinMode(CHANGE_TIME_BUTTON_PIN, INPUT);
}

void setup_rtc() {
  if (!rtc.begin()) {
     lcd.print("Failed to initialize RTC");
     delay_loop();  
  }

  if (rtc.lostPower()) {

  }

  rtc.disable32K();

  rtc.clearAlarm(1);
  rtc.writeSqwPinMode(DS3231_OFF);
  rtc.disableAlarm(2);

  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), clock_interrupt, FALLING);
}

void setup_lcd() {
  lcd.init();
  lcd.backlight();
  lcd.display();
}

// ---- INTERRUPT HANDLER ---- 
void clock_interrupt() {
  alarm = true;
}

// ---- DISPLAY CODE ---- 

void display_time() {
  DateTime now = rtc.now();
  lcd.setCursor(0, 1);
  lcd.print("Current Time: ");
  lcd.setCursor(0, 2);
  lcd.print(now.hour(), DEC);
  lcd.print(':');
  String printminute = String(now.minute());
  if (printminute.length() < 2) {
    lcd.print("0");
  }
  lcd.print(now.minute(), DEC);
  lcd.print(':');
  String printsecond = String(now.second());
  if (printsecond.length() < 2) {
    lcd.print("0");
  }
  lcd.print(now.second(), DEC);
}

void display_state() {
  if (lastCurtainState != curtainState) { 
    lcd.setCursor(0, 0);
    clear_line(0);
    lcd << "State: ";
    if (curtainState == CurtainState::Open) lcd << "Open"; else lcd << "Close"; 
    lastCurtainState = curtainState;
  }
}

// ---- MOTOR CONTROLLER --- 

void issue_motor(MotorDirection direction, int time) {
  motor_issued_time = rtc.now().unixtime();
  motor_direction = direction;
  motor_wait_time = time;
  motor_busy = true;
}

void motor_update() {
  if (motor_busy) {
    if (rtc.now().unixtime() - motor_issued_time >= motor_wait_time) {
      analogWrite(MOTOR_R_PWM_PIN, 0);
      analogWrite(MOTOR_L_PWM_PIN, 0);
      motor_busy = false;
      motor_wait_time = 0;
    } else {
      switch (motor_direction) {
        case MotorDirection::Right:
          analogWrite(MOTOR_R_PWM_PIN, 128);
          analogWrite(MOTOR_L_PWM_PIN, 0);
          break;
        case MotorDirection::Left:
          analogWrite(MOTOR_R_PWM_PIN, 0);
          analogWrite(MOTOR_L_PWM_PIN, 128);
          break;
      }
    }
  }
}

// ---- CURTAIN CODE ----

void curtain_reset() {
  issue_motor(MotorDirection::Right, 7);
  curtainState = CurtainState::Open;
}

void curtain_open() {
  if (curtainState != CurtainState::Open) {
    lastCurtainState = curtainState;
    curtainState = CurtainState::Open;
    issue_motor(MotorDirection::Right, 7);
  }
}

void curtain_close() {
  if (curtainState != CurtainState::Close) {
    lastCurtainState = curtainState;
    curtainState = CurtainState::Close;
    issue_motor(MotorDirection::Left, 7);
  }
}

// ---- MAIN LOOP ----
void loop() {
  if (!motor_busy) {
    openCloseButton.update([](int state) {
      if (state == 0) curtain_open(); else curtain_close();
    });
    button2.update([](int new_value) {
      Serial << "Button 2: " << new_value << endl;
    });
    button3.update([](int new_value) {
      Serial << "Button 3: " << new_value << endl;
    });
    button4.update([](int new_value) {
      Serial << "Button 4: " << new_value << endl;
    });
    button5.update([](int new_value) {
      Serial << "Button 5: " << new_value << endl;
    });
    button6.update([](int new_value) {
      Serial << "Button 6: " << new_value << endl;
    });
    button7.update([](int new_value) {
      Serial << "Button 7: " << new_value << endl;
    });
    button8.update([](int new_value) {
      Serial << "Button 8: " << new_value << endl;
    });
    setCurrentTimeButton.update([](int new_value) {
      Serial << "set Current time: " << new_value << endl;
    });
  }
  motor_update();
  display_time();
  display_state();
  delay(10);
}
