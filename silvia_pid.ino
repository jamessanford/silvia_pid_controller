/*
 * Arduino PID controller for boiler temperature on Rancilio Silvia.
 * Copyright 2012 James Sanford and Dan Lange.
 * All rights reserved.
 *
 * Use, modification, and redistribution of this software is permitted
 * provided that this notice is retained with the software.
 * This software is without warranty.
 */

// Controller for the LCD.
#include <LiquidCrystal.h>

// protothreads, external library in Documents/Arduino/libraries/pt/pt.h
#include <pt.h>

// github.com/br3ttb/Arduino-PID-Library in Documents/Arduino/libraries/PID_v1
#include <PID_v1.h>

// github.com/adafruit/MAX6675-library
#include <max6675.h>

// For teensy new operator, copy new.cpp and new.h from the arduino core.
#include <new.h>

// AVR watchdog reset
#include <avr/wdt.h>

#define FLASH(__s) F(__s)

// For testing, consider using 1000ms increments instead of 10000ms.
#define RELAY_PERIOD 10000

#define PIN_LED_BUILTIN PIN_D6
#define PIN_RELAY_CONTROL PIN_C7

#define LCD_RS PIN_F1
#define LCD_EN PIN_F0
#define LCD_DB4 PIN_F4
#define LCD_DB5 PIN_F5
#define LCD_DB6 PIN_F6
#define LCD_DB7 PIN_F7

#define PIN_BUTTON_UP PIN_B7
#define PIN_BUTTON_DOWN PIN_C6
#define PIN_BUTTON_LEFT PIN_D0
#define PIN_BUTTON_RIGHT PIN_D1

#define PIN_THERM_CLK PIN_B4
#define PIN_THERM_CS PIN_D4
#define PIN_THERM_DO PIN_D5

#define MIN_SET_TEMP 150
#define MAX_SET_TEMP 300

// Our protothreads.
static struct pt pt_led;
static struct pt pt_button_watch;
static struct pt pt_update_display;
static struct pt pt_relay;

static boolean active_alarm = false;
static uint8_t active_button = 0;  // set by button_watcher()->button_pressed()

// pid_controller will take 'current_temperature' and give us a duty cycle
// on 'pid_output' (between 0 and RELAY_PERIOD),
// as it tries to close in on 'set_temperature'
static double current_temperature = 0;
static double set_temperature = 0;
static double pid_output = 0;

// Tune these for how fast the boiler heats up and how slowly the water cools.
// static double Kp = 45.0, Ki = 10, Kd = 0;
static double Kp = 90.0, Ki = 10, Kd = 0;
PID pid_controller(&current_temperature,
                   &pid_output,
                   &set_temperature,
                   Kp, Ki, Kd, DIRECT);

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_DB4, LCD_DB5, LCD_DB6, LCD_DB7);

MAX6675 thermocouple(PIN_THERM_CLK, PIN_THERM_CS, PIN_THERM_DO);

// Timer from protothreads example.
struct s_timer { unsigned long start, interval; };

static void timer_set(struct s_timer *t, int interval) {
  t->interval = interval;
  t->start = millis();
}

static int timer_expired(const struct s_timer *t) {
  return (t->start + t->interval) <= millis();
}

void toggle_led(int pin_num) {
  boolean ledstate = digitalRead(pin_num);
  ledstate ^= 1;
  digitalWrite(pin_num, ledstate);
}

static int blink_led(struct pt *pt) {
  static s_timer led_timer;
  PT_BEGIN(pt);
  while(1) {
    timer_set(&led_timer, 1000);
    PT_WAIT_UNTIL(pt, timer_expired(&led_timer));
    toggle_led(PIN_LED_BUILTIN);
  }
  PT_END(pt);
}

// Return the pin number of the first button pressed down that we see.
// 'active_button' is a global used by others.
static int button_pressed(void) {
  if (digitalRead(PIN_BUTTON_UP) == LOW) {
    active_button = PIN_BUTTON_UP;
  }
  else if (digitalRead(PIN_BUTTON_DOWN) == LOW) {
    active_button = PIN_BUTTON_DOWN;
  }
  else if (digitalRead(PIN_BUTTON_LEFT) == LOW) {
    active_button = PIN_BUTTON_LEFT;
  }
  else if (digitalRead(PIN_BUTTON_RIGHT) == LOW) {
    active_button = PIN_BUTTON_RIGHT;
  }
  else {
    active_button = 0;
  }
  return active_button;
}

static void display_set_temperature(void) {
  lcd.setCursor(0, 1);
  lcd.print(set_temperature, 0);
  lcd.print("F");
}

static void display_current_temperature(void) {
  lcd.setCursor(16 - 4, 1);
  if (current_temperature < 100) {
    lcd.print(" ");
  }
  lcd.print(current_temperature, 0);  // round to whole degree
  lcd.print("F");
}

// Base class for each different display mode.
class SilviaDisplay {
  public:
    virtual void hide(void) = 0;
    virtual void show(void) = 0;
    virtual void press(int, int) = 0;
    virtual void release(int) = 0;
    virtual void periodic(void) = 0;
};

class DisplayNormal : public SilviaDisplay {
  public:
    virtual void hide(void) {
    }
    virtual void show(void) {
      lcd.clear();
      lcd.print(FLASH("Miss Silvia"));
      display_set_temperature();
      periodic();  // this is just to show the current temperature
    }
    virtual void press(int pin, int down_ms) {
      if (down_ms == 0 || down_ms >= 400) {
        if (pin == PIN_BUTTON_DOWN) {
          set_temperature -= 1;
          set_temperature = max(MIN_SET_TEMP, set_temperature);
          display_set_temperature();
        }
        else if (pin == PIN_BUTTON_UP) {
          set_temperature += 1;
          set_temperature = min(MAX_SET_TEMP, set_temperature);
          display_set_temperature();
        }
      }
    }
    virtual void release(int pin) {
      Serial.print(FLASH("Write "));
      Serial.print(set_temperature);
      Serial.println(FLASH(" to EEPROM"));
    }
    virtual void periodic(void) {
      display_current_temperature();
    }
};

class DisplayPID : public SilviaDisplay {
  private:
    boolean _blink_state;
    char*   _pid_letter;
    double* _pid_variable;
    int     _pid_min;
    int     _pid_max;
    double  _pid_delta;
    int     _pid_x;
  public:
    DisplayPID(char *pid_letter, double *pid_variable) {
      _blink_state = 0;
      _pid_letter = pid_letter;
      _pid_variable = pid_variable;
      _pid_min = 0;
      _pid_max = 99;
      _pid_delta = 1;
      if(_pid_variable == &Kp) {
        _pid_x = 1;
      } else if(_pid_variable == &Ki) {
        _pid_x = 6;
      } else if(_pid_variable == &Kd) {
        _pid_x = 11;
      }
    }
    void refresh_pid(void) {
      lcd.clear();
      lcd.print(FLASH(" P    I    D"));
//                    "90  10.5   0"
//                     012345678901
      lcd.setCursor(0, 1);
      if (Kp < 10) {
        lcd.print(" ");
      }
      lcd.print(Kp, 0);
      lcd.setCursor(4, 1);
      if (Ki < 10) {
        lcd.print(" ");
      }
      lcd.print(Ki, 1);
      lcd.setCursor(10, 1);
      if (Kd < 10) {
        lcd.print(" ");
      }
      lcd.print(Kd, 0);
      // redraw any currently-blank items, and the temperature.
      _blink_state ^= 1;  // negated by periodic();
      periodic();
    }
    virtual void hide(void) {
    }
    virtual void show(void) {
      _blink_state = 1;
      refresh_pid();
    }
    virtual void press(int pin, int down_ms) {
      if (down_ms == 0 || down_ms >= 400) {
        if (pin == PIN_BUTTON_DOWN) {
          *_pid_variable -= _pid_delta;
          *_pid_variable = max(_pid_min, *_pid_variable);
          refresh_pid();
        }
        else if (pin == PIN_BUTTON_UP) {
          *_pid_variable += _pid_delta;
          *_pid_variable = min(_pid_max, *_pid_variable);
          refresh_pid();
        }
      }
    }
    virtual void release(int pin) {
      pid_controller.SetTunings(Kp, Ki, Kd);
      Serial.print(FLASH("Update EEPROM "));
      Serial.print(*_pid_letter);
      Serial.print(FLASH(" = "));
      Serial.println(*_pid_variable);
    }
    virtual void periodic(void) {
      lcd.setCursor(_pid_x, 0);
      _blink_state ^= 1;
      if (_blink_state) {
        lcd.print(" ");
      } else {
        lcd.print(*_pid_letter);
      }
      display_current_temperature();
    }
};

class DisplayUnits : public SilviaDisplay {
  public:
    virtual void hide(void) {
    }
    virtual void show(void) {
      lcd.clear();
      lcd.print(FLASH("Display Units"));
      lcd.setCursor(2, 1);
      lcd.print(FLASH("Fahrenheit"));
      periodic();
    }
    virtual void press(int pin, int down_ms) {
    }
    virtual void release(int pin) {
    }
    virtual void periodic(void) {
      display_current_temperature();
    }
};

class DisplayRAM : public SilviaDisplay {
  public:
    virtual void hide(void) {
    }
    virtual void show(void) {
      periodic();
    }
    virtual void press(int pin, int down_ms) {
    }
    virtual void release(int pin) {
    }
    virtual void periodic(void) {
      extern int __heap_start, *__brkval;
      int v;
      lcd.clear();
      lcd.print(FLASH("Free RAM"));
      lcd.setCursor(2, 1);
      lcd.print(
        (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval)
      );
      lcd.print(FLASH(" bytes"));
      display_current_temperature();
    }
};

static uint8_t active_display = 0;  // Which display_panel[] is active.
#define DISPLAY_COUNT 6
static SilviaDisplay *display_panel[DISPLAY_COUNT] = {
  new DisplayNormal(),
  new DisplayPID((char *)"P", &Kp),
  new DisplayPID((char *)"I", &Ki),
  new DisplayPID((char *)"D", &Kd),
  new DisplayUnits(),
  new DisplayRAM() };

// Each display mode gets called periodically so it can blink text
// or update the temperature display.
static s_timer display_refresh_timer;  // NOTE: Also reset by change_display();
static int update_display(struct pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    timer_set(&display_refresh_timer, 1000);
    PT_WAIT_UNTIL(pt, timer_expired(&display_refresh_timer));
    display_panel[active_display]->periodic();
  }
  PT_END(pt);
}

// left/right buttons are hit to switch between different display modes
static void change_display(int pin, int button_down_ms) {
  if (button_down_ms == 0 || button_down_ms > 400) {
    timer_set(&display_refresh_timer, 1000);
    display_panel[active_display]->hide();
    if (pin == PIN_BUTTON_RIGHT) {
      active_display += 1;
      if (active_display >= DISPLAY_COUNT) {
        active_display = 0;
      }
    }
    else if (pin == PIN_BUTTON_LEFT) {
      if (active_display == 0) {
        active_display = DISPLAY_COUNT - 1;
      } else {
        active_display -= 1;
      }
    }
    display_panel[active_display]->show();
  }
}

// This logic ended up looking much more complicated than I imagined!
static int button_watcher(struct pt *pt) {
  static s_timer button_timer;
  static unsigned long button_down_ms;
  PT_BEGIN(pt);
  while(1) {
    button_down_ms = 0;
    PT_WAIT_UNTIL(pt, button_pressed());
    Serial.print(FLASH("button "));
    Serial.println(active_button);
    // debounce press
    timer_set(&button_timer, 10);
    PT_WAIT_UNTIL(pt, timer_expired(&button_timer));
    while(digitalRead(active_button) == LOW) {
      if (button_down_ms % 100 == 0) {
        // this will fire the initial '0ms' and then every 100ms after
        if (active_button == PIN_BUTTON_LEFT ||
            active_button == PIN_BUTTON_RIGHT) {
          change_display(active_button, button_down_ms);
        } else {
          display_panel[active_display]->press(active_button, button_down_ms);
        }
      }
      timer_set(&button_timer, 10);
      PT_WAIT_UNTIL(pt, timer_expired(&button_timer));
      button_down_ms += 10;
    }
    if (active_button == PIN_BUTTON_LEFT ||
        active_button == PIN_BUTTON_RIGHT) {
      // no release action
    } else {
      display_panel[active_display]->release(active_button);
    }
    // debounce release
    timer_set(&button_timer, 10);
    PT_WAIT_UNTIL(pt, timer_expired(&button_timer));
  }
  PT_END(pt);
}

static int filter_pid_output(int duty_cycle) {
  // May need to adjust, but for now don't allow the boiler to be on
  // when the temperature is high, and don't allow sub-400ms cycles.

  if (current_temperature >= set_temperature) {
    return 0;
  } else if (duty_cycle > 0 && duty_cycle < 400) {
    // turn on for a minimum of 400ms.
    return 400;
  } else if (duty_cycle > RELAY_PERIOD - 400) {
    // when duty_cycle is close to full relay period, use the full period.
    return RELAY_PERIOD;
  } else {
    return duty_cycle;
  }
}

static int update_relay(struct pt *pt) {
  static s_timer relay_timer;
  static int pid_duty_cycle;
  PT_BEGIN(pt);
  while (1) {
    pid_duty_cycle = filter_pid_output((int)pid_output);
    if(pid_duty_cycle > 0) {
      digitalWrite(PIN_RELAY_CONTROL, HIGH);
      Serial.print(FLASH("RELAY ON "));
      Serial.println(pid_duty_cycle);
      timer_set(&relay_timer, pid_duty_cycle);
      PT_WAIT_UNTIL(pt, timer_expired(&relay_timer));
    }
    if (RELAY_PERIOD - pid_duty_cycle > 0) {
      digitalWrite(PIN_RELAY_CONTROL, LOW);
      Serial.print(FLASH("RELAY OFF "));
      Serial.println(RELAY_PERIOD - pid_duty_cycle);
      timer_set(&relay_timer, RELAY_PERIOD - pid_duty_cycle);
      PT_WAIT_UNTIL(pt, timer_expired(&relay_timer));
    }
  }
  PT_END(pt);
}

static void get_temperature(void) {
  double tmp_c = thermocouple.readCelsius();
  if (isnan(tmp_c)) {
    Serial.println("temperature is NAN!");
    return;
  }
  current_temperature = (1.8 * tmp_c) + 32;
}

static void enable_alarm(const __FlashStringHelper *msg) {
  if (millis() < 5000) {
    // Only enable alarms when we have been on for five seconds.
    // TODO: Remove if check_alarm() gets smarter.
    return;
  }
  active_alarm = true;
  digitalWrite(PIN_RELAY_CONTROL, LOW);  // disable solid state relay
  lcd.clear();
  lcd.print(msg);
  display_current_temperature();
  Serial.print("ALARM: ");
  Serial.println(msg);
}

static void check_alarm(void) {
  // TODO: Consider a 'warning' for 5 seconds and then it becomes an alarm.
  if (active_alarm) {
    display_current_temperature();
  } else if (current_temperature >= 300) {
    // TODO: Verify steam temperature.
    enable_alarm(FLASH("TEMP TOO HOT"));
  } else if (current_temperature < 40) {
    // Should never be this low, probe not working?
    enable_alarm(FLASH("TEMP TOO LOW"));
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN_LED_BUILTIN, OUTPUT);
  pinMode(PIN_RELAY_CONTROL, OUTPUT);
  pinMode(PIN_BUTTON_UP, INPUT_PULLUP);
  pinMode(PIN_BUTTON_DOWN, INPUT_PULLUP);
  pinMode(PIN_BUTTON_LEFT, INPUT_PULLUP);
  pinMode(PIN_BUTTON_RIGHT, INPUT_PULLUP);

  digitalWrite(PIN_RELAY_CONTROL, LOW);

  PT_INIT(&pt_led);
  PT_INIT(&pt_button_watch);
  PT_INIT(&pt_update_display);
  PT_INIT(&pt_relay);

  lcd.begin(16, 2);  // also clears the display

  get_temperature();            // current temperature (Pv)
  set_temperature = 224.0;      // desired temperature (Sv)

  pid_controller.SetOutputLimits(0, RELAY_PERIOD);
  pid_controller.SetMode(AUTOMATIC);

  display_panel[active_display]->show();

  // reset microcontroller if 8 seconds elapse between calls to wdt_reset()
  wdt_enable(WDTO_8S);
}

void loop() {
  wdt_reset();  // Reset watchdog timer.

  // See if any of our protothreads have work to do.
  blink_led(&pt_led);
  get_temperature();
  check_alarm();
  if (active_alarm == false) {
    button_watcher(&pt_button_watch);
    update_display(&pt_update_display);

    pid_controller.Compute();
    update_relay(&pt_relay);
  }
  // TODO: sleep microcontroller for 10ms
}
