/*
 * Arduino PID controller for boiler temperature on Rancilio Silvia.
 * Copyright 2012 James Sanford and Dan Lange.
 * All rights reserved.
 *
 * Use, modification, and redistribution of this software is permitted
 * provided that this notice is retained with the software.
 * This software is without warranty.
 */

// protothreads, external library in Documents/Arduino/libraries/pt/pt.h
#include <pt.h>

// github.com/br3ttb/Arduino-PID-Library in Documents/Arduino/libraries/PID_v1
#include <PID_v1.h>

// AVR watchdog reset
#include <avr/wdt.h>

// Change the relay in 1000ms increments, this value is just for testing.
#define RELAY_PERIOD 1000

#define PIN_LED_BUILTIN 13
#define PIN_LED_EXTERNAL 3
#define PIN_RELAY_CONTROL 7

#define PIN_BUTTON_UP 2
#define PIN_BUTTON_DOWN 4
#define PIN_BUTTON_LEFT 5
#define PIN_BUTTON_RIGHT 6

#define MIN_SET_TEMP 150
#define MAX_SET_TEMP 300

// Our protothreads.
static struct pt pt_led;
static struct pt pt_led2;
static struct pt pt_button_watch;
static struct pt pt_update_display;
static struct pt pt_relay;
static struct pt pt_faketemp;

static int active_button = 0;  // set by button_watcher() -> button_pressed()
static int relay_active = 0;   // set by update_relay()

// pid_controller will take 'current_temperature' and give us a duty cycle
// on 'pid_output' (between 0 and RELAY_PERIOD),
// as it tries to close in on 'set_temperature'
static double current_temperature = 0;
static double set_temperature = 224.0;
static double pid_output = 0;

// Tune these for how fast the boiler heats up and how slowly the water cools.
// static double Kp = 45.0, Ki = 10, Kd = 0;
static double Kp = 90.0, Ki = 10, Kd = 0;
PID pid_controller(&current_temperature,
                   &pid_output,
                   &set_temperature,
                   Kp, Ki, Kd, DIRECT);

// Timer from protothreads example.
struct s_timer { unsigned long start, interval; };

static void timer_set(struct s_timer *t, int interval) {
  t->interval = interval;
  t->start = millis();
}

static int timer_expired(struct s_timer *t) {
  return (t->start + t->interval) <= millis();
}

// Fake LCD panel class.  Should be in a library.
class FakeLCD : public Print {
#define LCD_WIDTH_MAX 20
#define LCD_HEIGHT_MAX 2
  public:
   FakeLCD(int w, int h) {
     width = min(LCD_WIDTH_MAX, w);
     height = min(LCD_HEIGHT_MAX, h);
     cursorX = 0;
     cursorY = 0;
     clear();
   }

   void setCursor(int x, int y) {
     cursorX = max(0, min(width - 1, x));
     cursorY = max(0, min(height - 1, y));
   }

   void show(void) {
     int x;
     int y;
     Serial.print("* \r\n* ");
     for (y = 0; y < height; y++) {
       for (x = 0; x < width; x++) {
         Serial.print((char)paneldata[(y * width) + x]);
       }
       Serial.print(" *\r\n* ");
     }
     Serial.print("\r\n\r\n");
   }

   void clear(void) {
     cursorX = 0;
     cursorY = 0;
     int offset;
     for (offset = 0; offset < (width * height); offset++) {
       paneldata[offset] = 32;  // space, 0x20
     }
   }

   virtual size_t write(uint8_t value) {
     paneldata[(cursorY * width) + cursorX] = value;
     cursorX += 1;
     cursorX = min(width - 1, cursorX);
     // no wrapping to next line
     return 1;
   }

  private:
   int width;
   int height;
   int cursorX;
   int cursorY;
   uint8_t paneldata[LCD_WIDTH_MAX * LCD_HEIGHT_MAX];  // eww
};

FakeLCD fake_lcd(20, 2);

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

static int blink_led2(struct pt *pt) {
  static s_timer led_timer2;
  PT_BEGIN(pt);
  while(1) {
    timer_set(&led_timer2, 500);
    PT_WAIT_UNTIL(pt, timer_expired(&led_timer2));
    toggle_led(PIN_LED_EXTERNAL);
  }
   PT_END(pt);
}

// Return the pin number of the first button pressed down that we see.
// 'active_button' is a global used by others.
static int button_pressed(void) {
  if (digitalRead(PIN_BUTTON_UP) == HIGH) {
    active_button = PIN_BUTTON_UP;
  }
  else if (digitalRead(PIN_BUTTON_DOWN) == HIGH) {
    active_button = PIN_BUTTON_DOWN;
  }
  else if (digitalRead(PIN_BUTTON_LEFT) == HIGH) {
    active_button = PIN_BUTTON_LEFT;
  }
  else if (digitalRead(PIN_BUTTON_RIGHT) == HIGH) {
    active_button = PIN_BUTTON_RIGHT;
  }
  else {
    active_button = 0;
  }
  return active_button;
}

static void display_set_temperature(void) {
  fake_lcd.setCursor(0, 1);
  fake_lcd.print(set_temperature, 0);
  fake_lcd.print("F");
  fake_lcd.show();
}

static void display_current_temperature(void) {
  fake_lcd.setCursor(20 - 4, 1);
  if (current_temperature < 100) {
    fake_lcd.print(" ");
  }
  fake_lcd.print(current_temperature, 0);  // round to whole degree
  fake_lcd.print("F");
  fake_lcd.show();
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
      fake_lcd.clear();
      fake_lcd.print("Miss Silvia");
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
      Serial.print("Write ");
      Serial.print(set_temperature);
      Serial.println(" to EEPROM");
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
        _pid_min = 0;
        _pid_max = 50;
        _pid_delta = 0.5;
        _pid_x = 6;
      } else if(_pid_variable == &Kd) {
        _pid_x = 11;
      }
    }
    void refresh_pid(void) {
      fake_lcd.clear();
      fake_lcd.print(" P    I    D");
//                   "90  10.5   0"
//                    012345678901
      fake_lcd.setCursor(0, 1);
      if (Kp < 10) {
        fake_lcd.print(" ");
      }
      fake_lcd.print(Kp, 0);
      fake_lcd.setCursor(4, 1);
      if (Ki < 10) {
        fake_lcd.print(" ");
      }
      fake_lcd.print(Ki, 1);
      fake_lcd.setCursor(10, 1);
      if (Kd < 10) {
        fake_lcd.print(" ");
      }
      fake_lcd.print(Kd, 0);
      // redraw any currently-blank items, and the temperature.
      _blink_state ^= 1;  // negated by periodic();
      periodic();         // includes fake_lcd.show();
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
      Serial.print("Update EEPROM ");
      Serial.print(*_pid_letter);
      Serial.print(" = ");
      Serial.println(*_pid_variable);
    }
    virtual void periodic(void) {
      fake_lcd.setCursor(_pid_x, 0);
      _blink_state ^= 1;
      if (_blink_state) {
        fake_lcd.print(" ");
      } else {
        fake_lcd.print(*_pid_letter);
      }
      display_current_temperature();  // includes fake_lcd.show();
    }
};

class DisplayUnits : public SilviaDisplay {
  public:
    virtual void hide(void) {
    }
    virtual void show(void) {
      fake_lcd.clear();
      fake_lcd.print("Display Units");
      fake_lcd.setCursor(2, 1);
      fake_lcd.print("Fahrenheit");
//      fake_lcd.print("Celcius");
      fake_lcd.show();
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

static int active_display = 0;  // Which display_panel[] is active.
#define DISPLAY_COUNT 5
static SilviaDisplay *display_panel[DISPLAY_COUNT] = {
  new DisplayNormal(),
  new DisplayPID((char *)"P", &Kp),
  new DisplayPID((char *)"I", &Ki),
  new DisplayPID((char *)"D", &Kd),
  new DisplayUnits() };

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
    if (pin == PIN_BUTTON_LEFT) {
      active_display += 1;
      if (active_display >= DISPLAY_COUNT) {
        active_display = 0;
      }
    }
    else if (pin == PIN_BUTTON_RIGHT) {
      active_display -= 1;
      if (active_display < 0) {
        active_display = DISPLAY_COUNT - 1;
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
    Serial.print("button ");
    Serial.println(active_button);
    // debounce press
    timer_set(&button_timer, 10);
    PT_WAIT_UNTIL(pt, timer_expired(&button_timer));
    while(digitalRead(active_button) == HIGH) {
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

static int update_relay(struct pt *pt) {
  static s_timer relay_timer;
  static int pid_duty_cycle;
  PT_BEGIN(pt);
  while (1) {
    pid_duty_cycle = (int)pid_output;
    if(pid_duty_cycle > 0) {
      relay_active = 1;
      Serial.print("RELAY ON ");
      Serial.println(pid_duty_cycle);
      timer_set(&relay_timer, pid_duty_cycle);
      PT_WAIT_UNTIL(pt, timer_expired(&relay_timer));
    }
    if (RELAY_PERIOD - pid_duty_cycle > 0) {
      relay_active = 0;
      Serial.print("RELAY OFF ");
      Serial.println(RELAY_PERIOD - pid_duty_cycle);
      timer_set(&relay_timer, RELAY_PERIOD - pid_duty_cycle);
      PT_WAIT_UNTIL(pt, timer_expired(&relay_timer));
    }
  }
  PT_END(pt);
}

// Fake boiler heater and tank that fluctuates temperature.
// This implementation is only reliable if we get called every millisecond.
static int update_faketemp(struct pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    // this isn't actually reliable, but it's just for fun.
    if (relay_active && (millis() % 500 == 0)) {
      current_temperature += 1;
      Serial.print("Temperature ");
      Serial.println(current_temperature);
    }
    else if (!relay_active && (millis() % 2000 == 0)) {
      current_temperature -= 1;
      Serial.print("Temperature ");
      Serial.println(current_temperature);
    }
    PT_YIELD(pt);
  }
  PT_END(pt);
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN_LED_BUILTIN, OUTPUT);
  pinMode(PIN_LED_EXTERNAL, OUTPUT);
  pinMode(PIN_RELAY_CONTROL, OUTPUT);
  pinMode(PIN_BUTTON_UP, INPUT);
  pinMode(PIN_BUTTON_DOWN, INPUT);
  pinMode(PIN_BUTTON_LEFT, INPUT);
  pinMode(PIN_BUTTON_RIGHT, INPUT);

  PT_INIT(&pt_led);
  PT_INIT(&pt_led2);
  PT_INIT(&pt_button_watch);
  PT_INIT(&pt_update_display);
  PT_INIT(&pt_relay);
  PT_INIT(&pt_faketemp);

  current_temperature = 180.0;  // fake boiler temperature
  set_temperature = 224.0;      // desired temperature

  pid_controller.SetOutputLimits(0, RELAY_PERIOD);
  pid_controller.SetMode(AUTOMATIC);

  fake_lcd.clear();
  display_panel[active_display]->show();

  // reset microcontroller if 8 seconds elapse between calls to wdt_reset()
  wdt_enable(WDTO_8S);
}

void loop() {
  wdt_reset();  // Reset watchdog timer.

  // See if any of our protothreads have work to do.
  blink_led(&pt_led);
  blink_led2(&pt_led2);
  button_watcher(&pt_button_watch);
  update_display(&pt_update_display);

  pid_controller.Compute();
  update_relay(&pt_relay);

  // make the temp float around depending on relay status and time
  update_faketemp(&pt_faketemp);
}
