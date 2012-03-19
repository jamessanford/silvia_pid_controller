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

#define PIN_BUTTON_UP 2
#define PIN_BUTTON_DOWN 4

// Our protothreads.
static struct pt pt_watchdog;
static struct pt pt_led;
static struct pt pt_led2;
static struct pt pt_serial;
static struct pt pt_button_watch;
static struct pt pt_relay;
static struct pt pt_faketemp;

static int active_button = 0;  // set by button_watcher() -> button_pressed()
static int relay_active = 0;   // set by update_relay()

// pid_controller will take 'current_temperature' and give us a duty cycle
// on 'pid_output' (between 0 and RELAY_PERIOD,
// as it tries to close in on 'set_temperature'
static double current_temperature = 0;
static double set_temperature = 224.0;
static double pid_output = 0;

// Tune these for how fast the boiler heats up and how slowly the water cools.
static double Kp = 45.0, Ki = 10, Kd = 0;
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

static int reset_watchdog(struct pt *pt) {
  static s_timer watchdog_timer;
  PT_BEGIN(pt);
  while(1) {
    timer_set(&watchdog_timer, 1000);
    PT_WAIT_UNTIL(pt, timer_expired(&watchdog_timer));
    wdt_reset();
  }
  PT_END(pt);
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

static int serial2000(struct pt *pt) {
  static s_timer serialtimer;
  PT_BEGIN(pt);
  while(1) {
    timer_set(&serialtimer, 2000);
    PT_WAIT_UNTIL(pt, timer_expired(&serialtimer));
    Serial.print("Delay ");
    Serial.println(millis() - serialtimer.start);
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
  else {
    active_button = 0;
  }
  return active_button;
}

void button_event(int button, unsigned long down_ms) {
  // Repeat every 800ms.  We get called every 100ms, but we don't see every
  // single millisecond, so divide by 100 and then test if multiple of 8.
  if (down_ms == 0 || (down_ms / 100) % 8 == 0) {
    if (button == PIN_BUTTON_DOWN) {
      current_temperature -= 20;
      if(current_temperature < 150) {
        current_temperature = 150;  // limit button range
      }
    }
    else if (button == PIN_BUTTON_UP) {
      current_temperature += 10;
      if (current_temperature > 300) {
        current_temperature = 300;  // limit button range
      }
    }
    Serial.print("Temperature ");
    Serial.println(current_temperature);
  }
}

static int button_watcher(struct pt *pt) {
  static s_timer button_timer;
  static unsigned long button_down_ms;
  PT_BEGIN(pt);
  while(1) {
    button_down_ms = 0;
    PT_WAIT_UNTIL(pt, button_pressed());
    timer_set(&button_timer, 10);
    PT_WAIT_UNTIL(pt, timer_expired(&button_timer));
    while(digitalRead(active_button) == HIGH) {
      if (button_down_ms % 100 == 0) {
        // this will fire the initial '0ms' and then every 100ms after
        button_event(active_button, button_down_ms);
      }
      timer_set(&button_timer, 10);
      PT_WAIT_UNTIL(pt, timer_expired(&button_timer));
      button_down_ms += 10;
    }
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
    if (pid_duty_cycle != RELAY_PERIOD) {
      relay_active = 0;
      Serial.print("RELAY OFF ");
      Serial.println(RELAY_PERIOD - pid_duty_cycle);
      timer_set(&relay_timer, RELAY_PERIOD - pid_duty_cycle);
      PT_WAIT_UNTIL(pt, timer_expired(&relay_timer));
    }
  }
  PT_END(pt);
}

// Fake boiler heater and tank that flucuates temperature.
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
  pinMode(PIN_BUTTON_UP, INPUT);
  pinMode(PIN_BUTTON_DOWN, INPUT);

  PT_INIT(&pt_watchdog);
  PT_INIT(&pt_led);
  PT_INIT(&pt_led2);
  PT_INIT(&pt_serial);
  PT_INIT(&pt_button_watch);
  PT_INIT(&pt_relay);
  PT_INIT(&pt_faketemp);

  current_temperature = 180.0;  // fake boiler temperature
  set_temperature = 224.0;      // desired temperature

  pid_controller.SetOutputLimits(0, RELAY_PERIOD);
  pid_controller.SetMode(AUTOMATIC);

  // reset microcontroller if 8 seconds elapse between calls to wdt_reset()
  wdt_enable(WDTO_8S);
}

void loop() {
  reset_watchdog(&pt_watchdog);
  blink_led(&pt_led);
  blink_led2(&pt_led2);
  serial2000(&pt_serial);
  button_watcher(&pt_button_watch);

  // not entirely convinced about calling Compute so often
  // yet discarding the resulting duty cycle?  check the PID code.
  pid_controller.Compute();
  update_relay(&pt_relay);

  // make the temp float around depending on relay status and time
  update_faketemp(&pt_faketemp);
}
