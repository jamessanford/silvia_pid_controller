/*
 * An API similar to an LCD that dumps to the serial port.
 * Copyright 2012 James Sanford and Dan Lange.
 * All rights reserved.
 *
 * Use, modification, and redistribution of this software is permitted
 * provided that this notice is retained with the software.
 * This software is without warranty.
 */

#include <Arduino.h>

#ifndef FAKELCD_H
#define FAKELCD_H

// Fake LCD panel class.  Should be in a library.
class FakeLCD : public Print {
#define LCD_WIDTH_MAX 20
#define LCD_HEIGHT_MAX 2
  public:
   FakeLCD(int w, int h);
   void setCursor(int x, int y);
   void show(void);
   void clear(void);
   virtual size_t write(uint8_t value);

  private:
   int width;
   int height;
   int cursorX;
   int cursorY;
   uint8_t paneldata[LCD_WIDTH_MAX * LCD_HEIGHT_MAX];  // eww
};
#endif  // FAKELCD_H
