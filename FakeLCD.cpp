/*
 * An API similar to an LCD that dumps to the serial port.
 * Copyright 2012 James Sanford and Dan Lange.
 * All rights reserved.
 *
 * Use, modification, and redistribution of this software is permitted
 * provided that this notice is retained with the software.
 * This software is without warranty.
 */

#include "FakeLCD.h"

#define FLASH(__s) F(__s)

FakeLCD::FakeLCD(int w, int h) {
  width = min(LCD_WIDTH_MAX, w);
  height = min(LCD_HEIGHT_MAX, h);
  cursorX = 0;
  cursorY = 0;
  clear();
}

void FakeLCD::setCursor(int x, int y) {
  cursorX = max(0, min(width - 1, x));
  cursorY = max(0, min(height - 1, y));
}

void FakeLCD::show(void) {
  int x;
  int y;
  Serial.print(FLASH("* \r\n* "));
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      Serial.print((char)paneldata[(y * width) + x]);
    }
    Serial.print(FLASH(" *\r\n* "));
  }
  Serial.print(FLASH("\r\n\r\n"));
}

void FakeLCD::clear(void) {
  cursorX = 0;
  cursorY = 0;
  int offset;
  for (offset = 0; offset < (width * height); offset++) {
    paneldata[offset] = 32;  // space, 0x20
  }
}

size_t FakeLCD::write(uint8_t value) {
  paneldata[(cursorY * width) + cursorX] = value;
  cursorX += 1;
  cursorX = min(width - 1, cursorX);
  // no wrapping to next line
  return 1;
}
