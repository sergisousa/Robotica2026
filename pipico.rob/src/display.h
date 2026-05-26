#ifndef DISPLAY_H
#define DISPLAY_H

void display_setup(void);

void display_loop(void);

// constants and variable to allow the main code running on core 0 to set the
// display that is being controlled by core 1
enum {
  SCREEN_MAIN = 0,
  SCREEN_MAP,
  SCREEN_DATA,
};
static const int SCREEN_COUNT = 3;

extern volatile int current_screen;

#endif
