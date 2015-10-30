#include <stdio.h>
#include <stdlib.h>

#define SETUP_HANDLER 0

short int CURRENT_STATE = SETUP_HANDLER;

void setupHandler() {
  // Hardware definitions
  // Setup next state
}

void loop() {
  while(1) {
    switch(CURRENT_STATE) {
      case SETUP_HANDLER: setupHandler(); break;
    }
  }
}

int main(void) {
  loop();
  return 0;
}
