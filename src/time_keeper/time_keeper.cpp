#include "time_keeper.h"

static uint32_t currentTimeSec; // current time in seconds since midnight

void timebaseInit() {
  currentTimeSec = 0; // Initialize time to midnight
}

void timebaseTick() {
  currentTimeSec++;
  if (currentTimeSec >= 86400) { // Wrap around after 24 hours
    currentTimeSec = 0;
  }
}

void setTimeSecondsOfTheDay(uint32_t seconds) {
  currentTimeSec = seconds % SECONDS_PER_DAY;
}

uint32_t getTime() {
  return currentTimeSec;
}

void timeAdjustSeconds(int32_t deltaSeconds){
// Adjust the current time by a given number of seconds (can be negative). 
 int32_t timeIntermediate = (static_cast<int32_t>(currentTimeSec) + deltaSeconds) % SECONDS_PER_DAY;
  
    if (timeIntermediate < 0) {
        timeIntermediate += SECONDS_PER_DAY; // Wrap around for negative values
    }
    currentTimeSec = static_cast<uint32_t>(timeIntermediate);
}