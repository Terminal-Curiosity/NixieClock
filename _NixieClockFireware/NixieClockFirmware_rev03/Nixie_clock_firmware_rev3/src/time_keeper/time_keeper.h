#pragma once

#include <stdint.h>
constexpr int32_t SECONDS_PER_DAY = 86400;

void timebaseInit();
void timebaseTick();
void setTimeSecondsOfTheDay(uint32_t seconds);
uint32_t getTime();