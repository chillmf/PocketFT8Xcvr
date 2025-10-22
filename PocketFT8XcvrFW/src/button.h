/**
 * Defines button handles and functions
 */

#pragma once
#include <Arduino.h>

//-----Define button handles
#define BUTTON_CQ 0  // CQ
#define BUTTON_AB 1  // ABORT
#define BUTTON_TU 2  // TUNE
#define BUTTON_TX 3  // Reserved
#define BUTTON_M1 4  // Custom
#define BUTTON_M2 5  // Custom
#define BUTTON_M3 6  // Custom
#define BUTTON_M4 7  // Custom
#define BUTTON_SY 8  // Reserved

void set_startup_freq(void);
void terminate_transmit_armed(void);

