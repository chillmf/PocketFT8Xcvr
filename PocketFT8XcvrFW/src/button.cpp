
#include "button.h"

#include <Arduino.h>

#include "HX8357_t3n.h"
#include "NODEBUG.h"
#include "decode_ft8.h"
#include "gen_ft8.h"
#include "traffic_manager.h"
#include <EEPROM.h>
#include <SI4735.h>
#include <Wire.h>
#include "Process_DSP.h"
#include "Sequencer.h"
#include "TouchScreen_I2C.h"
#include "msgTypes.h"
#include "pins.h"

#define numButtons 9
#define button_height BUTTON_BAR_H  // TODO:  Clean-up old definitions for height and top line location
#define button_line BUTTON_BAR_Y
#define button_width 42
#define IOEXP16_ADDR 0x24

// Define which I2C bus we are using
#define WIRE WIRE_ETC

// This is calibration data for the raw touch data to the screen coordinates
// using 510 Ohm resistors to reduce the driven voltage to Y+ and X-
#define TS_MINX 123
#define TS_MINY 104
#define TS_MAXX 1715
#define TS_MAXY 1130

#define MINPRESSURE 120
#define PENRADIUS 3

extern HX8357_t3n tft;
extern TouchScreen ts;

extern int Transmit_Armned;

extern SI4735 si4735;
#define USB 2

uint16_t draw_x, draw_y, touch_x, touch_y;
int test;

extern int master_decoded;
extern void sync_FT8(void);
extern int tune_flag;

const float ft8_shift = 6.25;  // FT8 Hz/bin???
extern int log_flag, logging_on;

// Get a reference to the Sequencer singleton
static Sequencer& seq = Sequencer::getSequencer();






/**
 * @brief Dis-arms the transmitter, switches from xmit t recv, and clears the outbound FT8 message
 */
void terminate_transmit_armed(void) {
    Transmit_Armned = 0;
    receive_sequence();
}







