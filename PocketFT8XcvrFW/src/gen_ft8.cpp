/*
 * gen_ft8.c
 *
 *  Created on: Oct 24, 2019
 *      Author: user
 */

#include "gen_ft8.h"

#include <TimeLib.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "HX8357_t3n.h"
#include "DEBUG.h"
#include "constants.h"
#include "encode.h"
#include "pack.h"
extern HX8357_t3n tft;

#include <string.h>
#include "arm_math.h"
#include "decode_ft8.h"
#include "locator.h"
#include "UserInterface.h"
#include "msgTypes.h"
#include "traffic_manager.h"
#include "PocketFT8Xcvr.h"



char Target_Call[7];     // six character call sign + /0
int Target_RSL;          // their RSL

char message[18];   // FT8 message text pending transmission.
int message_state;  // Non-zero => message[] is valid/ready (but nothing checks it???)

extern time_t getTeensy3Time();
extern UserInterface ui;

/**
 * Setup required parameters for constructing messages to remote target station
 *
 * @param targetCall The remote station's callsign
 * @param rsl The remote station's RSL report
 *
 *
 **/
void setXmitParams(char* targetCall, int rsl) {
    strlcpy(Target_Call, targetCall, sizeof(Target_Call));
    Target_RSL = rsl;
}

/**
 * Retrieves the outbound message char[] string
 *
 * @return pointer to the outbound message char[] string
 *
 **/
char* get_message() {
    return message;
}

/**
 *  Builds an outbound FT8 message[] for later transmission
 *
 *  Constructs and displays the specified FT8 outbound message and sets the
 *  message_state flag indicating message[] is valid.  Does not actually
 *  start the transmitter.
 *
 *  Global variable usage includes:
 *    Station_Call[] -- Our station's callsign
 *    Locator[] --      Our station's maidenhead gridsquare
 *    TargetCall --     Worked station's callsign
 *    TargetRSL --      Worked station's Received Signal Level
 *    message[] --      The outbound message is constructed here
 *    message_state --  Status of message[]: 0==Invalid, 1==Valid
 *
 *  @param index Specifies the outbound FT8 message type, e.g.
 *              0 -- CQ KQ7B DN15
 *              1 -- AG0E KQ7B DN15
 *              2 -- AG0E KQ7B -12
 *              3 -- AG0E KQ7B RR73
 *              4 -- AG0E KQ7B R-8
 *              5 -- AG0E KQ7B RRR
 *
 **/
void set_message(uint16_t index) {
    uint8_t packed[K_BYTES];
    char seventy_three[] = "RR73";

    getTeensy3Time();
    char rtc_string[10];  // print format stuff
    snprintf(rtc_string, sizeof(rtc_string), "%2i:%2i:%2i", hour(), minute(), second());

    clearOutboundMessageText();
    clearOutboundMessageDisplay();

    switch (index) {
        case MSG_CQ:  // We are calling CQ from our Locator, e.g. CQ KQ7B DN15
            snprintf(message, sizeof(message), "%s %s %s", "CQ", thisStation.getCallsign(), thisStation.getLocator());
            break;

        case MSG_LOC:  // We are calling target from our Locator, e.g. AG0E KQ7B DN15
            snprintf(message, sizeof(message), "%s %s %s", Target_Call, thisStation.getCallsign(), thisStation.getLocator());
            break;

        case MSG_RSL:  // We are responding to target with their signal report, e.g. AG0E KQ7B -12
            snprintf(message, sizeof(message), "%s %s %i", Target_Call, thisStation.getCallsign(), Target_RSL);
            break;

        case MSG_RR73:  // We are responding to target with RR73, e.g. AG0E KQ7B RR73
            snprintf(message, sizeof(message), "%s %s %3s", Target_Call, thisStation.getCallsign(), seventy_three);
            break;

        case MSG_73:  // We are responding to target with 73, e.g. AG0E KQ7B 73
            snprintf(message, sizeof(message), "%s %s %s", Target_Call, thisStation.getCallsign(), "73");
            break;

        case MSG_RRSL:  // We are responding with Roger and their RRSL signal report, e.g. AG0E KQ7B R-3
            snprintf(message, sizeof(message), "%s %s R%i", Target_Call, thisStation.getCallsign(), Target_RSL);
            break;

        case MSG_RRR:  // We are responding with RRR, e.g. AG0E KQ7B RRR
            snprintf(message, sizeof(message), "%s %s RRR", Target_Call, thisStation.getCallsign());
            break;

        default:
            DPRINTF("***** ERROR:  Invalid set_message(%d) index\n", index);
            break;
    }
    DPRINTF("message='%s'\n", message);
    ui.applicationMsgs->setText(message);

    // TODO:  Check for empty message???
    pack77_1(message, packed);
    genft8(packed, tones);

    message_state = 1;

}  // set_message()

/**
 * @brief Set up a 13-char (max) free text for transmission
 * @param freeText The message
 */
void set_message(char* freeText) {
    uint8_t packed[K_BYTES];

    DPRINTF("set_message(%s)\n", freeText);

    // Ignore nonsense
    if ((freeText == NULL) || (strlen(freeText) == 0)) return;

    // strlcpy(message, blank, sizeof(message));
    clearOutboundMessageText();
    clearOutboundMessageDisplay();

    snprintf(message, sizeof(message), "%s", freeText);
    DPRINTF("message='%s'\n", message);
    ui.applicationMsgs->setText(message);

    // Prepare the outbound message as an array of tones for the FSK modulator
    packtext77(message, packed);  // Pack text into compressed bits
    genft8(packed, tones);        // Generate the FT8 tones for modulator

    message_state = 1;

}

void clearOutboundMessageText(void) {
    message[0] = 0;
}

void clearOutboundMessageDisplay(void) {
    message_state = 0;
}

