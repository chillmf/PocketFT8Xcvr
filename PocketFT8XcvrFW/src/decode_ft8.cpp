/*
 * decode_ft8.c
 *
 *  Created on: Sep 16, 2019
 *      Author: user
 */

#include "decode_ft8.h"

#include <Arduino.h>
#include <TimeLib.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "DEBUG.h"
#include "HX8357_t3n.h"
#include "PocketFT8Xcvr.h"
#include "Process_DSP.h"
#include "Sequencer.h"
#include "UserInterface.h"
#include "constants.h"
#include "decode.h"
#include "encode.h"
#include "gen_ft8.h"
#include "ldpc.h"
#include "unpack.h"

extern HX8357_t3n tft;

char erase[] = "                   ";

const int kLDPC_iterations = 10;
const int kMax_candidates = 20;
const int kMax_decoded_messages = 9;  // chhh 27 feb
const int kMax_message_length = 24;   // Was 22 (KQ7B)

const int kMin_score = 40;  // Minimum sync score threshold for candidates (40)

int validate_locator(char locator[]);
int strindex(const char s[], const char t[]);

extern uint32_t ft8_time;
extern uint8_t export_fft_power[ft8_msg_samples * ft8_buffer * 4];

extern int ND;
extern int NS;

extern int NN;
// Define the LDPC sizes
extern int N;
extern int K;

extern int M;

extern int K_BYTES;

extern UserInterface ui;

Decode new_decoded[20];

Calling_Station Answer_CQ[100];
CQ_Station Calling_CQ[8];

int num_calls;  // number of unique calling stations
int num_call_checks;
int num_CQ_calls;
int num_calls_to_CQ_station;
int message_limit = DISPLAY_DECODED_LINES;

int max_Calling_Stations = DISPLAY_DECODED_LINES;
int num_Calling_Stations;

extern float Station_Latitude, Station_Longitude;

extern float Target_Latitude, Target_Longitude;

extern float Target_Distance(char target[]);

extern int CQ_State;

extern char Target_Call[7];
extern int Target_RSL;  // four character RSL  + /0

extern time_t getTeensy3Time();
extern int log_flag, logging_on;

// Get a reference to the Sequencer singleton
static Sequencer &seq = Sequencer::getSequencer();

/**
 *  Retrieve address of the new_decoded[] messages
 *
 *
 **/
Decode *getNewDecoded() {
    return new_decoded;
}

/**
 * Decode received->FT8 signals into new_decoded[] of successfully decoded messages (if any)
 *
 * @return Number of successfully demodulated messages placed in new_decoded[]
 *
 * new_decoded[] can hold a hard-wired maximum of 20 messages.
 **/
int ft8_decode(void) {
    // DTRACE();

    // Find top candidates by Costas sync score and localize them in time and frequency
    Candidate candidate_list[kMax_candidates];
    int num_candidates = find_sync(export_fft_power, ft8_msg_samples, ft8_buffer, kCostas_map, kMax_candidates, candidate_list, kMin_score);
    char decoded[kMax_decoded_messages][kMax_message_length];

    const float fsk_dev = 6.25f;  // tone deviation in Hz and symbol rate


    // Go over candidates and attempt to decode their messages
    int num_decoded = 0;

    for (int idx = 0; idx < num_candidates; ++idx) {
        Candidate cand = candidate_list[idx];
        float freq_hz = (cand.freq_offset + cand.freq_sub / 2.0f) * fsk_dev;

        float log174[N];
        extract_likelihood(export_fft_power, ft8_buffer, cand, kGray_map, log174);

        // bp_decode() produces better decodes, uses way less memory
        uint8_t plain[N];
        int n_errors = 0;
        bp_decode(log174, kLDPC_iterations, plain, &n_errors);

        if (n_errors > 0) continue;  // Skip messages that can't be decoded

        // Extract payload + CRC (first K bits)
        uint8_t a91[K_BYTES];      // Bfr for the received message's packed bits
        pack_bits(plain, K, a91);  // Pack K bits into a91[] from K bool bytes in plain[]

        // Extract CRC and verify it with the computed CRC
        uint16_t chksum = ((a91[9] & 0x07) << 11) | (a91[10] << 3) | (a91[11] >> 5);  // Extracted CRC from transmitted message
        a91[9] &= 0xF8;
        a91[10] = 0;
        a91[11] = 0;
        uint16_t chksum2 = crc(a91, 96 - 14);  // Computed CRC for message as actually received
        if (chksum != chksum2) continue;       // Skip messages whose CRCs don't match

        // We have finally decoded the FT8 message bits and verified a valid CRC.  The message looks good.
        // Now we can unpack the FT8 encoding (see reference) into human-readable fields.
        char message[kMax_message_length];
        char field1[14];
        char field2[14];
        char field3[7];
        MsgType msgType;
        int rc = unpack77_fields(a91, field1, field2, field3, &msgType);
        if (rc < 0) continue;  // Unpack failure???

        snprintf(message, sizeof(message), "%s %s %s ", field1, field2, field3);

        // Check for duplicate messages (TODO: use hashing)
        bool found = false;
        for (int i = 0; i < num_decoded; ++i) {
            if (0 == strcmp(decoded[i], message)) {
                found = true;
                break;
            }
        }

        int raw_RSL;
        int display_RSL;
        float distance;

        getTeensy3Time();
        char rtc_string[10];  // print format stuff
        snprintf(rtc_string, sizeof(rtc_string), "%02i:%02i:%02i", hour(), minute(), second());

        if (!found && num_decoded < kMax_decoded_messages) {
            if (strlen(message) < kMax_message_length) {
                strlcpy(decoded[num_decoded], message, kMax_message_length);

                new_decoded[num_decoded].sync_score = cand.score;
                new_decoded[num_decoded].freq_hz = (int)freq_hz;
                strlcpy(new_decoded[num_decoded].field1, field1, 14);  // Destination station
                strlcpy(new_decoded[num_decoded].field2, field2, 14);  // Source station
                strlcpy(new_decoded[num_decoded].field3, field3, 7);   // Extra info passed to destination from source
                strlcpy(new_decoded[num_decoded].decode_time, rtc_string, 10);

                raw_RSL = new_decoded[num_decoded].sync_score;
                if (raw_RSL > 160) raw_RSL = 160;
                display_RSL = (raw_RSL - 160) / 6;
                new_decoded[num_decoded].snr = display_RSL;  // Their received signal level at our station
                new_decoded[num_decoded].msgType = msgType;  // Record the msgType

                char Target_Locator[] = "    ";

                // Assume field3 is a locator
                strlcpy(Target_Locator, new_decoded[num_decoded].field3, sizeof(Target_Locator));

                // Try to determine if field3 is really a locator (Note:  msgType is the preferred indicator *except* for CQ)
                if (validate_locator(Target_Locator) == 1) {
                    distance = Target_Distance(Target_Locator);
                    new_decoded[num_decoded].distance = (int)distance;
                    strlcpy(new_decoded[num_decoded].locator, Target_Locator, 7);  // Bug:  Save their perhaps-this-is-a-locator for logging
                } else {
                    new_decoded[num_decoded].distance = 0;    // We don't know distance to target
                    new_decoded[num_decoded].locator[0] = 0;  // We don't have a valid locator for target
                }

                // Inform QSO sequencer about newly received message
                new_decoded[num_decoded].sequenceNumber = seq.getSequenceNumber();
                seq.receivedMsgEvent(&new_decoded[num_decoded]);
                ++num_decoded;
            }
        }
    }  // End of big decode loop

    return num_decoded;

}  // ft8_decode()

/**
 * Display decoded received messages, if any, on the LCD (left side)
 *
 * @param decoded_messages Number of successfully decoded messages in new_decoded[] array
 *
 * The size of the LCD's message display region limits the maximum number of displayed
 * messages to message_limit=6.  When the number of decoded messages exceeds what can
 * be displayed, only the first message_limit messages appear.
 *
 * The LCD display region is rectangular, 240 pixels wide and 140 pixels high.  Text size 2
 * produces 12X16 (widthXheight) pixel characters.
 *
 * Warning:  fillRect() is amazingly time-consuming.  Thus, we erase old messages by
 * printing space chars.
 **/
static const unsigned lineHeight = TEXT2_LINE_HEIGHT;  // Height in pixels of one line of text (including leading)
static int previousMessageCount = 0;                   // Number of messages displayed in previous timeslot
void display_messages(int decoded_messages) {
    char message[kMax_message_length];
    // char big_gulp[60];

    // Erase the message display region on the LCD.  It turns out that fillRect() of a large region is amazingly slow, increasing the
    // risk of missing the following FT8 timeslot.  So... we erase with space characters.
    // DTRACE();
    // tft.fillRect(DISPLAY_DECODED_X, DISPLAY_DECODED_Y, DISPLAY_DECODED_W, DISPLAY_DECODED_H, HX8357_BLACK);

    // Display info about each decoded message.  field1 is receiving station's callsign or CQ, field2 is transmitting station's callsign,
    // field3 is an RSL or locator or ???.
    if (decoded_messages > 0) ui.decodedMsgs->reset();                  // Clear all the old messages
    for (int i = 0; i < decoded_messages && i <= message_limit; i++) {  // Charlie's leading handled 6 rows of text
        snprintf(message, sizeof(message), "%s %s %4s S%c", new_decoded[i].field1, new_decoded[i].field2, new_decoded[i].field3, rsl2s(new_decoded[i].snr));

        // Display messages not sent to our station in the Decoded Messages box
        if (strncmp(new_decoded[i].field1, thisStation.getCallsign(), 14) != 0) {
            AColor color = A_LIGHT_GREY;  // Chatter appears in light grey
            if (strncmp(new_decoded[i].field1, "CQ", 2) == 0) {
                color = A_WHITE;  // CQ messages appear in white
            }
            // For now, don't display messages with hashed callsigns as our FT8 library doesn't support them
            if (strchr(message, '<') == NULL) ui.decodedMsgs->addItem(ui.decodedMsgs, message, color);  // Display received message
        }
    }

    // Erase messages lines from the previous timeslot that weren't overwritten above
    message[0] = 0;                           // An empty line
    previousMessageCount = decoded_messages;  // Remember for next timeslot

}  // display_messages()



/**
 * Determine if a char[] appears to be a valid maidenhead locator
 *
 * @param locator[] The four character locator (e.g. DN15)
 *
 * @return 0==invalid, 1==valid
 *
 * Limitiation:  The code classifies the maritime location RR73, the Arctic
 * and Antarctica as invalid
 *
 **/
int validate_locator(char locator[]) {
    uint8_t A1, A2, N1, N2;
    uint8_t test = 0;

    A1 = locator[0] - 65;
    A2 = locator[1] - 65;
    N1 = locator[2] - 48;
    N2 = locator[3] - 48;

    if (A1 >= 0 && A1 <= 17) test++;
    if (A2 > 0 && A2 < 17) test++;  // block RR73 Artic and Anartica
    if (N1 >= 0 && N1 <= 9) test++;
    if (N2 >= 0 && N2 <= 9) test++;

    if (test == 4)
        return 1;
    else
        return 0;
}

int strindex(const char s[], const char t[]) {
    int i, j, k, result;

    result = -1;

    for (i = 0; s[i] != '\0'; i++) {
        for (j = i, k = 0; t[k] != '\0' && s[j] == t[k]; j++, k++);
        if (k > 0 && t[k] == '\0')
            result = i;
    }
    return result;
}

/**
 * Displays decoded messages received from stations calling my station, if any, in right-side window
 *
 * @param num_decoded Number of entries in new_decoded[]
 *
 * @return -1 if no callers, else the index of last caller in new_decoded[]???
 *
 * This function checks every message addressed to our station, including messages that
 * address our station but are not "in" a QSO with us.  We display all messages addressed
 * to us (e.g. multiple replies to our CQ), but the logging package must determine what to log.
 *
 * @var new_decoded[] Array of successfully decoded messages (may or may not be addressed to us)
 *
 **/
int Check_Calling_Stations(int num_decoded) {
    char big_gulp[60];
    char message[kMax_message_length];
    int message_test = 0;

    // Loop executed once for each entry in new_decoded[] of received messages
    for (int i = 0; i < num_decoded; i++) {
        // Was this received message sent to our station?
        if (strindex(new_decoded[i].field1, thisStation.getCallsign()) >= 0) {
            // Yes, assemble details (their callsign, our callsign, extra_info) into message buffer
            snprintf(message, sizeof(message), "%s %s %s", new_decoded[i].field1, new_decoded[i].field2, new_decoded[i].field3);

            // Display details of received message addressed to our station
            getTeensy3Time();
            snprintf(big_gulp, sizeof(message), "%02i/%02i/%4i %s %s", day(), month(), year(), new_decoded[i].decode_time, message);
            num_Calling_Stations++;
            message_test = i + 100;  // 100+index of this calling station.  Why the 100 bias???
        }


    }

    // Return index of final calling station in new_decoded[] or -1 if none????????????????????????
    if (message_test > 100)
        return message_test - 100;
    else {
        
        return -1;
    }

    

}  // Check_Calling_Stations()

/**
 * @brief Translate RSL (in dB) to S-Level
 * @param rsl Signal-to-Noise level in dB
 * @return Pointer to S-Level digit, 1..9
 *
 * WSJTX reports FT8 received signal levels (RSL) in dB but the Pocket FT8 Revisited's display
 * doesn't have room for reports such as "-16" so we make signal reports great again by
 * translating RSL back into a single digit (e.g. 1..9) S-Level (as in RST).
 */
char rsl2s(int rsl) {
    // static const char mapRSL2S[] = "112233445566778899";  // RSL Levels -17..0 dB
    char S = ' ';  // S-Level

    if (rsl <= -17) return '1';  // Check for really weak signal
    if (rsl >= 0) return '9';    // Check for really strong signal
    switch (rsl) {
        case -17:
        case -16:
            S = '1';
            break;
        case -15:
        case -14:
            S = '2';
            break;
        case -13:
        case -12:
            S = '3';
            break;
        case -11:
        case -10:
            S = '4';
            break;
        case -9:
        case -8:
            S = '5';
            break;
        case -7:
        case -6:
            S = '6';
            break;
        case -5:
        case -4:
            S = '7';
            break;
        case -3:
        case -2:
            S = '8';
            break;
        case -1:
        case 0:
            S = '9';
            break;
        default:
            break;
    }
    return S;
}  // rsl2s()

