/*
 * gen_ft8.h
 *
 *  Created on: Oct 30, 2019
 *      Author: user
 */

#ifndef GEN_FT8_H_
#define GEN_FT8_H_

#include "arm_math.h"

char* get_message();
void set_message(uint16_t index);
void set_message(char* freeText);
void clearOutboundMessageDisplay(void);
void setXmitParams(char* targetStation, int snr);
void clearOutboundMessageText(void);

#endif /* GEN_FT8_H_ */
