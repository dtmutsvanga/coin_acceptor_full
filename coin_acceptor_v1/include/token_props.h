#ifndef TOKEN_PROPS_H
#define TOKEN_PROPS_H
#include <stdint.h>
#include "bsp.h"
typedef struct token
{
	uint8_t coin_num;
	uint16_t SD_daim;
	uint16_t avg_diam;
	uint16_t SD_em;
	uint16_t avg_em;
} token_t;


#endif //TOKEN_PROPS_H
