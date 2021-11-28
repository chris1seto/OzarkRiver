#ifndef TOKENITERATE_H
#define TOKENITERATE_H

#include <stdint.h>

typedef struct
{
  uint8_t* Buffer;
  uint32_t Size;
  uint8_t Delimiter;
  uint32_t Index;
} TokenIterate_t;

void TokenIterator_Init(TokenIterate_t* t, const uint8_t delim, const uint8_t* ptr, const uint32_t size);
uint8_t TokenIterator_Forward(TokenIterate_t* t, uint8_t** token, uint32_t* length);

#endif