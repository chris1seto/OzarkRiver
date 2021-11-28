#include <stdint.h>
#include  "TokenIterate.h"
void TokenIterator_Init(TokenIterate_t* t, const uint8_t delim, const uint8_t* ptr, const uint32_t size)
{
  t->Buffer = (uint8_t*)ptr;
  t->Delimiter = delim;
  t->Size = size;
  t->Index = 0;
}

// Go from our current position and get the next token
uint8_t TokenIterator_Forward(TokenIterate_t* t, uint8_t** token, uint32_t* length)
{
  uint32_t tokenStart = t->Index;

  // Check if we're at the end of the string
  if (t->Index >= t->Size)
  {
    return 0;
  }

  // Get the end of this token
  while (t->Index < t->Size)
  {
    if (t->Buffer[t->Index++] == t->Delimiter)
    {
      break;
    }
  }

  if (t->Index < t->Size)
  {
    *length = t->Index - tokenStart - 1;
  }
  else
  {
    *length = t->Index - tokenStart;
  }

  *token = &t->Buffer[tokenStart];

  return 1;
}