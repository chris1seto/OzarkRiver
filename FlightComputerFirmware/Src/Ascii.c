#include <stdint.h>
#include <stdio.h>
#include "Ascii.h"

uint8_t Ascii_Int2HexDigit(const uint8_t x)
{
  if (x < 10)
  {
    return x + '0';
  }

  if (x >= 10 && x <= 15)
  {
    return x + 'a';
  }

  return '.';
}

uint8_t Ascii_Hex2int(const uint8_t x)
{
  if (x >= '0' && x <= '9')
  {
    return x - '0';
  }
    
  if (x >= 'A' && x <= 'F')
  {
    return x - 'A' + 10;
  }
    
  if (x >= 'a' && x <= 'f')
  {
    return x - 'a' + 10;
  }
    
  return 0xff;
}

float Ascii_atofl(const uint8_t* buffer, const uint32_t length)
{
  float rez = 0;
  float fact = 1;
  int32_t pointSeen = 0;
  uint8_t d;
  uint32_t ptr = 0;

  if (buffer[0] == '-')
  {
    ptr++;
    fact = -1;
  }

  for (; ptr < length; ptr++)
  {
    if (buffer[ptr] == '.')
    {
      pointSeen = 1; 
      continue;
    }

    d = buffer[ptr] - '0';

    if (d >= 0 && d <= 9)
    {
      if (pointSeen)
      {
        fact /= 10.0f;
      }
      
      rez = rez * 10.0f + (float)d;
    }
  }

  return rez * fact;
}

// A simple atoi() function
int32_t Ascii_atoil(const uint8_t* buffer, const uint32_t length)
{
  int32_t res = 0;
  int32_t sign = 1;
  uint32_t ptr = 0;

  // Try to select + or -
  switch (buffer[0])
  {
    case '-':
      sign = -1;
      ptr++;
      break;

    case '+':
      sign = 1;
      ptr++;
      break;
  }
  
  // Try to parse the number
  for (; ptr < length; ptr++)
  {
    if (buffer[ptr] < '0' || buffer[ptr] > '9')
    {
      return 0;
    }

    res = (res * 10) + buffer[ptr] - '0';
  }
  
  return res * sign;
}

uint8_t Ascii_IsAscii(const uint8_t x)
{
  if (x >= '!' && x <= '~')
  {
    return 1;
  }

  return 0;
}