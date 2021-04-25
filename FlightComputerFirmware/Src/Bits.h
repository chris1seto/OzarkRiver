#ifndef BITS_H
#define BITS_H

#define SET(bitfield, bit) bitfield |= bit; 
#define CLR(bitfield, bit) bitfield &= ~bit;
#define IS_SET(bitfield, bit) (bitfield & bit)

#endif
