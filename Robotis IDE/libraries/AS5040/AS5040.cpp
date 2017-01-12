/*
 * AS5040.cpp - Arduino library for AMS AS5040 magnetic rotary encoder chip
 * version 1.0 2014-03-05
 *
 * Copyright (c) 2014 Mark Tillotson.  All rights reserved.
 *
 * This file is part of "Mark's AS5040 library for Arduino"
 *
 *  "Mark's AS5040 library for Arduino" is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  "Mark's AS5040 library for Arduino" is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with "Mark's AS5040 library for Arduino".
 *  If not, see <http://www.gnu.org/licenses/>.
 */


#include "AS5040.h"


AS5040::AS5040 (char pinCLK, char pinCS, char pinDO, char pinPROG) 
{
  _pinCLK  = pinCLK ;
  _pinCS   = pinCS ;
  _pinDO   = pinDO ;
  _pinPROG = pinPROG ;

  _status  = 0xFF ;  // invalid status
}

bool AS5040::begin()
{
  int config_word =
    (0 ? 0x8000 : 0x0000) | 
    ((0 & 0x3FF) << 5) |
    (0 & 0x1F) ;

  pinMode (_pinCLK, 1) ;  digitalWrite (_pinCLK, 1) ;
  if (_pinPROG != 0xFF)
  {
    pinMode (_pinPROG, 1) ; digitalWrite (_pinPROG, 0) ;
  }
  pinMode (_pinCS, 1) ;   digitalWrite (_pinCS, 1) ;
  pinMode (_pinDO, INPUT_PULLUP) ;

  char count = 0 ;
  while (read (), (_status & AS5040_STATUS_OCF) == 0)
  {
    if (count > 30)
      return false ; // failed to initialize
    delay (1) ;
    count ++ ;
  }

  if (_pinPROG == 0xFF)
  {
    // no initial program sequence, we're done already
    return true ;
  }
  digitalWrite (_pinCS, 0) ;
  delayMicroseconds (1) ;
  digitalWrite (_pinCLK, 0) ;
  delayMicroseconds (1) ;
  digitalWrite (_pinPROG, 1) ;
  delayMicroseconds (5) ;
  digitalWrite (_pinCS, 1) ;
  delayMicroseconds (5) ;

  unsigned int MSK = 0x8000 ;
  for (char i = 0 ; i < 16 ; i++)
  {
    digitalWrite (_pinPROG, (config_word & MSK) ? 1 : 0) ;
    MSK >>= 1 ;
    digitalWrite (_pinCLK, 1) ;
    delayMicroseconds (1) ;
    digitalWrite (_pinCLK, 0) ;
    delayMicroseconds (1) ;
  }
  
  digitalWrite (_pinPROG, 0) ;
  delayMicroseconds (1) ;
  digitalWrite (_pinCS, 0) ;
  delayMicroseconds (1) ;
  digitalWrite (_pinCS, 1) ; // ready for reads
  delayMicroseconds (1) ;
  digitalWrite (_pinCLK, 1) ;
  return true ;
}


// read position value, squirrel away status
unsigned int AS5040::read ()
{
  digitalWrite (_pinCS, 0) ;
  unsigned int value = 0 ;
  for (char i = 0 ; i < 10 ; i++)
  {
    digitalWrite (_pinCLK, 0) ;
    digitalWrite (_pinCLK, 1) ;
    value = (value << 1) | digitalRead (_pinDO) ;
  }
  char status = 0 ;
  for (char i = 0 ; i < 6 ; i++)
  {
    digitalWrite (_pinCLK, 0) ;
    digitalWrite (_pinCLK, 1) ;
    status = (status << 1) | digitalRead (_pinDO) ;
  }
  digitalWrite (_pinCS, 1) ;
  _parity = even_parity (value >> 2) ^ even_parity (value & 3) ^ even_parity (status) ;
  _status = status >> 1 ;
  return value ;
}

char AS5040::even_parity (char val)
{
  val = (val >> 1) ^ val ;
  val = (val >> 2) ^ val ;
  val = (val >> 4) ^ val ;
  return val & 1 ;
}


// raw status from latest read, 5 bit value
char AS5040::status ()
{
  return _status ;
}

// indicate if latest status implies valid data
bool AS5040::valid ()
{
  return _parity == 0 && (_status & 0x18) == 0x10 && (_status & 3) != 3 ;
}

// motion in the Z-axis, +1 means mag field increasing (magnet approaching chip),
// -1 is decreasing, 0 is stable.
int AS5040::Zaxis ()
{
  switch (_status & 0x3)
    {
    case 0: return 0 ;
    case 1: return -1 ;
    case 2: return +1 ;
    }
  return 0 ; // invalid case, must return something harmless
}
