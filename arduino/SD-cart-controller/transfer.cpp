#ifndef TRANSFER_H
#define TRANSFER_H

#include "transfer.h"
byte sendbuffer[256];
volatile char* t_flag;

byte* transfer_init(char volatile* timeroverflow_flag) {
  t_flag = timeroverflow_flag;
  return (byte*)sendbuffer;
}

void sendstream(unsigned char length) {
  digitalWrite(8, LOW);
  delayMicroseconds(150);
  while(*t_flag==0);
  *t_flag=0;
  while(*t_flag==0);
  *t_flag=0;
  PORTC = 0x00;
  PORTD &= 0B00111111;
  for (unsigned char i=0; i<length; i++) {
    while(*t_flag==0);
    *t_flag=0;
    PORTC = sendbuffer[i];
    PORTD &= 0B00111111;
    PORTD |= sendbuffer[i] & 0B11000000;
  }
  while(*t_flag==0);
  *t_flag=0;
  PORTC = 0xFF;
  PORTD |= 0B11000000;
  while(*t_flag==0);
  *t_flag=0;
  digitalWrite(8, HIGH);
}
#endif // TRANSFER_H
