#include <Arduino.h>

byte* transfer_init(char volatile* timeroverflow_flag); // returns pointer to sendbuffer
void sendstream(unsigned char length);

