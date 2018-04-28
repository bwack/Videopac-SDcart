#include <Arduino.h>
#define BUSTR_EN_N 3
byte* transfer_init(char volatile* timeroverflow_flag); // returns pointer to sendbuffer
void sendstream(unsigned char length);

