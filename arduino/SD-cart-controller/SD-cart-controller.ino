#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "transfer.h"

#define LEDPIN 2

#define SDCART_CMD_IS_SD_CART_INSERTED    34
#define SDCART_CMD_SEND_DIRECTORY_LISTING 2
#define SDCART_CMD_LOAD_ROM 3

const byte rxPin = 2;
const byte txPin = 3;
SoftwareSerial softserial(rxPin,txPin);

Sd2Card card;
SdVolume vol;
SdFile root;

volatile char timeroverflow=0;
byte* sendbuff;

ISR(TIMER1_COMPA_vect)
{
  timeroverflow=1;
}

byte is_sd_card_inserted(void) {
  Serial.println("Initializing SD card...");
  
  if ( !card.init(SPI_HALF_SPEED, SS) ) {
    Serial.println("Card init fail\n");
    return 1;
  }
  Serial.println("card init OK");

  // Try to initialize the FAT volume.
  if (!vol.init(&card)) {
    Serial.println("vol init failed\n");
    return 2;
  }
  Serial.println("vol init OK");
  
  // Try to open root
  if (!root.openRoot(&vol)) {
    Serial.println("open root failed\n");
    return 3;
  }
  Serial.println("open root OK\n");
  root.close();
  
  return 0;
}

void send_is_sd_card_inserted(void) {
  sendbuff[0]= is_sd_card_inserted();
  sendstream(1);
}

/*
void printDirectory(File dir, int numTabs) {
   while(true) {

     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("tt");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}
*/

void setup() {
  // put your setup code here, to run once:

  sendbuff = transfer_init(&timeroverflow);

  pinMode(LEDPIN, OUTPUT);

  cli();
  TCCR1A = 0;        // set entire TCCR1A register to 0
  TCCR1B = 0;

  TCCR1B |= (1 << WGM12);// turn on CTC mode:
  TCCR1B |= (1 << CS10); // 16 MHz
  OCR1A = 1670;          // 1670 * 1/16MHz = 104.4 us
  TIMSK1 |= (1 << OCIE1A);
    sei();
  
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  //softserial.begin(9600);

  Serial.begin(19200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  DDRD = 0x00;

  SPI.begin();
  SPI.setBitOrder(MSBFIRST); 
  /*
  writesram(0x00,0x01);
  */
  DDRC = 0xFF;
  PORTC = 0xFF;
  DDRD  |= 0B11000000;
  PORTD |= 0B11000000;
}

void loop() {
  // put your main code here, to run repeatedly:

  if (softserial.available()) {
    // receive command
    char command = softserial.read();
    if (command==SDCART_CMD_IS_SD_CART_INSERTED) {
      send_is_sd_card_inserted();
    }
    else if (command==SDCART_CMD_SEND_DIRECTORY_LISTING) {
      //send_dir_listing();
    }
    else if (command==SDCART_CMD_LOAD_ROM) {
      while( !softserial.available() );
      char filenumber = softserial.read();
      //send_rom_file(filenumber);
      //hang_around_for_next_reset();
    }

  }
}


// old stuff stored for later.

  /*
  File dataFile = SD.open("shark.bin");

  // if the file is available, write to it:
  if (dataFile) {
    //Serial.println("datalog.txt open");
    countL=0;
    countH=0;
    DDRD = 255;
    digitalWrite(SHIFTREG_OE_N, LOW);

    while (dataFile.available()) {
      //buff[count]=dataFile.read();
      data = dataFile.read();
      if(countH<4) {
        writesram(countL,countH,data );
        writesram(countL,countH+4,data );
      }
      else {
        writesram(countL,countH+4,data );
        writesram(countL,countH+8,data );
      }
      if(countL==255) { countH++;
        // Serial.println(countH);
      }
      countL++;
      //Serial.write(".");
      // --------------------------
      //if (countL==5) break;
    }
    DDRD = 0;
    digitalWrite(SHIFTREG_OE_N, HIGH);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  //delay(1000); 
  */

/*
  for (count=0; count<16; count++) {
    writesram(count,buff[count]);
  }
*/
