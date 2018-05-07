#include <AltSoftSerial.h>

//MP3
#define ARDUINO_RX 8  // should connect to TX of the Serial MP3 Player module
#define ARDUINO_TX 9  // connect to RX of the module
AltSoftSerial mySerial(ARDUINO_RX, ARDUINO_TX);

static int8_t Send_buf[8] = {0};
int16_t soundTrack[6] = {0X1E01,0X1E03,0X1E05,0X1E06,0X1E07,0X1E08};//soundTrack:0X1E01:the first song
int volume = 30;

#define CMD_PLAY_NEXT    0x01
#define CMD_PLAY_PREV   0x02
#define CMD_PLAY_W_INDEX  0x03
#define CMD_SET_VOLUME    0x06
#define CMD_SEL_DEV     0x09
#define CMD_PLAY_W_VOL    0x22
#define CMD_PLAY      0x0D
#define CMD_PAUSE     0x0E
#define CMD_SINGLE_CYCLE  0x19
#define DEV_TF        0x02
#define SINGLE_CYCLE_ON   0x00
#define SINGLE_CYCLE_OFF  0x01

//PIR Sensor
int PIR = 7;
int pirState = LOW;
int detected = 0;

//microwave radar sensor 
int MicrowavePin = A1;
int MicrowaveRead = 0;
int MicrowaveState = LOW;

//Hall Sensor
int Hallsensor = A0;
int Hallread = 0;


//LED to show the detection
int ledPin = 13;
void setup() {
  // put your setup code here, to run once:
  mySerial.begin(9600);//begin mySerial mp3;
  Serial.begin(9600);
  delay(500);//Wait chip initialization is complete
  sendCommand(CMD_SEL_DEV, DEV_TF);//select the TF card
  delay(200);//wait for 200ms 
//  sendCommand(CMD_PLAY_W_VOL, 0X1E01);//play the first song with volume 30 class 

  pinMode(PIR,INPUT);// declare PIR sensor as input
  pinMode(ledPin, OUTPUT);// declare LED as output
  pinMode(Hallsensor,INPUT);// declare hall sensor as input
  pinMode(MicrowavePin,INPUT);// declare microwave radar sensor as input
}

void loop() {
  // put your main code here, to run repeatedly:

  MicrowaveRead = analogRead(MicrowavePin);
//  Serial.println(MicrowaveRead);
  
  detected = digitalRead(PIR);
  if(detected == HIGH || MicrowaveRead > 640){
      digitalWrite(ledPin, HIGH);  // turn LED ON
      if(pirState == LOW || MicrowaveState == LOW){
          Serial.println("Motion detected!");
          sendCommand(CMD_SET_VOLUME, volume);
          sendCommand(CMD_PLAY_W_VOL, soundTrack[random(6)]);//play a song randomly
          pirState = HIGH;
          MicrowaveState = HIGH;
        }
    }else{
          digitalWrite(ledPin, LOW); // turn LED OFF
          if(pirState == HIGH){
//              sendCommand(CMD_PAUSE, 0x0000);
              Serial.println("PIRMotion ended!");
              pirState = LOW;
            }
          if(MicrowaveState == HIGH){
              MicrowaveState = LOW;
              Serial.println("Microwave ended!");
            }
          }
          
     Hallread = digitalRead(Hallsensor); 
//     Serial.println(Hallread);

}

void sendCommand(int8_t command, int16_t dat)
{
  delay(20);
  Send_buf[0] = 0x7e; // starting byte
  Send_buf[1] = 0xff; // version
  Send_buf[2] = 0x06; // the number of bytes of the command without starting byte and ending byte
  Send_buf[3] = command;  //
  Send_buf[4] = 0x00;    // 0x00 = no feedback, 0x01 = feedback
  Send_buf[5] = (int8_t)(dat >> 8); // datah
  Send_buf[6] = (int8_t)(dat);      // datal
  Send_buf[7] = 0xef; // ending byte
  for(uint8_t i=0; i<8; i++)
  {
    mySerial.write(Send_buf[i]) ;
  }
}
