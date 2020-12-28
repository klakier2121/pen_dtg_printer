//Printer motor
//Source & info at: www.HomoFaciens.de/technics-base-circuits-encoder-disc_en_navion.htm

#include <util/delay.h>

unsigned long millisDot = 0;
int stepStatusFeed = 0;
byte sensorStatusPhdA = 0;
byte oldSensorStatusPhdA = 0;
byte sensorStatusPhdB = 0;


int stepStatusPhd = 0;
byte sensorStatusFeedA = 0;
byte oldSensorStatusFeedA = 0;
byte sensorStatusFeedB = 0;
byte oldSensorStatusFeedB = 0;

int dutyCyclePhd = 0;
signed long setPointPhd = 0;
signed long actualPointPhd = 0;

int dutyCycleFeed = 0;
signed long setPointFeed = 0;
signed long actualPointFeed = 0;

//byte readByte = 0;
byte relayDown = 0;

byte stepsDone = 0;
int val;
String odczyt1, odczyt2, readByte;

static char buffer[30];
//bool head_home = false;

#define FEED_LED 13
#define FEED_POS 7

#define HEAD_POS 12

#define PHD_SENSOR_A  3
#define PHD_SENSOR_B  2
#define PHD_DIRECTION 5
#define PHD_PWM       6

#define FEED_SENSOR_A       8
#define FEED_SENSOR_B       9
#define FEED_DIRECTION     10
#define FEED_PWM           11

#define RELAY              A1
#define DOT_PAUSE          1


#define P_FRACTION_PHD 2.0     //Proportional factor of control loop 0.001 - 10.0 (1.0)
#define STEP_MARGIN_PHD 10.0   //10 - 1000 (1)
#define MIN_DUTYCYCLE_PHD 110   //0 - 255 (125)
#define MAX_DUTYCYCLE_PHD 150  //0 - 255 (255)

#define P_FRACTION_FEED 2.0     //Proportional factor of control loop 0.001 - 10.0 (1.0)
#define STEP_MARGIN_FEED 1.05     //10 - 1000 (1)
#define MIN_DUTYCYCLE_FEED 120   //0 - 255 (125)
#define MAX_DUTYCYCLE_FEED 130  //0 - 255 (255)



void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('r');     // send a capital X to indicate that Arduino is running    
    delay(300);
    
  }
}

void change_step(){
  
  sensorStatusPhdA = digitalRead(PHD_SENSOR_A);
  sensorStatusPhdB = digitalRead(PHD_SENSOR_B);

  if(sensorStatusPhdA == 0 && sensorStatusPhdB == 0){
    if(stepStatusPhd == 3){
      actualPointPhd++;
    }
    if(stepStatusPhd == 1){
      actualPointPhd--;
    }
    stepStatusPhd = 0;
  }

  if(sensorStatusPhdA == 1 && sensorStatusPhdB == 0){
    if(stepStatusPhd == 0){
      actualPointPhd++;
    }
    if(stepStatusPhd == 2){
      actualPointPhd--;
    }
    stepStatusPhd = 1;
  }

  if(sensorStatusPhdA == 1 && sensorStatusPhdB == 1){
    if(stepStatusPhd == 1){
      actualPointPhd++;
    }
    if(stepStatusPhd == 3){
      actualPointPhd--;
    }
    stepStatusPhd = 2;
  }

  if(sensorStatusPhdA == 0 && sensorStatusPhdB == 1){
    if(stepStatusPhd == 2){
      actualPointPhd++;
    }
    if(stepStatusPhd == 0){
      actualPointPhd--;
    }
    stepStatusPhd = 3;
  }

}
void ustaw_podajnik()
{
  digitalWrite(FEED_LED, 1);
  while(!(digitalRead(FEED_POS)))
  {
     digitalWrite(FEED_DIRECTION, 0);
     analogWrite(FEED_PWM, 160);
  }
   
  digitalWrite(FEED_DIRECTION, 0);
  analogWrite(FEED_PWM, 0);
  digitalWrite(FEED_LED, 0);

}

void ustaw_karetke()
{  
  while(digitalRead(HEAD_POS) <= 0)
  {
      digitalWrite(PHD_DIRECTION, 0);
      analogWrite(PHD_PWM, 100);
  }

  //setPointPhd = 300;
  //digitalWrite(PHD_DIRECTION, 1);
  //analogWrite(PHD_PWM, 255 - MAX_DUTYCYCLE_PHD);
      
 // delay(100); 
  analogWrite(PHD_PWM, 0);
  digitalWrite(PHD_DIRECTION, 0);  
  
//actualPointFeed = 0;
  setPointPhd=100;
  
  // head_home = true;
}

void setup() {

  pinMode(FEED_LED, OUTPUT);
  
  pinMode(HEAD_POS, INPUT_PULLUP);
  pinMode(FEED_POS, INPUT);
  
  pinMode(PHD_DIRECTION, OUTPUT);
  pinMode(PHD_PWM, OUTPUT);

  pinMode(FEED_DIRECTION, OUTPUT);
  pinMode(FEED_PWM, OUTPUT);

  pinMode(RELAY, OUTPUT);

  digitalWrite(PHD_DIRECTION, 0);
  analogWrite(PHD_PWM, MAX_DUTYCYCLE_PHD);
  delay(20);
  digitalWrite(PHD_DIRECTION, 1);
  analogWrite(PHD_PWM, 255 - MAX_DUTYCYCLE_PHD);
  delay(20);
  analogWrite(PHD_PWM, 0);
  digitalWrite(PHD_DIRECTION, 0);
  delay(20);

  attachInterrupt(0, change_step, CHANGE);
  attachInterrupt(1, change_step, CHANGE);

  sensorStatusPhdA = digitalRead(PHD_SENSOR_A);
  sensorStatusPhdB = digitalRead(PHD_SENSOR_B);

  millisDot = 0;
  digitalWrite(RELAY, 0);
  delay(DOT_PAUSE);
  relayDown = 0;
  stepsDone = 0;
  
  // start serial port at 115200 bps:
  Serial.begin(115200);
  ustaw_podajnik();
  ustaw_karetke();
  
  establishContact();  // send a byte to establish contact until receiver responds
}

int readline(int readch, char *buffer, int len)
{
  static int pos = 0;
  int rpos;

  if (readch > 0) {
    switch (readch) {
      case '\n': // Ignore new-lines
        break;
      case '\r': // Return on CR
        rpos = pos;
        pos = 0;  // Reset position index ready for next time
        return rpos;
      default:
        if (pos < len-1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  // No end of line has been found, so return -1.
  return -1;
}


void loop() {
  
  if(millisDot > millis()){
    millisDot = millis();
  }
  if(millis() - millisDot > DOT_PAUSE && relayDown == 1){
    millisDot = millis();
    relayDown = 2;
  }

  if(millis() - millisDot > DOT_PAUSE && relayDown == 2){
    millisDot = 0;
    relayDown = 0;
  }
   
  sensorStatusFeedA = digitalRead(FEED_SENSOR_A);
  sensorStatusFeedB = digitalRead(FEED_SENSOR_B);

  if(sensorStatusFeedA == 0 && sensorStatusFeedB == 0){
    if(stepStatusFeed == 3){
      actualPointFeed++;
    }
    if(stepStatusFeed == 1){
      actualPointFeed--;
    }
    stepStatusFeed = 0;
  }

  if(sensorStatusFeedA == 1 && sensorStatusFeedB == 0){
    if(stepStatusFeed == 0){
      actualPointFeed++;
    }
    if(stepStatusFeed == 2){
      actualPointFeed--;
    }
    stepStatusFeed = 1;
  }

  if(sensorStatusFeedA == 1 && sensorStatusFeedB == 1){
    if(stepStatusFeed == 1){
      actualPointFeed++;
    }
    if(stepStatusFeed == 3){
      actualPointFeed--;
    }
    stepStatusFeed = 2;
  }

  if(sensorStatusFeedA == 0 && sensorStatusFeedB == 1){
    if(stepStatusFeed == 2){
      actualPointFeed++;
    }
    if(stepStatusFeed == 0){
      actualPointFeed--;
    }
    stepStatusFeed = 3;
  }

  dutyCycleFeed = (double)(abs(setPointFeed - actualPointFeed)) * (double)P_FRACTION_FEED;
  if(dutyCycleFeed < MIN_DUTYCYCLE_FEED){
    dutyCycleFeed = MIN_DUTYCYCLE_FEED;
  }
  if(dutyCycleFeed > MAX_DUTYCYCLE_FEED){
    dutyCycleFeed = MAX_DUTYCYCLE_FEED;
  }
  if(abs(setPointFeed - actualPointFeed) < STEP_MARGIN_FEED){
    analogWrite(FEED_PWM, 0);
    digitalWrite(FEED_DIRECTION, 0);
  }
  else{
    if(actualPointFeed < setPointFeed){
      digitalWrite(FEED_DIRECTION, 1);
      analogWrite(FEED_PWM, 255 - dutyCycleFeed);
    }
    if(actualPointFeed > setPointFeed){
      digitalWrite(FEED_DIRECTION, 0);
      analogWrite(FEED_PWM, dutyCycleFeed);
    }
  }



  dutyCyclePhd = (double)(abs(setPointPhd - actualPointPhd)) * (double)P_FRACTION_PHD;
  if(dutyCyclePhd < MIN_DUTYCYCLE_PHD){
    dutyCyclePhd = MIN_DUTYCYCLE_PHD;
  }
  if(dutyCyclePhd > MAX_DUTYCYCLE_PHD){
    dutyCyclePhd = MAX_DUTYCYCLE_PHD;
  }

  if(abs(setPointPhd - actualPointPhd) < STEP_MARGIN_PHD){
    analogWrite(PHD_PWM, 0);
    digitalWrite(PHD_DIRECTION, 0);
  }
  else{
    if(actualPointPhd < setPointPhd){
      digitalWrite(PHD_DIRECTION, 1);
      analogWrite(PHD_PWM, 255 - dutyCyclePhd);
    }
    if(actualPointPhd > setPointPhd){
      digitalWrite(PHD_DIRECTION, 0);
      analogWrite(PHD_PWM, dutyCyclePhd);
    }
  }


  if(abs(setPointFeed - actualPointFeed) < STEP_MARGIN_FEED && abs(setPointPhd - actualPointPhd) < STEP_MARGIN_PHD){
    stepsDone = 0;
  }


  if(stepsDone == 0 && relayDown == 0){

    
  if (readline(Serial.read(), buffer, 29) > 0) {
      //get incoming byte:
      
      //Serial.println(digitalRead(FEED_POS));
      
      Serial.print('r');   // send a 'r' to initiate next data from computer

     if(buffer > 0){

        if(buffer[0] == 'R' && buffer[1] == '1'){
          
         // delay(50);
          millisDot = millis();
          digitalWrite(RELAY, 1);
          relayDown = 1;
          
        } 
        if(buffer[0] == 'R' && buffer[1] == '0'){
          digitalWrite(RELAY, 0);
          relayDown = 0;
        }
        
      
        if(buffer[0] == 'k'){
          ustaw_podajnik();
          ustaw_karetke();
         
        }
          
        if(buffer[0] == 'e'){
          setPointFeed += 95;
          stepsDone = 1;
        }

        odczyt1 = String(buffer[0]);
        readByte = String(buffer);
        odczyt2 = readByte.substring(1);


        if(odczyt1 == "p")
        {
            setPointPhd+=(odczyt2.toInt());
            stepsDone = 1;
        }
        if(odczyt1 == "t")
        {
            setPointPhd-=(odczyt2.toInt());
            stepsDone = 1;
        }
           
      }

    }

  }

}


