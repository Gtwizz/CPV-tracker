#include <SoftwareSerial.h>

//Software serial for data transfer
SoftwareSerial SS1(13, 12); // RX, TX

#define ESpin 11 //Azimuth endstop pin (N)
#define TdirPin 5    // Tilt motor direction pin
#define TstepPin 2   // Tilt motor step pin
#define AZdirPin 7    // Azimuth motor direction pin
#define AZstepPin 4    // Azimuth motor step
#define stepsPerRevolution 3200  //steg per varv 200steg 16 micro
#define enablePin 8  // Enable DRV8825

#define LDRNWpin A3 // Nordväst, R2 = 220 ohm 
#define LDRNEpin A0 // Nordost, R2 = 220 ohm
#define LDRSWpin A1 // Sydväst, R2 = 220 ohm
#define LDRSEpin A2 // Sydost, R2 = 220 ohm

// LDR sensor values
int NWv = 0;  // North West light sensor Value
int NEv = 0;  // Notrh East Light sensor Value
int SEv = 0;  // South West Light sensor Value
int SWv = 0;  // South East Light sensor Value

// Stepper position values
int x = 0;
int homeadapt = -1;
int AZPOS = 0;  // Azimuth axle position
int AZMIN = 0;
int AZMAX = 0;
int AZTARGET = stepsPerRevolution/2; // Azimuth target position (south)
int TPOS = 0;  //Tilt axle position
int TMAX = 0;  // Upper tilt Limit, colision with azimuth
int TMIN = 0; // Lower tilt Limit, colision with azimuth
int TTARGET = 0;  // Target position Tilt axle
int steptime = 10;
int Tsteptime = 10;  // pulsewith for step (same for high & low)
int AZsteptime = 10;  // pulsewith for step (same for high & low)

// Controller values
int Navg = 0; // North sensors average, (NW+NE)/2
int Savg = 0; // South sensors average, (NW+NE)/2
int Wavg = 0; // East sensors average, (NW+SW)/2
int Eavg = 0; // West sensors average, (NW+NE/2)
int LDRavg = 0; // Average of all (daylight detection)
int LDRmax = 0; // Max LDR-value for sunlight detection
int TiltError = 0; // Tilt error (Navg - Savg)
int AzimuthError = 0; //Azimuth error (Wavg - Eavg)

// Time 
unsigned long startMillis;
unsigned long Time = 0;
unsigned long TAdapt = 900000;
unsigned long dTAdapt = 900000;
int delta[] = {-100,200,-200,100};

void setup() {
  // put your setup code here, to run once:
  // Disable motors during setup
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH); 

  // Define Endstop pins
  pinMode(ESpin,INPUT_PULLUP);
  //pinMode(ESTpin,INPUT_PULLUP);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);

   // Start serial comunication at9600 baud
  Serial.begin(115200);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  SS1.begin(115200);
  SS1.println("Experiment start");
  SS1.println("Time,AZPOS,TPOS,ESAZ,EST,NW,NE,SW,SE,AZe,Te,LDRavg,LDRmax");

  pinMode(AZstepPin, OUTPUT);
  pinMode(AZdirPin, OUTPUT);
  pinMode(TstepPin, OUTPUT);
  pinMode(TdirPin, OUTPUT);
 
  // Define LDRpins
    // Light sensor setup
  pinMode(LDRNWpin, INPUT);
  pinMode(LDRNEpin, INPUT);
  pinMode(LDRSWpin, INPUT);
  pinMode(LDRSEpin, INPUT);

  // enable motors
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);
  homing();
  FFCntrl();
}

void loop() {
  //SS1.println(TPOS);
  // Return to ref pos
  delay(1000);
  read_LDR();
  printV();
  # Run feedback everit
  if(abs(TiltError) > 5 || abs(AzimuthError) > 5 ||  millis()>(TAdapt)) && LDRmax > 550){
    LDRFBCntrl();
    TAdapt=millis()+dTAdapt;
  }
}

void homing(){
  // Tilt homing //
  //turn motor CCW (dirPin=LOW) until endstop is reached
  steptime=5;
  digitalWrite(TdirPin, LOW);
  while (digitalRead(ESpin) == HIGH){
    digitalWrite(TstepPin, HIGH);
    digitalWrite(TstepPin, LOW);
    delay(steptime);
    TPOS--;
  }
  delay(500);
  SS1.print("TMIN: ");
  SS1.print(TPOS);
  //Move CW, dirPin=HIGH until Endstop has released
  digitalWrite(TdirPin, HIGH);
  while (digitalRead(ESpin) == LOW){
    digitalWrite(TstepPin, HIGH);
    digitalWrite(TstepPin, LOW);
    delay(steptime);
    TPOS++;
  }
  TMIN = TPOS;
  SS1.print(",   TMIN: ");
  SS1.print(TPOS);
  delay(500);

  //Move CW, dirPin=HIGH to next endstop
  digitalWrite(TdirPin, HIGH);
  while (digitalRead(ESpin) == HIGH){
    digitalWrite(TstepPin, HIGH);
    digitalWrite(TstepPin, LOW);
    delay(steptime);
    TPOS++;
  }
  delay(500);
  SS1.print(",   TMAX: ");
  SS1.print(TPOS);

  //Move CW, dirPin=HIGH until Endstop has released
  digitalWrite(TdirPin, LOW);
  while (digitalRead(ESpin) == LOW){
    digitalWrite(TstepPin, HIGH);
    digitalWrite(TstepPin, LOW);
    delay(steptime);
    TPOS--;
  }
  TMAX=TPOS;
  delay(500);
  SS1.print("  TMAX: ");
  SS1.println(TPOS);

  int Td = TMAX - TMIN;
  TMAX = Td/2;
  TMIN = - TMAX;
  TPOS = TMAX;
 // SS1.print(TMAX, TMIN, TPOS);

  // AZ Homing //
  // Azimuth homing //
  // turn motor CCW (dirPin=LOW) until endstop is reached
  digitalWrite(AZdirPin, LOW);
  while (digitalRead(ESpin) == HIGH){
    digitalWrite(AZstepPin, HIGH);
    digitalWrite(AZstepPin, LOW);
    delay(steptime);
    AZPOS--;
  }
  
  delay(500);
  SS1.print("AZMIN: ");
  SS1.print(AZPOS);
  
  // Move CW, dirPin=HIGH until Endstop has released
  digitalWrite(AZdirPin, HIGH);
  while (digitalRead(ESpin) == LOW){
    digitalWrite(AZstepPin, HIGH);
    digitalWrite(AZstepPin, LOW);
    delay(steptime);
    AZPOS++;
  }
  AZPOS=0; 
  SS1.print(",   AZmin: ");
  SS1.print(AZPOS);
  delay(500);

  //Move CW, dirPin=HIGH to next endstop
  digitalWrite(AZdirPin, HIGH);
  while (digitalRead(ESpin) == HIGH){
    digitalWrite(AZstepPin, HIGH);
    digitalWrite(AZstepPin, LOW);
    delay(steptime);
    AZPOS++;
  }
  delay(500);
  SS1.print(",   AZmax: ");
  SS1.print(AZPOS);

  //Move CW, dirPin=HIGH until Endstop has released
  digitalWrite(AZdirPin, LOW);
  while (digitalRead(ESpin) == LOW){
    digitalWrite(AZstepPin, HIGH);
    digitalWrite(AZstepPin, LOW);
    delay(steptime);
    AZPOS--;
  }
  AZMAX=AZPOS;
  delay(500);
  SS1.print("  AZMAX: ");
  SS1.println(AZPOS);
  AZTARGET = (AZMAX - AZMIN)/2;
}

void adaption(){
  // Refpos + FBcntrl

  //SS1.println("Ref Pos 180,90");
  //SS1.println("Time,AZPOS,TPOS,ESAZ,EST,NW,NE,SW,SE,AZe,Te");
  /// Find Target
    LDRFBCntrl();
    delay(1000);
  // Svep AZ fram och tillbaka
  SS1.println("AZ sweep");
  printHead();  // print the variable header
  //AZ-svep
  AZTARGET = AZPOS;
  TTARGET = TPOS;
  for(int i=0; i<4; i++){
    AZTARGET += delta[i];
    FFCntrl();
  }

  SS1.println("Tilt sweep");
  printHead();  // print the variable header
  //tiltsvep
  for(int i=0; i<4; i++){
    TTARGET += delta[i];
    FFCntrl();
  }
}

void FFCntrl(){
  // Limit position
  // Move motors to ff position
  // move Azimuth motor
  SS1.println("FF control"); 
  printHead();  // print the variable header 
  while(AZPOS != AZTARGET){
  if(AZTARGET > AZPOS){
    digitalWrite(AZdirPin, HIGH);
    stepAZ();
    AZPOS += 1;
    read_LDR();
    printV();
  } else if (AZTARGET < AZPOS) {
    digitalWrite(AZdirPin, LOW);
    stepAZ();
    AZPOS -= 1;
    read_LDR();
    printV();
    }
  }

  // move Tiltmotor (add endstop info )
  while(TPOS != TTARGET){
  if(TTARGET > TPOS){
    digitalWrite(TdirPin, HIGH);
    stepT();
    TPOS += 1;
    read_LDR();
    printV();
  } else if (TTARGET < TPOS) {
    digitalWrite(TdirPin, LOW);
    stepT();
    TPOS -= 1;
    read_LDR();
    printV();
    }
  }
}

void LDRFBCntrl(){
/*  SS1.println("---------");
  SS1.println("FBsensor control");
  SS1.println("Time,AZPOS,TPOS,ESAZ,EST,NW,NE,SW,SE,AZe,Te,LDRavg,LDRmax");
*/
  read_LDR();
  SS1.println("FBsensor control");
  printHead();  // print the variable header 
  printV();  // print Variable values 
  // Move motors to minimize Az,Tilt error
  // Clockwise rotation E --> W
  // AZe = W - E
  while(abs(AzimuthError) > 2){
    set_speed();
    if(AzimuthError > 0){
      digitalWrite(AZdirPin, HIGH);
      stepAZ();
      AZPOS += 1;
      read_LDR();
      printV();
      } else if (AzimuthError < 0) {
      digitalWrite(AZdirPin, LOW);
      stepAZ();
      AZPOS -= 1;
      read_LDR();
      printV();
    }
  }
 
  // Clockwise rotation N --> S
  // AZe = N - S 
  // Negative error increase elevation = CW rotation (HIGH)
  // Positive error reduce elevation = CCW rotation (LOW)
  while(abs(TiltError) > 2){
    set_speed();
    if(TiltError > 0){
      digitalWrite(TdirPin, LOW);
      stepT();
      TPOS -= 1;
      read_LDR();
      printV();
    } else if (TiltError < 0) {
      digitalWrite(TdirPin, HIGH);
      stepT();
      TPOS += 1;
      read_LDR();
      printV();
    }
  }
}

void stepAZ(){
// Azimuth motor 1 step
  digitalWrite(AZstepPin, HIGH);
  digitalWrite(AZstepPin, LOW);
  delay(AZsteptime);
}

void stepT(){
// Tilt motor 1 step
  digitalWrite(TstepPin, HIGH);
  digitalWrite(TstepPin, LOW);
  delay(Tsteptime);
}

void read_LDR(){
  Time = millis();
  // Read each LDR 3times
  NWv = analogRead(LDRNWpin);
  NWv = analogRead(LDRNWpin);
  NWv = analogRead(LDRNWpin);

  NEv = analogRead(LDRNEpin);
  NEv = analogRead(LDRNEpin);
  NEv = analogRead(LDRNEpin);

  SWv = analogRead(LDRSWpin);
  SWv = analogRead(LDRSWpin);
  SWv = analogRead(LDRSWpin);

  SEv = analogRead(LDRSEpin);
  SEv = analogRead(LDRSEpin);
  SEv = analogRead(LDRSEpin);

  // Calc avg North, South, West, East, Tilt error, Azimuth error
  Navg = (NWv + NEv)/2 ; // North sensors average, (NW+NE)/2
  Savg = (SWv + SEv)/2; // South sensors average, (NW+NE)/2
  Wavg = (NWv + SWv)/2; // West sensors average, (NW+SW)/2
  Eavg = (NEv + SEv)/2; // East sensors average, (NE+SE/2)
  LDRavg = (NWv + NEv + SWv + SEv)/4; //Average of all for daylight detection
  TiltError = Navg -Savg; // Tilt error (Navg - Savg)
  AzimuthError = Wavg - Eavg; //Azimuth error (Wavg - Eavg)
  // LDR max value > 600 = sunlight
  int n = max(NWv,NEv);
  int s = max(SWv,SEv);
  LDRmax = max(n,s);

}

void set_speed(){
  AZsteptime = 50;
  Tsteptime = 30;
  if(abs(AzimuthError) < 25){ AZsteptime = 500; }
  if(abs(TiltError) < 25){ Tsteptime = 500; }  

}

void printHead(){
  SS1.println("Time,AZPOS,TPOS,ESAZ,EST,NW,NE,SW,SE,AZe,Te,LDRavg,LDRmax");
}

void printV(){
  //read_LDR();
  SS1.print(Time);
  SS1.print(",");
  SS1.print(AZPOS);
  SS1.print(",");
  SS1.print(TPOS);
  SS1.print(",");
  SS1.print(digitalRead(ESpin));
  SS1.print(",");
  SS1.print(digitalRead(ESpin));
  SS1.print(",");
  SS1.print(NWv);
  SS1.print(",");
  SS1.print(NEv);
  SS1.print(",");
  SS1.print(SWv);
  SS1.print(",");
  SS1.print(SEv);
  SS1.print(",");
  SS1.print(AzimuthError);
  SS1.print(",");
  SS1.print(TiltError);
  SS1.print(",");
  SS1.print(LDRavg);
  SS1.print(",");
  SS1.println(LDRmax);
}