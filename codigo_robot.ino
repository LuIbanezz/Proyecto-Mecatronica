//Code for WiperBot created by Akhil Padmanabha
//Check out my other work at https://akhilpadmanabha.wixsite.com/portfolio
//Feel free to contact me with questions at akhil.padmanabha@gmail.com
//Huge thanks to Jeff Rowberg for his libraries on the MPU6050 gyro/accelerometer.Check out his library at: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050


#include <math.h>

//servos
//#include <Servo.h>
//Servo right_servo;
//Servo left_servo;
//int right_motor_port = 6;
//int left_motor_port = 5;

/*** Motor Setup ***/
const int enR = 6;
const int enL = 5;
const int inR1 = 4;
const int inR2 = 2;
const int inL1 = 0;
const int inL2 = 1;



///speed values for gyro 
int right_start = 100; //86
int left_start = 100; //101
int right_speed = right_start;
int left_speed = left_start;
int right_start_back = 100; //103
int left_start_back = 100; //85
int right_back_speed = right_start_back;
int left_back_speed = left_start_back;

//IR 
int DF_table_IR = 12; // right front table IR in port 12
int IF_table_IR = 11; // left front table IR in port 8 
int DL_table_IR = 8; // right side table IR in port 8 
int IL_table_IR = 7; // left side table IR in port 8 

int IAF_IR = 14; // left front top IR in port 8 
int DAF_IR = 15; // right front top IR in port 8 
int IA_IR = 16; // left side top IR in port 8 
int DA_IR = 17; // right side topx IR in port 8 


int DF_table_val = LOW; //LOW means table 
int IF_table_val = LOW; //LOW means table 
int DL_table_val = LOW; //LOW means table 
int IL_table_val = LOW; //LOW means table 

int IAF_val = LOW; //LOW means object
int DAF_val = LOW; //LOW means object
int IA_val = LOW; //LOW means object
int DA_val = LOW; //LOW means object


//gyro
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#define OUTPUT_READABLE_YAWPITCHROLL
MPU6050 mpu;

//other global variables
bool at_edge = false;



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//driving initializations
float heading = 0; 


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() 
{
    ////servo 
    //right_servo.attach(right_motor_port);
    //left_servo.attach(left_motor_port );
    pinMode(enR, OUTPUT);
    pinMode(inR1, OUTPUT);
    pinMode(inR2, OUTPUT);

    pinMode(enL, OUTPUT);
    pinMode(inL1, OUTPUT);
    pinMode(inL2, OUTPUT);

    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity  https://www.i2cdevlib.com/forums/topic/91-how-to-decide-gyro-and-accelerometer-offsett/


    mpu.setXGyroOffset(220); 
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}


///////////MAIN CODE////////////////

void loop()
{
  Serial.println("Program started. Waiting for gyro");
  int count = 15;
  while (count>0)  //gyro stabilization countdown 
  {
    Serial.println(count);
    delay(1000);
    count-=1;
  }

  //allign in corner of table
  forward_till_table();
  delay(500);
  backward(.4);
  delay(500);
  left_turn();
  delay(500);
  forward_till_table();
  delay(500);
  backward(.4);
  delay(500);
  left_turn();
  delay(500);

  //commence wiping path
  int switch_turn = 0; 
  while (at_edge == false)
  {
      forward_till_table();
      delay(1000);
      Serial.println("did forward");
      backward(.4);
      delay(1000);
      Serial.println("did backward");
      if (switch_turn == 0)
      {
        left_uturn();
        Serial.println("did uturn");
  
      }  
      else
      {
        right_uturn();
      }
      switch_turn = 1- switch_turn; 
      
  }
  forward_till_table();
  while(1)
  {
    //do nothing
  }

  
}


float getyaw() {
    // if programming failed, don't try to do anything
    if (!dmpReady) 
    {
        Serial.println("programming failed");
        return getyaw();
    }
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();


    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        return getyaw();
       

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait

        while (fifoCount < packetSize) 
        {
          fifoCount = mpu.getFIFOCount();
        }

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            return (ypr[0] * 180/M_PI);
        #endif


        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    return getyaw();
}


void forward(float seconds)
{
  int yawvalue;
  int milliseconds = seconds*1000;
  heading = round(getyaw());
  int firsttime = millis();
  DF_table_val = digitalRead(DF_table_IR);
  IF_table_val = digitalRead(IF_table_IR);
  int currenttime = millis();
  Serial.println(firsttime);
  while ((currenttime - firsttime < milliseconds) and (DF_table_val == LOW) and (IF_table_val == LOW))
  {
    DF_table_val = digitalRead(DF_table_IR);
    IF_table_val = digitalRead(IF_table_IR);
    yawvalue = round(getyaw());
    if (yawvalue > heading)
    {
      Serial.println("tilting right");
      right_speed = right_speed - 1;
      left_speed = left_start;
    }
    else if (yawvalue < heading)
    {
      Serial.println("tilting left");
      left_speed = left_speed + 1;
      right_speed = right_start;
    }
    else
    {
      Serial.println("facing forward");
      left_speed = left_start;
      right_speed = right_start;
    }
    //right_servo.write(right_speed);
    //left_servo.write(left_speed);
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW); 
    analogWrite(enR, right_speed);

    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
    analogWrite(enR, left_speed);
    currenttime = millis();
    Serial.println(currenttime);
    delay(50);
  }
  if (DF_table_val == HIGH or IF_table_val == HIGH)
  {
    at_edge = true; 
  }
  
  stop();
  Serial.println("finished forward");
}

void backward(float seconds)
{
  int yawvalue;
  int milliseconds = seconds*1000;
  heading = round(getyaw());
  int firsttime = millis();
  int currenttime = millis();
  Serial.println(firsttime);
  while ((currenttime - firsttime < milliseconds))
  {
    yawvalue = round(getyaw());
    if (yawvalue > heading)
    {
      Serial.println("tilting right");
      left_back_speed = left_back_speed - 1;
      right_back_speed = right_start_back;

    }
    else if (yawvalue < heading)
    {
      Serial.println("tilting left");
      right_back_speed = right_back_speed + 1;
      left_back_speed = left_start_back;
    }
    else
    {
      Serial.println("facing backward");
      left_speed = left_start_back;
      right_speed = right_start_back;
    }
    //right_servo.write(right_back_speed);
    //left_servo.write(left_back_speed); 
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH); 
    analogWrite(enR, right_speed);

    digitalWrite(inL1, LOW);
    digitalWrite(inL2, HIGH);
    analogWrite(enR, left_speed);
    currenttime = millis();
    Serial.println(currenttime);
    delay(50);
  }
  stop();
  Serial.println("finished backward");
}

void forward_till_table() 
{
  int yawvalue;
  heading = round(getyaw());
  DF_table_val = digitalRead(DF_table_IR);
  IF_table_val = digitalRead(IF_table_IR);
  while (DF_table_val == LOW and IF_table_val == LOW) //didn't finish path
  {
    yawvalue = round(getyaw());
    if (yawvalue > heading)
    {
      Serial.println("tilting right");
      right_speed = right_speed - 1;
      left_speed = left_start;
    }
    else if (yawvalue < heading)
    {
      Serial.println("tilting left");
      left_speed = left_speed + 1;
      right_speed = right_start;
    }
    else
    {
      Serial.println("facing forward");
      left_speed = left_start;
      right_speed = right_start;
    }
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW); 
    analogWrite(enR, right_speed);

    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
    analogWrite(enR, left_speed);
    DF_table_val = digitalRead(DF_table_IR);
    IF_table_val = digitalRead(IF_table_IR);
  }
  
  while (DF_table_val != HIGH or IF_table_val != HIGH) //has reached end of table
  {
    if (DF_table_val == HIGH && DF_table_val == LOW) //right is in end of table, left has to align
    {
      //right_servo.write(93); //if value<200, value=angle
      //left_servo.write(100);
      digitalWrite(inR1, LOW);
      digitalWrite(inR2, LOW); 

      digitalWrite(inL1, HIGH);
      digitalWrite(inL2, LOW);
      analogWrite(enL, left_speed); //TO DO
      Serial.println("right saw cliff");
    }
    else if (IF_table_val == HIGH && DF_table_val == LOW)
    {
      //left_servo.write(90);
      //right_servo.write(80);
      digitalWrite(inR1, HIGH);
      digitalWrite(inR2, LOW); 
      analogWrite(enR, right_speed);

      digitalWrite(inL1, LOW);
      digitalWrite(inL2, LOW);
  
      Serial.println("left saw cliff");
    }
    DF_table_val = digitalRead(DF_table_IR);
    IF_table_val = digitalRead(IF_table_IR);
  }
  stop();
  Serial.println("Square up done");
}

void left_turn()
{
  int yawvalue;
  int heading = round(getyaw());
  int finalheading = heading - 82; //TODO
  Serial.print ("heading ");
  Serial.println(heading);
  Serial.print("finalheading ");
  Serial.println(finalheading);
  
  if (finalheading < -180)
  {
    finalheading = 180 + (finalheading + 180);
    while (heading <= finalheading)
    {
    //left_servo.write(80);                                                                                                                                                   
    //right_servo.write(83);
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW); 
    analogWrite(enR, right_speed); //TO DO

    digitalWrite(inL1, LOW);
    digitalWrite(inL2, LOW);
    heading = round(getyaw());
    Serial.println(heading);
    }
    while (heading >= finalheading)
    {
      //left_servo.write(80);                                                                                                                                                   
      //right_servo.write(83);
      digitalWrite(inR1, HIGH);
      digitalWrite(inR2, LOW); 
      analogWrite(enR, right_speed); //TO DO

      digitalWrite(inL1, LOW);
      digitalWrite(inL2, LOW);
      heading = round(getyaw());
      Serial.println(heading);
    }
  }
  else
  {
    while (heading >= finalheading)
    {
    //left_servo.write(80);                                                                                                                                                   
    //right_servo.write(83);
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW); 
    analogWrite(enR, right_speed); //TO DO

    digitalWrite(inL1, LOW);
    digitalWrite(inL2, LOW);
    heading = round(getyaw());
    Serial.println(heading);
    heading = round(getyaw());
    Serial.println(heading);
    }
  }
  Serial.print("modified finalheading: ");
  Serial.println(finalheading); 
  
  Serial.println("done turning");
  stop();
}

void right_turn() //works
{
  int yawvalue;
  int heading = round(getyaw());
  int finalheading = heading + 87;
  Serial.print ("heading ");
  Serial.println(heading);
  Serial.print("finalheading ");
  Serial.println(finalheading);
  if (finalheading > 180)
  {
    
    finalheading = -180 + (finalheading - 180);
    while (heading >= finalheading)
    {
    //left_servo.write(110);                                                                                                                                                   
    //right_servo.write(113);
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, LOW); 

    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
    analogWrite(enL, left_speed); //TO DO
    heading = round(getyaw());
    Serial.println(heading);
    }
    while (heading <= finalheading)
    {
      //left_servo.write(110);                                                                                                                                                   
      //right_servo.write(113);
      digitalWrite(inR1, LOW);
      digitalWrite(inR2, LOW); 

      digitalWrite(inL1, HIGH);
      digitalWrite(inL2, LOW);
      analogWrite(enL, left_speed); //TO DO
      heading = round(getyaw());
      Serial.println(heading);
    }
  }
  else
  {
    while (heading <= finalheading)
    {
      //left_servo.write(110);                                                                                                                                                   
      //right_servo.write(113);
      digitalWrite(inR1, LOW);
      digitalWrite(inR2, LOW); 

      digitalWrite(inL1, HIGH);
      digitalWrite(inL2, LOW);
      analogWrite(enL, left_speed); //TO DO
      heading = round(getyaw());
      Serial.println(heading);
    }
  }
  Serial.println("done turning");
  stop();
}

void right_uturn()
{
  right_turn();
  delay(500);
  forward(2); 
  delay(500);
  right_turn();
  delay(500);
}

void left_uturn()
{
  left_turn();
  delay(500);
  forward(2);
  delay(500);
  left_turn();
  delay(500);
}


void stop()
{
  //right_servo.write(93);
  //left_servo.write(90);
  //right_servo.detach();
  //left_servo.detach();
  //right_servo.attach(right_motor_port);
  //left_servo.attach(left_motor_port );
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW); 

  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}
void IR_sensor_test()
{
  while(1)
  {
    DF_table_val = digitalRead(DF_table_IR);
    IF_table_val = digitalRead(IF_table_IR);
    if (IF_table_val == HIGH and DF_table_val == HIGH)
    {
      Serial.println("both cliff");
    }
    else if (DF_table_val == HIGH)
    {
      Serial.println("right cliff");
    }
    else if (IF_table_val == HIGH)
    {
      Serial.println("left cliff");
    }
    
    else
    {
      Serial.println("both ground");
    }
    delay(500);
  }
}

