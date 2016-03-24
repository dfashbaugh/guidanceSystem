#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

#define DEBUGYOU

Servo TomServo;

float input = 0;
float angle = 0;

struct WayPoint
{
    double latitude;
    double longitude;
};

// Linked List
struct node
{
    node* next;
    WayPoint data;
};

// Add to list
void addToBackOfList(WayPoint theData, node* head)
{
    node* newNode = NULL;
    
    newNode = new node();
    
    newNode->next = NULL;
    newNode->data.latitude = theData.latitude;
    newNode->data.longitude = theData.longitude;
    
    node* tracker = head;
    while(tracker->next != NULL){tracker = tracker->next;}
    
    tracker->next = newNode;
}

WayPoint popFront(node** head)
{
    node* oldHead = *head;
    *head = (*head)->next;
    
    delete oldHead;
    
    return oldHead->data;
}

node* InitalizeLinkedList(WayPoint theData)
{
    node* head = new node();
    head->next = NULL;
    head->data = theData;
    
    return head;
}

/*
Connections Accelerometer / Magnetometer
===========
Connect SCL to analog 5
Connect SDA to analog 4
Connect VDD to 3.3V DC
Connect GROUND to common ground

Connections GPS
==========
Connect Vin to 5V DC
Connect GND to GND
Connect RX to Digital 2
Connect TX to Digital 3
*/
#define BNO055_SAMPLERATE_DELAY_MS (50)

Adafruit_BNO055 bno = Adafruit_BNO055();

// Drive Parameters
double targetLat = 40.6785;
double targetLongitude = -74.0182;
double curOffsetAngle = 0.0;
double curMagnetometerAngle = 0.0;
double curGPSAngle = 0.0;
double acceptableAngleForFullThrottle = 10;

// Pin Numbers
int LEDBlinker = 2; // GPS Lock
int LEDYellow = 3;
int LEDGreen  = 5;
int ButtonPin = 11;
int ServoPin = 4;
int MotorPin = 23;

boolean usingInterrupt = false;

boolean getGPSInfo(double &Latitude, double &Longitude)
{
  char GPSInfo [50];
  Latitude = 0.0;
  Longitude = 0.0;
  int longitudeIndex = 0;
  boolean fix = false;
  
  for(int i = 0; i<50; i++)
  {
    while(!Serial1.available()){}
    GPSInfo[i] = (char)Serial1.read();

    if(GPSInfo[i] == 'N' || GPSInfo[i] == 'S')
    {
      double deg = ((int)GPSInfo[i-10]-'0')*10.0 + ((int)GPSInfo[i-9]-'0')*1.0;
      double minutes = ((int)GPSInfo[i-8]-'0')*10.0 + ((int)GPSInfo[i-7]-'0')*1.0
                      + ((int)GPSInfo[i-5]-'0')*0.1 + ((int)GPSInfo[i-4]-'0')*0.01
                      + ((int)GPSInfo[i-3]-'0')*0.001 + ((int)GPSInfo[i-2]-'0')*0.0001;
      Latitude = deg + (minutes/60.0);

      if(GPSInfo[i] == 'S') Latitude *= -1;
    }

    
    if(GPSInfo[i] == 'W' || GPSInfo[i] == 'E')
    {
      double deg = ((int)GPSInfo[i-10]-'0')*10.0 + ((int)GPSInfo[i-9]-'0')*1.0;
      double minutes = ((int)GPSInfo[i-8]-'0')*10.0 + ((int)GPSInfo[i-7]-'0')*1.0
                      + ((int)GPSInfo[i-5]-'0')*0.1 + ((int)GPSInfo[i-4]-'0')*0.01
                      + ((int)GPSInfo[i-3]-'0')*0.001 + ((int)GPSInfo[i-2]-'0')*0.0001;
      Longitude = deg + (minutes/60.0);

      if(GPSInfo[i] == 'W') Longitude *= -1;

      // Save for finding Fix
      longitudeIndex = i;
    }
  }

  fix = ((int)GPSInfo[longitudeIndex] - '0') > 0;

  return fix;
}

boolean youGPGGA()
{
// If 50 Bytes are available
  if(Serial1.available() >= 50)
  {
    //Serial.println("Here");
    //delay(1000);
    // Until no more byte are available
    while(Serial1.available() > 5)
    {
      if(Serial1.read() == 'G')
      {
        if(Serial1.read() == 'P')
        {
          if(Serial1.read() == 'G')
          {
            if(Serial1.read() == 'G')
            {
              if(Serial1.read() == 'A')
              {
                Serial.println("InGPGGA");
                
                return true;
              }
            }
          }
        }
      }
    }
  }
  return false;
}

void setup()
{
    delay(1000);

    // Servo
    Serial.begin(9600);
    Serial1.begin(9600);

#ifdef DEBUGYOU
    Serial.println("Start of Setup");
#endif

    // Motor Connections
    TomServo.attach(ServoPin);
    pinMode(MotorPin, OUTPUT);
    digitalWrite(MotorPin, LOW);

    // DO IO
    pinMode(LEDBlinker, OUTPUT);
    digitalWrite(LEDBlinker, LOW);
    pinMode(LEDGreen, OUTPUT);
    digitalWrite(LEDGreen, LOW);
    pinMode(LEDYellow, OUTPUT);
    digitalWrite(LEDYellow, LOW);
    pinMode(ButtonPin, INPUT_PULLUP);
    digitalWrite(ButtonPin, HIGH);  

    // Magnetometer Initialization
    if(!bno.begin())
    {
        // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    delay(1000);
    bno.setExtCrystalUse(true);
    
    delay(1000);
    // Ask for firmware version
#ifdef DEBUGYOU
    Serial.println("End of Setup");
#endif
}

// Get our current target azimuth
double GetAzimuth(double targetLat, double targetLong, double thisLat, double thisLong)
{
    double longitudinalDifference = targetLong - thisLong;
    double latitudinalDifference = targetLat - thisLat;
    double azimuth = (3.141 * 0.5) - atan(latitudinalDifference / longitudinalDifference);
    
    if (longitudinalDifference > 0)       return azimuth;
    else if (longitudinalDifference < 0)  return azimuth + 3.141;
    else if (latitudinalDifference < 0)   return 3.141;
    
    return 0;
}

boolean isCloseEnough(double targetLat, double targetLong, double thisLat, double thisLong)
{
    double CloseEnoughFactor = 0.0001;
    
    double longitudinalDifference = targetLong - thisLong;
    double latitudinalDifference = targetLat - thisLat;
    longitudinalDifference = (longitudinalDifference < 0) ? (longitudinalDifference*-1) : longitudinalDifference;
    latitudinalDifference  =  (latitudinalDifference < 0) ? (latitudinalDifference*-1) : latitudinalDifference;
    
#ifdef DEBUGYOU
    Serial.print("Target Long: "); Serial.print(targetLong, 6);
    Serial.print(" This Long: "); Serial.println(thisLong, 6);
    Serial.print("Long Diff: "); Serial.print(longitudinalDifference, 6);
    Serial.print(" Lat Diff: "); Serial.println(latitudinalDifference, 6);
#endif

    if((longitudinalDifference < CloseEnoughFactor && latitudinalDifference < CloseEnoughFactor) || digitalRead(ButtonPin) == 0)
    {
        // Blink Lights
        do
        {
            digitalWrite(LEDBlinker, HIGH);
            delay(500);
            digitalWrite(LEDBlinker, LOW);
            delay(500);
        }while(digitalRead(ButtonPin) != 0);
        goingToPos(2);

        return true;
    }
    else
    {
        return false;
    }
}

void goingToPos(int position)
{
    if(position == 1)
    {
        digitalWrite(LEDGreen, HIGH);
        digitalWrite(LEDYellow, LOW);
    }
    else
    {
        digitalWrite(LEDYellow, HIGH);
        digitalWrite(LEDGreen, LOW);
    }
}

// Turn the azimuth into degrees
double GetDegreesAzimuth(double targetLat, double targetLong, double thisLat, double thisLong)
{
    double curAz = ((180 * GetAzimuth(targetLat, targetLong, thisLat, thisLong))/3.141);
    return curAz > 180.0 ? -360 + curAz : curAz;
}

uint32_t timer = millis();
uint32_t timerAcc = millis();
int LLSize = 0;
void loop()
{

#ifdef DEBUGYOU
    Serial.println("Begin Main Loop");
#endif

    /////////////////////////////////////////
    // LINKED LIST WAYPOINT INITIALIZATION
    LLSize = 0;
    
    /*
    // Boatie's First Jouney
    // Van Brunt x Pioneer
    WayPoint newpoint;
    newpoint.latitude = 40.678920;
    newpoint.longitude = -74.01163;
    node* head = InitalizeLinkedList(newpoint);
    LLSize = 1;
    
    // Van Brunt x Sullvian
    newpoint.latitude = 40.677908;
    newpoint.longitude = -74.012301;
    addToBackOfList(newpoint, head);
    LLSize++;

    // Van Brunt x Dikeman
    newpoint.latitude = 40.676900;
    newpoint.longitude = -74.013515;
    addToBackOfList(newpoint, head);
    LLSize++;

    // Conover x Dikeman
    newpoint.latitude = 40.677632;
    newpoint.longitude = -74.014683;
    addToBackOfList(newpoint, head);
    LLSize++;

    // Conover x Sullivan
    newpoint.latitude = 40.678698;
    newpoint.longitude = -74.013469;
    addToBackOfList(newpoint, head);
    LLSize++;

    // Conover x Pioneer
    newpoint.latitude = 40.679699;
    newpoint.longitude = -74.012428;
    addToBackOfList(newpoint, head);
    LLSize++;

    // Pioneer Works
    newpoint.latitude = 40.679376;
    newpoint.longitude = -74.011692;
    addToBackOfList(newpoint, head);
    LLSize++;
    */

    // Exploring Pioneer Works
    // Top of Imlay
    WayPoint newpoint;
    newpoint.latitude = 40.683538;
    newpoint.longitude = -74.006870;
    node* head = InitalizeLinkedList(newpoint);
    LLSize = 1;

    // Straight South down street
    newpoint.latitude = 40.675135;
    newpoint.longitude = -74.016822;
    addToBackOfList(newpoint, head);
    LLSize++;

    // END LINKED LIST WAYPOINT INITIALIZATION
    /////////////////////////////////////////
    
    // Indicate going to position 1.
    goingToPos(1);
    
    for(int i = 0; i<LLSize; i++)
    {
#ifdef DEBUGYOU
        Serial.println("Begin LL Loop");
#endif

        WayPoint curWayPoint = popFront(&head);
        targetLat = curWayPoint.latitude;
        targetLongitude = curWayPoint.longitude;

#ifdef DEBUGYOU
        Serial.print("New WAYPOINT: Latitude ->");
        Serial.print(targetLat,6);
        Serial.print(" longitude ->");
        Serial.println(targetLongitude,6);
        delay(1000);
#endif
        
        boolean closeEnough = false;
        while(!closeEnough)
        {

            if (timerAcc > millis())  timerAcc = millis();
            if(millis() - timerAcc > BNO055_SAMPLERATE_DELAY_MS)
            {
                timerAcc = millis();
                
                // Possible vector values can be:
                // - VECTOR_ACCELEROMETER - m/s^2
                // - VECTOR_MAGNETOMETER  - uT
                // - VECTOR_GYROSCOPE     - rad/s
                // - VECTOR_EULER         - degrees
                // - VECTOR_LINEARACCEL   - m/s^2
                // - VECTOR_GRAVITY       - m/s^2
                
                
                imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
                
                // absolute position in degrees
                float heading = atan2(magnet.x(), magnet.y());
                curMagnetometerAngle = -1*(heading* (57296 / 1000));
                //Serial.println(curMagnetometerAngle);
                curOffsetAngle = curGPSAngle - curMagnetometerAngle;
                
                input = curOffsetAngle;
                angle = map(input, -180, 180, 30, 130);
                angle = constrain(angle, 30, 130);
                TomServo.write(angle);

                // Start or end motor rotation
                Serial.print(acceptableAngleForFullThrottle);
                Serial.print(" ");
                Serial.println(abs(curOffsetAngle));
                if(abs(curOffsetAngle) < acceptableAngleForFullThrottle)
                {
                    analogWrite(MotorPin, 80);
                }
                else
                {
                    analogWrite(MotorPin, 0);
                }

#ifdef DEBUGYOU
                Serial.print("Current Angle is: ");
                Serial.println(angle);
#endif
            }
            
            if(millis() - timer > 2000)
            {
                timer = millis();
            if (youGPGGA())
            {
                double Latitude = 0.0;
                double Longitude = 0.0;
                if(getGPSInfo(Latitude, Longitude))
                {
                #ifdef DEBUGYOU
                Serial.print("Current: ");
                Serial.print(Latitude, 6);
                Serial.print(" , ");
                Serial.println(Longitude, 6);
                Serial.print("Target: ");
                Serial.print(targetLat, 6);
                Serial.print(" , ");
                Serial.println(targetLongitude, 6);
                #endif

                digitalWrite(LEDBlinker, HIGH);
                
                curGPSAngle = GetDegreesAzimuth(targetLat, targetLongitude, Latitude, Longitude);
                
                #ifdef DEBUGYOU
                Serial.print("After Get Azimuth: ");
                Serial.println(curGPSAngle);
                #endif

                closeEnough = isCloseEnough(targetLat, targetLongitude, Latitude, Longitude);
                
                #ifdef DEBUGYOU
                Serial.println("After Calculation");
                #endif
            }
            else
            {
                digitalWrite(LEDBlinker, LOW);
            }
            }
        }
            
        }
    }
}
