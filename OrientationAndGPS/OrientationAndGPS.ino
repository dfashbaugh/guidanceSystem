#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

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



//SoftwareSerial mySerial(3, 2);

Adafruit_GPS GPS(&Serial);

double targetLat = 40.6785;
double targetLongitude = -74.0182;
double curOffsetAngle = 0.0;
double curMagnetometerAngle = 0.0;
double curGPSAngle = 0.0;


#define GPSECHO  false
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()
{
    // Servo
    Serial.begin(9600);
    TomServo.attach(3);
    pinMode(10, OUTPUT);
    digitalWrite(10, LOW);
    
    
    // Magnetometer Initialization
    if(!bno.begin())
    {
        // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    delay(1000);
    bno.setExtCrystalUse(true);
    
    
    // GPS Initialization
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);
    useInterrupt(true);
    
    delay(1000);
    // Ask for firmware version
    Serial.println(PMTK_Q_RELEASE);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect)
{
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    #ifdef UDR0
    if (GPSECHO)
    if (c) UDR0 = c;
    // writing direct to UDR0 is much much faster than Serial.print
    // but only one character can be written at a time.
    #endif
}

// Turn on interrupt at low level in UNO... Will probably need to change for Teensy
void useInterrupt(boolean v)
{
    if (v)
    {
        // Timer0 is already used for millis() - we'll just interrupt somewhere
        // in the middle and call the "Compare A" function above
        OCR0A = 0xAF;
        TIMSK0 |= _BV(OCIE0A);
        usingInterrupt = true;
    } else
    {
        // do not call the interrupt function COMPA anymore
        TIMSK0 &= ~_BV(OCIE0A);
        usingInterrupt = false;
    }
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
    double CloseEnoughFactor = 20.0;
    
    double longitudinalDifference = targetLong - thisLong;
    double latitudinalDifference = targetLat - thisLat;
    
    if(longitudinalDifference < CloseEnoughFactor && latitudinalDifference < CloseEnoughFactor)
    {
        return true;
    }
    else
    {
        return false;
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
    /////////////////////////////////////////
    // LINKED LIST WAYPOINT INITIALIZATION
    LLSize = 0;
    
    WayPoint newpoint;
    newpoint.latitude = 2;
    newpoint.longitude = 4;
    node* head = InitalizeLinkedList(newpoint);
    LLSize = 1;
    
    newpoint.latitude = 5;
    newpoint.longitude = 3;
    addToBackOfList(newpoint, head);
    LLSize++;
    // END LINKED LIST WAYPOINT INITIALIZATION
    /////////////////////////////////////////
    
    
    for(int i = 0; i<LLSize; i++)
    {
        WayPoint curWayPoint = popFront(&head);
        targetLat = curWayPoint.latitude;
        targetLongitude = curWayPoint.longitude;
        
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
                angle = map(input, -180, 180, 0, 179);
                angle = constrain(angle, 0, 179);
                TomServo.write(angle);
                
                if (angle > 55 && angle < 125)
                {
                    if (angle > 86 && angle < 94)
                    {
                        analogWrite(5,0);
                        analogWrite(11, 255);
                    }
                    else
                    {
                        analogWrite(11, 255/(abs(90-angle)+1));
                        analogWrite(5, 255 - 255/(abs(90-angle)+1));
                    }
                }
                else
                {
                    analogWrite(11,0);
                    if (angle < 55)
                    {
                        analogWrite(5, map(angle, 0, 55, 0, 255));
                    }
                    if (angle > 125)
                    {
                        analogWrite(5, map(angle, 125, 179, 255, 0));
                    }
                }
                
            }
            
            // if a sentence is received, we can check the checksum, parse it...
            if (GPS.newNMEAreceived()) {
                if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
                return;  // we can fail to parse a sentence in which case we should just wait for another
            }
            
            // if millis() or timer wraps around, we'll just reset it
            if (timer > millis())  timer = millis();
            
            // approximately every 2 seconds or so, print out the current stats
            if (millis() - timer > 2000)
            {
                timer = millis(); // reset the timer
                
                if (GPS.fix)
                {
                    
                    // Debugging Output
                    /*  Serial.print("Location (in degrees): ");
                    Serial.print(GPS.latitudeDegrees, 4);
                    Serial.print(", ");
                    Serial.println(GPS.longitudeDegrees, 4);
                    
                    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
                    Serial.print("Angle: "); Serial.println(GPS.angle);
                    Serial.print("Altitude: "); Serial.println(GPS.altitude);
                    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
                    Serial.println();
                    Serial.print("Distance from Target: "); Serial.print(distanceToLat, 4); Serial.print(", ");
                    Serial.println(distanceToLong);
                    Serial.print("Angle to Target: "); Serial.println(GetDegreesAzimuth(targetLat, targetLongitude, GPS.latitudeDegrees, GPS.longitudeDegrees));
                    Serial.println();
                    */
                    
                    curGPSAngle = GetDegreesAzimuth(targetLat, targetLongitude, GPS.latitudeDegrees, GPS.longitudeDegrees);
                    closeEnough = isCloseEnough(targetLat, targetLongitude, GPS.latitudeDegrees, GPS.longitudeDegrees);
                }
            }
            
        }
    }
}
