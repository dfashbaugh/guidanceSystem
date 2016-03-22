void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
}

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
    if(Serial1.available())
  {
    if(Serial1.read() == 'G')
    {
      while(!Serial1.available()){}
      if(Serial1.read() == 'P')
      {
        while(!Serial1.available()){}
        if(Serial1.read() == 'G')
        {
          while(!Serial1.available()){}
          if(Serial1.read() == 'G')
          {
            while(!Serial1.available()){}
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
  return false;
}

void loop() {
    if(youGPGGA())
    {
      double Latitude = 0.0;
      double Longitude = 0.0;
      if(getGPSInfo(Latitude, Longitude))
      {
        Serial.print(Latitude,6);
        Serial.print(" , ");
        Serial.println(Longitude,6);
      }
    }
}
