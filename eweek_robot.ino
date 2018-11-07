 //TMP36 Pin Variables
int sensorPin = 0; //the analog pin the TMP36's Vout (sense) pin is connected to
                        //the resolution is 10 mV / degree centigrade with a
                        //500 mV offset to allow for negative temperatures

// This version uses discrete temperature ranges to determine how quickly to blink the LED

float friendHeatSignature = 100;

int LED_PIN = 13;

float LOW_TEMP         = 70;  // F
float HI_TEMP          = 98;  // F
float MAX_REASONABLE_F = 200; // F
unsigned long MAX_DELAY = 10000; // 0.2 Hz = 5000ms
unsigned long MIN_DELAY = 40;    // 25  Hz = 40ms
unsigned long MIN_FLASH = 60;

float updateTemp()
{
  //getting the voltage reading from the temperature sensor
  int reading = analogRead(sensorPin);  
 
  // converting that reading to voltage, for 3.3v arduino use 3.3
  float voltage = reading * 5.0;
  voltage /= 1024.0; 

  float temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
                                                //to degrees ((voltage - 500mV) times 100)
  return temperatureC;
}
 
/*
 * setup() - this function runs once when you turn your Arduino on
 * We initialize the serial connection with the computer
 */
void setup()
{
  Serial.begin(9600);  //Start the serial connection with the computer
                       //to view the result open the serial monitor 

  // Setup the built in LED Arduino LED to indicate when we've found our friend
  pinMode(LED_PIN, OUTPUT);
}

void setLEDsEnabled(bool on)
{
  if (on)
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }
}

unsigned long lookupDelay(float tempF)
{
  unsigned long msDelay = 2000;

  // Under LOW_TEMP (70 deg)
  if (tempF < LOW_TEMP)
  {
    msDelay = 4000;
  }
  // [70, 80)
  else if (tempF >= 70 && tempF < 80)
  {
    msDelay = 2000;
  }
  // [80, 90)
  else if (tempF >= 80 && tempF < 90)
  {
    msDelay = 1000;
  }
  // [90, 95)
  else if (tempF >= 90 && tempF < 95)
  {
    msDelay = 500;
  }
  // [95, 100)
  else if (tempF >= 95 && tempF < 100)
  {
    msDelay = 250;
  }

  return msDelay;
}

void loop()
{
  static bool ledOn     = false;
  static int  iterDelay = MIN_DELAY / 2; // Delay causes loop to run at 50 Hz
  
  static unsigned long msToNextFlash = 0;
  static unsigned long timeLastFlash = 0;
  static unsigned long timeNow       = 0;
  static unsigned long ledOnTime     = MIN_FLASH; 
  static unsigned long lastPrint     = 0;

  // Update the temperature
  float temperatureC = updateTemp();
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;

  // Update current time running
  timeNow = millis();

  if ( temperatureF > MAX_REASONABLE_F )
  {
    int i = 0;
    
    // Flash an emergency signal!
    for ( i = 0; i < 3; i++)
    {
      setLEDsEnabled(true);
      delay(100);    
      setLEDsEnabled(false);
      delay(100);    
    }

    delay(250);

    for ( i = 0; i < 3; i++)
    {
      setLEDsEnabled(true);
      delay(250);    
      setLEDsEnabled(false);
      delay(100);    
    }
    
    delay(250);

    for ( i = 0; i < 3; i++)
    {
      setLEDsEnabled(true);
      delay(100);    
      setLEDsEnabled(false);
      delay(100);    
    }

    delay(1000);
  }
  else if ( temperatureF >= 100 )
  {
    if (!ledOn)
    {
      ledOn = true;
      setLEDsEnabled(ledOn);
    }
  }
  else
  {
    // Determine if LED should be flashed
    if ( timeNow - timeLastFlash >= msToNextFlash )
    {
      // Flash the LEDs
      timeLastFlash = timeNow;
      
      // Turn the LED on
      if( !ledOn )
      {
        ledOn = true;
      }
    }
    // Determine if LED should be turned off
    else if (ledOn && timeNow - timeLastFlash >= ledOnTime )
    {
      ledOn = false;
    }

    // Update the output
    setLEDsEnabled(ledOn);
  }
  
  // Calculate how long to wait until the next flash
  msToNextFlash = lookupDelay(temperatureF);
  
  // Keep the LED on 20% of the time between flashes
  ledOnTime = max(msToNextFlash / 5, MIN_FLASH);

  if (timeNow - lastPrint > 250)
  {
    lastPrint = timeNow;
    Serial.print(temperatureF); Serial.println(" degrees F");
    Serial.print(timeNow); Serial.print("; "); Serial.print(timeLastFlash); Serial.print("; "); Serial.println(msToNextFlash); 
  }

  delay(iterDelay);
}
