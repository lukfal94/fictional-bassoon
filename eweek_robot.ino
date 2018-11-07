 //TMP36 Pin Variables
int sensorPin = 0; //the analog pin the TMP36's Vout (sense) pin is connected to
                        //the resolution is 10 mV / degree centigrade with a
                        //500 mV offset to allow for negative temperatures

float friendHeatSignature = 100;

float LOW_TEMP  = 70;   // F
float HI_TEMP   = 98;   // F
unsigned long MAX_DELAY = 10000; // 0.2 Hz = 5000ms
unsigned long MIN_DELAY = 40;    // 25  Hz = 40ms

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
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(8 , OUTPUT);
}

void setLEDsEnabled(bool on)
{
  if (on)
  {
    digitalWrite(13, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(11, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(8 , HIGH);
  }
  else
  {
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(10, LOW);
    digitalWrite(8 , LOW);
  }
}

void loop()
{
  static bool ledOn     = false;
  static int  iterDelay = MIN_DELAY / 2; // Delay causes loop to run at 50 Hz
  
  static unsigned long msToNextFlash = 0;
  static unsigned long timeLastFlash = 0;
  static unsigned long timeNow       = 0;
  static unsigned long ledOnTime     = map(LOW_TEMP, LOW_TEMP, HI_TEMP, MAX_DELAY, MIN_DELAY);
  static unsigned long lastPrint     = 0;

  // Update the temperature
  float temperatureC = updateTemp();
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;

  // Update current time running
  timeNow = millis();

  if ( temperatureF >= 100 )
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
  msToNextFlash = map(temperatureF, LOW_TEMP, HI_TEMP, MAX_DELAY, MIN_DELAY);
  
  // Keep the LED on 25% of the time between flashes
  ledOnTime = msToNextFlash / 4;

  if (timeNow - lastPrint > 250)
  {
    lastPrint = timeNow;
    Serial.print(temperatureF); Serial.println(" degrees F");
    Serial.print(timeNow); Serial.print("; "); Serial.print(timeLastFlash); Serial.print("; "); Serial.println(msToNextFlash); 
  }

  delay(iterDelay);
}
