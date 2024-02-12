// ARDUINO NANO ESP32 SPOD version 0.10
// VERSION DATE 1/15/2024
// WRITTEN BY ADAM GOLDEN FOR UALR IOST LAB
// special thanks to Dr. Phil Williams, Dr. Robert Belford, and authors of code examples used to make this project.
// https://circuitdigest.com/microcontroller-projects/interfacing-mq2-gas-sensor-with-arduino
// https://randomnerdtutorials.com/esp32-web-server-arduino-ide/


/*
Change Log
version 0.10 
implemented adapted code from the MKR 1010 WIFI spod. 
this version created and tested with no sensors connected.
pin choices made based on Arduino provided pinout. 
Current issues: 
wifi server example used will print connected despite not actually connecting. Need to implement checks from MKR 1010 WIFI, or something accomplishing same results. 
Current version will print bogus 0.0.0.0 IP address. 

version 0.11
Switched order of initializing wifi connection and initializing BME sensor.
Added check to ensure real wifi connection before claiming wifi was connected and stopping attempts.
Current issues:
Not all disconnected pins read 0 during server ping.
Check if these pins are set as outputs by default and see if setting as inputs during setup solves problem. 

version 0.12
making outputs more machine readable for purposes of easy spreadsheet insertion by condensing all output values to a single line, comma separated output variables.
two arrays that are written to by single functions then looped through to output to html
corrected typo in #define BME_MOSI 12 wrong pin, was previously 13.
*/


// libraries
// example code for wifi server only uses WiFi.h
// see if removing others breaks thigs, these libraries are from Phil's code
// SEE ABOVE: it compiled with the other 3 commented out, need further testing to see if it messes up
// it seems to work fine. leaving other libraries here in case they are useful.
#include <WiFi.h>
//#include <WiFiClient.h>
//#include <WebServer.h>
//#include <ESPmDNS.h>

#include <stdlib.h>

#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// define moxpin
#define moxPin0 A0
#define moxPin1 A1
#define moxPin2 A2
#define moxPin3 A3
#define moxPin4 A4
#define moxPin5 A5

// mox sensor array
int arrMox[6];

// define BME stuff
// these digital pins seem to make sense. testing needed to see if they don't work
#define BME_SCK D13
#define BME_MISO D11
#define BME_MOSI D12
#define BME_CS D10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme;

// bme sensor array
int arrBME[5];

// WIFI INFO
const char* ssid = "UALR-IoT";
const char* password = "";

// Set web server port 80
WiFiServer server(80);

void setup() {
  // i see this baud rate on every esp32 implementation. 
  // VERY IMPORTANT: THESE DELAYS SEEM TO MAKE IT WORK. DON'T TOUCH IT. 
  digitalWrite(LED_BLUE, LOW);
  Serial.begin(115200);
  delay(1000);
  digitalWrite(LED_BLUE, HIGH);
  //digitalWrite(LED_RED, LOW);
  //delay(1000);
  //while(!Serial);
  //digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  delay(1000);
  digitalWrite(LED_GREEN, HIGH);

  //mac address
  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  // connect to wifi network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Attempting to connect to ssid: ");
  Serial.println(ssid);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // start web server
  server.begin();

  // Initialize bme sensor 
  Serial.print(F("BME680 test"));
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    // while (1); //temporary comment out for testing purposes with no sensors connected. 
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

}

void loop() {
  // listen for incoming clients
  WiFiClient client = server.available();

  if (client) {

    Serial.println("new client");

    // an http request ends with a blank line

    boolean currentLineIsBlank = true;

    while (client.connected()) {

      if (client.available()) {

        char c = client.read();

        Serial.write(c);

        // if you've gotten to the end of the line (received a newline

        // character) and the line is blank, the http request has ended,

        // so you can send a reply

        if (c == '\n' && currentLineIsBlank) {

          // send a standard http response header

          client.println("HTTP/1.1 200 OK");

          client.println("Content-Type: text/html");

          client.println("Connection: close");  // the connection will be closed after completion of the response

          //client.println("Refresh: 5");  // refresh the page automatically every 5 sec ADAM NOTE: WE MAY NOT WANT THIS. we want the client to decide how often the data comes. see if getting rid of this line stops the constant refresh.

          client.println();

          client.println("<!DOCTYPE HTML>");

          client.println("<html>");

          // output the value of each analog input pin
/*
          for (int analogChannel = 0; analogChannel < 7; analogChannel++) {

            int sensorReading = analogRead(analogChannel);

            client.print("analog input ");

            client.print(analogChannel);

            client.print(" is ");

            client.print(sensorReading);

            client.println("<br />");

          }
*/
        //attempting to make outputs more easily machine readable for spreadsheet input purposes. condensing outputs to singe line, values separated by commas.
        //BEGIN MQ SENSOR LOOP CODE
        /* attempting to turn this into single function with static array output. see setup for static array. 
        //client.print("Analog Pin0 output: ");
        client.println(readSensor0());
        //client.println("<br />");
        client.println(",");
        //client.print("Analog Pin1 output: ");
        client.println(readSensor1());
        client.println(",");
        //client.println("<br />");
        //client.print("Analog Pin2 output: ");
        client.println(readSensor2());
        client.println(",");
        //client.println("<br />");
        //client.print("Analog Pin3 output: ");
        client.println(readSensor3());
        client.println(",");
        //client.println("<br />");
        //client.print("Analog Pin4 output: ");
        client.println(readSensor4());
        client.println(",");
        //client.println("<br />");
        //client.print("Analog Pin5 output: ");
        client.println(readSensor5());
        client.println(",");
        //client.println("<br />");
        client.println();
        */
        //loop through 0-5 arrMox values and print to client
        readSensor();
        for (int i = 0; i < 6; i++) {
          client.println(arrMox[i]);
          client.println(",");
        }
        //END MQ SENSOR LOOP CODE
        //BEGIN BME 680 LOOP CODE
          if (! bme.performReading()) {
            client.println("Failed to perform reading :(");
            return;
          }
          /*
          //client.print("Temperature = ");
          client.print(bme.temperature);
          //client.println(" *C");
          //client.println("<br />");
          client.println(",");

          //client.print("Pressure = ");
          client.print(bme.pressure / 100.0);
          //client.println(" hPa");
          //client.println("<br />");
          client.println(",");

          //client.print("Humidity = ");
          client.print(bme.humidity);
          //client.println(" %");
          //client.println("<br />");
          client.println(",");

          //client.print("Gas = ");
          client.print(bme.gas_resistance / 1000.0);
          //client.println(" KOhms");
          //client.println("<br />");
          client.println(",");

          //client.print("Approx. Altitude = ");
          client.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
          //client.println(" m");
          //client.println("<br />");
          */
        readBME();
        for (int i = 0; i < 5; i++) {
          client.println(arrBME[i]);
          client.println(",");
        }
        // not sure what these two println do
        //  client.println();
        //  client.println();
        //END BME 680 LOOP CODE
          client.println("</html>");
          

          break;

        }

        if (c == '\n') {

          // you're starting a new line

          currentLineIsBlank = true;

        } else if (c != '\r') {

          // you've gotten a character on the current line

          currentLineIsBlank = false;

        }

      }

    }

    // give the web browser time to receive the data

    delay(1);


    // close the connection:

    client.stop();

    Serial.println("client disconnected");
  }

}

// This is super ugly and simple but it works. will figure out how to condense this with input values later. SEE MKR1010 SPOD CODE FOR POSSIBLE FUNCTION CONDENSATION
/*
int readSensor0() {
  unsigned int sensorValue = analogRead(moxPin0);  // Read the analog value from sensor
  unsigned int outputValue = map(sensorValue, 0, 1023, 0, 255); // map the 10-bit data to 8-bit data
  return outputValue;             // Return analog value
}
int readSensor1() {
  unsigned int sensorValue = analogRead(moxPin1);  // Read the analog value from sensor
  unsigned int outputValue = map(sensorValue, 0, 1023, 0, 255); // map the 10-bit data to 8-bit data
  return outputValue;             // Return analog value
}
int readSensor2() {
  unsigned int sensorValue = analogRead(moxPin2);  // Read the analog value from sensor
  unsigned int outputValue = map(sensorValue, 0, 1023, 0, 255); // map the 10-bit data to 8-bit data
  return outputValue;             // Return analog value
}
int readSensor3() {
  unsigned int sensorValue = analogRead(moxPin3);  // Read the analog value from sensor
  unsigned int outputValue = map(sensorValue, 0, 1023, 0, 255); // map the 10-bit data to 8-bit data
  return outputValue;             // Return analog value
}
int readSensor4() {
  unsigned int sensorValue = analogRead(moxPin4);  // Read the analog value from sensor
  unsigned int outputValue = map(sensorValue, 0, 1023, 0, 255); // map the 10-bit data to 8-bit data
  return outputValue;             // Return analog value
}
int readSensor5() {
  unsigned int sensorValue = analogRead(moxPin5);  // Read the analog value from sensor
  unsigned int outputValue = map(sensorValue, 0, 1023, 0, 255); // map the 10-bit data to 8-bit data
  return outputValue;             // Return analog value
}
*/
// attempt writing sensor values into array for function condensation. figure out how to loop this later.
void readSensor() {
  /*
  for (int i = 0; i < 5; i++) { //or i <= 4
  unsigned int sensorValue = analogRead([i]);
  unsigned int outputValue = map(sensorValue, 0, 1023, 0, 255);
  }
  */
  unsigned int sensorValue = analogRead(moxPin0);  // Read the analog value from sensor
  unsigned int outputValue = map(sensorValue, 0, 1023, 0, 255); // map the 10-bit data to 8-bit data
  arrMox[0] = outputValue;
  sensorValue = analogRead(moxPin1);  // Read the analog value from sensor
  outputValue = map(sensorValue, 0, 1023, 0, 255); // map the 10-bit data to 8-bit data
  arrMox[1] = outputValue;
  sensorValue = analogRead(moxPin2);  // Read the analog value from sensor
  outputValue = map(sensorValue, 0, 1023, 0, 255); // map the 10-bit data to 8-bit data
  arrMox[2] = outputValue;
  sensorValue = analogRead(moxPin3);  // Read the analog value from sensor
  outputValue = map(sensorValue, 0, 1023, 0, 255); // map the 10-bit data to 8-bit data
  arrMox[3] = outputValue;
  sensorValue = analogRead(moxPin4);  // Read the analog value from sensor
  outputValue = map(sensorValue, 0, 1023, 0, 255); // map the 10-bit data to 8-bit data
  arrMox[4] = outputValue;
  sensorValue = analogRead(moxPin5);  // Read the analog value from sensor
  outputValue = map(sensorValue, 0, 1023, 0, 255); // map the 10-bit data to 8-bit data
  arrMox[5] = outputValue;
}
// bme read function write to array order: temperature,pressure,humidity,gas_resistance,altitude
void readBME() {
  arrBME[0] = bme.temperature;
  arrBME[1] = bme.pressure;
  arrBME[2] = bme.humidity;
  arrBME[3] = (bme.gas_resistance / 1000.0);
  arrBME[4] = bme.readAltitude(SEALEVELPRESSURE_HPA);
}
