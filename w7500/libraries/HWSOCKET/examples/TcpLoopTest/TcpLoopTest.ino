/*
  Web Server

 A simple web server that shows the value of the analog input pins.
 using an Arduino Wiznet Ethernet shield.

 Circuit:
 * Ethernet shield attached to pins 10, 11, 12, 13
 * Analog inputs attached to pins A0 through A5 (optional)

 created 18 Dec 2009
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe

 */

//#include <SPI.h>
#include <Ethernet.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
    0x00, 0x08, 0xDC, 0x01, 0x02, 0x03
};
IPAddress ip(192, 168, 77, 9);
IPAddress gateway(192, 168, 77, 1);
IPAddress subnet(255, 255, 255, 0);

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(5000);
boolean alreadyConnected = false; // whether or not the client was connected previously

void setup() 
{
    //Open serial communications and wait for port to open:
    //Serial.begin(9600);
    Serial.begin(115200);
    while (!Serial) 
    {
        ; // wait for serial port to connect. Needed for Leonardo only
    }

    // start the Ethernet connection and the server:
    // initialize the ethernet device
    Ethernet.begin(mac, ip, gateway, subnet);
    // start listening for clients
    server.begin();
    Serial.print("Chat server address:");
    Serial.println(Ethernet.localIP());
}

void loop() 
{
//    Serial.println("11111111111111111111111111111");
    // wait for a new client:
    EthernetClient client = server.available();
    // when the client sends the first byte, say hello:
    if (client) 
    {
#if 0
        if (!alreadyConnected)
        {
            // clead out the input buffer:
            client.flush();
            Serial.println("We have a new client");
            client.println("Hello, client!");
            alreadyConnected = true;
        }
#endif
//    Serial.println("22222222222222222222222222222");
        if (client.available() > 0) 
        {
            // read the bytes incoming from the client:
            char thisChar = client.read();
//    Serial.println("3333333333333333333333333333333");
            // echo the bytes back to the client:
            server.write(thisChar);
    Serial.println("4444444444444444444444444444444");
            // echo the bytes to the server as well:
            Serial.write(thisChar);
        }
    Serial.println("5555555555555555555555555555555");
    }
}