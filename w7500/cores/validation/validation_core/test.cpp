/*
  Copyright (c) 2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"

static void Interrupt_Pin3( void ) ; // Ext Int Callback in LOW mode
static void Interrupt_Pin4( void ) ; // Ext Int Callback in HIGH mode
static void Interrupt_Pin5( void ) ; // Ext Int Callback in FALLING mode
static void Interrupt_Pin6( void ) ; // Ext Int Callback in RISING mode
static void Interrupt_Pin7( void ) ; // Ext Int Callback in CHANGE mode

static uint32_t ul_Interrupt_Pin3 = 0 ;
static uint32_t ul_Interrupt_Pin4 = 0 ;
static uint32_t ul_Interrupt_Pin5 = 0 ;
static uint32_t ul_Interrupt_Pin6 = 0 ;
static uint32_t ul_Interrupt_Pin7 = 0 ;

int temps = 0;
int valX = 0;
int valY = 0;

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int sensorMin = 1023;        // minimum sensor value
int sensorMax = 0;           // maximum sensor value

// declare three strings:
String stringOne, stringTwo, stringThree;

void setup( void )
{
//  Serial5.begin( 115200 ) ; // Output to EDBG Virtual COM Port
  // turn on LED to signal the start of the calibration period:
  Serial.begin( 115200 ) ; // Output to EDBG Virtual COM Port

  stringOne = String("stringThree = ");
  stringTwo = String("this string");
  stringThree = String ();
  // send an intro:
  Serial.println("\n\nAdding strings together (concatenation):");
  Serial.println();


#if 0
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);

  // calibrate during the first five seconds
  while (millis() < 5000) {
    sensorValue = analogRead(A0/*sensorPin*/);

    // record the maximum sensor value
    if (sensorValue > sensorMax) {
      sensorMax = sensorValue;
    }

    // record the minimum sensor value
    if (sensorValue < sensorMin) {
      sensorMin = sensorValue;
    }
  }

  // signal the end of the calibration period
  digitalWrite(3, LOW);
#endif
  
  
#if 0
  pinMode( 0, OUTPUT ) ;
  pinMode( 1, OUTPUT ) ;
  pinMode( 4, OUTPUT ) ;
  pinMode( 5, OUTPUT ) ;
  pinMode( 6, OUTPUT ) ;
  pinMode( 7, OUTPUT ) ;
  pinMode( 8, OUTPUT ) ;
  pinMode( 9, OUTPUT ) ;
  pinMode( 10, OUTPUT ) ;
  pinMode( 11, OUTPUT ) ;
  pinMode( 12, OUTPUT ) ;
  pinMode( 13, OUTPUT ) ;
#endif

#if 0
  // Initialize the digital pin as an output.
  // Pin PIN_LED has an LED connected on most Arduino boards:
  //pinMode( PIN_LED, OUTPUT ) ;
  //digitalWrite( PIN_LED, LOW ) ;

  // Initialize the PIN_LED2 digital pin as an output.
  pinMode( PIN_LED2, OUTPUT ) ;
  digitalWrite( PIN_LED2, HIGH ) ;

  // Initialize the PIN_LED3 digital pin as an output.
  pinMode( PIN_LED3, OUTPUT ) ;
  digitalWrite( PIN_LED3, LOW ) ;

  // Initialize the PIN 2 digital pin as an input.
  pinMode( 2, INPUT ) ;

//**********************************************
// Clock output on pin 4 for measure

 /* pinPeripheral( 4, PIO_AC_CLK ) ; // Clock Gen 0
  pinPeripheral( 5, PIO_AC_CLK ) ; // Clock Gen 1
  pinPeripheral( 13, PIO_AC_CLK ) ; // Clock Gen 3*/

//**********************************************
  Serial5.begin( 115200 ) ; // Output to EDBG Virtual COM Port

  // Test External Interrupt
  attachInterrupt( 3, Interrupt_Pin3, LOW ) ;
  attachInterrupt( 4, Interrupt_Pin4, HIGH ) ;
  attachInterrupt( 5, Interrupt_Pin5, FALLING ) ;
  attachInterrupt( 6, Interrupt_Pin6, RISING ) ;
  attachInterrupt( 7, Interrupt_Pin7, CHANGE) ;
#endif
}

static void led_step1( void )
{
//  digitalWrite( PIN_LED, LOW ) ;  // set the LED on
  digitalWrite( PIN_LED2, LOW ) ;   // set the red LED off
  digitalWrite( PIN_LED3, HIGH ) ;   // set the red LED off
}

static void led_step2( void )
{
//  digitalWrite( PIN_LED, HIGH ) ;   // set the LED off
  digitalWrite( PIN_LED2, HIGH ) ;  // set the red LED on
  digitalWrite( PIN_LED3, LOW ) ;  // set the red LED on
}

void loop( void )
{
//  Serial.write( '\n' ) ;   // send a char
//  Serial.write( '\r' ) ;   // send a char
//  Serial5.write( '-' ) ;   // send a char

  // adding a constant integer to a string:
  stringThree =  stringOne + 123;
  Serial.println(stringThree);    // prints "stringThree = 123"

  // adding a constant long interger to a string:
  stringThree = stringOne + 123456789;
  Serial.println(stringThree);    // prints " You added 123456789"

  // adding a constant character to a string:
  stringThree =  stringOne + 'A';
  Serial.println(stringThree);    // prints "You added A"

  // adding a constant string to a string:
  stringThree =  stringOne +  "abc";
  Serial.println(stringThree);    // prints "You added abc"

  stringThree = stringOne + stringTwo;
  Serial.println(stringThree);    // prints "You added this string"

  // adding a variable integer to a string:
  int sensorValue = analogRead(A0);
  stringOne = "Sensor value: ";
  stringThree = stringOne  + sensorValue;
  Serial.println(stringThree);    // prints "Sensor Value: 401" or whatever value analogRead(A0) has

  // adding a variable long integer to a string:
  long currentTime = millis();
  stringOne = "millis() value: ";
  stringThree = stringOne + millis();
  Serial.println(stringThree);    // prints "The millis: 345345" or whatever value currentTime has

  // do nothing while true:
  while (true);



#if 0
  // get any incoming bytes:
  if (Serial.available() > 0) {
    int thisChar = Serial.read();

////////    int thisChar = 'a';
    // say what was sent:
    Serial.print("You sent me: \'");
    Serial.write(thisChar);
    Serial.print("\'  ASCII Value: ");
    Serial.println(thisChar);

    // analyze what was sent:
    if (isAlphaNumeric(thisChar)) {
      Serial.println("it's alphanumeric");
    }
    if (isAlpha(thisChar)) {
      Serial.println("it's alphabetic");
    }
    if (isAscii(thisChar)) {
      Serial.println("it's ASCII");
    }
    if (isWhitespace(thisChar)) {
      Serial.println("it's whitespace");
    }
    if (isControl(thisChar)) {
      Serial.println("it's a control character");
    }
    if (isDigit(thisChar)) {
      Serial.println("it's a numeric digit");
    }
    if (isGraph(thisChar)) {
      Serial.println("it's a printable character that's not whitespace");
    }
    if (isLowerCase(thisChar)) {
      Serial.println("it's lower case");
    }
    if (isPrintable(thisChar)) {
      Serial.println("it's printable");
    }
    if (isPunct(thisChar)) {
      Serial.println("it's punctuation");
    }
    if (isSpace(thisChar)) {
      Serial.println("it's a space character");
    }
    if (isUpperCase(thisChar)) {
      Serial.println("it's upper case");
    }
    if (isHexadecimalDigit(thisChar)) {
      Serial.println("it's a valid hexadecimaldigit (i.e. 0 - 9, a - F, or A - F)");
    }

    // add some space and ask for another byte:
    Serial.println();
    Serial.println("Give me another byte:");
    Serial.println();
  }
#endif

#if 0
  sensorValue = analogRead(A0);

  // apply the calibration to the sensor reading
  sensorValue = map(sensorValue, sensorMin, sensorMax, 0, 255);
  Serial.print("sensor = " );
  Serial.print(sensorValue);

  // in case the sensor value is outside the range seen during calibration
  outputValue = constrain(sensorValue, 0, 255);


  // print the results to the serial monitor:
  Serial.print("\t output = ");
  Serial.println(outputValue);

  // fade the LED using the calibrated value:
  analogWrite(9/*ledPin*/, sensorValue);
  
  delay(1);
#endif
#if 0
  // read the analog in value:
  sensorValue = analogRead(A0);
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);
  // change the analog out value:
  analogWrite(9/*analogOutPin*/, outputValue);

  // print the results to the serial monitor:
  Serial.print("sensor = " );
  Serial.print(sensorValue);
  Serial.print("\t output = ");
  Serial.println(outputValue);

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
 delay(2);
//  delay(1000);
#endif

#if 0
  digitalWrite( 0, HIGH ) ;  // set the red LED on
  digitalWrite( 0, LOW ) ;  // set the red LED on

  digitalWrite( 1, HIGH ) ;  // set the red LED on
  digitalWrite( 1, LOW ) ;  // set the red LED on

  digitalWrite( 4, HIGH ) ;  // set the red LED on
  digitalWrite( 4, LOW ) ;  // set the red LED on

  digitalWrite( 5, HIGH ) ;  // set the red LED on
  digitalWrite( 5, LOW ) ;  // set the red LED on

  digitalWrite( 6, HIGH ) ;  // set the red LED on
  digitalWrite( 6, LOW ) ;  // set the red LED on

  digitalWrite( 7, HIGH ) ;  // set the red LED on
  digitalWrite( 7, LOW ) ;  // set the red LED on

  digitalWrite( 8, HIGH ) ;  // set the red LED on
  digitalWrite( 8, LOW ) ;  // set the red LED on

  digitalWrite( 9, HIGH ) ;  // set the red LED on
  digitalWrite( 9, LOW ) ;  // set the red LED on

  digitalWrite( 10, HIGH ) ;  // set the red LED on
  digitalWrite( 10, LOW ) ;  // set the red LED on

  digitalWrite( 11, HIGH ) ;  // set the red LED on
  digitalWrite( 11, LOW ) ;  // set the red LED on

  digitalWrite( 12, HIGH ) ;  // set the red LED on
  digitalWrite( 12, LOW ) ;  // set the red LED on

  digitalWrite( 13, HIGH ) ;  // set the red LED on
  digitalWrite( 13, LOW ) ;  // set the red LED on
#endif

#if 0
//    int a = 123;
//    Serial.print(a, DEC);

//  for ( uint32_t i = A0 ; i <= A0+NUM_ANALOG_INPUTS ; i++ )
  for ( uint32_t i = 0 ; i <= NUM_ANALOG_INPUTS ; i++ )
  {

    int a = analogRead(i);
//    int a = 123;
    Serial.print(a, DEC);
    Serial.print(" ");
//   Serial.write( ' ' ) ;   // send a char
//   Serial.write( 0x30 + i ) ;   // send a char

//   Serial.write( 0x30 + ((a/1000)%10) ) ;   // send a char
//   Serial.write( 0x30 + ((a/100)%10) ) ;   // send a char
//   Serial.write( 0x30 + ((a/10)%10) ) ;   // send a char
//   Serial.write( 0x30 + (a%10) ) ;   // send a char
//   Serial.write( ' ' ) ;   // send a char
  }
  Serial.println();
#endif

#if 0
  volatile int pin_value=0 ;
  static volatile uint8_t duty_cycle=0 ;
  static volatile uint16_t dac_value=0 ;

  // Test digitalWrite
  led_step1() ;
  delay( 500 ) ;              // wait for a second
  led_step2() ;
  delay( 500 ) ;              // wait for a second

  // Test Serial output
  Serial5.write( '-' ) ;   // send a char
  Serial5.write( "test1\n" ) ;   // send a string
  Serial5.write( "test2" ) ;   // send another string

  // Test digitalRead: connect pin 2 to either GND or 3.3V. !!!! NOT on 5V pin !!!!
  pin_value=digitalRead( 2 ) ;
  Serial5.write( "pin 2 value is " ) ;
  Serial5.write( (pin_value == LOW)?"LOW\n":"HIGH\n" ) ;

  duty_cycle+=8 ;//=(uint8_t)(millis() & 0xff) ;
  analogWrite( 13, duty_cycle ) ;
  analogWrite( 12, duty_cycle ) ;
  analogWrite( 11, duty_cycle ) ;
  analogWrite( 10 ,duty_cycle ) ;
  analogWrite(  9, duty_cycle ) ;
  analogWrite(  8, duty_cycle ) ;

  dac_value += 64;
  analogWrite(A0, dac_value);



  Serial5.print("\r\nAnalog pins: ");

  for ( uint32_t i = A0 ; i <= A0+NUM_ANALOG_INPUTS ; i++ )
  {

    int a = analogRead(i);
    Serial5.print(a, DEC);
    Serial5.print(" ");

  }
  Serial5.println();

  Serial5.println("External interrupt pins:");
  if ( ul_Interrupt_Pin3 == 1 )
  {
    Serial5.println( "Pin 3 triggered (LOW)" ) ;
    ul_Interrupt_Pin3 = 0 ;
  }

  if ( ul_Interrupt_Pin4 == 1 )
  {
    Serial5.println( "Pin 4 triggered (HIGH)" ) ;
    ul_Interrupt_Pin4 = 0 ;
  }

  if ( ul_Interrupt_Pin5 == 1 )
  {
    Serial5.println( "Pin 5 triggered (FALLING)" ) ;
    ul_Interrupt_Pin5 = 0 ;
  }

  if ( ul_Interrupt_Pin6 == 1 )
  {
    Serial5.println( "Pin 6 triggered (RISING)" ) ;
    ul_Interrupt_Pin6 = 0 ;
  }

  if ( ul_Interrupt_Pin7 == 1 )
  {
    Serial5.println( "Pin 7 triggered (CHANGE)" ) ;
    ul_Interrupt_Pin7 = 0 ;
  }
#endif
}

// Ext Int Callback in LOW mode
static void Interrupt_Pin3( void )
{
  ul_Interrupt_Pin3 = 1 ;
}

// Ext Int Callback in HIGH mode
static void Interrupt_Pin4( void )
{
  ul_Interrupt_Pin4 = 1 ;
}

// Ext Int Callback in CHANGE mode
static void Interrupt_Pin5( void )
{
  ul_Interrupt_Pin5 = 1 ;
}

// Ext Int Callback in FALLING mode
static void Interrupt_Pin6( void )
{
  ul_Interrupt_Pin6 = 1 ;
}

// Ext Int Callback in RISING mode
static void Interrupt_Pin7( void )
{
  ul_Interrupt_Pin7 = 1 ;
}
