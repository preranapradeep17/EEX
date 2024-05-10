#include <Servo.h>

#include <Servo.h>

#include <Servo.h>

/* prog_lec_3_1_1_uart_tx.ino
 * PICO_DEFAULT_UART_BAUD_RATE 115200 
  
  Make sure that  Raspberry Pi Pico/RP2040 package is chosen and not the Mbed OS package

  HW connections: Connect this Rx board's pin 2 (GPIO 1, RX0) to 
  the Tx board's Pin no. 1 (GPIO 0, TX0), note that UART0 of both the boards
  are used to communicate the commands from Tx board to Rx board.
  Tx board receives the command from the user on Serial (USB) port and 
  passes to the Rx board through UART0. The Rx board uses uart_getc() SDK function
  to recieve the commands from the UART0 device.
  Caution: Though only Tx board is sending data to Rx board, make sure
  that both Tx and Rx pins are cross connected between Tx and Rx boards.
  Connect the GND (pin no. 3) of both the boards connected too.
  
 It blinks the built-in LED of the RX board connected through UART0
 The program runs on Rx board. It receives commands from tx board (USB Serial interface)
 It receives the following commands (single letter) on UART0 from the TX board
 The command controls the blinking rate of its built-in LED.
 Commands: 'o' or 'O': ON, 'f' or 'F': OFF, 
 from 1 to 9 -> blink at 100 to 900 msecs
 Ref UART files:
 C:\Users\Mouli\Documents\ArduinoData\packages\rp2040\hardware\rp2040\1.4.0\pico-sdk\src\rp2_common\hardware_uart\include\hardware

*/
#include <stdio.h>
//#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

bool LED_ON_flag = false;    // Default keep it blinking at default rate 1 sec

static int delayVal = 1000; // Default it is set to 1 second

#define CR_VAL 10

// The command coming in on Serial 0 from the serial monitor of this board
// to be sent on Serial 1 to the other board to control the built-in LED on the 
// other board
int incomingCMD = 0; 
int ledCMD = 0;       // It has the command from the other board to control this LED

// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(115200);    // USB Serial port
  uart_init(uart0, 115200);
  
  // Set the GPIO pin mux to the UART0 - 0 is TX0, 1 is RX0
  gpio_set_function(0, GPIO_FUNC_UART);
  gpio_set_function(1, GPIO_FUNC_UART);
  
  // Blink this LED based on the command from the other board
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, HIGH);
}

// the loop function runs over and over again forever
void loop() {
  if(LED_ON_flag)
    digitalWrite(LED_BUILTIN, HIGH); // keep it always ON without any blinking
  else if (delayVal != 0){
    digitalWrite(LED_BUILTIN, HIGH);   
    delay(delayVal);
    digitalWrite(LED_BUILTIN, LOW);
    delay(delayVal);
  } else 
    digitalWrite(LED_BUILTIN, LOW); // keep it always OFF without any blinking

  char rx_char; // received through Serial USB port
  // Check if there is any command from the Serial Monitor sent by the user
  // Receive it and send it across to the other board through Serial 1
  if(Serial.available() > 0) {
    // read the incoming command byte:
    incomingCMD = Serial.read();

    // say what you got:
    Serial.print("Serial: Received the command: ");
    Serial.println(incomingCMD);
    // if it is not a carriage return then send it across to the other board 
    // through uart 0
    if(incomingCMD != CR_VAL)
      uart_putc(uart0, incomingCMD);  // send it on uart0 to other board
  }
  
} // end of loop()
