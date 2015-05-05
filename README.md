# michaeldreher42-bluecontroller
Arduino support and ISP programmer for BlueController ATmega board BCA8-BTM
BlueController is an ATmega board using an ATMega328P or ATmega88PA. Instead of a USB serial port, it uses a BTM-222 Bluetooth module which makes it similar to an Arduino-BT.

For more information please read optiboot/doc/HowToUseBlueControllerWithArduino.pdf in this repository.

Features of this project
 * Integration of Bluecontroller in the Arduino environment
  * LED uses PB6 pin instead of PB5
  * Example sketch to set up the BTM-222 ("Setup_BlueController" in examples/)
  * added device definition and bootloaders so the bootloader can directly be flashed from Arduino IDE
 * ISP programmer with optimized communication speed and adapted avrdude (bluecontroller/examples/BlueC_ISP and bluecontroller/avrdude/avrdude-5.11.bt)
 * Adapted optiboot bootloader
  * Enter bootloader by pressing on-board button during reset (long timeout of 35 seconds)
  * Enter bootloader by writing a magic word to RAMSTART from the running sketch (long timeout of 35 seconds). This makes a remote update possible without touching any button.
  * further code size savings to make space for the additional features
  * the Bluetooth module send a "CONNECT" string which must be ignored by the bootloader (depending on the settings of the BT module)
  * set LED to half-bright when entering bootloader and switch it off when exiting bootloader

