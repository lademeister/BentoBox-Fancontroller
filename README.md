# BentoBox-Fancontroller
BentoBox FanController software for HEPA/activated charcoal modular 3D Printer Air purifier
With automatic fan speed controlm OLED display, menu button for settings and over-the air update over wifi.

Background: I did not like the manual power switches of the BentoBox designs out there, and I also did not like visible cables in the printer.
This code is part of my project "J1 BentoBase" for Snapmaker J1 printers.

It can be used for any, but is designed for BentoBox setups and particularly for the Snapmaker J1 BentoBase that features an integrated 128x32 OLED display and a mode button.
It is designed to control two fans with 4pin-connector that support PWM speed control and that output a tacho signal. BentoBox designs often use 4028 fans, but the code is not limited to this type.
The code has a preset-value for maximum fan speed, on which it calculates the bargraph height for fanspeed display. 
Because different fans may be used, the code features an auto-learning mechanism. The highest achieved fan speed is automatically saved as reference, all you need to do is run the fans at 100% once, and the code will adapt the display max value to your individual fan(s).

The code is intended to be used with Wemos ESP32-S2 mini V1.0.0, regarding used pins and memory allocation to save the values during power loss. The onboard button of that microcontroller board is used as menu button, and is directly actuated in the Snapmaker J1 BentoBase, where the board can directly be slid in.
If you want a different setup or microcontroller, you may need to change the used pins and if not using ESP32 microcontroller family, you may need to change how the values are saved.
Also, the code provides wifi setup and over the air-update capability, e.g. flashing directly over WiFi from Arduino IDE.
If you use other microcontrollers, they may not have Wifi capability at all.
During first setup, the device eill open a wifi access point for wifi setup. You can connect to that wifi and choose a network from a list of found networks. If you enter a WiFi password, this data will be saved to the microcontroller only.
The code does not connect to the internet, nor does it transfer any data to remote servers. Particularly, your wifi password is only stored locally on the chip and cannot be accessed from external.
MQTT control is not yet implemented but can be easily added, e.g. if you want to influence fan speed or read out information like chamber temperature of the BentoBox fan controller from your home automation system.
Fan start and fan stop are controlled by an input pin; in the Snapmaker J1 BentoBase a optical infrared sensor is used to detect bed homing position; you could of course also use a microswitch.
Although possible, I suggest not to use a on/off switch - the codes intention is to have an automatic fan controller that does not need user interaction. As the code will save the last mode setting and fan speed setting and keep it available during power loss, the fans would start with the next print if they were on or stay off if they were switched off before.
As the code also allows switching off the fans while still keeping track of the last fan speed setting, you do not need to set the fan speed again.
From OFF to AUTO mode is just one button press, so there is literally no need for bulky power switches.
Due to auto-off there should be no need for interaction at all, once you have found your desired setting.


Display Features: 
Fan speed in RPM and as bargraphs for two fans, temperature and humidity, and a menu for settings.

On startup, the OLED will display some different screens, e.g. if and which sensors were found, what settings were loaded from memory and what the current IP address of the device is.
After that, the device will enter the standard screen. Pressing the mode button toggles between modes AUTO, SET and OFF. 
In mode AUTO and mode SET a long press will enter programming mode that allows changing the desired fan target speed.
To indicate that programming is enabled, "SET" will flash. Using short presses, you now can toggle in 10% steps through fan target speeds. The fan speed will be live adjusted so that you can determine air flow and noise and choose your desired value Longpress or doing nothing for a couple of seconds will save the value and exit to auto mode.

In mode AUTO, the fans may be off if you are not currently printing, therefore in mode SET the fans will start to run at the currently set fan speed. In mode OFF the fans stey off all time, no matter if you print or not.
The last state (AUTO or OFF) is saved and kept in memory, even if you completely unpower the board.

Menu on OLED display: Modes AUTO, SET and OFF
  mode "AUTO":  Fan speed is displayed as live data in 1/min (RPM) and as bargraphs per individual fan
                Fans switch on with the programmed speed when the print starts and switch off when the print has finished.
                Additionally, a post-print purifying phase is dynamically calculated, based on printing time. This is intended for use with enclosed printers.
                The post-printing air purifying phase can be adjusted in different setpoints within the code, to create not only different post-purifying times (e.g. 1 min for a 5min print and 1 hour for a 10 hr                  print) but also allow more setpoints to create a non-linear result in post-purifying time.
                Additionally a factor allows to set a different fan speed during post print purifying phase in comparison to the programmed fan speed value during printing time.
                This factor can be below 1.0, exactly 1.0 or above 1.0 and the custom set fan speed during printing will be multiplied with that factor to calclulate post-printing fan speed.
                Examples:
                Fan speed during printing: 50%, postPrint_purifying_fanSpeed_factor = 0.78 --> 50% * 0.78 = 39% fan speed in post-print purifying.
                Fan speed during printing: 20%, postPrint_purifying_fanSpeed_factor = 4.5 --> 20% * 4.5 = 90% fan speed in post print purifying
                Fan speed during printing: 90%, postPrint_purifying_fanSpeed_factor = 1.0 --> 90% * 1.0 = 90% fan speed in post print purifying (identical, unchanged)
                Fan speed during printing: 60%, postPrint_purifying_fanSpeed_factor = 0.0 --> 60% * 0.0 = 0% fan speed, post print purifying = off
                
  mode "set": adjustment of the desired fan speed during printing time (will auto-save the setting to internal memory of microcontroller)
              while adjusting, the target value is shown as a line in the displayed fan speed bargraphs, until the fans reach the target speed
  mode "off": fans stay off, and sensor state of bed position sensor is displayed

Additionally,  in the lower area of the display the current temperature and humidity is displayed. Two sensors are supported without changes: BME280 I2C sensor and DHT22.
The Snapmaker J1 BentoBase has a slot for BME280 sensor, and the DHT22 sensor can be optionally added in the fan module between the fans.
The code automatically detects the presence of one or both sensor types and in case both sensors are connected, displays the higher value for temperature and humidity.

The fans Auto start/stop with print, detecting home position e.g. of print bed.
Of course it is technically possible for the microcontroller to detect and read out Snapmaker J1 printing state and bed/nozzle temperatures over wifi, but i wanted the code to be usable for other or similar applications as well, e.g. in other printer types, especially keeping in mind the wide spread use of BentoBox designs (but also other filtration systems).

WiFi is currently used for Software-Update over the Air (OTA) capability from e.g. Arduino IDE but is forseen to be enriched witch MQTT (hopefully by the community as i am running out of free time).
The code also features a nice progress bar and progress percentage display during OTA update on the OLED, while still controlling the fans in background during the update data upload.
I am well aware that the implemented 128x32 OLED display is smalland a vertical orientation is not typical, but I wanted to have nice high bargraphs for the fan speed, and I wanted to integrate the display into my BentoBase design for Snapmaker J1, while not blocking too much of the air outlet size.

--- how to get started ---
Download this repository and install Arduino IDE if you don't have it.
To add the ESP32 board package to the Arduino IDE, go to File > Preferences, paste https://dl.espressif.com/dl/package_esp32_index.json into the Additional Board Manager URLs field, and click OK.
If you are havon difficulties, use Google to get instructions on how to add ESP32 chip family to Arduino IDE.

Then open the .ino file in Arduino IDE, go to Tools/Board/ESP32 Arduino/ and select LOLIN S2 mini or alternatively ESP32 S2 Dev Module.
For the first upload to a bare chip you need to connect it via USB-C to your computer.
After connecting the microcontroller via USB you should find the port of your board in Tools/Port/
Select it and go to Sketch/Upload (or use CTRL+U) to compile and upload the code.

After first upload and setting up your wifi credentials using the configurartion site (the captive portal opens up automatically when connecting a computer or mobile phone to the WiFi hotspot that the chip opens up), any future code updates can be made using over the air-update (OTA) directly from arduino IDE. For that, you just change the port from the USB connection to the device that will show up in the port selection menu. Usually, Arduino IDE needs to be restarted to find the device.


--- what you need from hardware point of view ---
If you want to use the code without changes, use a Wemos ESP32-S2 mini V1.0.0 microcontroller.

Use 4pin fans that have PWM speed control and a tacho signal output.
Typical and cheap fans for BentoBox are 4028 fans with 4pin connector. The 4pin variants are usually available as 12V version, not 24V.
Also, those fans usually output 5V on the tacho pin, which is not compatible with ESP32 microcontrollers (3.3V GPIO input pin maximum voltage)

To be able to use those widely available 12V fans, you will need one or two buck converters:
The firstbuck converter that you need is 5V for the Wemos ESP32-S2 mini board (connect +5V to the pin labeled VBUS (this is the 5V input pin)
All ESP32 can run from 3.3V, when powering them using the 3.3V pin - but as we need a 5V reference voltage later, use the 5V input pin and set the buck converter to 5V. 

If your printer has a 12V output on the power supply or if you use an external 12V power supply, connect the fans to that 12V supply.
If you want to connect the fans to a printer that has 24V power supply, jou need a buck converter (step-down-module) to get down to 12V.
Set this buck converter to 12V for the fan(s). Connect +12V output and GND to the fans.

For the issue with the 5V tacho signal output and simoultanously to solve signal detection problems for the tacho signals, I advise and strongly suggest to use a level converter board (usually below 1€/1$) instead of trying to build an RC circuit (resistor-capacitro circuit) on your own, this is way faster and just as reliable.
Do NOT connect the tacho signal of the fans directly and without a level converter to ESP32. It would fry the chip relatively fast.

To connect the level converter, it needs two voltage references: low voltage and high voltage. The voltage that you apply on those two pins of the level converter msut be 3.3V for LOW voltage and 5V for HIGH voltage. Those pins are usually labeled with LV and HV. 
For the 3.3V voltage reference you can directly connect LV to the 3.3V pin of the Wemos board. 
For the 5V voltage reference you can directly connect HV to the 35V pin of the Wemos board (labeled VBUS on the board). 
Basically the level converter board is in between ESP32 and the fan, particularly for the Tacho signal pins. 
It usually works just fine to directly connect the output pins of ESP32 (the two pins that are used for fan speed control) to the fans directly. 
But tacho signal needs to be sent through the level converter.





