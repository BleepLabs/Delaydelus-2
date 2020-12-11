### The new sampling delay instrument.  
Limited edition device avaiale in mid 2019  
https://bleeplabs.com/product/delaydelus-2-preorder/   
Can playback 4 stereo samples at once with full speed control using the included aditions to the teensy audio library.  
Based onthe teensy 3.6. Requires my branch of the audio library https://github.com/BleepLabs/Audio. 
  
### Version 8 update  
Adds the ability to copy recodrings from the device to the sd card <br>

To upload this code to your delaydelus you'll need to install the [arduino IDE and teensyduino](https://github.com/BleepLabs/Dadageek-August20/wiki/Arduino-software-first-steps).  Once you have them installed just open teensyduino. I'll look like this.   
![](https://github.com/BleepLabs/Dadageek-August20/raw/master/images/teensyduino%20window.jpg?raw=true)  
  
You'll need a 2mm allen wrench to remove the three screws on the bottom and a 1/16" allen for the four on the top in the corners top corners. Don't remove the two that are in the middle surrounded by the banana plug hotels on the top.  
Now you'll see the usb micro port inside. 
  
Click the button in the top left of select file>open HEX and give it [this hex file](https://github.com/BleepLabs/Delaydelus-2/blob/master/delaydelus_2_poduction_8.ino.hex).  
Now press the tiny white button shown here.    
![](https://raw.githubusercontent.com/BleepLabs/Dadageek-August20/master/images/teensy%20button.jpg)   
teensyduino will say uploading and then you're done.   
   
   
To copy the files to the SD hold mode and the large right button while it turns on. Keep holding them for about 3 seconds until the light turns dim blue. 
Then it will take a bit to copy the files. Once it turns orange it's all done and you can remove the card. 
Every time you do this it will make a new folder called userX where X is the next number and fill it with 10 wav files from each of the banks.
The samples inside the device will not be affected at all UNLESS you hold sample and mode, that will copy over them from the SD card. So make sure it's mode and the large right button you're holding.    
  
