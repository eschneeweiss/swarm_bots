## Time for Arduino

### What we need
* Arduino board, for example https://www.pololu.com/product/3117
* micro USB cable
* know nothing? watch some youtube videos like https://www.youtube.com/watch?v=nL34zDTPkcs

### Just read instructions
* for my board, https://www.pololu.com/docs/0J63/5.2, or https://www.pololu.com/product/3117/resources

### How to run the demos (on windows)
* Arduino IDE  
download Arduino IDE from https://www.arduino.cc/en/Main/Software
* (optional driver)  
install driver, for my board, in https://www.pololu.com/product/3117/resources
* Additional Boards Manager URLs  
open Arduino IDE, in Preferences dialog, find the "Additional Boards Manager URLs", paste `https://files.pololu.com/arduino/package_pololu_index.json`. (This is in instruction)
* install board profile  
in Tools > Board menu, select "Boards Manager…", find "Pololu A-Star", install it.
* select board  
now in Tools > Board you should see **"Pololu A-Star 32U4"**, select it.
* select port  
in Tools > Port, select the **port** you have, if you don't have any, check the driver.
* install library  
for my board there is a special library, https://github.com/pololu/a-star-32u4-arduino-library, install it by open the Sketch > Include Library > Manage Libraries, Search for "AStar32U4", find it and install it.
* find demos  
then you can run the **demos**! demos are under file > examples > examples from custom libraries, other examples might be unable to run.
* run demos  
connect the board to the computer, turn it on by the switch, in Arduino IDE, open a demo, click Verify and click upload, you will see **Done uploading** is succeed. and Arduino should be running your demo automatically, no buttons needed.