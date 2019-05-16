# Install Ubuntu on Raspberry Pi and run OpenCV face detection
### Things we need before start
1. Raspberry Pi 3 b+
2. pi camera v2
3. SD (tf) card and its adapter
4. before control pi by VNC, you will need an HDMI screen and mouse & keyboard
5. micro USB cable (like the old ones on Android), ethernet cable

### Install Ubuntu Mate
Using etcher or Rufus etc. to flash img file into SD card.  
I used etcher: https://www.balena.io/etcher/  
Using ubuntu mate: https://ubuntu-mate.org/raspberry-pi/  
followed https://zhuanlan.zhihu.com/p/64681595?edition=yidianzixun&utm_source=yidianzixun  

### (optional) open function or change settings with raspi-config
* Resolution
* Camera
* I2C
```
(In terminal ctrl alt + T)
raspi-config
```
### Install VNC server
```
(optional)
sudo apt install openssh-server
sudo apt install sshguard
```
chose realVNC: https://www.realvnc.com/en/connect/download/vnc/raspberrypi/
```
use dpkg to install: dpkg -i blah.deb
(make you VNC start at boot)
systemctl enable vncserver-x11-serviced.service
```
**trouble shooting (for windows)** 
* For any connection, three ip address should be identical:  
    - **ubuntu ip address provide by `ifconfig` (ipv4: eth0 inet)**, for example 169.254.118.32, and physical address b8:27:eb:09:e0:d5.
    - this id address should also be shown on **VNC server GUI**
    - In windows cmd, `apr -a` will show some ipv4 address, one of them should be 169.254.118.32 for example, and physical address be b8:27:eb:09:e0:d5
* tests passed under several conditions, ubuntu mate as server and windows as the viewer, both signed in.
    - **when EVERYTHING GOES WRONG**, try to back to this setting: windows turn off WLAN sharing, ubuntu set ethernet connnetion ipv4 to be **Manual**, ip addresses be **fixed**, 169.254.19.19 for example, netmusk be 255.255.255.0, ipv6 be Link local only; on windows side, set ipv4 be automatic, then restart everything.
    - without WLAN sharing: windows turn off WLAN sharing, ubuntu set ethernet connnetion ipv4 to be **Link local only**, three ip addresses are identical 
    - with WLAN sharing: windows turn on WLAN sharing, ubuntu set ethernet connnetion ipv4 to be **Automatic DHCP**, three ip addresses are identical, and in windows `apr -a`, there are two ip addresses with a same physical address, the second one will be b8:27:eb:09:e0:d5 for example. (This method seems failed under Xfinity wifi)
    - things to try: disconnect wired connection; restart VNC by `systemctl (stop|start) vncserver-x11-serviced.service`

* If you turn off the VNC server, and it doesn’t self-starting again, just run `systemctl enable vncserver-x11-serviced.service` again
```
(other commands)
systemctl (enable|disable|start|stop) vncserver-x11-serviced.service
```

### Install packages for face detection
```
sudo apt-get update
sudo apt-get install libraspberrypi-dev
sudo apt install python3-pip
pip3 install picamera
sudo apt-get install python3-opencv
```
**optional packages**
```
sudo apt-get install bluefish
sudo apt install vim
sudo apt install git
```


## KNOWN ISSUES
* under voltage detected
Tried sudo apt-get update && sudo apt-get dist-upgrade -y 
on: https://www.element14.com/community/thread/64029/l/under-voltage-detection-raspberry-pi-3-b?displayFullThread=true
Doesn't work
* cannot turn on onboard (on screen keyborad)