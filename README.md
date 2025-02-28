# Autonomous Drone Dvelopment Challenge 2025
This repository belongs to team aeroKLE . It includes the  proposed approach used for the 3rd SAEISS ADDC (Autonomous Drone Development Challenge) 2025 

# Setting up Raspberry Pi 4 model B
1.Using 'Rasbperry Pi Imager', install Raspberry Pi OS compatible with the Raspberry Pi 4 (Recommended: Raspberry Pi OS (Debian Bullseye) Legacy 32-Bit Full with Desktop environment and recommended applications) onto the SD Card (Recommended: Class 10 32 GB Micro SD Card). </br>

2.Access the Raspberry Pi through Wi-Fi via SSH </br>

3.Set up serial connection and type the following in SSH: </br>
```sudo raspi-config``` </br>

Change the folowing settings: </br>
a) Go to interface settings </br>

b) Enable Legacy camera </br>

c) Enable SSH </br>
 
d) Enable VNC </br>

e) Go to serial </br>

f) When prompted, select no to 'Would you like a login shell to be accessible over serial?' </br>

g) When prompted, select yes to 'Would you like the serial port hardware to be enabled?'. </br>

h) Reboot the Raspberry Pi using sudo reboot when you are done. </br>

i) The Raspberry Pi’s serial port will now be usable on /dev/ttyAMA0. </br>

If you encounter Cannot currently show the Desktop, go to ```sudo nano /boot/config.txt``` and type the following lines after #hdmi_safe=1: </br>
```
    hdmi_force_hotplug=1 
    hdmi_group=2
    hdmi_mode=9
```
Save the file and exit the text editor (in ```nano```, you do this by pressing CTRL + X, then Y, and Enter). </br>

4. Rnn the following Commands </br>
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python3-pip
sudo apt-get install python-dev
sudo pip install future
sudo apt-get install screen
sudo apt-get install python-wxgtk4.*
sudo apt-get install libxml2-dev
sudo apt-get install libxslt1-dev
pip install lxml
sudo pip3 install pyserial
sudo pip3 install dronekit
sudo pip3 install geopy
sudo pip3 install MAVProxy
```
5. Install OpenCV
  ```pip install opencv-python```
Note : refer to the troubleshooting section , if any errors raised . </br>

# Integrate Raspberry Pi with Pixhawk Flight Controller
1. Set the follwing parameters in Mission Planner (Ardupilot Ground Control Station) </br>
  ```SERIAL2_PROTOCOL = 2``` </br>
 ```SERIAL2_BAUD = 921```  </br>
```LOG_BACKEND_TYPE = 3``` </br>

2. Connect Pixhawk and Raspberry Pi as shown in below figure :

![321070134-f1a79c68-ce5e-46aa-9fdd-fc60bfb1db5b](https://github.com/user-attachments/assets/efb5acdc-3b89-4daf-8a3c-0f1de88bfd5b) 

3. Power the Raspberry Pi using BEC Module </br>
a) Check Port </br>
  ```ls/dev/ttyAMAO``` </br>

b) Add the follwing two lines at bottom of the file ```sudo nano/boot/config.txt```: </br>
```
  enable_uart=1
  dtoverlay=disable-bt
```
Save the file and exit the text editor (in ```nano```, you do this by pressing CTRL + X, then Y, and Enter). </br>

4. Now type the following command to get the telemetry data of PixHawk: 
``` mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 ``` </br>

5. Type the following if you want telemetry data to be displayed in Mission Planner:
```
   mavproxy.py --master=/dev/serial0 --baudrate 921600 --out udp:127.0.0.1:14552

/*Here,
 '127.0.0.1' Your PC's IP Adress, Obtained by typing 'ipconfig' in command prompt
 '14552' is the port to which you need to connect to Mission Planner using UDP
*/
```
#Executing the Mission







