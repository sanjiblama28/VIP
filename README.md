# PC Setup using:
- Ubuntu 20.04
- ROS Foxy

## 1. Install ROS on Remote PC

Open the terminal and type the commands listed below one at a time.

```
sudo apt update
sudo apt upgrade
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
```

![image](https://user-images.githubusercontent.com/92040822/200147279-93635a26-e453-4ef7-81ec-3d65961c7964.png)

```
chmod 755 ./install_ros_noetic.sh 
$ bash ./install_ros_noetic.sh
```
![image](https://user-images.githubusercontent.com/92040822/200147308-5d60e464-2113-4098-9ad6-5bf30fee222d.png)


## 2. Install Dependent ROS Packages

Open the terminal with Ctrl+Alt+T from Remote PC.

```
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc ros-noetic-rgbd-launch ros-noetic-rosserial-arduino ros-noetic-rosserial-python ros-noetic-rosserial-client ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

![image](https://user-images.githubusercontent.com/92040822/200147456-6e07351b-d873-4a23-af36-dd7b33e2d682.png)

## 3 Install TurtleBot3 Packages

Download the source codes and build them.

```
cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws && catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

![image](https://user-images.githubusercontent.com/92040822/200147701-b75fadd2-5197-4183-b06b-9e17efbfb6d9.png)

![image](https://user-images.githubusercontent.com/92040822/200147808-b976d8f7-2505-4337-92a3-e7a4c286b708.png)

## 4. Network Configuration

Connect PC to a WiFi device and find the assigned IP address with the command below.

```
ifconfig
```

![image](https://user-images.githubusercontent.com/92040822/200147852-c903d9f7-cc37-4418-9bc9-e3f6918dc929.png)

Open the file and update the ROS IP settings with the command below.

```
nano ~/.bashrc
```

Press Ctrl+END or Alt+/ to move the cursor to the end of line.
Modify the address of localhost in the ROS_MASTER_URI and ROS_HOSTNAME with the IP address acquired from the above terminal window.

#image

Source the bashrc with below command.

```
source ~/.bashrc
```

# SBC Setup using:
- Pen drive
- TurtleBot3 SBC Image (Raspberry Pi 4B (2GB or 4GB) ROS Neotic image)
- Raspberry Pi Imager

Following the preparation of a pen drive, downloading the correct image file for my hardware and the ROS version that is neotic, unzipping the downloaded image file, extracting the.img file, saving it to the local disk and burning the image file using a Raspberry Pi Imager was done. Then the following includes:

```
Click CHOOSE OS.
Click Use custom and select the extracted .img file from local disk.
Click CHOOSE STORAGE and select the microSD.
Click WRITE to start burning the image.
```

![image](https://github.com/sanjiblama28/San/blob/main/Veed%20Recording%20-%206%20November%202022%20(1).gif)

GParted GUI utility must first be installed using the provided code.

```
sudo apt-get install gparted
```

![image](https://user-images.githubusercontent.com/92040822/200148610-46d5d475-1fe8-4e39-8df6-da0478224778.png)

Next, gparted is launched. 

```
Select microSD card from the menu (mounted location may vary by system).
Right click on the yellow partition.
Select Resize/Move option.
Drag the right edge of the partition to all the way to the right end.
Click Resize/Move button.
Click the Apply All Operations green check button at the top.
```

![image](https://github.com/sanjiblama28/San/blob/main/Veed%20Recording%20-%206%20November%202022%20(3).gif)

## Configure the WiFi Network Setting

Launch a terminal window, and navigate to the netplan directory on the microSD card and open the 50-cloud-init.yaml file and begin editing it with a superuser permission sudo.

```
cd /media/$USER/writable/etc/netplan
sudo nano 50-cloud-init.yaml
```

Replace the WIFI SSID and WIFI PASSWORD with your wifi SSID and password once the editor has been opened.

Press Ctrl+S to save the document and Ctrl+X to close it.

![image](https://github.com/sanjiblama28/San/blob/main/Veed%20Recording%20-%206%20November%202022%20(4).gif)
![image](https://github.com/sanjiblama28/Github/blob/main/20221201_030911.jpg)
![image](https://user-images.githubusercontent.com/92040822/204874933-6a3d4aa7-dc98-49bc-9071-e6c457810958.png)

## Boot Up the Raspberry Pi
a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.
b. Connect input devices to the USB port of Raspberry Pi.
c. Insert the microSD card.
d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.
e. Login with ID ubuntu and PASSWORD turtlebot.

## ROS Network Configuration

Please follow the instructions below on the SBC (Raspberry Pi).

Confirm the WiFi IP address.
```
ifconfig
```

Edit the .bashrc file.
```
nano ~/.bashrc
```

Find the ROS_MASTER_URI and ROS_HOSTNAME setting section, then modify the IP adddresses accordingly.
```
export ROS_MASTER_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311
export ROS_HOSTNAME={IP_ADDRESS_OF_RASPBERRY_PI_3}
```

Save the file with Ctrl + S and exit the nano editor with Ctrl + X.

Apply changes with the command below.
```
source ~/.bashrc
```

## NEW LDS-02 Configuration

Please follow the steps below on the TurtleBot3 SBC (Raspberry Pi).

1. Install the LDS-02 driver and update the TurtleBot3 package.

```
sudo apt update
sudo apt install libudev-dev
cd ~/catkin_ws/src
git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
cd ~/catkin_ws/src/turtlebot3 && git pull
rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
cd ~/catkin_ws && catkin_make
```

![image](https://user-images.githubusercontent.com/92040822/204878291-2f56bb73-87af-4312-a460-f5bb49b0a4c3.png)

![image](https://user-images.githubusercontent.com/92040822/204879981-fdcf8708-b207-4d3e-a49f-adec275807a8.png)

![image](https://user-images.githubusercontent.com/92040822/204987063-623a27cb-3cb1-4345-b4c5-4e0a60985e3e.png)

Now, 
2. Export the LDS MODEL to the bashrc file. LDS-01 or LDS-02, depending on your LDS model.

```
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
source ~/.bashrc
```
![image](https://user-images.githubusercontent.com/92040822/204987255-ab70dec1-7135-4448-a9c7-7a7fd64bd49a.png)

# OpenCR Setup

1. Connect the OpenCR to the Rasbperry Pi using the micro USB cable.
2. Install required packages on the Raspberry Pi to upload the OpenCR firmware.

```
sudo dpkg --add-architecture armhf
sudo apt update
sudo apt install libc6:armhf 
```

![image](https://user-images.githubusercontent.com/92040822/204988172-e33c0099-0d12-4723-a9ba-ecddeb6efbb6.png)

3. Depending on the platform, use either burger or waffle for the OPENCR_MODEL name.

```
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
rm -rf ./opencr_update.tar.bz2
```

![image](https://user-images.githubusercontent.com/92040822/204989231-5613bc9d-d103-4406-a464-6ec586d6ce74.png)

4. Download the firmware and loader, then extract the file.

```
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2 
tar -xvf ./opencr_update.tar.bz2
```

![image](https://user-images.githubusercontent.com/92040822/204989421-5b95e2be-012a-4d2d-a585-8e5998686761.png)

5. Upload firmware to the OpenCR.

```
cd ~/opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```

![image](https://user-images.githubusercontent.com/92040822/204989511-07ced6dd-7abe-431b-8127-651a0c2ef6c3.png)


6. OpenCR Test

We perform the OpenCR Test by following the below steps:
--> PUSH SW1 / PUSH SW2 were used to check if the robot has been properly assembeled or not and this process tests the left and right DYNAMIXEL's and the OpenCR board.
--> After assembling TurtleBot3, we connected the power to OpenCR and turn on the power switch of OpenCR. The red `power LED` will be turned on. 
--> And then the robot was placed in a flat ground in an open area.
--> After that, `PUSH SW 1` button was pushed for a few seconds to move the robot forward around 30 centimeters as recommended. 
--> And `PUSH SW 2` for a few seconds to command the robot to rotate 180 degrees in place.

# Bringuo

## Run roscore

Run roscore from PC.

```
roscore
```

# Bringup TurtleBot3

TIP: Before executing this command, you have to specify the model name of TurtleBot3. The ${TB3_MODEL} is the name of the model you are using in burger, waffle, waffle_pi.

Open a new terminal from PC with Ctrl + Alt + T and connect to Raspberry Pi with its IP address.
The default password is turtlebot.

```
ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```

Bring up basic packages to start TurtleBot3 applications.

```
export TURTLEBOT3_MODEL=${TB3_MODEL}
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

# Simulation of Obstacle Detection

## Launch Simulation World

In this instruction, TurtleBot3 World will be used.
Please use the proper keyword among burger, waffle, waffle_pi for the TURTLEBOT3_MODEL parameter.

```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

## Run SLAM Node

Open a new terminal from Remote PC with Ctrl + Alt + T and run the SLAM node. Gmapping SLAM method is used by default.
Please use the proper keyword among burger, waffle, waffle_pi for the TURTLEBOT3_MODEL parameter.

```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

## Run Teleoperation Node

Open a new terminal from Remote PC with Ctrl + Alt + T and run the teleoperation node from the Remote PC.
Please use the proper keyword among burger, waffle, waffle_pi for the TURTLEBOT3_MODEL parameter.

```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit
 ```
 
 ## Run Obstacle launch
 
 ```
 export TURTLEBOT3_MODEL=burger
 roslaunch turtlebot3_example turtlebot3_obstacle.launch
 ```
 
 # Simulation of Pick-up and Drop
 
## Run Gazebo

[Remote PC] Load TurtleBot3 with OpenMANIPULATOR-X into Gazebo world with the following command.

```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
```

## Run move_group Node

[Remote PC] In order to use Moveit feature, launch move_group node. If you press [▶] button in Gazebo to start simulation, use the following command.
With a successful launch, “You can start planning now!” message will be printed on the terminal.

```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```

## Run RViz

[Remote PC] Use Moveit feature in RViz by reading moveit.rviz file where Moveit enviroment data is configured.
You can control the mounted manipulator using an interactive marker, and simulate the motion of goal position, which helps preventing a possible physical contact by simulating the motion in advance.

```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
```

## Run ROBOTIS GUI Controller

[Remote PC] You can also use ROBOTIS GUI to control the OpenMANIPULATOR-X in Gazebo. The GUI supports Task Space and Joint Space controls. Use any control methods you prefer.

Task Space Control: Control based on the valid gripping position (represented as a small red cube between the grippers) of the end-effector of the OpenMANIPULATOR-X.
Joint Space Control: Control based on each joint angle.

```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```










 
 
 
 
 
 




