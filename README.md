# UnrealVR_ROS
Contains files that use ROS Integration with Unreal Engine 4 and ROS Melodic. For this repository, the files are used to integrate ROS capabalities alongside with UE4 to create a virtual reality interface to conduct unmanned air traffic management (UTM) for unmanned aerial systems (UAS). The project uses the Airsim plugin with ROSIntegration.<br/>

Access files from "development" branch

## Setting Up and Using ROS Integration
To setup ROS Integration follow the setup steps from https://github.com/code-iai/ROSIntegration

### IMPORTANT NOTE:
However, do not install ROSBridge with their method. Instead, do the following...<br/>

$cd ~/"your workspace name"/src<br/>
$git clone -b 0.9.0 https://github.com/RobotWebTools/rosbridge_suite.git<br/>
$cd ..<br/>
$catkin build<br/>
$source ./devel/setup.bash<br/>

The reason for this is because ROS Integration is not compatible with ROSBridge's new transmission method. Which is why ROSBridge is downgraded to 0.9.0, which is compatible with the current version of ROSIntegration.

##Using ROSIntegration with UE4
When using ROSIntegration, the sequence is the following:

1.)Initialize ROSBridge server in bson_only_mode<br/>
2.)Run any ROS scripts<br/>
3.)Insert C++ actors that is used for ROS into environment in UE4<br/>
4.)Press play in UE4<br/>

## FAQ:
### I am experiencing large FPS drops in my project.
If you are experiencing large FPS drops, make sure that you are connected to ROSBridge before pressing play. The reason for doing this is because in UE4 it is constantly attempting to connect to a ROSBridge server. If it does not connect it will continually spawn ROSIntegrationCore, then destroys it afterwards due to it not connecting.
### I am not seeing the ROS topic when I echo it out in Linux terminal.
If you do not see the ROS topic, make sure to check the following:<br/>
1.) Check if you can compile and build your C++ actors from UE4<br/>
2.) ROSrun you scripts from your workspace<br/>
3.) Use ROS info [/topic] to check if the correct nodes are used<br/>
### Will ROSIntegration work for nodes when its actor is spawned during runtime?
If the actor spawns during runtime and the rostopic/rosnode is initialize during BeginPlay, it WILL NOT WORK. For any publishing/subscribing actors that are going to be used, the actors must be spawned before running project in UE4. Unless it initializes the rostopic/rosnode during runtime. 
### ROSIntegration is not loading correctly, causing my UE4 project to not load.
If this happens to you, delete the plugin from your project and reinstall/re-clone the ROSIntegration repository from 'code-iai' again, then rebuild your project. This solution has worked for me.
