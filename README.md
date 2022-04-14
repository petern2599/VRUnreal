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
2.)Make sure to insert the WSL 2 application's IP address in the address parameter for the ROSIntegration gameinstance <br/>
3.)Run any ROS scripts<br/>
4.)Insert C++ actors that intialize rostopic and messages at the start of runtime into environment in UE4<br/>
5.)Press play in UE4<br/>

## FAQ:
### I am experiencing large FPS drops in my project.
If you are experiencing large FPS drops, make sure that you are connected to ROSBridge before pressing play. The reason for doing this is because in UE4 it is constantly attempting to connect to a ROSBridge server. If it does not connect it will continually spawn ROSIntegrationCore, then destroys it afterwards due to it not connecting.<br/>
Another reason for experiencing FPS drops is because the frame rate being conjested with handling the "game logic". Always try to AVOID using the NativeTick function like shown in the ROSIntegration documentation. Instead use the "Timer Handle" node in Unreal Engine 4 to specify the time intervals to run a function with ROSIntegration.
### I am not seeing the ROS topic when I echo it out in Linux terminal.
If you do not see the ROS topic, make sure to check the following:<br/>
1.) Check if you can compile and build your C++ actors from UE4<br/>
2.) ROSrun you scripts from your workspace<br/>
3.) Use ROS info [/topic] to check if the correct nodes are used<br/>
3.) Check if you roslaunch the rosbridge_server like shown from ROSIntegration repository and README.md file in this repository<br/>
If you still not seeing the ROS topic, close Unreal Engine 4 and open it up again and try to publish/subscribe again.
### Will ROSIntegration work for nodes when its actor is spawned during runtime?
If the actor spawns during runtime and the rostopic/rosnode is initialize during BeginPlay, it WILL NOT WORK. BeginPlay refers to the first frame of runtime, so if you spawn in actor after the first frame to initialize rostopic/rosnode it will not execute the functions. Thus in order to run the functions that intialize the rostopic/rosnode and publish/subscribe to the ROS messages, you need to create C++ functions that have some custom event in Unreal Engine 4 to execute it during runtime.
### ROSIntegration is not loading correctly, causing my UE4 project to not load.
If this happens to you, delete the plugin from your project and reinstall/re-clone the ROSIntegration repository from 'code-iai' again, then rebuild your project. This solution has worked for me.
### Can I clone or download your UE4 project from here?
Unfortunately I am unable to upload the project files in Github as it exceeds the allowable file size to upload. Even with Github LFS (if I configured it correctly...), I was unable to upload it.
