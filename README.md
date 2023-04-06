# Intent of the repository : 

1. To be a container for and workspace for various packages that will be created for ENPM661 ENPM673 semester projects

# Initial Setup for Development on PC/Laptop:

1. Install vs-code editor on your ubuntu 20.04 host OS : https://linuxize.com/post/how-to-install-visual-studio-code-on-ubuntu-20-04/
2. Install docker on your ubuntu 20.04 host OS : https://docs.docker.com/engine/install/ubuntu/
3. Please remember to follow all the steps in docker installation, and verifying it's successful installation @ https://docs.docker.com/engine/install/ubuntu/#next-steps > https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user (Step 1 2 3 4)
4. Install following extensions on your VSCode: 
    1. Docker : https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker
    2. Remote-Containers :  https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers
5. Clone this repository using:
    > ``` git clone --recurse-submodules https://github.com/vedran97/plan-percep-ws.git ```
6. From the current working directory, cd to plan-percep-ws repo which u just cloned
7. Open a terminal in plan-percep-ws and type 
    > ``` code . ```
8. A VSCode popup should arise which says "Open folder in Container" where you choose Yes OR press F1,search for "Open Folder in Container" and execute the command
9. Now wait for the build process to finish, once it's completed, you have a fully functional ROS1 workspace with example packages
11. When the container is built for the first time, a error will popup saying "Failed to enable ROS Extension" , just choose the reload window option
10. Set up ROS dependencies using the following section

# VS Code tasks:

0. How to use this? Ans: Open this directory as a container(or not) and press "Ctrl+Shift+B" to get a drop down list of available tasks.<br>Like this: <br> ![Tasks](./docs/tasks.png)
1. "ROS:Build a Package" task -> This task lets you build a selected few, or all packages. CMake parrallelism limits have been added to prevent crashing during builds.
    1. Choose a build config:<br>![Build-Config](./docs/buildmode.png)
    2. Choose a package name:<br>![Package-Name](./docs/pkgname.png)
<br>choosing no package name, will build all packages in this directory
2. "ROS:Install ROS dependencies" task -> This task lets you install ros-deps which are mentioned in any package's package.xml file, which is included in this workspace.
3. "ROS: Clean" task -> This task cleans the workspace off all build artifacts

# Setting up ROS-Dependencies :

1. Run the VSCode task "ROS:Install ROS dependencies"

# Initial Setup for Development on Raspberry PI:

1. Setup github ssh on RPI.
2. Clone this repository using:
    > ``` git clone --recurse-submodules https://github.com/vedran97/plan-percep-ws.git ```
3. > ```cd plan-percep-ws```
4. > ```sudo bash ./robot-scripts/install-ros.bash $(pwd) ```
5. > ```sudo  bash ./robot-scripts/dep.bash  ```

# Generating custom msgs

1. Add the message file in terpbot_msgs/msg ,edit the CMAKELISTS.txt of this package accordingly
2. Build
3. Source
4. execute this : ```rosrun rosserial_client make_libraries ./ terpbot_msgs```
5. This builds headers for all the message types which are recognized by rosmsg command, and all these files are generated in ./ros_lib folder,relative to this workspace
6. Copy the newly generated terpbot_msgs directory from ./ros_lib, and paste it in Arduino IDE's rosserial library install folder/src/

# Command to send Gains:
 All ITRS are for left motor as of now
1. ITR1
```rostopic pub -r 10 /GAINS terpbot_msgs/Gains '{kp: 20.0, ki: 0.0, kd: 0.0, i_clamp: 0.0, isleft: true}' ```
2. ITR2
```rostopic pub -r 10 /GAINS terpbot_msgs/Gains '{kp: 30.0, ki: 0.0, kd: 0.0, i_clamp: 0.0, isleft: true}' ```
3. ITR3 works decent under no load
```rostopic pub -r 10 /GAINS terpbot_msgs/Gains '{kp: 30.0, ki: 0.0, kd: 0.0, i_clamp: 0.0, isleft: true}' ```
4. ITR
```rostopic pub -r 10 /GAINS terpbot_msgs/Gains '{kp: 32.5, ki: 0.0, kd: 0.0, i_clamp: 0.0, isleft: true}' ```

```rostopic pub -r 10 /GAINS terpbot_msgs/Gains '{kp: 50.0, ki: 0.01, kd: 0.0, i_clamp: 100.0, isleft: true}' ```

```rostopic pub -r 10 /GAINS terpbot_msgs/Gains '{kp: 30.0, ki: 0.05, kd: 0.1, i_clamp: 100.0, isleft: false}' ```
