# skeleton_info
skeleton_info is a ros package used for accessing skeletal tracking information from the skeleton_tracker ros package. the skeleton_tracker package tracks and recognizes users and the information about the recognized users is provided in the form of an array of skeleton objects or joints present in the tf, which is a package that lets the user keep track of multiple coordinate frames over time.

## Install
We need to install the following ros package. and

`{ros-indigo-openni2-camera, ros-indigo-openni2-launch}`

## Build
Before starting building skeleton_info you need to succefully build skeleton_tracker package first to do so please follow [this link](https://gist.github.com/Wessi/85afb5d78b53a38e304e)
To build skeleton_info as ros package, perform the following steps at the shell prompt(in the terminal)

`cd ~/catkin_ws/src/`

`git clone https://github.com/Wessi/skeleton_info.git`

`cd ../`

`catkin_make`

`source ./devel/setup.bash`

## Run
--- First start the skeleton_tracker as descrebed in the link above and then execute the following command in the terminal.
```
rosrun skeleton_info users.py    #For bundling users
rosrun skeleton_info users_tf_info.py      #For bundling each user's transforms
```
