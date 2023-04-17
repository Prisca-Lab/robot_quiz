# Code Annex of Sweet Robot O’Mine

This repository contains the code developed for replicating the experiment described in the paper "Sweet Robot O’Mine - How a Cheerful Robot Boosts Users' Performance in a Game Scenario".

The submodules `hri_msgs`, `pal_msgs` and `play_motion` are required for building the project and frozen to a specific commit.

The core of the codebase is contained in the four packages:
1. quiz
2. picovoice_ros
3. robot_behavior
4. keyboard_quiz

The collected data is stored in the `results` folder.

## Install
```sh
mkdir ~/ros_ws
cd ~/ros_ws
```

Clone the repo as source folder
```sh
git clone git@github.com:Prisca-Lab/robot_quiz.git src
git submodule update --init --recursive
```

Build and source
```sh
cd ~/ros_ws
catkin build
source devel/setup.sh
```

## Prepare the robot

Kill the alive movements from `ari-19c:8080`
The nodes to be paused are:
- interaction_profile_manager
- head_manager
- pal_chrome

or kill from the terminal

```sh
rosnode kill /interaction_profile_manager 
rosnode kill /head_manager
rosnode kill /pal_chrome
```
## Let's start the quiz

Launch the quiz:
`roslaunch quiz run.launch` is using the microphone for intent recognition

Both launch files require arguments `user_id` and `condition`. The arguments can also be accessed from the ros param server.

### Standard Mode
In this mode, the user input is expected to be received via the microphone with `_device:=-1` (check the audio device with `arecord -l`)

The parameter for selecting the audio device is found in `quiz/launch/run.launch:L14`

```sh
roslaunch quiz run.launch user_id:=1 condition:=2
```

The conditions are following:
1. ANTAGONIST
2. AGREEABLENESS

If a bag is already recorded containing data from the user, an error is raised and returned to user.


### State Machine of the interaction
![img](/quiz/data/sm.png)

For testing the logic, use the keyboard mode.

### Keyboard Mode

```sh
roslaunch quiz run_keyboard.launch user_id:=1 condition:=2
```



