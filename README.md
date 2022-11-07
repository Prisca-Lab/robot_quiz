# futuro_remoto

```sh
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

Clone the repo
```sh
git clone git@github.com:Prisca-Lab/quiz_futuro_remoto.git quiz
```

Build and source
```sh
cd ~/ros_ws
catkin build
source devel/setup.sh
```

Launch the quiz, the arguments can also be accessed from the ros param server.
```sh
roslaunch quiz run.launch user_id:=1 condition:=2
```

The conditions are following:
1. FRONT ANTAGONIST
2. FRONT AGREEABLENESS
3. SIDE ANTAGONIST
4. SIDE AGREEABLENESS

If a bag is already recorded containing data from the user, an error is raised and returned to user.


State Machine

![img](quiz/data/sm.png)
