# futuro_remoto

```sh
mkdir ~/ros_ws
cd ~/ros_ws
```

Clone the repo as source folder
```sh
git clone git@github.com:Prisca-Lab/quiz_futuro_remoto.git src
git submodule update --init --recursive
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


```
rosrun picovoice_driver picovoice_driver_rhino _access_key:=30uXi1g4+23OWdquCouuIax2CfKWmTzfvC9IFkSrYmhKTAapX1q4Zg== _model_url:=...

```