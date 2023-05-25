# Run Week 1 
`roslaunch puzzlebot_sim week1.launch 2> >(grep -v TF_REPEATED_DATA)`
`rosrun puzzlebot_sim data_gathering.py`
# Run Week 2 
`roslaunch puzzlebot_sim week2.launch 2> >(grep -v TF_REPEATED_DATA)`
`rosrun puzzlebot_sim teleop_twist_keyboard.py`
# Run Week 4
`roslaunch puzzlebot_sim week4.launch 2> >(grep -v TF_REPEATED_DATA)`

`rosrun puzzlebot_sim bug2.py`

`rosservice call /puzzlebot_setGoal [GOAL]`
# Run Week 6
`roslaunch puzzlebot_sim week6.launch 2> >(grep -v TF_REPEATED_DATA)`

