# Run Week 1 
`roslaunch puzzlebot_sim week1.launch 2> >(grep -v TF_REPEATED_DATA)`
`rosrun puzzlebot_sim data_gathering.py`
# Run Week 2 
`roslaunch puzzlebot_sim week2.launch 2> >(grep -v TF_REPEATED_DATA)`
`rosrun puzzlebot_sim teleop_twist_keyboard.py`
# Run Week 2 
`roslaunch puzzlebot_sim week4.launch 2> >(grep -v TF_REPEATED_DATA)`
