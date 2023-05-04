# Run Week 1 
`roslaunch puzzlebot_sim week1.launch 2> >(grep -v TF_REPEATED_DATA)`
`rosrun puzzlebot_sim data_gathering.py`
