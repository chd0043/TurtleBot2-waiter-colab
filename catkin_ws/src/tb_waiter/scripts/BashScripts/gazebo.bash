#!/bin/bash

# positions
x_array=( -2.73  -6.38   0.63   4.70   6.37   3.83  -7.82  -8.34   5.90  -2.00 )
y_array=(  7.06  -1.42   0.31  -2.97   0.27  -1.00   1.94  -1.10  -3.60  -0.30 )

# Random position for Robot 1
random1=$(($RANDOM%10))
robot1_x=${x_array[$random1]}
robot1_y=${x_array[$random1]}

# Random position for Robot 2
random2=$(($RANDOM%10)) #1st execution
while [ $random2 == $random1 ]; do
   random2=$(($RANDOM%10)) #loop execution
done
robot2_x=${x_array[$random2]}
robot2_y=${x_array[$random2]}

# Random position for Robot 3
random3=$(($RANDOM%10)) #1st execution
while [ $random3 == $random1 ] || [ $random3 == $random2 ]; do
   random3=$(($RANDOM%10)) #loop execution
done
robot3_x=${x_array[$random3]}
robot3_y=${x_array[$random3]}


echo ${x_array[$random1]}
echo ${y_array[$random1]}

echo ${x_array[$random2]}
echo ${y_array[$random2]}

echo ${x_array[$random3]}
echo ${y_array[$random3]}

roslaunch tb_tables tb_tables.launch  r1_x:=$random1 r1_y:=$random2 r2_x:=$random3 r2_y:=$random4 r3_x:=$random5 r3_y:=$random6 







