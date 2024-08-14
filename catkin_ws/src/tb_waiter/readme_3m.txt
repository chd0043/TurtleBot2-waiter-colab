// Simulacion con 2 turtlebots y 3 mesas (Robot0 y Robot1)

// 1.
roslaunch tb_waiter tb_2tables.launch

// 2. 
roslaunch tb_waiter runspade.launch

//3.
roslaunch tb_waiter move_base_2.launch

//------------------------------------------------//
// Contract-Net

rosrun tb_waiter cn_service0_sim.bash

rosrun tb_waiter cn_service1_sim.bash

rosrun tb_waiter assigner.bash

//------------------------------------------------//
// Negociation

rosrun tb_waiter neg_service0_sim.bash

rosrun tb_waiter neg_service1_sim.bash

rosrun tb_waiter dealer.bash

//------------------------------------------------//
//------------------------------------------------//
//------------------------------------------------//
// Contract-Net (test para STAGE)

roslaunch tb_waiter runspade.launch

rosrun tb_waiter cn_serv0_sim.bash

rosrun tb_waiter cn_serv1_sim.bash

rosrun tb_waiter assigner2.bash

roslaunch tb_waiter tb_2stage.launch

rosrun stage_ros stageros $(rospack find tb_waiter)/worlds/stage/lab.world

//------------------------------------------------//
rosrun stage_ros stageros $(rospack find tb_waiter)/worlds/stage/lab_4.world

roslaunch tb_waiter tb_2stage_cave.launch

roslaunch tb_waiter move_base_stage_cave.launch

roslaunch tb_waiter stage_rviz.launch
