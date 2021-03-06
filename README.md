# Robotic arm with computer vision

![Straight line trajectory](Content/IK_DH_params.png)

Serial servos: Lx-16a

Chess engine: Stockfish (https://stockfishchess.org/) with python API (https://pypi.org/project/stockfish/)

Computer vision: Potential dataset: https://github.com/maciejczyzewski/neural-chessboard 
		 		    https://par.nsf.gov/servlets/purl/10099572
		 		    
Inverse Kinematics (kinematic coupling):
https://www.researchgate.net/publication/261281825_A_screw_dual_quaternion_operator_for_serial_robot_kinematics
https://www.researchgate.net/publication/276159627_Using_cuckoo_optimization_algorithm_and_imperialist_competitive_algorithm_to_solve_inverse_kinematics_problem_for_numerical_control_of_robotic_manipulators

All the good stuff related to kinematics (and robotics in general) is in this book:
> Robotics: Modelling, Planning and Control by Bruno Siciliano, Lorenzo Sciavicco, Luigi Villani,Giuseppe Oriolo

Slowly getting to grips with 3D printer and putting it together. I've just managed to put together the first 3 joints and the servos for the first 2 degrees of freedom:

![Initial phase](Content/First_2_DoF.gif)



Created a 3d model of the robot and used what it seems to be the standard for their simulations. At a first glance it looks medieval compared to multibody simulations in other industries.


![Initial phase](Content/Gazebo_manual_simulation.gif)

It's been a while since Kaggle allowed me to work on this project.
This is a great source on QP, simple, clean and well explained (https://scaron.info/teaching/inverse-kinematics.html)


Implementation of this paper where decision variables of the controller are the incremental angle of the joints rather than speeds which seems to be the norm.
Least square problem is transformed to QP problem with two inequality constraints, the incremental joint angles are limited by a max/min joint angles and max/min incremental values.

https://roam.me.columbia.edu/files/seasroamlab/publications/humanoids2013.pdf


![Full robot moving](Content/robot_full.gif)


![Straight line trajectory](Content/Linear_trajectory.PNG)