#include <project4/purePursuit.h>
#include <stdio.h>

purePursuit::purePursuit(){

}

control purePursuit::get_control(point x_robot, point x_goal){
    // Compute the angle difference between robot's heading 
     //and look-ahead point direction
    double PI = 3.14; 
    double treshold1, treshold2;
    treshold1 = (PI*60)/180;
    treshold2 = (PI*20)/180;

    control ctrl_temp;
    //First create vector between robot and point
    double diff_angle;
    double vector_diff[2];
    vector_diff[0] = x_goal.x - x_robot.x;
    vector_diff[1] = x_goal.y - x_robot.y;

    //Creates vector for the angle
    double vector_angle[2];	
    vector_angle[0] = std::cos(x_robot.th);
    vector_angle[1] = std::sin(x_robot.th);


    //Find the angle between the vectors
    double alfa = std::atan2(vector_diff[1], vector_diff[0]) - std::atan2(vector_angle[1], vector_angle[0]);

    if (alfa < 0) //Make alfa always positive value
    	alfa += 2*PI;

    //Check first treshold to turn fast
 	if (alfa > treshold1 && alfa < 2*PI-treshold1){
 		//Stop robot, turn to the right angle fast
 		if (alfa < PI){
 			ctrl_temp.v = 0;
 			ctrl_temp.w = -0.6;
 		}
 		else{
 			ctrl_temp.v = 0;
	 		ctrl_temp.w = 0.6;
 		}
 	}
 	else if (alfa > treshold2 && alfa < 2*PI -treshold2){
 		//Stop robot, turn to the right angle slow
		if (alfa < PI){
 			ctrl_temp.v = 0;
 			ctrl_temp.w = -0.25;
 		}
 		else{
 			ctrl_temp.v = 0;
	 		ctrl_temp.w = 0.25;
 		}

 	} 

 	else{
 		//Do pure pursuit algorithm
	    double vector_goal_robot[2];
	    vector_goal_robot[0]=x_goal.x-x_robot.x;
	    vector_goal_robot[1]=x_goal.y-x_robot.y;
	    double l= sqrt(vector_goal_robot[0]*vector_goal_robot[0]+vector_goal_robot[1]*vector_goal_robot[1]);
	    double relative_x = (std::sin(alfa)*l);
		double yetta = (-2*relative_x)/(l*l);

		ctrl_temp.v = 0.2;
		ctrl_temp.w = ctrl_temp.v*yetta;
	}
	ctrl = ctrl_temp;
	return ctrl_temp;
}

