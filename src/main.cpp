// TO DO BEFORE TESTING
// Have to tune the robot with the margins and maxstep, as specified on etl
// Test this on the real world map given
//state definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <project4/purePursuit.h>
#include <project4/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <time.h>

//map spec
cv::Mat map;
cv::Mat dynamic_map; //use this variable as dynamic mapping
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

//way points
std::vector<point> waypoints;

//path
std::vector<point> path_RRT;
std::vector<point> back_up_points;

//robot
point robot_pose;
geometry_msgs::Twist cmd_vel;

//point cloud data from 
pcl::PointCloud<pcl::PointXYZ> point_cloud;

//Number of balloons for each goal
int BALLOONS[3];
double PI = 3.14;
//int MARGIN = 7;
//int TREE_ITER_SIZE = 2000;

//FSM state
int state;

//function definition
bool isCollision();
void dynamic_mapping();
void set_waypoints();
void generate_path_RRT();
void generate_path_RRT_update(int);
void callback_state(gazebo_msgs::ModelStatesConstPtr msgs);
void callback_points(sensor_msgs::PointCloud2ConstPtr msgs);
void setcmdvel(double v, double w);
double normalize_angle(double angle_deg);
double relativeX(int);
bool reachedDegree(double degrees, int goal);
bool freePassage();
void deleteSquare(double, double, double);
void scanAndClear();


int main(int argc, char** argv){
    ros::init(argc, argv, "rrt_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Subscriber gazebo_pose_sub = n.subscribe("/gazebo/model_states",1,callback_state);
    ros::Subscriber gazebo_kinect_sub = n.subscribe("/camera/depth/points",1,callback_points);
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);
    ros::ServiceClient gazebo_spawn = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::ServiceClient gazebo_set = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    printf("Initialize topics\n");

    // Load Map
    char* user = getlogin();
    map = cv::imread((std::string("/home/")+
                      std::string(user)+
                      std::string("/catkin_ws/src/project4/src/ground_truth_map_sin1.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    map_y_range = map.cols;
    map_x_range = map.rows;
    printf("map.rows = %d\n", map.rows);
    map_origin_x = map_x_range/2.0 - 0.5;
    map_origin_y = map_y_range/2.0 - 0.5;
    /* //Testing implementation
    world_x_min = -10.0;
    world_x_max = 10.0;
    world_y_min = -10.0;
    world_y_max = 10.0;*/
    //Real world implementation
    world_x_min = -1.0;
    world_x_max = 2.0;
    world_y_min = -2.0;
    world_y_max = 2.0; 
    res = 0.01; //0.05 //0.01 Latter for real world
    printf("Load map\n");
    dynamic_map = map.clone();

    // Set Way Points
    int number_balloons = 0;
    set_waypoints();
    printf("Set way points\n");

    // RRT
    generate_path_RRT();
    printf("Generate RRT\n");

    // Goals
    int goals_reached = 0;

    // FSM
    state = INIT;
    bool running = true;
    purePursuit pure_pursuit;
    int look_ahead_idx;
    ros::Rate control_rate(10);
    while(running){
        switch (state) {
        case INIT: {
            look_ahead_idx = 0;

            //visualize path
            for(int i = 0; i < path_RRT.size(); i++){
                gazebo_msgs::SpawnModel model;
                model.request.model_xml = std::string("<robot name=\"simple_ball\">") +
                        std::string("<link name=\"ball\">") +
                        std::string("<inertial>") +
                        std::string("<mass value=\"1.0\" />") +
                        std::string("<origin xyz=\"0 0 0\" />") +
                        std::string("<inertia  ixx=\"1.0\" ixy=\"0.0\"  ixz=\"0.0\"  iyy=\"1.0\"  iyz=\"0.0\"  izz=\"1.0\" />") +
                        std::string("</inertial>") +
                        std::string("<visual>") +
                        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                        std::string("<geometry>") +
                        std::string("<sphere radius=\"0.09\"/>") +
                        std::string("</geometry>") +
                        std::string("</visual>") +
                        std::string("<collision>") +
                        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                        std::string("<geometry>") +
                        std::string("<sphere radius=\"0\"/>") +
                        std::string("</geometry>") +
                        std::string("</collision>") +
                        std::string("</link>") +
                        std::string("<gazebo reference=\"ball\">") +
                        std::string("<mu1>10</mu1>") +
                        std::string("<mu2>10</mu2>") +
                        std::string("<material>Gazebo/Blue</material>") +
                        std::string("<turnGravityOff>true</turnGravityOff>") +
                        std::string("</gazebo>") +
                        std::string("</robot>");

                std::ostringstream ball_name;
                ball_name << i;
                model.request.model_name = ball_name.str();
                model.request.reference_frame = "world";
                model.request.initial_pose.position.x = path_RRT[i].x;
                model.request.initial_pose.position.y = path_RRT[i].y;
                model.request.initial_pose.position.z = 0.7;
                model.request.initial_pose.orientation.w = 0.0;
                model.request.initial_pose.orientation.x = 0.0;
                model.request.initial_pose.orientation.y = 0.0;
                model.request.initial_pose.orientation.z = 0.0;

                gazebo_spawn.call(model);

                ros::spinOnce();
                ros::Rate(10).sleep();
            }
            printf("Spawn path\n");

            //initialize robot position
            geometry_msgs::Pose model_pose;
            model_pose.position.x = waypoints[0].x;
            model_pose.position.y = waypoints[0].y;
            model_pose.position.z = 0.3;
            model_pose.orientation.x = 0.0;
            model_pose.orientation.y = 0.0;
            model_pose.orientation.z = 0.0;
            model_pose.orientation.w = 0.0;

            geometry_msgs::Twist model_twist;
            model_twist.linear.x = 0.0;
            model_twist.linear.y = 0.0;
            model_twist.linear.z = 0.0;
            model_twist.angular.x = 0.0;
            model_twist.angular.y = 0.0;
            model_twist.angular.z = 0.0;

            gazebo_msgs::ModelState modelstate;
            modelstate.model_name = "RosAria";
            modelstate.reference_frame = "world";
            modelstate.pose = model_pose;
            modelstate.twist = model_twist;

            gazebo_msgs::SetModelState setmodelstate;
            setmodelstate.request.model_state = modelstate;

            gazebo_set.call(setmodelstate);
            ros::spinOnce();
            ros::Rate(0.33).sleep();
            printf("Initialize ROBOT\n");
            //Create a initial back-up point, so it wont crash if it hits something before getting to a point
            back_up_points.push_back(waypoints[0]);

            state = RUNNING;
        } break;

        case RUNNING: {
            printf("path_RRT = %lu\n", path_RRT.size());

            if (path_RRT.size() == 0){
                printf("path_RRT = 0, fault \n\n\n\n\n\n");
                state = FINISH;
                break;
            }
            
            while(look_ahead_idx < path_RRT.size()){

                scanAndClear();
                if (BALLOONS[goals_reached] == 0){
                    printf("There is a problem with the finding path algo\n");
                    state = FINISH;
                    break;
                }

                if (isCollision()){
                    printf("COLLIDED \n\n\n\n\n\n\n");
                    state = PATH_PLANNING;
                    number_balloons = 0;
                    setcmdvel(0,0);
                    ros::spinOnce();
                    control_rate.sleep();
                    break;
                }



                control ctrl = pure_pursuit.get_control(robot_pose, path_RRT[look_ahead_idx]);
                setcmdvel(ctrl.v, ctrl.w);
                cmd_vel_pub.publish(cmd_vel);

                // Find the distance to the goal
                double dist_goal = std::sqrt((path_RRT[look_ahead_idx].x-robot_pose.x)*(path_RRT[look_ahead_idx].x-robot_pose.x)+
                    (path_RRT[look_ahead_idx].y-robot_pose.y)*(path_RRT[look_ahead_idx].y-robot_pose.y));
                
                if (dist_goal < 0.2){//Break if distance is less than 0.2 m
                    look_ahead_idx++;
                    number_balloons++;
                    back_up_points.push_back(robot_pose);
                    printf("Balloon reached, look_ahead_idx = %d\n", look_ahead_idx);
                    if (number_balloons == BALLOONS[goals_reached]){
                        printf("Goal Reached!!!!! = %d \n\n\n\n\n\n\n", goals_reached+1);
                        goals_reached++;
                        number_balloons = 0;
                        //scanAndClear();
                        setcmdvel(0, 0);
                        cmd_vel_pub.publish(cmd_vel);
                        ros::spinOnce();
                        control_rate.sleep();
                        generate_path_RRT_update(goals_reached);
                        look_ahead_idx = 0;
                    }
                }
                ros::spinOnce();
                control_rate.sleep();
            }
            if (look_ahead_idx >= path_RRT.size()){ //Error function
                printf("look_ahead_idx when done = %d\n", look_ahead_idx);
                state = FINISH;
            }
            if (goals_reached == 3){
                printf("Finished properly\n");
                state = FINISH;
            }
            
        } break;

        case PATH_PLANNING: { // First makes it turn around for a couple of seconds while its dynamicaly mapping it
            
            dynamic_mapping(); 
            double accumelator = 0;
            double old_distance = robot_pose.th;
            double new_distance;

            while(accumelator < 25.0/180 * PI){
                setcmdvel(0, -0.2);
                cmd_vel_pub.publish(cmd_vel);
                ros::spinOnce();
                control_rate.sleep();
                new_distance = robot_pose.th;
                accumelator += std::abs(std::abs(old_distance) - std::abs(new_distance));
                old_distance = new_distance;
                dynamic_mapping();
            }
            setcmdvel(0, 0);
            cmd_vel_pub.publish(cmd_vel);
            ros::spinOnce();
            control_rate.sleep();
            old_distance = robot_pose.th;
            accumelator = 0;
            while(accumelator < 50.0/180 * PI){
                setcmdvel(0, 0.2);
                cmd_vel_pub.publish(cmd_vel);
                ros::spinOnce();
                control_rate.sleep();
                new_distance = robot_pose.th;
                accumelator += std::abs(std::abs(old_distance) - std::abs(new_distance));
                old_distance = new_distance;
                dynamic_mapping();
            }
            setcmdvel(0, 0);
            cmd_vel_pub.publish(cmd_vel);
            ros::spinOnce();
            control_rate.sleep();
            while (!reachedDegree(0, look_ahead_idx)){
                setcmdvel(0, -0.2);
                cmd_vel_pub.publish(cmd_vel);
                ros::spinOnce();
                control_rate.sleep();
            }

            point old_pos = robot_pose;
            accumelator = 0;
            while(accumelator < 0.4){
                setcmdvel(-0.2,0);
                cmd_vel_pub.publish(cmd_vel);
                ros::spinOnce();
                control_rate.sleep();
                accumelator += std::sqrt((robot_pose.x - old_pos.x)*(robot_pose.x - old_pos.x) + (robot_pose.y - old_pos.y)*(robot_pose.y - old_pos.y));
                old_pos = robot_pose;
                
            }
            setcmdvel(0, 0);
            cmd_vel_pub.publish(cmd_vel);
            ros::spinOnce();
            control_rate.sleep();
            generate_path_RRT_update(goals_reached);
            printf("generate_path_RRT_update finnished\n");
            look_ahead_idx = 0;

            state = RUNNING;
        } break;

        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd_vel);
            running = false;
            printf("Finish state reached\n");
            ros::spinOnce();
            control_rate.sleep();
        } break;

        default: {
        } break;
        }

        printf("curr state : %d\ncurr robot pos : %.2f,%.2f\ncurr robot vel : %.2f,%.2f\n",state,robot_pose.x,robot_pose.y,cmd_vel.linear.x,cmd_vel.angular.z);
        printf("path_RRT = %lu\n", path_RRT.size());
        printf("goals_reached = %d\n", goals_reached);
        printf("look_ahead_idx = %d\n", look_ahead_idx);
    }
    return 0;
}

void generate_path_RRT()
{

    std::vector<point> result;
    rrtTree tree;

    int i=0;
    while (i < 2) //Basically order_size - 1, have to change this one everytime i change the waypoints
    {
        tree = rrtTree(waypoints[i], waypoints[i+1], map, map_origin_x, map_origin_y, res, 3); //last one is margin to wall , testing is 8
        tree.generateRRTst(world_x_max, world_x_min, world_y_max, world_y_min, 2000, 1); //K = 500, MAX_STEP = 1, testing is 3

        result = tree.backtracking();
        std::reverse(result.begin(),result.end());
        int k;
        for (k=0; k<result.size(); k++){
            path_RRT.push_back(result[k]);
        }
        BALLOONS[i] = result.size();
        i++;
    }

    tree = rrtTree(waypoints[0], waypoints[0+1], map, map_origin_x, map_origin_y, res, 3); //last one is margin to wall
    tree.generateRRTst(world_x_max, world_x_min, world_y_max, world_y_min, 2000, 1); //K = 500, MAX_STEP = 1

    result = tree.backtracking();
}



void generate_path_RRT_update(int start){  // Updating the path after a crash

    std::vector<point>().swap(path_RRT); // For clearing a vector safely 

    int back_up_iter = back_up_points.size()-1;
    int old_start = start;

    std::vector<point> result;
    rrtTree tree;
    waypoints[start] = back_up_points[back_up_iter];

    while (start < 3) //order_size - 1
    {
        tree = rrtTree(waypoints[start], waypoints[start+1], dynamic_map, map_origin_x, map_origin_y, res, 2); //last one is margin to wall, testing is 5
        tree.generateRRTst(world_x_max, world_x_min, world_y_max, world_y_min, 2000, 1); //Testing is 3
        result = tree.backtracking();
        std::reverse(result.begin(),result.end());

        if (result.size() == 0){ //This means it didnt work, have to try again, only runs on the first run
            back_up_iter--;
            waypoints[start] = back_up_points[back_up_iter];
            printf("Trying to find another starting point for generate_path_RRT_update\n");
        }
        else{
            int k;
            for (k=0; k<result.size(); k++){
                path_RRT.push_back(result[k]);
            }
            BALLOONS[start] = result.size();
            start++;
        }
        printf("start = %d\n", start);
    }
    tree = rrtTree(waypoints[old_start], waypoints[old_start+1], dynamic_map, map_origin_x, map_origin_y, res, 2); //last one is margin to wall
    tree.generateRRTst(world_x_max, world_x_min, world_y_max, world_y_min, 2000, 1);
    result = tree.backtracking();
}



void set_waypoints()
{

	//For the real world
	waypoint_candid[0].x = 1.5;
	waypoint_candid[0].y = -1.5;
	waypoint_candid[1].x = 1.5;
	waypoint_candid[1].y = 1.5;
	waypoint_candid[2].x = -0.5;
	waypoint_candid[2].y = -1.5;
	int order [] = {0, 1, 2};
	int order_size = 3;
    /*// scenario 1 sample way points
    point waypoint_candid[3];
    waypoint_candid[0].x = -6.0;
    waypoint_candid[0].y = 0.0;
    waypoint_candid[1].x = 3.0;
    waypoint_candid[1].y = 1.0;
    waypoint_candid[2].x = -8.0;
    waypoint_candid[2].y = 7.0;
    int order[] = {0,1,2};
    int order_size = 3;*/

    /*// scenario 2 sample way points
    point waypoint_candid[3];
    waypoint_candid[0].x = -5.0;
    waypoint_candid[0].y = -4.0;
    waypoint_candid[1].x = 5.0;
    waypoint_candid[1].y = 6.0;
    waypoint_candid[2].x = -8.0;
    waypoint_candid[2].y = 8.0;
    int order[] = {0,1,2};
    int order_size = 3;*/

    /*// scenario 3 sample way points
    point waypoint_candid[4];
    waypoint_candid[0].x = 5.0;
    waypoint_candid[0].y = -7.0;
    waypoint_candid[1].x = -3.0; // -3 //4
    waypoint_candid[1].y = -6.0; //-6 //2
    waypoint_candid[2].x = -8.0;
    waypoint_candid[2].y = 8.0;
    waypoint_candid[3].x = 8.0;
    waypoint_candid[3].y = 8.0;
    int order[] = {0,1,2,3};
    int order_size = 4;*/



    /*point waypoint_candid[4];
    waypoint_candid[0].x = 5.0;
    waypoint_candid[0].y = -8.0;
    waypoint_candid[1].x = -6.0;
    waypoint_candid[1].y = -7.0;
    waypoint_candid[2].x = -8.0;
    waypoint_candid[2].y = 8.0;
    waypoint_candid[3].x = 3.0;
    waypoint_candid[3].y = 7.0;

    int order[] = {3,1,2,3};
    int order_size = 4;*/

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}


void callback_state(gazebo_msgs::ModelStatesConstPtr msgs){
    for(int i; i < msgs->name.size(); i++){
        if(std::strcmp(msgs->name[i].c_str(),"RosAria") == 0){
            robot_pose.x = msgs->pose[i].position.x;
            robot_pose.y = msgs->pose[i].position.y;
            robot_pose.th = tf::getYaw(msgs->pose[i].orientation);
        }
    }
}

void callback_points(sensor_msgs::PointCloud2ConstPtr msgs){
    pcl::fromROSMsg(*msgs,point_cloud);
}

bool isCollision() // Detects collision in front of the kinect sensor
{
    point relative;
    double x_world_cord, y_world_cord, robot_angle, rotation_theta;

    int j;
    for (j = 0; j < point_cloud.size(); j++){
        if (point_cloud[j].x < 0.4 && point_cloud[j].x > -0.4 && point_cloud[j].y > 0 && 
            point_cloud[j].y < 1 && point_cloud[j].z >0 &&
            point_cloud[j].z < 0.54){ 

            printf("point_cloud[j].z %f\n", point_cloud[j].z);
            printf("point_cloud[j].x %f\n", point_cloud[j].x);
            printf("point_cloud[j].y %f\n", point_cloud[j].y);
            relative.x=point_cloud[j].z;
            relative.y=-point_cloud[j].x;

            // Finding rotation angle for changing from kinect frame to world frame
            rotation_theta = normalize_angle(robot_pose.th);
            x_world_cord = relative.x*std::cos(rotation_theta)-std::sin(rotation_theta)*relative.y+robot_pose.x;
            y_world_cord = relative.x*std::cos(rotation_theta)+std::sin(rotation_theta)*relative.y+robot_pose.y;

            // Returns true if not an obstacle already in grid map
            if (!(dynamic_map.at<uchar>(round(x_world_cord/res + map_origin_x), round(y_world_cord/res + map_origin_y)) == 0))
                return true;
        }
    }
    return false;
}

void dynamic_mapping() // Maps the new obstacles to dynamic_map
{
    point relative;

    int j;
    double x_world_cord, y_world_cord, robot_angle, rotation_theta;
    for (j=0; j<point_cloud.size(); j++)
    {
        if (point_cloud[j].x < 0.5 && point_cloud[j].x > -0.5 && point_cloud[j].y > 0 && 
            point_cloud[j].y < 1 && point_cloud[j].z >0 &&
            point_cloud[j].z < 0.54)
        {
            relative.x=point_cloud[j].z;
            relative.y=-point_cloud[j].x;

                // Finding rotation angle for changing from kinect frame to world frame
            rotation_theta = normalize_angle(robot_pose.th);
            x_world_cord = relative.x*std::cos(rotation_theta)-std::sin(rotation_theta)*relative.y+robot_pose.x;
            y_world_cord = relative.x*std::cos(rotation_theta)+std::sin(rotation_theta)*relative.y+robot_pose.y;

                // Add the point on the grid map
            dynamic_map.at<uchar>(round(x_world_cord/res + map_origin_x), round(y_world_cord/res + map_origin_y)) = 0;
        }
    }
}
void setcmdvel(double v, double w){
    cmd_vel.linear.x = v;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = w;
}

double normalize_angle(double angle_deg){
    if (angle_deg < 0){
        angle_deg += 2*3.14;
    }
    double theta;

    if (0<angle_deg && angle_deg<(90*3.14)/180)
        theta=(90*3.14)/180-angle_deg;
    else if ((90*3.14)/180<angle_deg && angle_deg<(180*3.14)/180) 
        theta=angle_deg-(90*3.14)/180;
    else if ((180*3.14)/180<angle_deg && angle_deg<(270*3.14)/180)
        theta=angle_deg-(180*3.14)/180 - 3.14; 
    else if ((270*3.14)/180<angle_deg && angle_deg<(360*3.14)/180)
        theta=angle_deg-(270*3.14)/180 - 3.14;
    else
        printf("This should never happen \n\n\n\n\n\n\n\n\n\n\n\n\n");
    return theta;
}

double relativeX(int goal){
    //Find alfa to the next balloon
    double vector_diff[2];
    vector_diff[0] = path_RRT[goal].x - robot_pose.x;
    vector_diff[1] = path_RRT[goal].y - robot_pose.y;

    //Creates vector for the angle
    double vector_angle[2]; 
    vector_angle[0] = std::cos(robot_pose.th);
    vector_angle[1] = std::sin(robot_pose.th);


    //Find the angle between the vectors
    double alfa = std::atan2(vector_diff[1], vector_diff[0]) - std::atan2(vector_angle[1], vector_angle[0]);

    if (alfa < 0) //Make alfa always positive value
        alfa += 2*PI;

    double vector_goal_robot[2];
    double l= sqrt(vector_diff[0]*vector_diff[0]+vector_diff[1]*vector_diff[1]);

    double relative_x = alfa;

    return relative_x;
}

bool reachedDegree(double degrees, int goal){
    double rel_x = relativeX(goal);
    if (rel_x > (360.0-degrees-45)/180*PI && rel_x < (360.0-degrees)/180*PI)
        return true;
    else
        return false;
}

bool freePassage(){
    int j;
    for (j = 0; j < point_cloud.size(); j++){
        if (point_cloud[j].x < 2 && point_cloud[j].x > -2 && point_cloud[j].y > 0 && 
            point_cloud[j].y < 1 && point_cloud[j].z >0 &&
            point_cloud[j].z < 0.7){ 

            //printf("point_cloud[j].z %f\n", point_cloud[j].z);
            //printf("point_cloud[j].x %f\n", point_cloud[j].x);
            //printf("point_cloud[j].y %f\n", point_cloud[j].y);

            return false;
        }
    }
    return true;
}

void deleteSquare(double cord_x, double cord_y, double size){
    double x_cord_grid = round(cord_x/res + map_origin_x);
    double y_cord_grid = round(cord_y/res + map_origin_y);
    int i;
    int j;
    
    for (i = 0; i < size/2; i++){
        for (j = 0; j < size/2; j++){
            if(x_cord_grid - i > 0 && x_cord_grid < 800 && y_cord_grid - j> 0 && y_cord_grid < 800){
                dynamic_map.at<uchar>(x_cord_grid - i, y_cord_grid - j) = 255;
                dynamic_map.at<uchar>(x_cord_grid - i, y_cord_grid + j) = 255;
                dynamic_map.at<uchar>(x_cord_grid + i, y_cord_grid - j) = 255;
                dynamic_map.at<uchar>(x_cord_grid + i, y_cord_grid + j) = 255;
            }
        }
    }
}

void scanAndClear(){
    if (freePassage()){
        //Find the point 0.7 in front of the robot
        point relative;
        double x_world_cord, y_world_cord, robot_angle, rotation_theta;
        double k, y, x;
        point old_robot_pos = robot_pose;
        rotation_theta = normalize_angle(old_robot_pos.th);
        relative.y = -0.1;
        relative.x = 0.7;
        double i;
        for (i = 0; i < 0.3; i += 0.1){
            relative.y += i;    
            x_world_cord = relative.x*std::cos(rotation_theta)-std::sin(rotation_theta)*relative.y+old_robot_pos.x;
            y_world_cord = relative.x*std::cos(rotation_theta)+std::sin(rotation_theta)*relative.y+old_robot_pos.y;
            for(k = 0; k < 10; k++){
                x = x_world_cord + (old_robot_pos.x-x_world_cord)*k/9; 
                y = y_world_cord + (old_robot_pos.y-y_world_cord)*k/9;     
                deleteSquare(x, y, 6);           
            }
        }
    }
}
