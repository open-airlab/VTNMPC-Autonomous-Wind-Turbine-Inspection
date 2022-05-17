#include <gnc_functions.hpp>
//include API 

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	//wait4start();

	//create local reference frame 
	initialize_local_frame();
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";



    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
	//request takeoff
	takeoff(300);
    
	//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = -750/100;
	nextWayPoint.y = 8500/100;
	nextWayPoint.z = 2600/50;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = -2750/100;
	nextWayPoint.y = 1400/100;
	nextWayPoint.z = 1400/100;
	nextWayPoint.psi = -90;
	waypointList.push_back(nextWayPoint);

	//waypointList.push_back(nextWayPoint);
	//nextWayPoint.x = 0;
	//nextWayPoint.y = 0;
	//nextWayPoint.z = 3;
	//nextWayPoint.psi = 180;
	//waypointList.push_back(nextWayPoint);
	//nextWayPoint.x = 0;
	//nextWayPoint.y = 0;
	//nextWayPoint.z = 3;
	//nextWayPoint.psi = 0;
	//waypointList.push_back(nextWayPoint);

     
	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(10.0);
	int counter = 0;


	 ros::Time last_request = ros::Time::now();
	while(ros::ok())
	{
        if( current_state_g.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state_g.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
		ros::spinOnce();
		rate.sleep();
		if(check_waypoint_reached(.3) == 1)
		{
			if (counter < waypointList.size())
			{
				set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
				counter++;	
			}else{
				//land after all waypoints are reached
			//	land();
			}	
		}	
	   
	}
	return 0;
}
