#include "chassis.h"
#include "motion.h"
#include "hardware.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot");

	ros::NodeHandle nh;

	// We run the ROS loop in a separate thread as external calls, such
	// as service callbacks loading controllers, can block the (main) control loop
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// set process priority
	struct sched_param params{.sched_priority = 98};
  	if (sched_setscheduler(0, SCHED_FIFO, &params) == -1)
    	ROS_ERROR("Set scheduler failed, RUN THIS NODE AS SUPER USER.\n");

	//Create Nodes
	Chassis chassis;
	Motion motion;

	ros::Rate loop_rate(ROBOT_SAMPLING_RATE);

	//Process Jobs
	while (ros::ok())
	{
		//Update Subnodes
		chassis.update();
		motion.update();

		//Update Hardware
		Hardware()->update();

		//Loop
		loop_rate.sleep();
	}

	//Release Hardware
	ReleaseHardware();
	return 0;
}
