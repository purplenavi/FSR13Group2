#include <math.h>

#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <nav_msgs/Path.h>


static boost::mutex mutex;

static const char NODENAME[] = "point2point_navigation";

static actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *client = NULL;
static nav_msgs::Path currentPath;
static unsigned int currentGoalIndex = 0;

void sendPositionGoal(bool next);


void routePlanCallback(const nav_msgs::Path::ConstPtr& msg)
{
	ROS_INFO("gotcallback!");
	if (!ros::ok())
		return;
	mutex.lock();
	currentPath = nav_msgs::Path();
	currentGoalIndex = 0;
	currentPath = *msg;
	sendPositionGoal(false);
	mutex.unlock();
}

void doneCb(const actionlib::SimpleClientGoalState& state, const
		move_base_msgs::MoveBaseResultConstPtr& result) {
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	
	if (!ros::ok())
		return;
	switch (state.state_) {
		case actionlib::SimpleClientGoalState::PENDING:
			break;
		case actionlib::SimpleClientGoalState::ACTIVE:
			break;
		case actionlib::SimpleClientGoalState::RECALLED:
			break;
		case actionlib::SimpleClientGoalState::REJECTED:
		case actionlib::SimpleClientGoalState::PREEMPTED:
		case actionlib::SimpleClientGoalState::ABORTED:
		case actionlib::SimpleClientGoalState::LOST:
			ROS_WARN("Goal %s. Sending...", state.toString().c_str());
			mutex.lock();
			sendPositionGoal(false);
			mutex.unlock();
			break;
		case actionlib::SimpleClientGoalState::SUCCEEDED:
			ROS_WARN("Goal %s. Sending next.", state.toString().c_str());
			mutex.lock();
			sendPositionGoal(true);
			mutex.unlock();
			break;
	}
}


void activeCb() {
	ROS_DEBUG("Goal active");
}


void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
	const geometry_msgs::PoseStamped &base_position = feedback->base_position;
	mutex.lock();
	double goalX = currentPath.poses[currentGoalIndex].pose.position.x;
	double goalY = currentPath.poses[currentGoalIndex].pose.position.y;
	double dist_to_goal = sqrt(
			pow(base_position.pose.position.x - goalX, 2.0) +
			pow(base_position.pose.position.y - goalY, 2.0)
			);
	if (dist_to_goal <= 0.3) {
		ROS_INFO("Dist to goal is %lf. Sending the next goal from the CB", dist_to_goal);
		sendPositionGoal(true);
	}
	mutex.unlock();
}

void sendPositionGoal(bool next) {
	if (!ros::ok())
		return;
	if (currentPath.poses.size() == 0)
		return;
	if (next) {
		if (currentPath.poses.size() <= ++currentGoalIndex)
			currentGoalIndex = 0;
	}
	ROS_INFO("Sending next goal to point(%lf, %lf)", currentPath.poses[currentGoalIndex].pose.position.x,
							 currentPath.poses[currentGoalIndex].pose.position.y);
	move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = currentPath.poses[currentGoalIndex];
	goal.target_pose.header.stamp = ros::Time::now();
	client->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, NODENAME);
	ros::NodeHandle n;
	client = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
	while(! client->waitForServer(ros::Duration(5.0))) {
		ROS_WARN("Could not establish connection with ActionServer.");
	}
	ros::Subscriber sub_rp = n.subscribe("gui_plan", 1, routePlanCallback);
	
	ros::spin(); 
	client->cancelAllGoals();
	delete client;
	return 0;
}


