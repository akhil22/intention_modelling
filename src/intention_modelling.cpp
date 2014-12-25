/* Author: Akhil Kumar Nagariya 
   RRC IIIT Hyderabad */
#include<ros/ros.h>
#include<stdio.h>
#include<string>
#include<geometry_msgs/Pose.h>
#include<gaussian_process_catkin/covarianceFunctions.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include<vector>
#include<std_msgs/Header.h>
#include<fstream>
#include<sstream>
#include<stdlib.h>
//structure to contain trajectory data
struct Trajectory{
	int num_points;
	TDoubleVector time;
	TDoubleVector x;
	TDoubleVector y;
	ros::Time then;
};

class IntentionModelling{
	public:
		IntentionModelling();

	private:
		// node handler
		ros::NodeHandle nh_;
		int sliding_window_size_;
		Trajectory *trajectories_;
		int num_goals_;
		int **goal_locations_;
		int max_num_agents_;
		double timeout_;
		std::string goal_file_;
		//trajectory data subscriber
		ros::Subscriber traj_sub_;

		//trajectory subscriber call back function
		void trajCB (const visualization_msgs::MarkerArray::ConstPtr& msg);
		void readGoals();

};

IntentionModelling::IntentionModelling(){

	//get parmeters
	ros::NodeHandle private_nh_("~");
	private_nh_.param ("sliding_window_size", sliding_window_size_, int(4));
	private_nh_.param ("max_num_agents", max_num_agents_, int(10));
	private_nh_.param ("timout", timeout_, double(2));
	private_nh_.param ("goal_file", goal_file_, std::string("~/goals.txt"));
	
	//subscribe to get trajectory
	traj_sub_ = nh_.subscribe ("human_pose", 100, &IntentionModelling::trajCB, this);

	//read goals
	readGoals();
	
	//initialize trajectories
	trajectories_ = new Trajectory [max_num_agents_];
	for(int i = 0; i< max_num_agents_; i++){
		trajectories_[i].then = ros::Time::now();
	}
}

void IntentionModelling::trajCB(const visualization_msgs::MarkerArray::ConstPtr& msg){
	int num_markers = msg->markers.size();
	for(int i = 0; i < num_markers; i++){
		int marker_id = msg->markers[i].id;
		ros::Time current = ros::Time::now();
		double del_t = (current - trajectories_[marker_id].then).toSec();
		if (del_t > timeout_){
			trajectories_[marker_id].num_points = 0;
		}
		int n_points = trajectories_[marker_id].num_points;
		trajectories_[marker_id].x.insert_element(n_points,msg->markers[i].pose.position.x);
		trajectories_[marker_id].y.insert_element(n_points,msg->markers[i].pose.position.y);
		trajectories_[marker_id].time.insert_element(n_points,msg->markers[i].header.stamp.toSec());
		trajectories_[marker_id].num_points++;
	}
}

void IntentionModelling::readGoals(){
	std::ifstream filep(goal_file_.c_str());
	if(!filep){
		ROS_INFO("goal file doesnot exist");
		return ;
	}
}
int main(){
	return 0;
}
