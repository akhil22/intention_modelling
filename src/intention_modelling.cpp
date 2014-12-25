#include<ros/ros.h>
#include<stdio.h>
#include<string>
#include<geometry_msgs/Pose.h>
#include<gaussian_process_catkin/covarianceFunctions.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>

//structure to contain trajectory data
struct Trajectory{
	int id;
	int num_points;
	TDoubleVector time;
	TDoubleVector x;
	TDoubleVector y;
	TDoubleVector vx;
	TDoubleVector vy;
};

class IntentionModelling{
	public:
		IntentionModelling();
	private:
		// node handler
		ros::NodeHandle nh_;
		int sliding_window_size_;

		//trajectory data subscriber
		ros::Subscriber traj_sub_;

		//trajectory subscriber call back function
		void trajCB(const visualization_msgs::MarkerArray::ConstPtr& msg); 

};

IntentionModelling::IntentionModelling(){
	ros::NodeHandle private_nh_("~");
	private_nh_.param ("sliding_window_size", sliding_window_size_, int(4));
	traj_sub_ = nh_.subscribe ("human_pose", 100, &IntentionModelling::trajCB, this);
}

void IntentionModelling::trajCB(const visualization_msgs::MarkerArray::ConstPtr& msg){
	int num_markers = msg->markers.size();
}

int main(){
	return 0;
}
