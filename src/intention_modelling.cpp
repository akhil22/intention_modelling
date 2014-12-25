/* Author: Akhil Kumar Nagariya 
   RRC IIIT Hyderabad */
#include<ros/ros.h>
#include<stdio.h>
#include<string>
#include<geometry_msgs/Pose.h>
#include<gaussian_process_catkin/covarianceFunctions.h>
#include<gaussian_process_catkin/SingleGP.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include<vector>
#include<std_msgs/Header.h>
#include<fstream>
#include<sstream>
#include<stdlib.h>
#include<math.h>
//structure to contain trajectory data
struct Trajectory{
	int num_points;
	TDoubleVector time;
	TDoubleVector x;
	TDoubleVector y;
	ros::Time then;
};

double evalNormalDensity(double mu, double sigma, double x){
	return (1.0/(sigma*sqrt(2*M_PI)))*exp(-0.5*(x-mu)*(x-mu)/(sigma*sigma));
}

class IntentionModelling{
	public:
		IntentionModelling();
		void getIntentions(double *intention_prob, int traj_num);

	private:
		// node handler
		ros::NodeHandle nh_;
		int sliding_window_size_;
		Trajectory *trajectories_;
		int num_goals_;
		float **goal_locations_;
		int max_num_agents_;
		double timeout_;
		std::string goal_file_;

		//gp hyperparam
		double length_scale_;
		double sigmaf_;
		double sigman_;
		
		//trajectory data subscriber
		ros::Subscriber traj_sub_;

		//trajectory subscriber call back function
		void trajCB (const visualization_msgs::MarkerArray::ConstPtr& msg);
		void readGoals();

};

void IntentionModelling::getIntentions(double *intention_prob, int traj_num){
	int num_points = trajectories_[traj_num].num_points;
	
	CovFuncND cov(1,length_scale_,sigmaf_);
	gaussian_process::SingleGP gpx(cov,sigman_);
	gaussian_process::SingleGP gpy(cov,sigman_);
	TDoubleVector in_x(sliding_window_size_);
	TDoubleVector in_y(sliding_window_size_);
	TDoubleVector in_time(sliding_window_size_);
	TDoubleVector test_time(sliding_window_size_); 
	double mean_x[sliding_window_size_];
	double mean_y[sliding_window_size_];
	double var_x[sliding_window_size_];
	double var_y[sliding_window_size_];
	
	for(int i = num_points - sliding_window_size_,j =0 ; i < num_points ; i++,j++){
		in_x.insert_element(j,trajectories_[traj_num].x(i));
		in_y.insert_element(j,trajectories_[traj_num].y(i));
		in_time.insert_element(j,trajectories_[traj_num].time(i));
	}
	TVector<TDoubleVector> in_time_temp;
	in_time_temp.insert_element(0,in_time);
	gpx.SetData(in_time_temp,in_x);
	gpy.SetData(in_time_temp,in_y);
	
	for(int i=0 ; i< sliding_window_size_ ;i++){
		TDoubleVector time(1);
		time.insert_element(1,in_time(i));
		gpx.Evaluate(time, mean_x[i],var_x[i]);
		gpy.Evaluate(time, mean_y[i],var_y[i]);
	}

	for(int i = 0; i< num_goals_; i++){
		double prob = 1.0;
		for(int j = 1 ; j < sliding_window_size_ ;j++){
			double heading = atan2(in_y(j) - in_y(j-1),in_x(j) - in_x(j-1));
			double target = atan2(goal_locations_[i][1] - in_y(j), goal_locations_[i][0] - in_x(j));
			double desired = target - heading;
			if(desired > M_PI){
				desired = desired - M_PI;
			}
			if(desired < -1*M_PI){
				desired = desired + M_PI;
			}
			prob = prob * evalNormalDensity(0,0.2,desired);
		}
		intention_prob[i] = prob;
	}
}

IntentionModelling::IntentionModelling(){

	//get parmeters
	ros::NodeHandle private_nh_("~");
	private_nh_.param ("sliding_window_size", sliding_window_size_, int(4));
	private_nh_.param ("max_num_agents", max_num_agents_, int(10));
	private_nh_.param ("timout", timeout_, double(2));
	private_nh_.param ("goal_file", goal_file_, std::string("~/goals.txt"));
	private_nh_.param ("length_scale", length_scale_, double(0.0));
	private_nh_.param ("sigmaf", sigmaf_, double(0.0));
	private_nh_.param ("sigman", sigman_, double(log(0.1)));

	length_scale_ = exp(length_scale_);
	sigmaf_ = exp(sigmaf_);
	sigman_ = exp(sigman_);
	
	//subscribe to get trajectory
	traj_sub_ = nh_.subscribe ("human_pose", 100, &IntentionModelling::trajCB, this);

	//read goals
	readGoals();
	
	//initialize trajectories
	trajectories_ = new Trajectory [max_num_agents_];
	for(int i = 0; i< max_num_agents_; i++){
		trajectories_[i].then = ros::Time::now();
		trajectories_[i].x.resize(100);
		trajectories_[i].y.resize(100);
		trajectories_[i].time.resize(100);
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
	
	ROS_INFO("reading goal file %s",goal_file_.c_str());

	//read file
	std::ifstream filep(goal_file_.c_str());
	if(!filep){
		ROS_WARN("goal file doesnot exist");
		return ;
	}

	//get number of goals
	string line;
	getline(filep,line);
	sscanf(line.c_str(),"%d", &num_goals_);
	ROS_INFO("loading %d goal locations", num_goals_);

	//initialize goal locations;
	goal_locations_ = new float* [num_goals_];
	for(int i=0; i<num_goals_; i++){
		goal_locations_[i] = new float [2];
	}
	int j = 0;

	//get all the goal locations
	while(getline(filep,line)){
		stringstream ss2;
		ss2<<line;
		string value;
		int k=0;
		while(getline(ss2,value,',')){
			sscanf(value.c_str(),"%f",&goal_locations_[j][k]);
			k++;
		}
		j++;
		if (j == num_goals_)
			break;
	}

	//display all goal locations
	for(int i = 0; i < num_goals_ ;i++){
		ROS_WARN("goal %d (%f,%f)",i,goal_locations_[i][0],goal_locations_[i][1]);
	}
}
int main(int argc, char **argv){
	ros::init(argc,argv,"intention_modelling");
	IntentionModelling intention_modelling;
	return 0;
}
