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
		int num_goals;

	private:
		// node handler
		ros::NodeHandle nh_;

		//num of trajectory points to be considered to calculate intention
		int sliding_window_size_;
		//holds trajectory information
		Trajectory *trajectories_;
		//holds goal locations
		float **goal_locations_;
		//maximum number of agents to track 
		int max_num_agents_;
		//time out
		double timeout_;
		//name of goal file
		std::string goal_file_;

		//gp hyperparameters
		double length_scale_;
		double sigmaf_;
		double sigman_;
		
		//trajectory data subscriber
		ros::Subscriber traj_sub_;

		//trajectory subscriber call back function
		void trajCB (const visualization_msgs::MarkerArray::ConstPtr& msg);

		//calculates intentions 
		void getIntentions(double *intention_prob, int traj_num);

		//reads goal locations
		void readGoals();

};

void IntentionModelling::getIntentions(double *intention_prob, int traj_num){
	int num_points = trajectories_[traj_num].num_points;

	//create gp for initial smoothing
	CovFuncND cov(1,length_scale_,sigmaf_);
	gaussian_process::SingleGP gpx(cov,sigman_);
	gaussian_process::SingleGP gpy(cov,sigman_);

	//hold x, y and time informations for points currently in sliding window
	TDoubleVector in_x(sliding_window_size_);
	TDoubleVector in_y(sliding_window_size_);
	TVector<TDoubleVector> in_time(sliding_window_size_);
	TDoubleVector test_time(sliding_window_size_);

	//holds filtered values of x and y to be considered for intention prediction
	double mean_x[sliding_window_size_];
	double mean_y[sliding_window_size_];
	double var_x[sliding_window_size_];
	double var_y[sliding_window_size_];

	//create input data for gaussian process
	for(int i = num_points - sliding_window_size_,j =0 ; i < num_points ; i++,j++){
		in_x.insert_element(j,trajectories_[traj_num].x(i));
		in_y.insert_element(j,trajectories_[traj_num].y(i));
		TDoubleVector temp_in_time(1);
		temp_in_time.insert_element(0,trajectories_[traj_num].time(i));
		in_time.insert_element(j, temp_in_time);
	}

	//set the input data 
	gpx.SetData(in_time,in_x);
	gpy.SetData(in_time,in_y);

	//calculate filtered values of x and y
	for(int i=0 ; i< sliding_window_size_ ;i++){
		TDoubleVector time(1);
		time.insert_element(0,in_time(i)(0));
		gpx.Evaluate(time, mean_x[i],var_x[i]);
		gpy.Evaluate(time, mean_y[i],var_y[i]);
	}

	//calculate intention probability for each goal
	for(int i = 0; i< num_goals; i++){
		double prob = 1.0;
		for(int j = 1 ; j < sliding_window_size_ ;j++){

			//current heading
			double heading = atan2(in_y(j) - in_y(j-1),in_x(j) - in_x(j-1));
			//goal direction
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

	//matlab gpml gives log values of hyperparameters calculate antilog to get hyperparam
	length_scale_ = exp(length_scale_);
	sigmaf_ = exp(sigmaf_);
	sigman_ = exp(sigman_);
	
	//subscribe to get trajectories
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

		//if no data is being pulished on a trajectory for timeout time then reinitilize trajectory 
		ros::Time current = ros::Time::now();
		double del_t = (current - trajectories_[marker_id].then).toSec();
		
		if (del_t > timeout_){
			trajectories_[marker_id].num_points = 0;
		}
		
		//add x y and time location to the trajectory data
		int n_points = trajectories_[marker_id].num_points;
		trajectories_[marker_id].x.insert_element(n_points,msg->markers[i].pose.position.x);
		trajectories_[marker_id].y.insert_element(n_points,msg->markers[i].pose.position.y);
		trajectories_[marker_id].time.insert_element(n_points,msg->markers[i].header.stamp.toSec());
		trajectories_[marker_id].num_points++;
		trajectories_[marker_id].then = ros::Time::now();
	}

	//calculate inentions
	double intention_prob[num_goals];
	for(int i = 0; i< num_markers; i++){
		int marker_id = msg->markers[i].id;

		//if points in a trajectory is less then points required then continue otherwise calculate intentions
		if(trajectories_[marker_id].num_points < sliding_window_size_){
			ROS_WARN("%d trajectory doesnot have sufficient points", marker_id);
			continue;
		}
		else{
			ROS_WARN("%d trajectory got sufficient points" , marker_id);
			double intention_prob[num_goals];
			getIntentions(intention_prob, marker_id);
			for(int j = 0; j< num_goals; j++){
		                int n_points = trajectories_[marker_id].num_points;
				ROS_WARN("%d current point(%f,%f) num_points = %d trajectory intention probability of(%f,%f) is %f",marker_id,trajectories_[marker_id].x(n_points-1),trajectories_[marker_id].y(n_points-1),n_points, goal_locations_[j][0], goal_locations_[j][1], intention_prob[j]);
			}
		}
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
	sscanf(line.c_str(),"%d", &num_goals);
	ROS_INFO("loading %d goal locations", num_goals);

	//initialize goal locations;
	goal_locations_ = new float* [num_goals];
	for(int i=0; i<num_goals; i++){
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
		if (j == num_goals)
			break;
	}

	//display all goal locations
	for(int i = 0; i < num_goals ;i++){
		ROS_WARN("goal %d (%f,%f)",i,goal_locations_[i][0],goal_locations_[i][1]);
	}
}
int main(int argc, char **argv){
	ros::init(argc,argv,"intention_modelling");
	IntentionModelling intention_modelling;
	ros::spin();
}
