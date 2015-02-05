/* Author: Akhil Kumar Nagariya 
   RRC IIIT Hyderabad */
#include<sstream>
#include<iostream>
#include<ros/ros.h>
#include<stdio.h>
#include<string>
#include<geometry_msgs/Pose.h>
#include<gaussian_process_catkin/covarianceFunctions.h>
#include<gaussian_process_catkin/SingleGP.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include<people_msgs/Person.h>
#include<people_msgs/PositionMeasurementArray.h>
#include<vector>
#include<std_msgs/Header.h>
#include<fstream>
#include<sstream>
#include<stdlib.h>
#include<math.h>
#include<map>

//structure to contain trajectory data
struct Trajectory{
	int num_points;
	bool new_traj = true;
	TDoubleVector time;
	TDoubleVector x;
	TDoubleVector y;
	ros::Time then;
};
typedef std::map< std::string,Trajectory>::iterator it_type; 

double evalNormalDensity(double mu, double sigma, double x){
	return (1.0/(sigma*sqrt(2*M_PI)))*exp(-0.5*(x-mu)*(x-mu)/(sigma*sigma));
}

class IntentionModelling{
	public:
		IntentionModelling();
		int num_goals;
		void getIntentions();
		double calc_distance(float x1, float y1, float x2, float y2);

	private:
		// node handler
		ros::NodeHandle nh_;

		//num of trajectory points to be considered to calculate intention
		int sliding_window_size_;
		//holds trajectory information
	 	std::map< std::string,Trajectory>trajectories_;
		//holds goal locations
		float **goal_locations_;
		//maximum number of agents to track 
		int max_num_agents_;
		//time out
		double timeout_;
		//min distance between trajectory points
		double dist_thresh_;
		//name of goal file
		std::string goal_file_;

		//gp hyperparameters
		double length_scale_;
		double sigmaf_;
		double sigman_;
		
		//trajectory data subscriber
		ros::Subscriber traj_sub_;

		//trajectory subscriber call back function
		void trajCB (const people_msgs::Person::ConstPtr& msg);

		//calculates intentions 

		//reads goal locations
		void readGoals();

};

double IntentionModelling::calc_distance(float x1, float y1, float x2, float y2){
	return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}

void IntentionModelling::getIntentions(){
	for(it_type itr = trajectories_.begin(); itr != trajectories_.end();itr++){
		int n_points = itr->second.num_points;
		ROS_INFO("found %s at (%f, %f) time = %f",itr->first.c_str(),itr->second.x(n_points-1),itr->second.y(n_points-1),itr->second.time(n_points-1));
	}
}
/*	int num_points = trajectories_[traj_num].num_points;

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
}*/

IntentionModelling::IntentionModelling(){

	//get parmeters
	ros::NodeHandle private_nh_("~");
	private_nh_.param ("sliding_window_size", sliding_window_size_, int(4));
	private_nh_.param ("max_num_agents", max_num_agents_, int(10));
	private_nh_.param ("timout", timeout_, double(2));
	private_nh_.param ("goal_file", goal_file_, std::string("/home/akhil/goals.txt"));
	private_nh_.param ("length_scale", length_scale_, double(0.0));
	private_nh_.param ("sigmaf", sigmaf_, double(0.0));
	private_nh_.param ("sigman", sigman_, double(log(0.1)));
	private_nh_.param ("dist_thresh", dist_thresh_, double(0.01));

	//matlab gpml gives log values of hyperparameters calculate antilog to get hyperparam
	length_scale_ = exp(length_scale_);
	sigmaf_ = exp(sigmaf_);
	sigman_ = exp(sigman_);
	
	//subscribe to get trajectories
	traj_sub_ = nh_.subscribe ("human_pose", 100, &IntentionModelling::trajCB, this);

	//read goals
	readGoals();
	
}

void IntentionModelling::trajCB(const people_msgs::Person::ConstPtr& msg){
	std:string index = msg->name;
	if(trajectories_[index].new_traj){
		trajectories_[index].then = msg->stamp;
		trajectories_[index].new_traj = false;
		trajectories_[index].num_points = 0;
		if(trajectories_[index].x.size() == 0){
			trajectories_[index].x.resize(1000);
			trajectories_[index].y.resize(1000);
			trajectories_[index].time.resize(1000);
		}
	}
	int n_points = trajectories_[index].num_points; 
	if(n_points != 0 && calc_distance(msg->position.x,msg->position.y,trajectories_[index].x(n_points-1),trajectories_[index].y(n_points-1)<= dist_thresh_)){
//		return;
	}
	ros::Time current = msg->stamp;
	double del_t = (current - trajectories_[index].then).toSec();

	if (del_t > timeout_){
		trajectories_[index].num_points = 0;
		trajectories_[index].new_traj = true;

	}
	//add x y and time location to the trajectory data
	trajectories_[index].x.insert_element(n_points,msg->position.x);
	trajectories_[index].y.insert_element(n_points,msg->position.y);
	trajectories_[index].time.insert_element(n_points,msg->stamp.toSec());
	trajectories_[index].num_points++;
	trajectories_[index].then = current;
	if(trajectories_[index].num_points == 1000){
		 trajectories_[index].new_traj = true;
	}


/*	//calculate inentions
	double intention_prob[num_goals];
//if points in a trajectory is less then points required then continue otherwise calculate intentions
	if(trajectories_[index].num_points < sliding_window_size_){
		ROS_WARN("%s_%s trajectory doesnot have sufficient points",msg->name.c_str(),msg->object_id.c_str());
	}
	else{
		ROS_WARN("%s_%s trajectory got sufficient points" , msg->name.c_str(),msg->object_id.c_str());
		double intention_prob[num_goals];
//		getIntentions(intention_prob, person_id);
		for(int j = 0; j< num_goals; j++){
			int n_points = trajectories_[index].num_points;
			ROS_WARN("person(%s_%s) current point(%f,%f) num_points = %d trajectory intention probability of(%f,%f) is %f",msg->name.c_str(),msg->object_id.c_str(),trajectories_[index].x(n_points-1),trajectories_[index].y(n_points-1),n_points, goal_locations_[j][0], goal_locations_[j][1], intention_prob[j]);
		}
	}*/
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
	ros::Rate r(10);
	while(ros::ok()){
	    ros::spinOnce();
	    intention_modelling.getIntentions();
	    r.sleep();
	}
}
