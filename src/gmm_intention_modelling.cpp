/* Author: Akhil Kumar Nagariya 
   RRC IIIT Hyderabad */
#include<sstream>
#include<iostream>
#include<ros/ros.h>
#include<stdio.h>
#include<string>
#include<tf/transform_datatypes.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Path.h>
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
	geometry_msgs::Quaternion orientation;
};
typedef std::map< std::string,Trajectory>::iterator it_type; 

double evalNormalDensity(double mu, double sigma, double x){
	return (1.0/(sigma*sqrt(2*M_PI)))*exp(-0.5*(x-mu)*(x-mu)/(sigma*sigma));
}
class IntentionModelling{
	public:
		IntentionModelling();
		
		int num_goals;
		//get intentions
		void getIntentions();
		//predict intentions
		void predictIntent(it_type itr);
		
		double calc_distance(float x1, float y1, float x2, float y2);
		
		double getHeading(float , float ,float ,float);
		//fit the gp to the given trajectory
		void gpFit(it_type itr,double mean_x[],double mean_y[],double var_x[],double var_y[]);
		//calculate observations 
		void calcObservations(double mean_x[],double mean_y[],double observations[]);

		//add dynamic goals if consider is true
		int addDynamicGoals(double** current_goals,double x_last,double y_last,it_type itr,bool consider);
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
		//step size
		double step_size_;

		//gp hyperparameters
		double length_scale_;
		double sigmaf_;
		double sigman_;

		//trajectory data subscriber
		ros::Subscriber traj_sub_;

		ros::Publisher** smooth_human_pose_;
		//trajectory subscriber call back function
		void trajCB (const people_msgs::Person::ConstPtr& msg);

		//calculates intentions 

		//reads goal locations
		void readGoals();
		//smooth the data

};

double IntentionModelling::calc_distance(float x1, float y1, float x2, float y2){
	return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}
double IntentionModelling::getHeading(float x1,float y1,float x2,float y2){
	double dist = calc_distance(x1,y1,x2,y2);
	double x_cap = (x2 - x1)/dist;
	double y_cap = (y2 - y1)/dist;
	return atan2(y_cap,x_cap);
}
void IntentionModelling::predictIntent(it_type itr){}

void IntentionModelling::gpFit(it_type itr,double mean_x[],double mean_y[],double var_x[],double var_y[]){
	CovFuncND cov(1,length_scale_,sigmaf_);
	gaussian_process::SingleGP gpx(cov,sigman_);
	gaussian_process::SingleGP gpy(cov,sigman_);
	TDoubleVector in_x(sliding_window_size_/3+1);
	TDoubleVector in_y(sliding_window_size_/3+1);
	TVector<TDoubleVector> in_time(sliding_window_size_/3+1);
	double temp_time_values[sliding_window_size_+1];
	int num_points = itr->second.num_points;
        int initial_point = num_points - sliding_window_size_-1;
	double time_increment = (itr->second.time(num_points-1) - itr->second.time(num_points-sliding_window_size_-1))/(sliding_window_size_+1.0);
	for(int i = num_points - sliding_window_size_-1,j =0 ; i < num_points ; i= i+3,j++){
		in_x.insert_element(j,itr->second.x(i));
		in_y.insert_element(j,itr->second.y(i));
		TDoubleVector temp_in_time(1);
		temp_in_time.insert_element(0,itr->second.time(i) - itr->second.time(initial_point));
		in_time.insert_element(j,temp_in_time);
	}
	gpx.SetData(in_time,in_x);
	gpy.SetData(in_time,in_y);
	gpx.OptimizeGP();
	gpy.OptimizeGP();
	for(int l =0;l<=sliding_window_size_;l++){
		TDoubleVector time(1);
		time.insert_element(0,l*time_increment);
		gpx.Evaluate(time, mean_x[l],var_x[l]);
		gpy.Evaluate(time, mean_y[l],var_y[l]);
	}

}
void IntentionModelling::calcObservations(double* mean_x,double* mean_y,double* observations){
	for(int i = 0; i<sliding_window_size_;i++){
		observations[i] = getHeading(mean_x[i] ,mean_y[i],mean_x[i+1],mean_y[i+1]);
	}
}
int IntentionModelling::addDynamicGoals(double** current_goals,double x_last,double y_last,it_type itr,bool consider){}
void IntentionModelling::getIntentions(){
	for(it_type itr = trajectories_.begin(); itr != trajectories_.end();itr++){
		int agent_num = 0;
		int camera_num = 0;
		sscanf(itr->first.c_str(),"%d_%d",&camera_num,&agent_num);
		int num_points = itr->second.num_points;

		//should have sufficient points for intent prediction
		if(num_points <= sliding_window_size_+2){
			ROS_INFO("NOT ENOUGH POINTS %d",num_points);
			return;
		}
		double mean_x[sliding_window_size_+1];
		double mean_y[sliding_window_size_+1];
		double var_x[sliding_window_size_+1];
		double var_y[sliding_window_size_+1];
		
		//smooth trajecotry using Gaussian process
		gpFit(itr,mean_x,mean_y,var_x,var_y);

		//publish smooth trajectory 
		nav_msgs::Path path;
		path.header.frame_id = "map";
		path.poses.resize(sliding_window_size_+1);
		for(int i = 0 ,j = num_points - sliding_window_size_-1 ;i<=sliding_window_size_,j<num_points;j++,i++){
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = "map";
			pose.pose.position.x = mean_x[i];
			pose.pose.position.y = mean_y[i];
			pose.pose.position.z = 0;
			path.poses[i] = pose;
		}
		smooth_human_pose_[agent_num][0].publish(path);
		double observations[sliding_window_size_];

		//calculate the observations
		calcObservations(mean_x,mean_y,observations);
		for(int i =0;i<sliding_window_size_;i++)
			ROS_INFO("headin = %f", observations[i]);


	}
}
IntentionModelling::IntentionModelling(){

	//get parmeters
	ros::NodeHandle private_nh_("~");
	private_nh_.param ("sliding_window_size", sliding_window_size_, int(20));
	private_nh_.param ("max_num_agents", max_num_agents_, int(10));
	private_nh_.param ("timout", timeout_, double(2));
	private_nh_.param ("goal_file", goal_file_, std::string("/home/akhil/goals.txt"));
	private_nh_.param ("length_scale", length_scale_, double(log(1.3)));
	private_nh_.param ("sigmaf", sigmaf_, double(log(4.42)));
	private_nh_.param ("sigman", sigman_, double(log(1.7168)));
//	private_nh_.param ("length_scale", length_scale_, double(0));
//	private_nh_.param ("sigmaf", sigmaf_, double(log(1)));
//	private_nh_.param ("sigman", sigman_, double(log(1)));
	private_nh_.param ("dist_thresh", dist_thresh_, double(-5.7949));
	private_nh_.param ("step_size", step_size_, double(0.5));

	//matlab gpml gives log values of hyperparameters calculate antilog to get hyperparam
	length_scale_ = exp(length_scale_);
	sigmaf_ = exp(sigmaf_);
	sigman_ = exp(sigman_);

	//subscribe to get trajectories
	traj_sub_ = nh_.subscribe ("human_pose", 100, &IntentionModelling::trajCB, this);

	//read goals
	readGoals();
	smooth_human_pose_ = new ros::Publisher* [max_num_agents_];
	ros::NodeHandle nh;
	for (int i =0; i<max_num_agents_; i++){
		smooth_human_pose_[i] = new ros::Publisher [num_goals];
		for(int j =0 ;j<num_goals;j++){
			char topic[100];
			sprintf(topic,"agent_%d_goal_%d",i,j);
			smooth_human_pose_[i][j] = nh.advertise<nav_msgs::Path>(topic,20);
		}
	}
}


void IntentionModelling::trajCB(const people_msgs::Person::ConstPtr& msg){
	std::string index = msg->name;
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
	trajectories_[index].orientation = msg->orientation;
	trajectories_[index].time.insert_element(n_points,msg->stamp.toSec());
	trajectories_[index].num_points++;
	trajectories_[index].then = current;
	if(trajectories_[index].num_points == 1000){
		 trajectories_[index].new_traj = true;
	}


/*	//calculate inentions
	double intention_prob[num_goals];
//if poivnts in a trajectory is less then points required then continue otherwise calculate intentions
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
		ROS_WARN("goal %d `(%f,%f)",i,goal_locations_[i][0],goal_locations_[i][1]);
	}
}
int main(int argc, char **argv){
	ros::init(argc,argv,"intention_modelling");
	ros::NodeHandle nh;
	IntentionModelling intention_modelling;
	ros::Publisher smooth_human_pose = nh.advertise<nav_msgs::Path>("human_pose_smooth",20);
	ros::Rate r(10);
	while(ros::ok()){
	    ros::spinOnce();
	    intention_modelling.getIntentions();
	    r.sleep();
	}
}
