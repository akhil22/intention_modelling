/* Author: Akhil Kumar Nagariya 
   RRC IIIT Hyderabad */
#include<sstream>
#include<iostream>
#include<ros/ros.h>
#include<stdio.h>
#include<string>
#include<tf/transform_datatypes.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseArray.h>
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
		void getIntentions();
		double calc_distance(float x1, float y1, float x2, float y2);
		void getHeading(float , float ,float ,float,float&,float& );

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

};

double IntentionModelling::calc_distance(float x1, float y1, float x2, float y2){
	return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}
void IntentionModelling::getHeading(float x1,float y1,float x2,float y2,float& x_cap,float& y_cap){
	double dist = calc_distance(x1,y1,x2,y2);
	x_cap = (x2 - x1)/dist;
	y_cap = (y2 - y1)/dist;
}

void IntentionModelling::getIntentions(){
	for(it_type itr = trajectories_.begin(); itr != trajectories_.end();itr++){
		int agent_num = 0;
		int camera_num = 0;
		sscanf(itr->first.c_str(),"%d_%d",&camera_num,&agent_num);
		int n_points = itr->second.num_points;
		//		ROS_INFO("found %s at (%f, %f) time = %f",itr->first.c_str(),itr->second.x(n_points-1),itr->second.y(n_points-1),itr->second.time(n_points-1));
		int num_points = itr->second.num_points;
		if(num_points <= sliding_window_size_){
			ROS_INFO("NOT ENOUGH POINTS %d",num_points);
			return;
		}
		//create gp for initial smoothing
		CovFuncND cov(1,length_scale_,sigmaf_);
		gaussian_process::SingleGP gpx(cov,sigman_);
		gaussian_process::SingleGP gpy(cov,sigman_);


		//hold x, y and time informations for points currently in sliding window
		TDoubleVector in_x(sliding_window_size_+1);
		TDoubleVector in_y(sliding_window_size_+1);
		TVector<TDoubleVector> in_time(sliding_window_size_+1);
		TDoubleVector test_time(sliding_window_size_+1);

		//holds filtered values of x and y to be considered for intention prediction
		double mean_x[sliding_window_size_+1];
		double mean_y[sliding_window_size_+1];
		double var_x[sliding_window_size_+1];
		double var_y[sliding_window_size_+1];
		//create input data for gaussian process
		int initial_point  =  num_points - sliding_window_size_;
		double arc_length = 0;
		for(int i = num_points - sliding_window_size_,j =0 ; i < num_points ; i++,j++){
			if(j!= 0)
				arc_length = arc_length + calc_distance(itr->second.x(i-1),itr->second.y(i-1),itr->second.x(i),itr->second.y(i));
			in_x.insert_element(j,itr->second.x(i));
			in_y.insert_element(j,itr->second.y(i));
			//		in_x.insert_element(j,1);
			//		in_y.insert_element(j,1);
			TDoubleVector temp_in_time(1);
			temp_in_time.insert_element(0,itr->second.time(i) - itr->second.time(initial_point));
			//	temp_in_time.insert_element(0,i-initial_point);
			in_time.insert_element(j, temp_in_time);
		}
		double avg_velocity = arc_length/(itr->second.time(num_points-1) - itr->second.time(initial_point));
		ROS_INFO("average_velocity= %f",avg_velocity);
		for (int l = 0 ;l<num_goals;l++){
			float x_cap=0;
			float y_cap=0;
			getHeading(itr->second.x(num_points-1),itr->second.y(num_points-1),goal_locations_[l][0],goal_locations_[l][1],x_cap,y_cap);
			double x_future = itr->second.x(sliding_window_size_-1)+x_cap*step_size_;
			double y_future = itr->second.y(sliding_window_size_-1)+y_cap*step_size_;
			double theta = atan2(y_cap,x_cap);
		//	theta = theta*180.0/M_PI;
			ROS_INFO("Heading for goal %d = %f",l,theta);
			ROS_INFO("CURRENT = (%f,%f) Future =(%f,%f) goal = (%f,%f)",itr->second.x(num_points-1),itr->second.y(num_points-1),x_future,y_future,goal_locations_[l][0],goal_locations_[l][1]);
			TDoubleVector future_time(1);
			double dist_to_goal = calc_distance(itr->second.x(num_points-1),itr->second.y(num_points-1),goal_locations_[l][0],goal_locations_[l][1]);
			//future_time.insert_element(0,itr->second.time(num_points-1) + step_size_/avg_velocity);
		//	future_time.insert_element(0,itr->second.time(num_points-1)-itr->second.time(initial_point) + step_size_);
			future_time.insert_element(0,itr->second.time(num_points-1)-itr->second.time(initial_point)+dist_to_goal);
//			in_x.insert_element(sliding_window_size_,x_future);
//			in_y.insert_element(sliding_window_size_,y_future);
			in_x.insert_element(sliding_window_size_,goal_locations_[l][0]);
			in_y.insert_element(sliding_window_size_,goal_locations_[l][1]);
			in_time.insert_element(sliding_window_size_,future_time);
		//	in_time.insert_element(sliding_window_size_,calc_distance(,itr->second.x(num_points-1),itr->second.y(num_points-1),goal_location[l]);

			//set the input data 
			gpx.SetData(in_time,in_x);
			gpy.SetData(in_time,in_y);
			geometry_msgs::PoseArray path;
			path.poses.resize(20);
		//	float del_time = future_time(0)/sliding_window_size_;
			float del_time = 0;
			if(dist_to_goal > 1.5)
				del_time =1.5/sliding_window_size_;
			else
				del_time = dist_to_goal/sliding_window_size_;

		//	ROS_INFO("del_time = %f",del_time);
			//calculate filtered values of x and y
			for(int i=0 ; i< sliding_window_size_ ;i++){
				TDoubleVector time(1);
				time.insert_element(0,in_time(sliding_window_size_-1)(0)+del_time*i);
				gpx.Evaluate(time, mean_x[i],var_x[i]);
				gpy.Evaluate(time, mean_y[i],var_y[i]);
				geometry_msgs::Pose temp_pose;
				temp_pose.position.x = mean_x[i];
				temp_pose.position.y = mean_y[i];
				temp_pose.position.z = 0.2;
			//	temp_pose.orientation = itr->second.orientation;
				getHeading(mean_x[i],mean_y[i],goal_locations_[l][0],goal_locations_[l][1],x_cap,y_cap);
				double theta = atan2(y_cap,x_cap);
				temp_pose.orientation = tf::createQuaternionMsgFromYaw(theta);
		//		temp_pose.orientation.y =  0;
		//		temp_pose.orientation.z =  sin(theta/2);
		//		temp_pose.orientation.w = cos(theta/2);
		//		ROS_INFO("cos theta/2= %f",cos(theta));
				path.poses[i] = temp_pose;
		//		ROS_INFO("actual_points (%f %f) time = %f",in_x(i),in_y(i),in_time(i)(0));
		//		ROS_INFO("predicted_points (%f %f) time = %f",mean_x[i],mean_y[i],in_time(i)(0));

			}
			path.header.frame_id = "map";
			smooth_human_pose_[agent_num][l].publish(path);

/*			for(int i = 0; i< num_goals; i++){
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
				//ROS_INFO("Intention Prob for goal %d = %f", i, prob);
			}*/
		}
	}
}
IntentionModelling::IntentionModelling(){

	//get parmeters
	ros::NodeHandle private_nh_("~");
	private_nh_.param ("sliding_window_size", sliding_window_size_, int(10));
	private_nh_.param ("max_num_agents", max_num_agents_, int(10));
	private_nh_.param ("timout", timeout_, double(2));
	private_nh_.param ("goal_file", goal_file_, std::string("/home/akhil/goals.txt"));
	private_nh_.param ("length_scale", length_scale_, double(log(1.3)));
	private_nh_.param ("sigmaf", sigmaf_, double(log(4.42)));
	private_nh_.param ("sigman", sigman_, double(log(1.7168)));
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
			smooth_human_pose_[i][j] = nh.advertise<geometry_msgs::PoseArray>(topic,20);
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
	ros::Publisher smooth_human_pose = nh.advertise<geometry_msgs::PoseArray>("human_pose_smooth",20);
	ros::Rate r(10);
	while(ros::ok()){
	    ros::spinOnce();
	    intention_modelling.getIntentions();
	    r.sleep();
	}
}
