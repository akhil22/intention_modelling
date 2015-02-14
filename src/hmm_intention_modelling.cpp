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

double valid_angle(double theta){
	if(theta < 0){
		return (2*M_PI + theta);
	}
	return theta;
}
	
double evalNormalDensity(double mu, double sigma, double x){
	double diff = abs(x - mu);
	if (diff > M_PI){
		diff = 2*M_PI - diff;
	}
	return (1.0/(sigma*sqrt(2*M_PI)))*exp(-0.5*(diff)*(diff)/(sigma*sigma));
}

double get_observation_probability(double* current_goal,double observation,double x,double y){
	double mu_theta = valid_angle(atan2(current_goal[1] - y,current_goal[0] - x));
//	ROS_INFO("mu_theta = %f",mu_theta);
	return evalNormalDensity(mu_theta,1.5,observation);
}

double get_transition_probability(double **current_goals,int i,int j,double x,double y,int num_current_goals){
	double sum = 0;
	double prob = 0;
	double mu_theta = valid_angle(atan2(current_goals[i][1] - y,current_goals[i][0] - x));
	for(int k = 0; k < num_current_goals; k++){
		double temp_theta = valid_angle(atan2(current_goals[k][1] - y ,current_goals[k][0] - x));
		double theta_limit = temp_theta - mu_theta;
		theta_limit = abs(theta_limit);
		if(theta_limit > M_PI){
			theta_limit = 2*M_PI - theta_limit;
		}
	//	sum  = sum + cos(theta_limit/2);
		sum  = sum + M_PI - theta_limit;
		if(k == j){
//			prob = cos(theta_limit/2);
			prob = M_PI - theta_limit;
		}
	}
	return (prob/sum);
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
		int addDynamicGoals(double** current_goals,double x_last,double y_last,it_type itr,double last_observation,bool consider);

		//calculate hmm parameter alpha
		void calcAlpha(double* mean_x,double* mean_y,double* observations,double** current_goals,int num_current_goals,double** alpha);

		//calculate hmm parameter beta 
		void calcBeta(double* mean_x,double* mean_y,double* observations,double** current_goals,int num_current_goals,double** beta);
		
		void calcGamma(double** alpha,double** beta,double** gamma,int num_current_goals);
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
        return valid_angle(atan2(y_cap,x_cap));
}
		
void IntentionModelling::calcAlpha(double* mean_x,double* mean_y,double* observations,double** current_goals,int num_current_goals,double** alpha){
	double sum1 = 0;
	for(int i = 0 ;i< num_current_goals;i++){
			
		alpha[0][i] = (1.0/num_current_goals)*get_observation_probability(current_goals[i],observations[0],mean_x[0],mean_y[0]);
		sum1 = sum1+alpha[0][i];
//		alpha[0][i] = get_observation_probability(current_goals[i],observations[0],mean_x[0],mean_y[0]);
	}
	for(int i =0 ;i< num_current_goals;i++){
		alpha[0][i] = alpha[0][i]/sum1;
	}
	int i,t,j;
	for(t = 1; t<sliding_window_size_;t++){
		double sum_temp = 0;
		for(i = 0;i< num_current_goals;i++){
			double sum = 0;
			for(j = 0; j< num_current_goals; j++){
				sum = sum + alpha[t-1][j]*get_transition_probability(current_goals,j,i,mean_x[t],mean_y[t],num_current_goals);
			}
			alpha[t][i] = sum*get_observation_probability(current_goals[i],observations[t],mean_x[t],mean_y[t]);
			sum_temp = sum_temp + alpha[t][i];
			//			alpha[t][i] = get_observation_probability(current_goals[i],observations[t],mean_x[t],mean_y[t]);
			//		alpha[t][i] = sum;
		}
		for(int i = 0;i<num_current_goals;i++){
			alpha[t][i] = alpha[t][i]/sum_temp;
		}
	}
}
void IntentionModelling::calcBeta(double* mean_x,double* mean_y,double* observations,double** current_goals,int num_current_goals,double** beta){
	double sum1 = 0;
	for(int i = 0 ;i< num_current_goals;i++){
		beta[sliding_window_size_-1][i] = 1.0;
	}
	int i,t,j;
	for(t = sliding_window_size_-2; t>=0;t--){
		double sum_temp = 0;
		for(i = 0;i< num_current_goals;i++){
			double sum = 0;
			for(j = 0; j< num_current_goals; j++){
				sum = sum + beta[t+1][j]*get_transition_probability(current_goals,i,j,mean_x[t],mean_y[t],num_current_goals)*get_observation_probability(current_goals[j],observations[t+1],mean_x[t+1],mean_y[t+1]);
			}
			beta[t][i] = sum;
			sum_temp = sum_temp + beta[t][i];
			//                      alpha[t][i] = get_observation_probability(current_goals[i],observations[t],mean_x[t],mean_y[t]);
			//              alpha[t][i] = sum;
		}
		for(int i = 0;i<num_current_goals;i++){
			beta[t][i] = beta[t][i]/sum_temp;
		}
	}
}

void IntentionModelling::calcGamma(double** alpha,double** beta,double** gamma,int num_current_goals){
	int t,i,j;
	for(t = 0;t<sliding_window_size_;t++){
		double sum = 0;
		for(i = 0;i<num_current_goals;i++){
			gamma[t][i] = alpha[t][i]*beta[t][i];
			sum = sum + gamma[t][i];
		}
		for(i =0;i<num_current_goals;i++){
			gamma[t][i] = gamma[t][i]/sum;
		}
	}
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
int IntentionModelling::addDynamicGoals(double** current_goals,double x_last,double y_last,it_type itr,double last_observation,bool consider){
	int i;
	for(i =0; i<num_goals;i++){
		current_goals[i][0] = goal_locations_[i][0];
		current_goals[i][1] = goal_locations_[i][1];
	}
	if(!consider){
		return num_goals;
	}
	for(it_type itr2 = trajectories_.begin();itr2 != trajectories_.end();itr2++){
		if(itr2 == itr){
			continue;
		}
		int num_points = itr2->second.num_points;
		double theta1=getHeading(x_last,y_last,itr2->second.x(num_points-1),itr2->second.y(num_points-1));
		double theta_limit = theta1 - last_observation;
		theta_limit = abs(theta_limit);
		if(theta_limit >= M_PI){
			theta_limit = theta_limit - M_PI;
		}
		if(theta_limit < M_PI/2.0){
			current_goals[i][0] = itr2->second.x(num_points-1);
			current_goals[i][1] = itr2->second.y(num_points-1);
			i++;
		}
	}
	return i;

}
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
		double **current_goals;
		current_goals = new double* [100];
		for(int a = 0 ;a<100;a++){
			current_goals[a] = new double [2];
		}
		//calculate the observations
		calcObservations(mean_x,mean_y,observations);

		int num_current_goals = addDynamicGoals(current_goals,mean_x[sliding_window_size_-1],mean_y[sliding_window_size_-1],itr,observations[sliding_window_size_-1],true);
//		for(int o = 0 ;o< sliding_window_size_;o++){
//			ROS_INFO("obs %d = %f", o,observations[o]);
//		}
		//initialize hmm paramters 
		double **alpha;
		double **beta;
		double **gamma;
		
		alpha = new double* [sliding_window_size_];
		beta = new double* [sliding_window_size_];
		gamma = new double* [sliding_window_size_];

		for(int a = 0; a<sliding_window_size_ ;a++){
			alpha[a] = new double [num_current_goals];
			beta[a] = new double [num_current_goals];
			gamma[a] = new double [num_current_goals];
		}
		calcAlpha(mean_x,mean_y,observations,current_goals,num_current_goals,alpha);
		calcBeta(mean_x,mean_y,observations,current_goals,num_current_goals,beta);
		calcGamma(alpha,beta,gamma,num_current_goals);

/*		current_goals[0][0] = -1;
		current_goals[0][1] =  0;
		current_goals[1][0] = 1;
		current_goals[1][1] = 1;
		current_goals[2][0] = 1;
		current_goals[2][1] = -1;
		double prob1 = get_transition_probability(current_goals,0,0,0,0,3);
		double prob2 = get_transition_probability(current_goals,0,1,0,0,3);
		double prob3 = get_transition_probability(current_goals,0,2,0,0,3);

		double temp_current_goal[2];
		temp_current_goal[0]=0;
		temp_current_goal[1]=-1;
		double prob4 = get_observation_probability(temp_current_goal,5*M_PI/4.0,0,0);
		ROS_INFO("curretn probs= %f,%f,%f,%f",prob1,prob2,prob3,prob4); */
//		for(int t = 0 ; t<sliding_window_size_;t++){
//		for(int a=0; a<num_current_goals;a++){
//			ROS_INFO("gamma[%d,%d]= %lf",t,a,gamma[t][a]);
//		}
//		}
//		for(int i =0;i<sliding_window_size_;i++)
//			ROS_INFO("headin = %f", observations[i]);
/*		double temp_goal[2];
		temp_goal[0] = 0.0;
		temp_goal[1] = -1.0;
		double temp = get_observation_probability(temp_goal,-M_PI/3.0,0,0);
		ROS_INFO("prob = %f",temp);

*/


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
