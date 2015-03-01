This pacakage uses HMM and extends Bayesian human motion intentionality prediction to predict human motion from trajectory informations. Provide the possible goals(x,y) that a person might go to in a given environment using the golas.txt file in the following format:

num of goals
goal one x coordinate,goal one y coordinate
goal two x coordinate,goal two y coordinate
goal three x coordinate,goal three y coordinate
goal four x coordinate,goal four y coordinate
....

The algorithm gives the probabiltiy of following each of these goals.To calculate these intention probability we consider past p observations in a particular trajectory.The algorithm uses gaussian process to first smooth the trajectory. The algorithm has following parameters:

1. sliding window size - num of points (p) to be considered before calculating intentions 
2. max_num_agents- maximum number of humans that can be there in the environment.
3. timeout - if no points (x,y) are publihsed on a perticular trajectory for this time then stop the prediction of this trajectory.
4. goal_file - specify static goals in the given environment.
5. length_scale - gaussian process hyperparameter.
6. sigmaf       - gaussian process hyperparameter.
7. sigman       - gaussian process hyperparameter.
