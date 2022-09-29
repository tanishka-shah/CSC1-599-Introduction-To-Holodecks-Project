# CSC1-599-Introduction-To-Holodecks-Project

Objectives:

1. An algorithm whose objective is to implement zone strategy to identify which dispatchers will dispatch FLSs for a particular point cloud while minimizing flight distance. The algorithm implemented will be a clustering algorithm like k-means clustering algorithm. There are two methods that will be implemented for static illumination: One, where an equal number of FLSs will be dispatched by each dispatcher and the formation will be done in the center of the workspace and then moving the formation to the desired location. Second, where zones will be marked and number of FLSs dispatching will depend on the clustering algorithm, in the second algorithm the formation will be formed in the desired location. Here, both algorithms will aim to generate a balance between distance traveled and execution time. 

2. An optimization algorithm (Soft computing algorithms like gradient descent, evolutionary algorithms will be considered for optimization) that will consider the shortlisted candidate algorithms from the first deliverable that minimize distance, fine tuning this algorithm by setting up thresholds for other parameters like rendition time, latency time and collision avoidance. Comparing the performance based on flight conflicts and a method to reduce them. 

3. Simulation of the proposed algorithms to get a visual representation of the algorithm's performance. A Matlab simulation to demonstrate the dispatch of FLSs and the rose illumination. And simulation of a tweaked algorithm considering collision avoidance in AirSim/Gazebo.

4. Considering the force and torque generated from a haptic interaction. Figure out the affected FLSs and the displacement of each of them. Calculating the altered points in the point cloud. The distance each displaced FLSs should cover with what velocity to achieve the next point cloud formation with minimum latency.

5. A research paper that describes the implementation of algorithms and their tradeoff. 

