#ifndef PHEROMONE_H
#define PHEROMONE_H

#include <geometry_msgs/Pose2D.h>
#include <vector>

/*****
 * Implementation of the iAnt Pheromone object used by the iAnt CPFA. iAnts build and maintain a list of these pheromone waypoint objects to use during
 * the informed search component of the CPFA algorithm.
 *****/

class Pheromone {

    public:

        Pheromone(geometry_msgs::Pose2D newLocation, std::vector<geometry_msgs::Pose2D> newTrail, ros::Time newTime, double newDecayRate);

        void Update(ros::Time time);
        void Deactivate();
        bool IsActive();
        double GetWeight();
        geometry_msgs::Pose2D GetLocation();
        std::vector<geometry_msgs::Pose2D> GetTrail();

    private:
        
        geometry_msgs::Pose2D location;
        std::vector<geometry_msgs::Pose2D> trail;

        ros::Time lastUpdated;
        double decayRate;
        double weight;
        double threshold;
};

#endif
