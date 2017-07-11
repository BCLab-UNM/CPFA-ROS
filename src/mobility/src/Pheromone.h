#ifndef PHEROMONE_H
#define PHEROMONE_H

#include <geometry_msgs/Pose2D.h>
#include <vector>

/*****
 * Implementation of the iAnt Pheromone object used by the iAnt CPFA. iAnts build and maintain a list of these pheromone waypoint objects to use during
 * the informed search component of the CPFA algorithm.
 *****/

class Pheromone 
{

    public:

        Pheromone(const geometry_msgs::Pose2D new_location, 
                  const std::vector<geometry_msgs::Pose2D> new_trail, 
                  const ros::Time new_time, 
                  const double new_decay_rate);

        void update(const ros::Time& time);
        bool isActive();
        double getWeight();
        geometry_msgs::Pose2D getLocation();
        std::vector<geometry_msgs::Pose2D> getTrail();

    private:
        
        geometry_msgs::Pose2D location;
        std::vector<geometry_msgs::Pose2D> trail;

        ros::Time last_updated;
        double decay_rate;
        double weight;
        double threshold;
};

#endif
