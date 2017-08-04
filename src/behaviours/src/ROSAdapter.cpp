#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include "behaviours/Rover.h"
#include "behaviours/PheromoneTrail.h"

// Include Controllers
#include "LogicController.h"

// Standard libraries
#include <vector>
#include <map>
#include <queue>

#include "Point.h"
#include "TagPoint.h"

// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>


using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator* rng;

// Create logic controller
LogicController logicController;

// Behaviours Logic Functions
void sendDriveCommand(double linearVel, double angularVel);
void openFingers(); // Open fingers to 90 degrees
void closeFingers();// Close fingers to 0 degrees
void raiseWrist();  // Return wrist back to 0 degrees
void lowerWrist();  // Lower wrist to 50 degrees
void resultHandler();

// Return pose of tag in odom
geometry_msgs::Pose2D getTagPose(apriltags_ros::AprilTagDetection tag);
float distanceToCenter();

// Stores information on all other rovers
map<string, behaviours::Rover> rovers;

// Center queue flags
queue<string> centerQueue;
bool frontOfLine = false;
bool queuedForCenter = false;
bool waitingForCenter = false;

// Center location has been updated
bool centerUpdated = false;

// Numeric Variables for rover positioning
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D currentLocationMap;
geometry_msgs::Pose2D currentLocationAverage;

geometry_msgs::Pose2D centerLocation;
geometry_msgs::Pose2D centerLocationMap;
geometry_msgs::Pose2D centerLocationOdom;

int currentMode = 0;
const float behaviourLoopTimeStep = 0.1; // time between the behaviour loop calls
const float status_publish_interval = 1;
const float heartbeat_publish_interval = 2;
const float waypointTolerance = 0.1; //10 cm tolerance.

// used for calling code once but not in main
bool initilized = false;

float linearVelocity = 0;
float angularVelocity = 0;

float prevWrist = 0;
float prevFinger = 0;

Result result;

std_msgs::String msg;


geometry_msgs::Twist velocity;
char host[128];
string publishedName;
char prev_state_machine[128];

// Publishers
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher infoLogPublisher;
ros::Publisher driveControlPublish;
ros::Publisher heartbeatPublisher;
ros::Publisher pheromoneTrailPublish;
ros::Publisher roverPublisher;

// Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber mapSubscriber;
ros::Subscriber pheromoneTrailSubscriber;
ros::Subscriber roverSubscriber;


// Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer publish_heartbeat_timer;
ros::Timer roverPublishTimer;

// records time for delays in sequanced actions, 1 second resolution.
time_t timerStartTime;

// An initial delay to allow the rover to gather enough position data to 
// average its location.
unsigned int startDelayInSeconds = 8;
float timerTimeElapsed = 0;

//Transforms
tf::TransformListener *tfListener;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mapHandler(const nav_msgs::Odometry::ConstPtr& message);
void roverHandler(const behaviours::Rover& message);
void pheromoneTrailHandler(const behaviours::PheromoneTrail& message);
void behaviourStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);
void publishRoverTimerEventHandler(const ros::TimerEvent&);
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);

// Converts the time passed as reported by ROS (which takes Gazebo simulation rate into account) into milliseconds as an integer.
long int getROSTimeInMilliSecs();

int main(int argc, char **argv) {
  
  gethostname(host, sizeof (host));
  string hostname(host);

  if (argc >= 2) {
    publishedName = argv[1];
    cout << "Welcome to the world of tomorrow " << publishedName
         << "!  Behaviour turnDirectionule started." << endl;
  } else {
    publishedName = hostname;
    cout << "No Name Selected. Default is: " << publishedName << endl;
  }

  // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
  ros::init(argc, argv, (publishedName + "_BEHAVIOUR"), ros::init_options::NoSigintHandler);
  ros::NodeHandle mNH;

  // Register the SIGINT event handler so the node can shutdown properly
  signal(SIGINT, sigintEventHandler);

  joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
  modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
  targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
  odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
  mapSubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, mapHandler);
  pheromoneTrailSubscriber = mNH.subscribe("/pheromones", 10, pheromoneTrailHandler);
  roverSubscriber = mNH.subscribe("/rovers", 10, roverHandler);

  message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(mNH, (publishedName + "/sonarLeft"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(mNH, (publishedName + "/sonarCenter"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(mNH, (publishedName + "/sonarRight"), 10);

  status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
  stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
  fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);
  wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);
  infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);
  driveControlPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/driveControl"), 10);
  heartbeatPublisher = mNH.advertise<std_msgs::String>((publishedName + "/behaviour/heartbeat"), 1, true);
  pheromoneTrailPublish = mNH.advertise<behaviours::PheromoneTrail>("/pheromones", 10, true);
  roverPublisher = mNH.advertise<behaviours::Rover>("/rovers", 10, true);

  publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
  stateMachineTimer = mNH.createTimer(ros::Duration(behaviourLoopTimeStep), behaviourStateMachine);
  roverPublishTimer = mNH.createTimer(ros::Duration(1), publishRoverTimerEventHandler);
  publish_heartbeat_timer = mNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

  message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
  sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

  tfListener = new tf::TransformListener();
  std_msgs::String msg;
  msg.data = "Log Started";
  infoLogPublisher.publish(msg);

  stringstream ss;
  ss << "Rover start delay set to " << startDelayInSeconds << " seconds";
  msg.data = ss.str();
  infoLogPublisher.publish(msg);

  timerStartTime = time(0);

  ros::spin();

  return EXIT_SUCCESS;
}


// This is the top-most logic control block organised as a state machine.
// This function calls the dropOff, pickUp, and search controllers.
// This block passes the goal location to the proportional-integral-derivative
// controllers in the abridge package.
void behaviourStateMachine(const ros::TimerEvent&) {
  std_msgs::String stateMachineMsg;

  // time since timerStartTime was set to current time
  timerTimeElapsed = time(0) - timerStartTime;

  // init code goes here. (code that runs only once at start of
  // auto mode but wont work in main goes here)
  if (!initilized) {
    if (timerTimeElapsed > startDelayInSeconds) {
      centerLocation.x = 1.2 * cos(currentLocation.theta);
      centerLocation.y = 1.2 * sin(currentLocation.theta);
      
      // Set arena size depending on the number of rovers in arena
      logicController.SetArenaSize(rovers.size());

      // initialization has run
      initilized = true;
    } else {
      return;
    }

  }


  // Robot is in automode
  if (currentMode == 2 || currentMode == 3) {

    cout << "Rover: " << publishedName << endl;
    cout << "timerTimeElapsed: " << timerTimeElapsed << endl;

    if(distanceToCenter() > 0.75) {
      centerUpdated = false;
    }

    if (logicController.layPheromone()) {
      behaviours::PheromoneTrail trail;
      Point pheromone_location_point = logicController.getTargetLocation();

      geometry_msgs::Pose2D pheromone_location;
      pheromone_location.x = pheromone_location_point.x - centerLocation.x;
      pheromone_location.y = pheromone_location_point.y - centerLocation.y;

      trail.waypoints.push_back(pheromone_location);
      pheromoneTrailPublish.publish(trail);

      
      cout << "================================================================" << endl;
      // Return to allow pheromone trail handler to receive the pheromone location
      return;
    }

    //TODO: this just sets center to 0 over and over and needs to change
    Point centerOdom;
    centerOdom.x = centerLocation.x;
    centerOdom.y = centerLocation.y;
    centerOdom.theta = centerLocation.theta;
    logicController.SetCenterLocationOdom(centerOdom);

    //update the time used by all the controllers
    logicController.SetCurrentTimeInMilliSecs( getROSTimeInMilliSecs() );

    //ask logic controller for the next set of actuator commands
    result = logicController.DoWork();

    bool wait = false;

    //if a wait behaviour is thrown sit and do nothing untill logicController is ready
    if (result.type == behavior) {
      if (result.b == wait) {
        wait = true;
      }
    }

    //do this when wait behaviour happens
    if (wait) {
      sendDriveCommand(0.0,0.0);
      std_msgs::Float32 angle;

      angle.data = prevFinger;
      fingerAnglePublish.publish(angle);
      cout << "sendDriveCommand(0, 0) fingerAngle: " << angle.data;
      angle.data = prevWrist;
      cout << " wristAngle " << angle.data << endl;
      wristAnglePublish.publish(angle);
    }

    //normally interpret logic controllers actuator commands and deceminate them over the appropriate ROS topics
    else {

      sendDriveCommand(result.pd.left,result.pd.right);
      cout << "sendDriveCommand(" << result.pd.left << ", " << result.pd.right << ")" << endl;

      std_msgs::Float32 angle;
      if (result.fingerAngle != -1) {
        angle.data = result.fingerAngle;
        fingerAnglePublish.publish(angle);
        prevFinger = result.fingerAngle;
        cout << "fingerAngle: " << angle.data << endl;
      }
      if (result.wristAngle != -1) {
        angle.data = result.wristAngle;
        wristAnglePublish.publish(angle);
        prevWrist = result.wristAngle;
        cout << "wristAngle: " << angle.data << endl;
      }
    }

    //publishHandeling here
    //logicController.getPublishData() suggested;


    //adds a blank space between sets of debugging data to easly tell one tick from the next
    cout << endl;

    cout << "================================================================" << endl; //you can remove or comment this out it just gives indication something is happening to the log file
  }

  // mode is NOT auto
  else {
    // publish current state for the operator to see
    stateMachineMsg.data = "WAITING";
  }


  // publish state machine string for user, only if it has changed, though
  if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0) {
    stateMachinePublish.publish(stateMachineMsg);
    sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
  }
}

void sendDriveCommand(double left, double right)
{
  velocity.linear.x = left,
      velocity.angular.z = right;

  // publish the drive commands
  driveControlPublish.publish(velocity);
}

/*************************
 * ROS CALLBACK HANDLERS *
 *************************/

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {
  CPFAState cpfa_state = logicController.GetCPFAState();
  bool ignored_tag = false;

  // Number of resource tags
  int num_tags = 0;

  if (message->detections.size() > 0) {
    vector<TagPoint> tags;

    for (int i = 0; i < message->detections.size(); i++) {


      TagPoint loc;
      loc.id = message->detections[i].id;
      
      if (loc.id == 0) {
        num_tags++;
      }

      geometry_msgs::PoseStamped tagPose = message->detections[i].pose;
      loc.x = tagPose.pose.position.x;
      loc.y = tagPose.pose.position.y;
      loc.z = tagPose.pose.position.z;
      //loc.theta =
      
      // While in a travel state ignore resource tags
      switch(cpfa_state) {

        case return_to_nest:
        case travel_to_search_site:
        case set_target_location:
        case start_state:
          {
            if(loc.id == 0 && !ignored_tag)
            {
              ignored_tag = true;
              cout << "ROSAdapter: targetHandler -> ignored a 0 tag!" << endl << endl;
            }

            if (loc.id == 0)
            {
              break;
            }
          }

        default:
          {
            if(loc.id == 0 && distanceToCenter() < 1.2) 
            {
              CPFAState cpfa_state = logicController.GetCPFAState();
              if (cpfa_state == sense_local_resource_density) 
              {
                  CPFASearchType cpfa_search_type = logicController.GetCPFASearchType();
                  if (cpfa_search_type == random_search) 
                  {
                      logicController.SetCPFAState(search_with_uninformed_walk);
                  } else 
                  {
                    logicController.SetCPFAState(search_with_informed_walk);        
                  }
              break;
            }
            tags.push_back(loc);
            }
          }

      }

      //if ((cpfa_state == return_to_nest || cpfa_state == travel_to_search_site || cpfa_state == set_target_location || cpfa_state == start_state) && loc.id == 0 ) {

        //if(!ignored_tag) {
          //ignored_tag = true;
          //cout << "ROSAdapter: targetHandler -> ignored a 0 tag!" << endl << endl;
        //}

        //continue;
      //} else {
        //tags.push_back(loc);
      //}
    }

    logicController.SetAprilTags(tags);

    if(cpfa_state == sense_local_resource_density) {
      logicController.senseLocalResourceDensity(num_tags);
    }

  }

}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
  currentMode = message->data;
  sendDriveCommand(0.0, 0.0);
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {

  logicController.SetSonarData(sonarLeft->range, sonarCenter->range, sonarRight->range);

}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
  //Get (x,y) location directly from pose
  currentLocation.x = message->pose.pose.position.x;
  currentLocation.y = message->pose.pose.position.y;

  //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  currentLocation.theta = yaw;

  linearVelocity = message->twist.twist.linear.x;
  angularVelocity = message->twist.twist.angular.z;


  Point currentLoc;
  currentLoc.x = currentLocation.x;
  currentLoc.y = currentLocation.y;
  currentLoc.theta = currentLocation.theta;
  logicController.SetPositionData(currentLoc);
  logicController.SetVelocityData(linearVelocity, angularVelocity);
}

void mapHandler(const nav_msgs::Odometry::ConstPtr& message) {
  //Get (x,y) location directly from pose
  currentLocationMap.x = message->pose.pose.position.x;
  currentLocationMap.y = message->pose.pose.position.y;

  //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  currentLocationMap.theta = yaw;

  linearVelocity = message->twist.twist.linear.x;
  angularVelocity = message->twist.twist.angular.z;

  Point currentLoc;
  currentLoc.x = currentLocation.x;
  currentLoc.y = currentLocation.y;
  currentLoc.theta = currentLocation.theta;
  logicController.SetPositionData(currentLoc);
  logicController.SetMapVelocityData(linearVelocity, angularVelocity);
}

void roverHandler(const behaviours::Rover& msg) {
    behaviours::Rover prevMsg = rovers[msg.name];
    rovers[msg.name] = msg;

    // Rover location should be in same frame as this rover
    rovers[msg.name].currentLocation.x += centerLocation.x;
    rovers[msg.name].currentLocation.y += centerLocation.y;

    // Center queue status has changes for this rover
    if(msg.queuedForCenter != prevMsg.queuedForCenter) {

        if(msg.queuedForCenter) {
            // Rover is now waiting
            ROS_INFO_STREAM(publishedName << "CPFA: Entered " << msg.name << " into centerQueue");
            centerQueue.push(msg.name);
        } else {
            // Rover has just finished dropped off its block
            ROS_INFO_STREAM(publishedName << "CPFA: Received " << msg.name << " popping!");
            centerQueue.pop();
        }
    }
}


void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message) {
  if (currentMode == 0 || currentMode == 1) {
    sendDriveCommand(abs(message->axes[4]) >= 0.1 ? message->axes[4] : 0, abs(message->axes[3]) >= 0.1 ? message->axes[3] : 0);
  }
}

void pheromoneTrailHandler(const behaviours::PheromoneTrail& message) {
    // Framework implemented for pheromone waypoints, but we are only using
    // the first index of the trail momentarily for the pheromone location
    behaviours::PheromoneTrail trail = message;

    // Adjust pheromone location to account for center location
    trail.waypoints[0].x += centerLocation.x;
    trail.waypoints[0].y += centerLocation.y;

    vector<Point> pheromone_trail;

    for(int i = 0; i < trail.waypoints.size(); i++) {
      Point waypoint(trail.waypoints[i].x, trail.waypoints[i].y, trail.waypoints[i].theta);
      pheromone_trail.push_back(waypoint);
    }

    logicController.insertPheromone(pheromone_trail);
    return;
}

void publishStatusTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "online";
  status_publisher.publish(msg);
}

void publishRoverTimerEventHandler(const ros::TimerEvent&) {
    // Contains information on this rover
    behaviours::Rover msg;

    // Rover name
    msg.name = publishedName;

    // Relative pose to center 
    msg.currentLocation.x = currentLocation.x - centerLocation.x;
    msg.currentLocation.y = currentLocation. y - centerLocation.y;
    msg.currentLocation.theta = currentLocation.theta;

    msg.queuedForCenter = queuedForCenter;

    roverPublisher.publish(msg);
}

void sigintEventHandler(int sig) {
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "";
  heartbeatPublisher.publish(msg);
}

geometry_msgs::Pose2D getTagPose(apriltags_ros::AprilTagDetection tag) {
    // Transforms pose of tag in camera_link to odom
    geometry_msgs::PoseStamped tagPoseOdom;
    string x = "";

    try { //attempt to get the transform of the camera frame to odom frame.
        tfListener->waitForTransform(publishedName + "/camera_link", publishedName + "/odom", ros::Time::now(), ros::Duration(1.0));
        tfListener->transformPose(publishedName + "/odom", tag.pose, tagPoseOdom);
    }

    catch(tf::TransformException& ex) {
        ROS_INFO("Received an exception trying to transform a point from \"camer_link\" to \"odom\": %s", ex.what());
        x = "Exception thrown " + (string)ex.what();
        std_msgs::String msg;
        stringstream ss;
        ss << "Exception in getTagPose() " + (string)ex.what();
        msg.data = ss.str();
        infoLogPublisher.publish(msg);
    }

    geometry_msgs::Pose2D newCenterLocation;
    newCenterLocation.x = tagPoseOdom.pose.position.x;
    newCenterLocation.y = tagPoseOdom.pose.position.y;

    return newCenterLocation;
}


long int getROSTimeInMilliSecs()
{
  // Get the current time according to ROS (will be zero for simulated clock until the first time message is recieved).
  ros::Time t = ros::Time::now();

  // Convert from seconds and nanoseconds to milliseconds.
  return t.sec*1e3 + t.nsec/1e6;
  
}

float distanceToCenter() {
    // Returns the distance of the rover to the nest
    return hypot(centerLocation.x - currentLocation.x, centerLocation.y - currentLocation.y);
}

