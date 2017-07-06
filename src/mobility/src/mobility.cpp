#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

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
#include "mobility/Rover.h"
#include "mobility/PheromoneTrail.h"

// Include Controllers
#include "PickUpController.h"
#include "DropOffController.h"
#include "CPFASearchController.h"

// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

// Standard Libraries
#include <utility>
#include <map>
#include <queue>


using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator* rng;

// Create controllers
PickUpController pickUpController;
DropOffController dropOffController;
CPFASearchController searchController;

// Mobility Logic Functions
void sendDriveCommand(double linearVel, double angularVel);
void openFingers(); // Open fingers to 90 degrees
void closeFingers();// Close fingers to 0 degrees
void raiseWrist();  // Return wrist back to 0 degrees
void lowerWrist();  // Lower wrist to 50 degrees
void mapAverage();  // constantly averages last 100 positions from map

// Return pose of tag in odom
geometry_msgs::Pose2D getTagPose(apriltags_ros::AprilTagDetection tag);
float distanceToCenter();

// Numeric Variables for rover positioning
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D currentLocationMap;
geometry_msgs::Pose2D currentLocationAverage;
geometry_msgs::Pose2D goalLocation;

geometry_msgs::Pose2D centerLocation;
geometry_msgs::Pose2D centerLocationMap;
geometry_msgs::Pose2D centerLocationOdom;
geometry_msgs::Pose2D previousCenterLocation;

// Stores information on all other rovers
map<string, mobility::Rover> rovers;

queue<string> centerQueue;
bool frontOfLine = false;
bool queuedForCenter = false;
bool waitingForCenter = false;

bool centerUpdated = false;
int totalTimeSearching = 0;
int currentMode = 0;
float mobilityLoopTimeStep = 0.1; // time between the mobility loop calls
float status_publish_interval = 1;
float killSwitchTimeout = 10;
bool targetDetected = false;
bool targetCollected = false;
float heartbeat_publish_interval = 2;

// Set true when the target block is less than targetDist so we continue
// attempting to pick it up rather than switching to another block in view.
bool lockTarget = false;

// Failsafe state. No legitimate behavior state. If in this state for too long
// return to searching as default behavior.
bool timeOut = false;

// Set to true when the center ultrasound reads less than 0.14m. Usually means
// a picked up cube is in the way.
bool blockBlock = false;

// central collection point has been seen (aka the nest)
bool centerSeen = false;

// Set true when we are insie the center circle and we need to drop the block,
// back out, and reset the boolean cascade.
bool reachedCollectionPoint = false;

// used for calling code once but not in main
bool init = false;

// used to remember place in mapAverage array
int mapCount = 0;

// How many points to use in calculating the map average position
const unsigned int mapHistorySize = 500;

// An array in which to store map positions
geometry_msgs::Pose2D mapLocation[mapHistorySize];

bool avoidingObstacle = false;

float searchVelocity = 0.2; // meters/second

std_msgs::String msg;

// state machine states
#define STATE_MACHINE_TRANSFORM 0
#define STATE_MACHINE_ROTATE 1
#define STATE_MACHINE_SKID_STEER 2
#define STATE_MACHINE_PICKUP 3
#define STATE_MACHINE_DROPOFF 4

int stateMachineState = STATE_MACHINE_TRANSFORM;

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
ros::Publisher transformPublish;;

// Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber mapSubscriber;
ros::Subscriber pheromoneTrailSubscriber;
ros::Subscriber roverSubscriber;;


// Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer targetDetectedTimer;
ros::Timer publish_heartbeat_timer;
ros::Timer roverPublishTimer;

// records time for delays in sequanced actions, 1 second resolution.
time_t timerStartTime;

// An initial delay to allow the rover to gather enough position data to 
// average its location.
unsigned int startDelayInSeconds = 10;
float timerTimeElapsed = 0;

//Transforms
tf::TransformListener *tfListener;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mapHandler(const nav_msgs::Odometry::ConstPtr& message);
void roverHandler(const mobility::Rover& message);
void pheromoneTrailHandler(const mobility::PheromoneTrail& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void targetDetectedReset(const ros::TimerEvent& event);
void publishRoverTimerEventHandler(const ros::TimerEvent&);
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);

int main(int argc, char **argv) {
    gethostname(host, sizeof (host));
    string hostname(host);

    // instantiate random number generator
    rng = new random_numbers::RandomNumberGenerator();

    //set initial random heading
    //goalLocation.theta = rng->uniformReal(0, 2 * M_PI);

    //select initial search position 50 cm from center (0,0)
    //goalLocation.x = 0.5 * cos(goalLocation.theta+M_PI);
    //goalLocation.y = 0.5 * sin(goalLocation.theta+M_PI);

    centerLocationOdom.x = 0;
    centerLocationOdom.y = 0;

    for (int i = 0; i < 100; i++) {
        mapLocation[i].x = 0;
        mapLocation[i].y = 0;
        mapLocation[i].theta = 0;
    }

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName
             << "!  Mobility turnDirectionule started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    searchController = CPFASearchController(publishedName);

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    // Register the SIGINT event handler so the node can shutdown properly
    signal(SIGINT, sigintEventHandler);

    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
    mapSubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, mapHandler);
    pheromoneTrailSubscriber = mNH.subscribe("/pheromones", 10, pheromoneTrailHandler);
    roverSubscriber = mNH.subscribe("/rovers", 10, roverHandler);

    status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
    stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
    fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);
    wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);
    infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);
    driveControlPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/driveControl"), 10);
    heartbeatPublisher = mNH.advertise<std_msgs::String>((publishedName + "/mobility/heartbeat"), 1, true);
    pheromoneTrailPublish = mNH.advertise<mobility::PheromoneTrail>("/pheromones", 10, true);
    roverPublisher = mNH.advertise<mobility::Rover>("/rovers", 10, true);
    transformPublish = mNH.advertise<geometry_msgs::PoseStamped>("/transformedPose", 10, true);

    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    targetDetectedTimer = mNH.createTimer(ros::Duration(0), targetDetectedReset, true);
    roverPublishTimer = mNH.createTimer(ros::Duration(1), publishRoverTimerEventHandler);
    publish_heartbeat_timer = mNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);

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
void mobilityStateMachine(const ros::TimerEvent&) {

    if(distanceToCenter() > 0.75) {
        centerUpdated = false;
    }

    std_msgs::String stateMachineMsg;
    float rotateOnlyAngleTolerance = 0.4;
    int returnToSearchDelay = 5;

    // calls the averaging function, also responsible for
    // transform from Map frame to odom frame.
    mapAverage();

    // Robot is in automode
    if (currentMode == 2 || currentMode == 3) {


        // time since timerStartTime was set to current time
        timerTimeElapsed = time(0) - timerStartTime;

        // init code goes here. (code that runs only once at start of
        // auto mode but wont work in main goes here)
        if (!init) {
            centerLocation.x = cos(currentLocation.theta);
            centerLocation.y = sin(currentLocation.theta);

            if (timerTimeElapsed > startDelayInSeconds) {
                // Set arena size depending on the number of rovers in arena
                int numRovers = rovers.size();
                searchController.setArenaSize(numRovers);

                // Set the location of the center circle location in the map
                // frame based upon our current average location on the map.
                centerLocationMap.x = currentLocationAverage.x;
                centerLocationMap.y = currentLocationAverage.y;
                centerLocationMap.theta = currentLocationAverage.theta;

                // initialization has run
                init = true;
            } else {
                return;
            }

        }

        // If no collected or detected blocks set fingers
        // to open wide and raised position.
        if (!targetCollected && !targetDetected) {
            // set gripper
            std_msgs::Float32 angle;

            // open fingers
            angle.data = M_PI_2;

            fingerAnglePublish.publish(angle);
            angle.data = 0;

            // raise wrist
            wristAnglePublish.publish(angle);
        }

        // Select rotation or translation based on required adjustment
        switch(stateMachineState) {

        // If no adjustment needed, select new goal
        case STATE_MACHINE_TRANSFORM: {
            stateMachineMsg.data = "TRANSFORMING";

            // If returning with a target
            if (targetCollected && !avoidingObstacle) {

                if(distanceToCenter() <= 1) {

                    if(!queuedForCenter) {
                        ROS_INFO_STREAM(publishedName << "CPFA: Enterting centerQueue");
                        queuedForCenter = true;
                    }

                    int size = centerQueue.size();
                    if(queuedForCenter && (size == 0 || size > 0 && centerQueue.front() != publishedName)) {
                        if(!waitingForCenter) {
                            waitingForCenter = true;
                            ROS_INFO_STREAM(publishedName << "CPFA: Waiting for center to open...");
                        }
                        sendDriveCommand(0, 0);
                        break;
                    } else if(!frontOfLine) {
                        frontOfLine = true;
                        ROS_INFO_STREAM(publishedName << "CPFA: Front of the line! Dropping off block...");
                    }
                }
                // calculate the euclidean distance between
                // centerLocation and currentLocation
                dropOffController.setCenterDist(hypot(centerLocation.x - currentLocation.x, centerLocation.y - currentLocation.y));
                dropOffController.setDataLocations(centerLocation, currentLocation, timerTimeElapsed);

                DropOffResult result = dropOffController.getState();

                if (result.timer) {
                    timerStartTime = time(0);
                    reachedCollectionPoint = true;
                }

                std_msgs::Float32 angle;

                if (result.fingerAngle != -1) {
                    angle.data = result.fingerAngle;
                    fingerAnglePublish.publish(angle);
                }

                if (result.wristAngle != -1) {
                    angle.data = result.wristAngle;
                    wristAnglePublish.publish(angle);
                }

                if (result.reset) {
                    timerStartTime = time(0);
                    targetCollected = false;
                    targetDetected = false;
                    lockTarget = false;
                    sendDriveCommand(0.0,0);

                    // move back to transform step
                    stateMachineState = STATE_MACHINE_TRANSFORM;
                    reachedCollectionPoint = false;;
                    centerLocationOdom = currentLocation;

                    dropOffController.reset();

                    ROS_INFO_STREAM(publishedName << "CPFA: Dropped off block! Leaving the queue...");
                    frontOfLine = false;
                    waitingForCenter = false;
                    queuedForCenter = false;
                } else if (result.goalDriving && timerTimeElapsed >= 5 ) {
                    goalLocation = result.centerGoal;
                    stateMachineState = STATE_MACHINE_ROTATE;
                    timerStartTime = time(0);
                }
                // we are in precision/timed driving
                else {
                    goalLocation = currentLocation;
                    sendDriveCommand(result.cmdVel,result.angleError);
                    stateMachineState = STATE_MACHINE_TRANSFORM;

                    break;
                }
            }
            //If angle between current and goal is significant
            //if error in heading is greater than 0.4 radians
            else if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > rotateOnlyAngleTolerance) {
                stateMachineState = STATE_MACHINE_ROTATE;
            }
            //If goal has not yet been reached drive and maintane heading
            else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
                stateMachineState = STATE_MACHINE_SKID_STEER;
            }
            //Otherwise, drop off target and select new random uniform heading
            //If no targets have been detected, assign a new goal
            else if (!targetDetected && timerTimeElapsed > returnToSearchDelay) {
                goalLocation = searchController.CPFAStateMachine(currentLocation, centerLocation);
            }

            //Purposefully fall through to next case without breaking
        }

        // Calculate angle between currentLocation.theta and goalLocation.theta
        // Rotate left or right depending on sign of angle
        // Stay in this state until angle is minimized
        case STATE_MACHINE_ROTATE: {
            stateMachineMsg.data = "ROTATING";
            // Calculate the diffrence between current and desired
            // heading in radians.
            float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);

            // If angle > 0.4 radians rotate but dont drive forward.
            if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > rotateOnlyAngleTolerance) {
                // rotate but dont drive  0.05 is to prevent turning in reverse
                sendDriveCommand(0.05, errorYaw);
                break;
            } else {
                // move to differential drive step
                stateMachineState = STATE_MACHINE_SKID_STEER;
                //fall through on purpose.
            }
        }

        // Calculate angle between currentLocation.x/y and goalLocation.x/y
        // Drive forward
        // Stay in this state until angle is at least PI/2
        case STATE_MACHINE_SKID_STEER: {
            stateMachineMsg.data = "SKID_STEER";

            // calculate the distance between current and desired heading in radians
            float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);

            // goal not yet reached drive while maintaining proper heading.
            if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
                // drive and turn simultaniously
                sendDriveCommand(searchVelocity, errorYaw/2);
            }
            // goal is reached but desired heading is still wrong turn only
            else if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1) {
                 // rotate but dont drive
                sendDriveCommand(0.0, errorYaw);
            }
            else {
                // stop
                sendDriveCommand(0.0, 0.0);
                avoidingObstacle = false;

                // move back to transform step
                stateMachineState = STATE_MACHINE_TRANSFORM;
            }

            break;
        }

        case STATE_MACHINE_PICKUP: {
            stateMachineMsg.data = "PICKUP";

            PickUpResult result;

            // we see a block and have not picked one up yet
            if (targetDetected && !targetCollected) {
                result = pickUpController.pickUpSelectedTarget(blockBlock);
                sendDriveCommand(result.cmdVel,result.angleError);
                std_msgs::Float32 angle;

                if (result.fingerAngle != -1) {
                    angle.data = result.fingerAngle;
                    fingerAnglePublish.publish(angle);
                }

                if (result.wristAngle != -1) {
                    angle.data = result.wristAngle;

                    // raise wrist
                    wristAnglePublish.publish(angle);
                }

                if (result.giveUp) {
                    targetDetected = false;
                    stateMachineState = STATE_MACHINE_TRANSFORM;
                    sendDriveCommand(0,0);
                    pickUpController.reset();

                    if(searchController.getSearchLocationType() == RANDOM){
                        searchController.setState(SEARCH_WITH_UNINFORMED_WALK);
                    }else{
                         searchController.setState(SEARCH_WITH_INFORMED_WALK);
                    }

                    goalLocation = searchController.CPFAStateMachine(currentLocation, centerLocation);

                }

                if (result.pickedUp) {
                    // Picked up block successfully. Next CPFA state will then
                    // be to return to this location using site fidelity
                    searchController.setSearchLocationType(SITE_FIDELITY);
                    searchController.setState(SET_SEARCH_LOCATION);

                    // Just picked up a block, determine if we should publish
                    // a pheromone trail
                    if(searchController.layPheromone()) {
                        
                        mobility::PheromoneTrail trail;
                        geometry_msgs::Pose2D pheromoneLocation = searchController.getTargetLocation();

                        // Move pheromoneLocation to coordinate frame relative to the current center location using dead reckoning
                        pheromoneLocation.x -= centerLocation.x;
                        pheromoneLocation.y -= centerLocation.y;

                        trail.waypoints.push_back(pheromoneLocation);

                        pheromoneTrailPublish.publish(trail);
                        ROS_INFO_STREAM(publishedName << "CPFA: Laying pheromone...");
                    }

                    pickUpController.reset();

                    // assume target has been picked up by gripper
                    targetCollected = true;
                    result.pickedUp = false;
                    stateMachineState = STATE_MACHINE_ROTATE;

                    goalLocation.theta = atan2(centerLocation.y - currentLocation.y, centerLocation.x - currentLocation.x);

                    // set center as goal position
                    goalLocation.x = centerLocation.x;
                    goalLocation.y = centerLocation.y;

                    // lower wrist to avoid ultrasound sensors
                    std_msgs::Float32 angle;
                    angle.data = 0.8;
                    wristAnglePublish.publish(angle);
                    sendDriveCommand(0.0,0);

                    return;
                }
            } else {
                stateMachineState = STATE_MACHINE_TRANSFORM;
            }

            break;
        }

        case STATE_MACHINE_DROPOFF: {
            stateMachineMsg.data = "DROPOFF";
            break;
        }

        default: {
            break;
        }

        } /* end of switch() */
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

void sendDriveCommand(double linearVel, double angularError)
{
    velocity.linear.x = linearVel,
    velocity.angular.z = angularError;

    // publish the drive commands
    driveControlPublish.publish(velocity);
}

/*************************
 * ROS CALLBACK HANDLERS *
 *************************/

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {
    CPFAState state = searchController.getState();

    // If in manual mode do not try to automatically pick up the target
    if (currentMode == 1 || currentMode == 0) return;

    // if a target is detected and we are looking for center tags
    if (message->detections.size() > 0 && !reachedCollectionPoint) {

        float cameraOffsetCorrection = 0.020; //meters;

        centerSeen = false;
        double count = 0;
        double countRight = 0;
        double countLeft = 0;
        int resourceCount = 0;



        // this loop is to get the number of center tags
        for (int i = 0; i < message->detections.size(); i++) {

            if (message->detections[i].id == 256) {

                if(!centerUpdated) {
                    centerLocation = getTagPose(message->detections[i]);
                    ROS_INFO_STREAM(publishedName << "CPFA: CenterLocation Updated x: " << centerLocation.x << " y: " << centerLocation.y);
                    centerUpdated = true;
                }

                geometry_msgs::PoseStamped cenPose = message->detections[i].pose;

                // checks if tag is on the right or left side of the image
                if (cenPose.pose.position.x + cameraOffsetCorrection > 0) {
                    countRight++;

                } else {
                    countLeft++;
                }

                centerSeen = true;
                count++;
            } else if (!targetCollected && distanceToCenter() > sqrt(2)) { 
                // If a block is seen away from the center 
                if(state == SEARCH_WITH_INFORMED_WALK || state == SEARCH_WITH_UNINFORMED_WALK){
                    searchController.setSearchLocationType(SITE_FIDELITY);
                    searchController.setState(SENSE_LOCAL_RESOURCE_DENSITY);
                    state = SENSE_LOCAL_RESOURCE_DENSITY;

                    geometry_msgs::Pose2D tagLocation = getTagPose(message->detections[i]);
                    searchController.setTargetLocation(tagLocation, centerLocation);
                    goalLocation = tagLocation;
                }
                resourceCount++;
            }
        }

        if(state == SENSE_LOCAL_RESOURCE_DENSITY){
            searchController.senseLocalResourceDensity(resourceCount);
        }

        if (centerSeen && targetCollected) {
            stateMachineState = STATE_MACHINE_TRANSFORM;
            goalLocation = currentLocation;
        }

        dropOffController.setDataTargets(count,countLeft,countRight);

        // if we see the center and we dont have a target collected
        if (centerSeen && !targetCollected) {

            float centeringTurn = 0.15; //radians
            stateMachineState = STATE_MACHINE_TRANSFORM;

            // this code keeps the robot from driving over
            // the center when searching for blocks
            if (right) {
                // turn away from the center to the left if just driving
                // around/searching.
                goalLocation.theta += centeringTurn;
            } else {
                // turn away from the center to the right if just driving
                // around/searching.
                goalLocation.theta -= centeringTurn;
            }

            // continues an interrupted search
            goalLocation = searchController.continueInterruptedSearch(currentLocation, goalLocation, centerLocation);

            targetDetected = false;
            pickUpController.reset();

            return;
        }
    }
    // end found target and looking for center tags

    // Rover needs to ignore blocks
    if(targetCollected || distanceToCenter() < sqrt(2) || state == TRAVEL_TO_SEARCH_SITE || state == RETURN_TO_NEST || state == SET_SEARCH_LOCATION) return;

    // found a target april tag and looking for april cubes;
    // with safety timer at greater than 5 seconds.
    PickUpResult result;

    if (message->detections.size() > 0 && !targetCollected && timerTimeElapsed > 5) {
        targetDetected = true;

        // pickup state so target handler can take over driving.
        stateMachineState = STATE_MACHINE_PICKUP;
        result = pickUpController.selectTarget(message);

        std_msgs::Float32 angle;

        if (result.fingerAngle != -1) {
            angle.data = result.fingerAngle;
            fingerAnglePublish.publish(angle);
        }

        if (result.wristAngle != -1) {
            angle.data = result.wristAngle;
            wristAnglePublish.publish(angle);
        }
    }
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
    currentMode = message->data;
    sendDriveCommand(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
    if ((!targetDetected || targetCollected) && (message->data > 0)) {
        // obstacle on right side
        if (message->data == 1) {
            // select new heading 0.2 radians to the left
            goalLocation.theta = currentLocation.theta + 0.6;
        }

        // obstacle in front or on left side
        else if (message->data == 2) {
            // select new heading 0.2 radians to the right
            goalLocation.theta = currentLocation.theta - 0.6;
        }

        // continues an interrupted search
        goalLocation = searchController.continueInterruptedSearch(currentLocation, goalLocation, centerLocation);

        // switch to transform state to trigger collision avoidance
        stateMachineState = STATE_MACHINE_ROTATE;

        avoidingObstacle = true;
    }

    // the front ultrasond is blocked very closely. 0.14m currently
    if (message->data == 4) {
        blockBlock = true;
    } else {
        blockBlock = false;
    }
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
}

void roverHandler(const mobility::Rover& msg) {
    mobility::Rover prevMsg = rovers[msg.name];
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

void pheromoneTrailHandler(const mobility::PheromoneTrail& message) {
    mobility::PheromoneTrail trail = message;

    // Adjust pheromone location to account for center location
    trail.waypoints[0].x += centerLocation.x;
    trail.waypoints[0].y += centerLocation.y;

    searchController.insertPheromone(trail.waypoints);
    return;
}

void publishStatusTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "online";
    status_publisher.publish(msg);
}

void publishRoverTimerEventHandler(const ros::TimerEvent&) {
    // Contains information on this rover
    mobility::Rover msg;

    // Rover name
    msg.name = publishedName;

    // Relative pose to center 
    msg.currentLocation.x = currentLocation.x - centerLocation.x;
    msg.currentLocation.y = currentLocation. y - centerLocation.y;
    msg.currentLocation.theta = currentLocation.theta;

    msg.queuedForCenter = queuedForCenter;

    roverPublisher.publish(msg);
}


void targetDetectedReset(const ros::TimerEvent& event) {
    targetDetected = false;

    std_msgs::Float32 angle;
    angle.data = 0;

    // close fingers
    fingerAnglePublish.publish(angle);

    // raise wrist
    wristAnglePublish.publish(angle);
}

void sigintEventHandler(int sig) {
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

void mapAverage() {
    // store currentLocation in the averaging array
    mapLocation[mapCount] = currentLocationMap;
    mapCount++;

    if (mapCount >= mapHistorySize) {
        mapCount = 0;
    }

    double x = 0;
    double y = 0;
    double theta = 0;

    // add up all the positions in the array
    for (int i = 0; i < mapHistorySize; i++) {
        x += mapLocation[i].x;
        y += mapLocation[i].y;
        theta += mapLocation[i].theta;
    }

    // find the average
    x = x/mapHistorySize;
    y = y/mapHistorySize;
    
    // Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    theta = theta/100;
    currentLocationAverage.x = x;
    currentLocationAverage.y = y;
    currentLocationAverage.theta = theta;


    // only run below code if a centerLocation has been set by initilization
    if (init) {
        // map frame
        geometry_msgs::PoseStamped mapPose;

        // setup msg to represent the center location in map frame
        mapPose.header.stamp = ros::Time::now();

        mapPose.header.frame_id = publishedName + "/map";
        mapPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, centerLocationMap.theta);
        mapPose.pose.position.x = centerLocationMap.x;
        mapPose.pose.position.y = centerLocationMap.y;
        geometry_msgs::PoseStamped odomPose;
        string x = "";

        try { //attempt to get the transform of the center point in map frame to odom frame.
            tfListener->waitForTransform(publishedName + "/map", publishedName + "/odom", ros::Time::now(), ros::Duration(1.0));
            tfListener->transformPose(publishedName + "/odom", mapPose, odomPose);
        }

        catch(tf::TransformException& ex) {
            ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
            x = "Exception thrown " + (string)ex.what();
            std_msgs::String msg;
            stringstream ss;
            ss << "Exception in mapAverage() " + (string)ex.what();
            msg.data = ss.str();
            infoLogPublisher.publish(msg);
        }

        // Use the position and orientation provided by the ros transform.
        //centerLocation.x = odomPose.pose.position.x; //set centerLocation in odom frame
        //centerLocation.y = odomPose.pose.position.y;


    }
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


    transformPublish.publish(tagPoseOdom);
    geometry_msgs::Pose2D newCenterLocation;
    newCenterLocation.x = tagPoseOdom.pose.position.x;
    newCenterLocation.y = tagPoseOdom.pose.position.y;

    return newCenterLocation;
}


float distanceToCenter() {
    // Returns the distance of the rover to the nest
    return hypot(centerLocation.x - currentLocation.x, centerLocation.y - currentLocation.y);
}

