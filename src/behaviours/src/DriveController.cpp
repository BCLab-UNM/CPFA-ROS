#include "DriveController.h"

using namespace std;
DriveController::DriveController() {

  fastVelPID.SetConfiguration(fastVelConfig());
  fastYawPID.SetConfiguration(fastYawConfig());

  slowVelPID.SetConfiguration(slowVelConfig());
  slowYawPID.SetConfiguration(slowYawConfig());

  constVelPID.SetConfiguration(constVelConfig());
  constYawPID.SetConfiguration(constYawConfig());


}

DriveController::~DriveController() {}
	
void DriveController::Reset() {
	cout<<"driver controller: reset..."<<endl;
  waypoints.clear();
  cout<<"stateMachineState="<<stateMachineState<<endl;
  if (stateMachineState == STATE_MACHINE_ROTATE || stateMachineState == STATE_MACHINE_SKID_STEER) {
    stateMachineState = STATE_MACHINE_WAYPOINTS;
  }
  cout<<"stateMachineState="<<stateMachineState<<endl;
}

Result DriveController::DoWork() {
 cout <<"CPFAStatus: DriveController::DoWork()"<<endl;
 
  if(result.type == behavior) {
    if(result.b == noChange) {
		cout<<"behavior, no change"<<endl;
      //if drive controller gets a no change command it is allowed to continue its previouse action
      //normally this will be to follow waypoints but it is not specified as such.
    } else if(result.b == wait) {
		cout<<"wait..."<<endl;
      //do nothing till told otherwise
      left = 0.0;
      right = 0.0;
      stateMachineState = STATE_MACHINE_WAITING;

    }
  } else if(result.type == precisionDriving) {
    //interpret input result as a precision driving command
    cout<<"precision driving 1..."<<endl;
    stateMachineState = STATE_MACHINE_PRECISION_DRIVING;

  } else if(result.type == waypoint) {
	  cout<<"waypoint..."<<endl;
    //interpret input result as new waypoints to add into the queue
    ProcessData();

  }

  switch(stateMachineState) {


  //Handlers and the final state of STATE_MACHINE are the only parts allowed to call INTERUPT
  //This should be done as little as possible. I Suggest to Use timeouts to set control bools false.
  //Then only call INTERUPT if bool switches to true.
  case STATE_MACHINE_PRECISION_DRIVING: {
    cout<<"precision driving 2..."<<endl;
    ProcessData();
    break;
  }

    //Handles route planning and navigation as well as makeing sure all waypoints are valid.
  case STATE_MACHINE_WAYPOINTS: {
    cout<<"state machine: waypoints"<<endl;
    bool tooClose = true;
    while (!waypoints.empty() && tooClose) {
		cout<<"waypoints is not empty and too close"<<endl;
      if (hypot(waypoints.back().x-currentLocation.x, waypoints.back().y-currentLocation.y) < waypointTolerance) {
		  cout<<"too close..."<<endl;
        waypoints.pop_back();
      }
      else {
		  cout<<"not too close"<<endl;
        tooClose = false;
      }
    }
    cout<<"Drive: waypoints size="<<waypoints.size()<<endl;
    if (waypoints.empty()) {
		cout<<"waypoint is empty..."<<endl;
      stateMachineState = STATE_MACHINE_WAITING;
      cout<<"stateMachineState: waiting"<<endl;
      result.type = behavior;
      interupt = true;
      return result;
    }
    else {
      stateMachineState = STATE_MACHINE_ROTATE;
      cout<<"stateMachineState: rotate"<<endl;
      
      //fall through on purpose
    }


  }
    // Calculate angle between currentLocation.theta and waypoints.front().theta
    // Rotate left or right depending on sign of angle
    // Stay in this state until angle is minimized
  case STATE_MACHINE_ROTATE: {
    cout<<"ROTATE..."<<endl;
    waypoints.back().theta = atan2(waypoints.back().y - currentLocation.y, waypoints.back().x - currentLocation.x);
    // Calculate the diffrence between current and desired heading in radians.
    float errorYaw = angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta);

    result.pd.setPointVel = 0.0;
    result.pd.setPointYaw = waypoints.back().theta;
    // If angle > rotateOnlyAngleTolerance radians rotate but dont drive forward.
    cout <<"currentLocation.theta="<<currentLocation.theta<<endl;
    cout<<"waypoints.back().theta="<<waypoints.back().theta<<endl;
    cout<<"rotateOnlyAngleTolerance="<<rotateOnlyAngleTolerance<<endl;
    if (fabs(angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta)) > rotateOnlyAngleTolerance) {
      // rotate but dont drive.
      cout<<"rotate but dont drive"<<endl;
      if (result.PIDMode == FAST_PID) {
        fastPID(0.0,errorYaw, result.pd.setPointVel, result.pd.setPointYaw);
      }
      break;
    } else {
      // move to differential drive step
      cout<<"move to differential drive step..."<<endl;
      stateMachineState = STATE_MACHINE_SKID_STEER;
      //fall through on purpose.
    }
  }
    // Calculate angle between currentLocation.x/y and waypoints.back().x/y
    // Drive forward
    // Stay in this state until angle is at least PI/2
  case STATE_MACHINE_SKID_STEER: {
	  cout<<"skid steer..."<<endl;
    // calculate the distance between current and desired heading in radians
    float errorYaw = angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta);

    result.pd.setPointYaw = waypoints.back().theta;

    // goal not yet reached drive while maintaining proper heading.
    if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(waypoints.back().y - currentLocation.y, waypoints.back().x - currentLocation.x))) < M_PI_2) {
      // drive and turn simultaniously
      result.pd.setPointVel = searchVelocity;
      if (result.PIDMode == FAST_PID){
        fastPID(searchVelocity - linearVelocity,errorYaw, result.pd.setPointVel, result.pd.setPointYaw);
      }
    }
    // goal is reached but desired heading is still wrong turn only
    else if (fabs(angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta)) > finalRotationTolerance) {
      // rotate but dont drive
      result.pd.setPointVel = 0.0;
      if (result.PIDMode == FAST_PID){
        fastPID(0.0,errorYaw, result.pd.setPointVel, result.pd.setPointYaw);
      }
    }
    else {
      // stopno change
      left = 0.0;
      right = 0.0;

      // move back to transform step
      stateMachineState = STATE_MACHINE_WAYPOINTS;
      cout<<"Reach goal..."<<endl;
    }

    break;
  }

  default: {
    break;
  }


  }

  result.pd.right = right;
  result.pd.left = left;

  return result;

}

bool DriveController::ShouldInterrupt() {

  if (interupt) {
    interupt = false;
    return true;
  }
  else {
    return false;
  }
}

bool DriveController::HasWork() {

}



void DriveController::ProcessData()
{
	
	cout <<"1 Driver: result.wpts.waypoints size="<<result.wpts.waypoints.size()<<endl;
    for(int i=0; i<result.wpts.waypoints.size(); i++){
		cout<<i<<".x= "<<result.wpts.waypoints[i].x<<endl;
		}
	
  if (result.type == waypoint) {
	  cout<<"driver, waypoint..."<<endl;
    result.type = behavior;
    result.b = noChange;

    if(result.reset) {
		cout<<"reset result..."<<endl;
      waypoints.clear();
    }

    if (!result.wpts.waypoints.empty()) {
		cout<<"D: wpts.waypoint is not empty..."<<endl;
      waypoints.insert(waypoints.end(),result.wpts.waypoints.begin(), result.wpts.waypoints.end());
      stateMachineState = STATE_MACHINE_WAYPOINTS;
    }
  }
  else if (result.type == precisionDriving) {
    if (result.PIDMode == FAST_PID){
		cout<<"fast PID"<<endl;
      float vel = result.pd.cmdVel -linearVelocity;
      float setVel = result.pd.cmdVel;
      fastPID(vel,result.pd.cmdAngularError, setVel, result.pd.setPointYaw);
    }
    else if (result.PIDMode == SLOW_PID) {
		cout<<"slow pid"<<endl;
      //will take longer to reach the setPoint but has less chanse of an overshoot
      float vel = result.pd.cmdVel -linearVelocity;
      float setVel = result.pd.cmdVel;
      slowPID(vel,result.pd.cmdAngularError, setVel, result.pd.setPointYaw);
    }
    else if (result.PIDMode == CONST_PID) {
		cout<<"const pid..."<<endl;
      float vel = result.pd.cmdVel - linearVelocity;
      float angular = result.pd.cmdAngular - angularVelocity;
      constPID(vel, angular ,result.pd.setPointVel, result.pd.setPointYaw);
    }
  }
  cout <<"2 Driver: result.wpts.waypoints size="<<result.wpts.waypoints.size()<<endl;
    for(int i=0; i<result.wpts.waypoints.size(); i++){
		cout<<i<<".x= "<<result.wpts.waypoints[i].x<<endl;
		}
}


void DriveController::fastPID(float errorVel, float errorYaw , float setPointVel, float setPointYaw) {

  float velOut = fastVelPID.PIDOut(errorVel, setPointVel);
  float yawOut = fastYawPID.PIDOut(errorYaw, setPointYaw);

  int left = velOut - yawOut;
  int right = velOut + yawOut;

  int sat = 255;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}

  this->left = left;
  this->right = right;
}

void DriveController::slowPID(float errorVel,float errorYaw, float setPointVel, float setPointYaw) {

  float velOut = slowVelPID.PIDOut(errorVel, setPointVel);
  float yawOut = slowYawPID.PIDOut(errorYaw, setPointYaw);

  int left = velOut - yawOut;
  int right = velOut + yawOut;

  int sat = 255;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}

  this->left = left;
  this->right = right;
}

void DriveController::constPID(float erroVel,float constAngularError, float setPointVel, float setPointYaw) {

  float velOut = fastVelPID.PIDOut(erroVel, setPointVel);
  float yawOut = fastYawPID.PIDOut(constAngularError, setPointYaw);

  int left = velOut - yawOut;
  int right = velOut + yawOut;

  int sat = 255;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}

  this->left = left;
  this->right = right;
}


void DriveController::SetVelocityData(float linearVelocity,float angularVelocity) {
  this->linearVelocity = linearVelocity;
  this->angularVelocity = angularVelocity;
}




PIDConfig DriveController::fastVelConfig() {
  PIDConfig config;

  config.Kp = 140;
  config.Ki = 10;
  config.Kd = 0.8;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 320; //gives 127 pwm at 0.4 commandedspeed
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::fastYawConfig() {
  PIDConfig config;

  config.Kp = 100;
  config.Ki = 10;
  config.Kd = 14;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/2;
  config.errorHistLength = 4;
  config.alwaysIntegral = false;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 0;
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::slowVelConfig() {
  PIDConfig config;

  config.Kp = 100;
  config.Ki = 8;
  config.Kd = 1.1;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/2;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 320; //gives 127 pwm at 0.4 commandedspeed
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::slowYawConfig() {
  PIDConfig config;

  config.Kp = 70;
  config.Ki = 16;
  config.Kd = 10;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/4;
  config.errorHistLength = 4;
  config.alwaysIntegral = false;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 0;
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/6;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::constVelConfig() {
  PIDConfig config;

  config.Kp = 140;
  config.Ki = 10;
  config.Kd = 0.8;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/2;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 320; //gives 127 pwm at 0.4 commandedspeed
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::constYawConfig() {
  PIDConfig config;

  config.Kp = 100;
  config.Ki = 5;
  config.Kd = 1.2;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/4;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 120;
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.6;

  return config;

}
CPFAState DriveController::GetCPFAState() 
{
  return cpfa_state;
}

void DriveController::SetCPFAState(CPFAState state) {
  cpfa_state = state;
  result.cpfa_state = state;
}

