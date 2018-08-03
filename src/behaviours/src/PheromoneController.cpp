#include "PheromoneController.h"

using namespace std;

PheromoneController::PheromoneController()
{
	rng = new random_numbers::RandomNumberGenerator();
  
}

PheromoneController::~PheromoneController()
{
}

void PheromoneController::Reset()
{
	//cout<<"PheromoneStatus: reset pheromone controller..."<<endl;
	//pheromones.clear(); //qilu 08/2017
  
  targetHeld = true;//qilu 12/2017
  //cout<<"TestStatus: reset sense_local_density_completed"<<endl;
  sense_local_density_completed = false;//qilu 12/2017
  drive_to_pheromone =false; //qilu 12/2017
  total_resource = 0;
  sense_local_resource_density = false;
    rotateAngle =0;
    detect_resource_angle.clear();
	  
	  num_resource_detected.clear();
	  result.lay_pheromone = false;
}

void PheromoneController::ProcessData() 
{
}


CPFAState PheromoneController::GetCPFAState() 
{
  return cpfa_state;
}

void PheromoneController::SetCPFAState(CPFAState state) 
{
  cpfa_state = state;
  result.cpfa_state = state;
}

bool PheromoneController::SensingLocalDensity()
{
	return sense_local_resource_density;	
}
void PheromoneController::DriveToPheromoneTrail()
{
	drive_to_pheromone= true;
}

Result PheromoneController::DoWork()
{
  cout<<"PheromoneStatus: PheromoneController::DoWork()"<<endl;
  //cout<<"PheromoneStatus: targetHeld="<<targetHeld<<endl;
  //if (targetHeld)
  if(!sense_local_resource_density)
  {
    //sense_local_density_completed = false;
    //targetHeld = false;
    //time_searching = current_time/1e3;//no reference. should be removed. qilu 12/2017
    //time_sensing = current_time;
    //cout<<"TestStatus: time_sensing="<<time_sensing<<endl;
    //cout<<"TestStatus: rotateAngle="<<rotateAngle<<endl;
    startAngle = current_location.theta*180/M_PI;
    previousAngle = startAngle;
    total_resource = 0;
  }
  
  currentAngle = current_location.theta*180/M_PI;
  //cout<<"TestAngle: currentAngle="<<currentAngle<<endl;
  //cout<<"TestAngle: previousAngle="<<previousAngle<<endl;
  
  diffAngle = currentAngle - previousAngle;
  //cout<<"TestAngle:diffAngle="<<diffAngle<<endl;
  
  if (diffAngle < -180)
  {
	  diffAngle += 360;
	  }
  
  previousAngle = currentAngle;
  
  rotateAngle += diffAngle;
  
  //cout<<"TestStatus: sense_local_density_completed="<<sense_local_density_completed<<endl;
  //cout<<"TestStatus: drive_to_pheromone="<<drive_to_pheromone<<endl;
  if (!sense_local_density_completed && !drive_to_pheromone)
  {
    //cout<<"TestStatus: rotateAngle="<<rotateAngle<<endl;
    if(rotateAngle >= 315)
    {
		//cout<<"PheromoneStatus: sense completed...# of tags="<<total_resource<<endl;
		int firstIdx =0;
	    int max_num_tags =num_resource_detected[0];
	    bool flag = false;
	    
	    for(int i = 1; i < detect_resource_angle.size(); i++)
        {
			//cout<<"detect_resource_angle["<<i<<"]="<<detect_resource_angle[i]<<endl;
			//cout<<"detect_resource_angle["<<firstIdx<<"]="<<detect_resource_angle[firstIdx]<<endl;
			
			diffAngle = detect_resource_angle[i]-detect_resource_angle[firstIdx];
			//cout<<"TestStatus: diffAngle="<<diffAngle<<endl;
			if(diffAngle<0)
			{
				diffAngle += 360;
			}
			if(diffAngle > 45)
			{
				//cout<<"TestStatus: max_num_tags="<<max_num_tags<<endl;
				total_resource += max_num_tags;
				max_num_tags = num_resource_detected[i];
				//cout<<"TestStatus: max_num_tags="<<max_num_tags<<endl;
				firstIdx = i;
				//cout<<"TestStatus: firstIdx="<<firstIdx<<endl;
				
				flag = false;
			}
			else
			{
				cout<<"TestStatus: num_resource_detected["<<i<<"]="<<num_resource_detected[i]<<endl;
				if(num_resource_detected[i]> max_num_tags)
				{
					max_num_tags = num_resource_detected[i];
				}
				flag = true;
			}
	    }
	    if(flag)
	    {
			total_resource += max_num_tags;
			}
	  cout<<"wpTestStatus: the final density="<<total_resource<<endl;
	  detect_resource_angle.clear();
	  num_resource_detected.clear();
	  //cout<<"PheromoneStatus: num_resource_detected size="<<num_resource_detected.size()<<endl;
      result.type = behavior;
      //result.b = COMPLETED;
      result.b = nextProcess;
      result.reset = true;
      
      sense_local_density_completed = true;
      //cout<<"TestStatus: complete sensing. sense_local_density_completed="<<sense_local_density_completed<<endl; 
      sense_local_resource_density = false;
      rotateAngle =0;
      targetHeld = true;
      resource_density = total_resource;
      //create a pheromone trail
      result.lay_pheromone = true;
     //cout<<"PheromoneStatus: currentLocation = ("<<current_location.x<<", "<<current_location.y<<")" <<endl;
      
    }
    else
    { 
		//cout<<"TestStatus: Sensing...."<<endl;
		 sense_local_resource_density = true;
		 result.wristAngle = 1.25;//lower the gripper to sense local density. qilu 12/2017
        result.type = precisionDriving;
        result.PIDMode = CONST_PID;
      
        result.pd.cmdVel = 0.0;
        result.pd.cmdAngular = 2*M_PI/target_search_time; //2 PI radians divided by seconds to choose how long it takes to spin 360 degrees
    }
    
  }
  
  if(drive_to_pheromone)
  {
	  cout <<"wpTestStatus: drive to pheromone..."<<endl;
	  
	 
	  cout<<"wpTestStatus: selected_pheromone=["<<selected_pheromone.x<<", "<<selected_pheromone.y<<"]"<<endl;
      //cout<<"PheromoneStatus: current_location.x="<<current_location.x<<endl;
	  if (hypot(selected_pheromone.x - current_location.x, selected_pheromone.y - current_location.y) < 0.15 || attemptCount>=ATTEMPT_MAX) 
	  {
		  attemptCount=0;
          drive_to_pheromone= false;
          sense_local_density_completed  = false;
          //cout <<"TestStatus: sense_local_density_completed="<<sense_local_density_completed<<endl;
          cout <<"wpTestStatus: Reached pheromone waypoint..."<<endl;
		  result.type = behavior;
		  result.b = COMPLETED;
		  /*if (attemptCount >=15)
          {
		      cout <<"wpTest: give up to pheromone wp and start to search..."<<endl;
		  }*/
	  } 
	  else if(attemptCount<ATTEMPT_MAX)
	  {
		  attemptCount++;
          cout <<"wpTestStatus: travel to pheromone_waypoint ["<<selected_pheromone.x<<", "<<selected_pheromone.y<<"]"<<endl;
          cout<<"wpTestStatus: pw attemptCount="<<attemptCount<<endl;
		  result.type = waypoint;
		  result.PIDMode = FAST_PID;
		  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), selected_pheromone);
	  }		    
  }
  return result;
}

void PheromoneController::UpdatePheromoneList()
{
  vector<Pheromone> newPheromoneList;
  //cout<<"wpTestStatus: pheromones.size()="<<pheromones.size()<<endl;
  //cout<<"PheromoneStatus: update pheromone, current_time="<<current_time<<endl;
  for(int i = 0; i < pheromones.size(); i++) 
  {
    Point pLoc = pheromones[i].getLocation();
    pheromones[i].update(current_time);
    
    if(pheromones[i].isActive()) 
    {
		//cout<<"ParameterStatus: pheromones["<<i<<"]=("<<pheromones[i].getLocation().x<<","<<pheromones[i].getLocation().y<<")"<<endl;
      newPheromoneList.push_back(pheromones[i]);
    }
    else
    {
		cout<<"wpTestStatus: the pheromone ["<<pheromones[i].getLocation().x<<","<<pheromones[i].getLocation().y<<"] is inactive"<<endl;
		}
  }
  pheromones = newPheromoneList;
}


void PheromoneController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
   }
}

void PheromoneController::SetRoverInitLocation(Point location) 
{
  roverInitLocation = location;
  cout<<"TestStatus: rover init location=["<<roverInitLocation.x<<","<<roverInitLocation.y<<"]"<<endl;
}



bool PheromoneController::SelectPheromone()
{
	//cout << "TestStatus: Selecting pheromone..." << endl;
  double maxStrength = 0.0;
  double randomWeight = 0.0;
  bool isPheromoneSet = false;
  //cout<<"TestStatus: pheromones.size()="<<pheromones.size()<<endl;
  if(pheromones.size()==0) return isPheromoneSet; //the case of no pheromone.
  
  //Calculate a maximum strength based on active pheromone weights.
  for(int i = 0; i < pheromones.size(); i++) 
  {
    if(pheromones[i].isActive()) 
    {
      maxStrength += pheromones[i].getWeight();
    }
  }
    
  // Calculate a random weight.
  randomWeight = rng->uniformReal(0.0, maxStrength);

  //Randomly select an active pheromone to follow. 
  for(int i = 0; i < pheromones.size(); i++) 
  {

    if(randomWeight < pheromones[i].getWeight()) 
    {
      //We've chosen a pheromone! 

      //SetTarget(pheromones[i].GetLocation());
      //target_location = pheromones[center_idx][i].getLocation();
      //target_location = pheromones[i].getLocation();
      
      selected_pheromone = pheromones[i].getLocation();
      cout << "wpTestStatus: selected pheromoneLocation=[" <<selected_pheromone.x << ", " << selected_pheromone.y<<"]"<< endl;
      //cout<<"TestStatus: centerLocation=["<<centerLocation.x <<", "<<centerLocation.y<<"]"<<endl;
      //cout<<"TestStatus: roverInitLocation=["<<roverInitLocation.x <<", "<<roverInitLocation.y<<"]"<<endl;
      
      selected_pheromone.x -= roverInitLocation.x;
      selected_pheromone.y -= roverInitLocation.y;
      
// TrailToFollow = pheromones[i].GetTrail();
      isPheromoneSet = true;
      //If we pick a pheromone, break out of this loop. 
      break;
    }

    //We didn't pick a pheromone! Remove its weight from randomWeight. 
    randomWeight -= pheromones[i].getWeight();
  }
  
	return isPheromoneSet = true;
}

void PheromoneController::InsertPheromone( const vector<Point> &pheromone_trail, double pheromone_decay_rate)
{
  // At this point in time we are not making a pheromone trail
  // the first index of the trail is the same position as the pheromone location
  Point new_location = pheromone_trail[0];
  //Pheromone pheromone(new_location, pheromone_trail, ros::Time::now(), rate_of_pheromone_decay);
  cout<<"wpTestStatus: create pheromone..."<<endl;
  Pheromone pheromone(new_location, pheromone_trail, current_time, pheromone_decay_rate);

  pheromones.push_back(pheromone);
  //pheromones[center_id].push_back(pheromone);
  cout << "wpTestStatus: pheromoneLocation=[" << pheromone_trail[0].x << ", " << pheromone_trail[0].y<<"]" << endl;
  
  /*for(map<int, vector<Pheromone>>::iterator it= pheromones.begin(); it!=pheromones.end(); ++it) {
                for(int i=0; i<it->second.size(); i++){
				cout<<"pheromone["<<it->first<<"]["<<i<<"]="<<it->second[i].getLocation().x<<", "<<it->second[i].getLocation().y<<endl;
			}
		}*/
  cout << "================================================================" << endl;
}


bool PheromoneController::SenseCompleted()
{
	return sense_local_density_completed;
	}
bool PheromoneController::ShouldInterrupt()
{
 //cout<<"pheromone controller should interrupt...";
  //qilu 12/2017
  //ProcessData(); 
  if(sense_local_density_completed)
  {
	 //cout<<"True"<<endl;
    return true;
  }
  else
  {
	 //cout<<"false"<<endl;
    return false;
  }
  
}

bool PheromoneController::HasWork()
{
	//cout<<"Pheromone has work...True"<<endl;
  bool has_work = true;
  
  return has_work;
  
}


void PheromoneController::SetTagData(std::vector<Tag> tags)
{
  int target_count = 0;
  int idx = 0;
 
  for (int i = 0; i < tags.size(); i++) 
  {
     if (tags[i].getID() == 0) 
     {
		if((tags[i].getPositionX()>=-0.015 || tags[i].getPositionX()<=-0.03) && tags[i].getPositionY()<=0.04 && tags[i].getPositionZ()>=0.15)//ignore the picked target
		{
		   target_count++;
	    }
      }
   }
   //resource_density += target_count;//qilu 12/2017
   //if(target_count>0)
   //{
       //cout<<"TestStatus: target_count="<<target_count<<endl;
	   num_resource_detected.push_back(target_count);
	   currentAngle = current_location.theta*180/M_PI;
	   detect_resource_angle.push_back(currentAngle);
	   //}
   
		
}	
 
int PheromoneController::GetResourceDensity()
 {
	 return resource_density;
 }
 /*double PheromoneController::getPoissonCDF(const double lambda)
{
  double sumAccumulator       = 1.0;
  double factorialAccumulator = 1.0;
  cout <<"LayStatus: local_resource_density="<<resource_density<<endl;
  for (size_t i = 1; i <= resource_density; i++) {
    factorialAccumulator *= i;
    sumAccumulator += pow(lambda, i) / factorialAccumulator;
  }

  return (exp(-lambda) * sumAccumulator);
}*/



