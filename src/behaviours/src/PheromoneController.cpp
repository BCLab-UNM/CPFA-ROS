#include "PheromoneController.h"

using namespace std;

PheromoneController::PheromoneController()
{
}

PheromoneController::~PheromoneController()
{
}

void PheromoneController::Reset()
{
	cout<<"DensityStatus: reset pheromone controller..."<<endl;
  targetHeld = true;//qilu 12/2017
  sense_local_density_completed = false;//qilu 12/2017
  drive_to_pheromone =false; //qilu 12/2017
  total_resource = 0;
  sense_local_resource_density = false;
    rotateAngle =0;
    detect_resource_angle.clear();
	  
	  num_resource_detected.clear();
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
	cout<<"DensityStatus CPFAStatus: PheromoneController::DoWork()"<<endl;
  cout<<"DensityStatus: targetHeld="<<targetHeld<<endl;
  //if (targetHeld)
  if(!sense_local_resource_density)
  {
    //sense_local_density_completed = false;
    //targetHeld = false;
    time_searching = current_time/1e3;//no reference. should be removed. qilu 12/2017
    //cout<<"Pheromone: time_searching="<<time_searching<<endl;
    startAngle = current_location.theta*180/M_PI;
    previousAngle = startAngle;
    total_resource = 0;
    //cout<<"AngleStatus: startAngle="<<startAngle<<endl;
  }
  
  currentAngle = current_location.theta*180/M_PI;
  //cout<<"AngleStatus: currentAngle = "<<currentAngle<<endl;
  
  diffAngle = currentAngle - previousAngle;
  previousAngle = currentAngle;
  cout<<"1. DensityStatus: diffAngle = "<<diffAngle<<endl;
  if (diffAngle < -1)
  {
	  diffAngle += 360;
	  }
  
  //cout<<"2. AngleStatus: diffAngle = "<<diffAngle<<endl;
  rotateAngle += diffAngle;
  
  //rotateAngle =  current_location.theta*180/M_PI - startAngle;
  cout<<"DensityStatus: rotateAngle="<<rotateAngle<<endl;
  //cout<<"AngleStatus: 2. rotateAngle="<<rotateAngle<<endl;
  int time_elapsed = current_time/1e3 - time_searching;
  cout<<"DensityStatus: sense_local_density_completed="<<sense_local_density_completed<<endl;
  cout<<"DensityStatus: drive_to_pheromone="<<drive_to_pheromone<<endl;
  if (!sense_local_density_completed && !drive_to_pheromone)
  {
    //if (time_elapsed >target_search_time)
    if(rotateAngle >= 320)
    {
		cout<<"CPFAStatus: DensityStatus: sense completed...# of tags="<<total_resource<<endl;
		//cout<<"AngleStatus: DensityStatus: current_location.theta="<<current_location.theta*180/M_PI<<endl;
		int firstIdx =0;
	    int max_num_tags =num_resource_detected[0];
	    bool flag = false;
	    
	    for(int i = 1; i < detect_resource_angle.size(); i++)
        {
			cout<<"DensityStatus: first angle="<<detect_resource_angle[firstIdx]<< ", idx="<<firstIdx<<endl;
			cout<<"DensityStatus: angle="<<detect_resource_angle[i]<<" , idx="<<i<<endl;
			diffAngle = detect_resource_angle[i]-detect_resource_angle[firstIdx];
			if(diffAngle<0)
			{
				diffAngle += 360;
			}
			if(diffAngle > 40)
			{
				cout<<"DensityStatus: 1. max_num_tags="<<max_num_tags<<endl;
				total_resource += max_num_tags;
				max_num_tags = num_resource_detected[i];
				firstIdx = i;
				flag = false;
			}
			else
			{
				if(num_resource_detected[i]> max_num_tags)
				{
					cout<<"DensityStatus: 2. num_resource_detected["<<i<<"]="<<num_resource_detected[i]<<endl;
					max_num_tags = num_resource_detected[i];
					
				}
				flag = true;
			}
	    }
	    if(flag)
	    {
			total_resource += max_num_tags;
			}
	  cout<<"LayStatus: the final density="<<total_resource<<endl;
	  detect_resource_angle.clear();
	  num_resource_detected.clear();
	  cout<<"DensityStatus: num_resource_detected size="<<num_resource_detected.size()<<endl;
      result.type = behavior;
      //result.b = COMPLETED;
      result.b = nextProcess;
      result.reset = true;
      sense_local_density_completed = true;
      sense_local_resource_density = false;
      rotateAngle =0;
      targetHeld = true;
      resource_density = total_resource;
    }
    else
    { 
		cout<<"DensityStatus: Sensing...."<<endl;
		 //cout<<"CPFAStatus: result.wristAngle="<<result.wristAngle<<endl;
		 //cout <<"AngleStatus: DensityStatus: Pheromone: time_elapsed="<<time_elapsed<<endl;
		 //cout<<"Pheromone: current_time="<<current_time/1e3<<endl;
		 cout<<"DensityStatus: current_location.theta="<<current_location.theta*180/M_PI<<endl;
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
	  cout <<"CPFAStatus: dirve to pheromone..."<<endl;
	  selected_pheromone.x = 2.0;
	  selected_pheromone.y = 2.0;
	  cout<<"selected_pheromone.x="<<selected_pheromone.x<<endl;
	  cout<<"current_location.x="<<current_location.x<<endl;
    if (hypot(selected_pheromone.x - current_location.x, selected_pheromone.y - current_location.y) < 0.15) 
    {
	  drive_to_pheromone= false;
	  sense_local_density_completed  = false;
	  cout <<"CPFAStatus: Reached pheromone waypoint..."<<endl;
      result.type = behavior;
      result.b = COMPLETED;
      //targetHeld =true;
      cout <<"result.wpts.waypoints size="<<result.wpts.waypoints.size()<<endl;
      for(int i=0; i<result.wpts.waypoints.size(); i++)
      {
		cout<<i<<".x= "<<result.wpts.waypoints[i].x<<endl;
		}
    
    } 
    else 
    {
	  cout <<"CPFAStatus: SF: travel to pheromone_waypoint, "<<selected_pheromone.x<<endl;
      result.type = waypoint;
      result.PIDMode = FAST_PID;
      result.wpts.waypoints.insert(result.wpts.waypoints.begin(), selected_pheromone);
      cout <<"result.wpts.waypoints size="<<result.wpts.waypoints.size()<<endl;
      for(int i=0; i<result.wpts.waypoints.size(); i++)
      {
		cout<<i<<".x= "<<result.wpts.waypoints[i].x<<endl;
		}
    }
  }
  
  return result;
}

bool PheromoneController::SenseCompleted()
{
	return sense_local_density_completed;
	}
bool PheromoneController::ShouldInterrupt()
{
  cout<<"pheromone controller should interrupt..."<<endl;
  //qilu 12/2017
  //ProcessData(); 
  if(sense_local_density_completed)
  {
    return true;
  }
  else
  {
    return false;
  }
  
}

bool PheromoneController::HasWork()
{
	cout<<"Pheromone has work...True"<<endl;
  bool has_work = true;
  
  return has_work;
  
}


void PheromoneController::SetTagData(std::vector<Tag> tags)
{
  //cout<<"DensityStatus: PheromoneController::SetTagData"<<endl;
  int target_count = 0;
  int idx = 0;
 
  for (int i = 0; i < tags.size(); i++) 
  {
     if (tags[i].getID() == 0) 
     {
		if((tags[i].getPositionX()>=-0.015 || tags[i].getPositionX()<=-0.03) && tags[i].getPositionY()<=0.04 && tags[i].getPositionZ()>=0.15)//ignore the picked target
		{
		   cout<<"DensityStatus: tags[i].getPositionX()="<<tags[i].getPositionX()<<endl;
           target_count++;
	    }
      }
   }
   cout<<"DensityStatus: sense ...."<<target_count<<endl;
   //resource_density += target_count;//qilu 12/2017
   //if(target_count>0)
   //{
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



