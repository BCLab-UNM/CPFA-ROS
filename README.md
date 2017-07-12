# CPFA-ROS

Please make sure that you have read all of the [documentation](https://github.com/BCLab-UNM/CPFA-ROS) and have set up all of the software regarding the Swarmathon as well as having read the [paper](https://www.cs.unm.edu/~melaniem/Publications_files/Hecker_Beyond_Pheromones_Swarm_Intelligence_2015.pdf)  that contains the algorithm.

This repository is a ROS (Robot Operating System) controller framework for the Swarmie robots used in the [NASA Swarmathon](http://www.nasaswarmathon.com), a national swarm robotics competition. This particular framework is a ROS implementation of the CPFA (central-place foraging algorithm) developed for [iAnt robot swarms](http://swarms.cs.unm.edu) at the [University of New Mexico](http://www.unm.edu/).


### Running the CPFA simulation

Assuming you have installed everything as the Swarmathon-ROS repository has instructed and you have read the paper, then we can begin. The CPFA relies a set of parameters that can be modified depending on the resource distribution that will be used for a run. 

In the ` ~/rover_workspace/CPFA_parameters/` directory you will see a set of files with a `.yaml` extension. Each file corresponds to the appropriate resource distribution and have been hand tuned to have paramters that will work for each of those distributions, albeit not optimally. 

You will see all the parameters that the paper describes as the paramters evolved by the GA. 

![Alt text](https://github.com/BCLab-UNM/CPFA-ROS/blob/documentation/readmeImages/parameters.png "CPFA Parameters")

To start the simulation you will run the `~/rover_workspace/run.sh` script like normal to start up the GUI. Selecting the uniform distribution option will run the   `uniform.yaml` parameters file, the clustered option will run the `clustered.yaml` paramteters file, and the power law option will run the `powerlaw.yaml` file. 

![Alt text](https://github.com/BCLab-UNM/CPFA-ROS/blob/documentation/readmeImages/gui1.png "Opening Screen")

If you decide to choose a custom world, then you will also be allows to pick any of the `.yaml` files available by clicking on the CPFA button as well as make any custom paramter files with custom paramters as long as they follow the same format as the default parameter files.

![Alt text](https://github.com/BCLab-UNM/CPFA-ROS/blob/documentation/readmeImages/gui2.png "Custom Paramters Widget")

#### ROS Paramters

The `.yaml`files will get loaded into the rosparam namespace as soon as the simulation starts. If you would like to see a list of all the CPFA parameters, then run the following command:

```
rosparam list | grep CPFA
```

This will list out all of the rosparameters that have been set and filters out any of them containing CPFA. You can then use
`rosparam get <parameter name>` to see the value of the paramter to confirm. Some other useful ways to use this if you want to modify the
parameters in real time, you can use `rosparam set <paramter name>` to dynamically modify the parameter and in the code you can reset that value. 

![Alt text](https://github.com/BCLab-UNM/CPFA-ROS/blob/documentation/readmeImages/rosparamlist.png "rosparam list")
