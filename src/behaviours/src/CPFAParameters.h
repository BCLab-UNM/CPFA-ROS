#ifndef CPFA_PARAMETERS_H
#define CPFA_PARAMETERS_H

struct CPFAParameters {
  const float random_seed = -1;
  //***For no obst.
  /*const float probability_of_switching_to_searching = 0.2;
  const float probability_of_returning_to_nest = 0.05;
  // In radians
  const float uninformed_search_variation = 0.7; //range [0, pi]
  // Lower causes it to decay slower
  const float rate_of_informed_search_decay = 0.1;
  // Reduce these to increase probability
  const float rate_of_following_site_fidelity = 6; //0.3; original is 0.3; range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_laying_pheromone = 8; //range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_pheromone_decay = 0.1; //range [0, 1];
  */
  //***For obst.
  /*const float probability_of_switching_to_searching = 0.2;
  const float probability_of_returning_to_nest = 0.005;
  // In radians
  const float uninformed_search_variation = 0.4; //range [0, pi]
  // Lower causes it to decay slower
  const float rate_of_informed_search_decay = 0.1;
  // Reduce these to increase probability
  const float rate_of_following_site_fidelity = 10; //0.3; original is 0.3; range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_laying_pheromone = 14; //range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_pheromone_decay = 0.1; //range [0, 1];
   */
   
 //***For obst.
  /*const float probability_of_switching_to_searching = 0.5;
  const float probability_of_returning_to_nest = 0.005;
  // In radians
  const float uninformed_search_variation = 1.0; //range [0, pi]
  // Lower causes it to decay slower
  const float rate_of_informed_search_decay = 0.1;
  // Reduce these to increase probability
  const float rate_of_following_site_fidelity = 14; //0.3; original is 0.3; range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_laying_pheromone = 18; //range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_pheromone_decay = 0.1; //range [0, 1];*/
  
   //***For obst.
  const float probability_of_switching_to_searching = 0.1;
  const float probability_of_returning_to_nest = 0.05;
  // In radians
  const float uninformed_search_variation = 0.3; //range [0, pi]
  // Lower causes it to decay slower
  const float rate_of_informed_search_decay = 0.1;
  // Reduce these to increase probability
  const float rate_of_following_site_fidelity = 10.0; //0.3; original is 0.3; range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_laying_pheromone = 10.0; //range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_pheromone_decay = 0.2; //range [0, 1];
};

#endif

