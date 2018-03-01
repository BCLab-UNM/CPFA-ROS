#ifndef CPFA_PARAMETERS_H
#define CPFA_PARAMETERS_H

struct CPFAParameters {
  const float random_seed = -1;

  // CPFA Parameters according to beyond pheromones
  
  //For uniform preliminary  
  /*const float probability_of_switching_to_searching = 0.03;
  const float probability_of_returning_to_nest = 0.001;
  // In radians
  const float uninformed_search_variation = 0.5; //range [0, pi]
  // Lower causes it to decay slower
  const float rate_of_informed_search_decay = 0.1666;
  // Reduce these to increase probability
  const float rate_of_following_site_fidelity = 10; //original is 0.3; range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_laying_pheromone = 10; //range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_pheromone_decay = 0.025; //range [0, 1];
  */
  //For powerlaw preliminary
  /*const float probability_of_switching_to_searching = 0.02;
  const float probability_of_returning_to_nest = 0.005;
  // In radians
  const float uninformed_search_variation = 1.0; //range [0, pi]
  // Lower causes it to decay slower
  const float rate_of_informed_search_decay = 0.1666;
  // Reduce these to increase probability
  const float rate_of_following_site_fidelity = 8; //original is 0.3; range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_laying_pheromone = 10; //range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_pheromone_decay = 0.025; //range [0, 1];
  */
  
  //For clustered preliminary; Get 41 in Uniform 
  /*const float probability_of_switching_to_searching = 0.03;
  const float probability_of_returning_to_nest = 0.01;
  // In radians
  const float uninformed_search_variation = 0.5; //range [0, pi]
  // Lower causes it to decay slower
  const float rate_of_informed_search_decay = 0.1666;
  // Reduce these to increase probability
  const float rate_of_following_site_fidelity = 1.0; //original is 0.3; range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_laying_pheromone = 3; //range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_pheromone_decay = 0.0025; //range [0, 1];
  */
  //For clustered
  const float probability_of_switching_to_searching = 0.03;//0.05;
  const float probability_of_returning_to_nest = 0.05;//0.01; //0.05;
  // In radians
  const float uninformed_search_variation = 0.5; //range [0, pi]
  // Lower causes it to decay slower
  const float rate_of_informed_search_decay = 0.1666;
  // Reduce these to increase probability
  const float rate_of_following_site_fidelity = 2.3; //0.3; original is 0.3; range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_laying_pheromone = 3; //range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_pheromone_decay = 0.01; //range [0, 1];
  
  
};

#endif

