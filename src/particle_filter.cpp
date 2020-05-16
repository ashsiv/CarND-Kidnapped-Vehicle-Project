/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 * Modified on May 15, 2020
 * Author: Aswin Sivakumar
 */

//Inlcudes
#include "particle_filter.h"
#include "helper_functions.h"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

using namespace std;
using std::string;
using std::vector;
using std::normal_distribution;
using std::discrete_distribution;

//DEBUG
int DEBUG=0;

//Random generator
std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles
  
  double std_x,std_y,std_theta;
  std_x     = std[0];
  std_y     = std[1];
  std_theta = std[2];
  
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);
  
  for(int i=0;i<num_particles;i++)
  {
    Particle particle;
    particle.id     =i;
    particle.x      =dist_x(gen);
    particle.y      =dist_y(gen);
    particle.theta  =dist_theta(gen);
    particle.weight =1.0;
    particles.push_back(particle);
    weights.push_back(1.0);
    if(DEBUG)
    {
        if(i==0)
        {
    	  cout<<i<<"\t"<<x<<"\t"<<y<<"\t"<<theta<<endl;
          cout<<i<<"\t"<<particle.x<<"\t"<<particle.y<<"\t"<<particle.theta<<endl;
        }
    }
    
  }
    
  is_initialized = true;
  return;
  
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

   for(int i=0;i<num_particles;i++)
  {
     Particle particle;
     if(fabs(yaw_rate)<0.0001)
     {
       particles[i].x = particles[i].x + ( (velocity*delta_t) * (cos( particles[i].theta)) );
       particles[i].y = particles[i].y + ( (velocity*delta_t) * (sin( particles[i].theta)) ); 
     }
     else
     {
       particles[i].x = particles[i].x + ( (velocity/yaw_rate) * (   sin( particles[i].theta + (yaw_rate*delta_t) ) - sin(particles[i].theta) ) );
       particles[i].y = particles[i].y + ( (velocity/yaw_rate) * ( - cos( particles[i].theta + (yaw_rate*delta_t) ) + cos(particles[i].theta) ) );
       particles[i].theta = particles[i].theta + (yaw_rate*delta_t);
     } 
       normal_distribution<double> dist_x(particles[i].x,         std_pos[0]);
       normal_distribution<double> dist_y(particles[i].y,         std_pos[1]);
       normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);
       
       particles[i].x     =dist_x(gen);
       particles[i].y     =dist_y(gen);
       particles[i].theta =dist_theta(gen);     
     
       if(DEBUG)
    		{
              if(i==0)
              {
                cout<<"Predicted"<<endl;
                cout<<i<<"\t"<<particles[i].x<<"\t"<<particles[i].y<<"\t"<<particles[i].theta<<endl;
              }
    		}
   }
  

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

   // Step 1: For each transformed observation, find the predicted measurement that is the closest
   for (unsigned int p=0; p < observations.size();p++)
   {
     double minimum;
     int pred_id;
     //Initialize minimum and pred_id
     minimum =sqrt( pow((observations[p].x-predicted[0].x),2) + pow((observations[p].y-predicted[0].y),2) );
     pred_id = predicted[0].id;
     for (unsigned int m=0; m<predicted.size();m++)
      {
      	double dist;
        dist = sqrt( pow((observations[p].x-predicted[m].x),2) + pow((observations[p].y-predicted[m].y),2) );
        if(dist<minimum)
        {
          minimum = dist;
          pred_id = predicted[m].id;
        }
     }
   // Step 2: Assign the observed measurement to the closest landmark
   observations[p].id = pred_id;
  
   }

  return;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  
   // Screening the landmarks based on sensor range
   
  
   for (int i=0;i<num_particles;i++)
   {
    vector<LandmarkObs> f_landmarks;
   //STEP 1: For each particle, filter the landmarks using sensor range (from the particle)
     for (unsigned int j=0;j<map_landmarks.landmark_list.size();j++)
     {
       double dist;
       dist = sqrt( pow((particles[i].x-map_landmarks.landmark_list[j].x_f),2) + pow((particles[i].y-map_landmarks.landmark_list[j].y_f),2) );
       if(dist<sensor_range)
       { 
         LandmarkObs lndmrk;
         lndmrk.x  = map_landmarks.landmark_list[j].x_f;
         lndmrk.y  = map_landmarks.landmark_list[j].y_f;
         lndmrk.id = map_landmarks.landmark_list[j].id_i;
         f_landmarks.push_back(lndmrk);
       }
     }
   
    if(DEBUG)
    {
     cout<<"Number of Map Landmarks"     <<map_landmarks.landmark_list.size()<<endl;
     cout<<"Number of Filtered Landmarks"<<f_landmarks.size()<<endl; 
    }
  
    // Step 2: Transform the observations to map coordinates with respect to the particle
    vector<LandmarkObs> T_obs;
    for (unsigned int j=0;j<observations.size();j++)
       {
           LandmarkObs transformed_obs;  
           transformed_obs.x  =  particles[i].x + (observations[j].x * cos(particles[i].theta)) - (sin(particles[i].theta)*observations[j].y);
           transformed_obs.y  =  particles[i].y + (observations[j].x * sin(particles[i].theta)) + (cos(particles[i].theta)*observations[j].y);
           transformed_obs.id = observations[j].id;
           T_obs.push_back(transformed_obs);
           if(DEBUG)
            {
              if(j==5)
              {
                cout<<"Transformed obs 0: "<<T_obs[j].x<<endl; 
              } 
            }
       }
   
    // Step 3:Associate transformed observations to filtered landmarks for each particle
    dataAssociation(f_landmarks,T_obs);
    vector<int> a;
    vector<double> sx; 
    vector<double> sy;
    for (unsigned int j=0;j<T_obs.size();j++)
      {
            a.push_back(T_obs[j].id);
           sx.push_back(T_obs[j].x);
           sy.push_back(T_obs[j].y);
      }
    SetAssociations(particles[i], a,sx,sy);   

   // Step 4: Calculate weights 
    double sigma_x,sigma_y;
    sigma_x = std_landmark[0];
    sigma_y = std_landmark[1];
    double factor;
    factor = (1/ ( 2*3.14*sigma_x*sigma_y));
    double prob=1.0;

    for (unsigned int k=0;k<T_obs.size();k++)
       { 
          for (unsigned int l=0;l<f_landmarks.size();l++)
          {
            if(DEBUG)
    		{
            cout<<i<<"\t f_landmark\t"<<f_landmarks[l].id<<"\t"<<f_landmarks[l].x<<endl;
            cout<<i<<"\t observations\t"<<T_obs[k].id<<"\t"<< T_obs[k].x<<endl;
            }
            if(f_landmarks[l].id == T_obs[k].id)
            {
              
               prob = prob * ( factor * exp( -( (pow((f_landmarks[l].x - T_obs[k].x),2)/(2*sigma_x*sigma_x)) +
                                                (pow((f_landmarks[l].y - T_obs[k].y),2)/(2*sigma_y*sigma_y)) )));
            }
          }
       }
    particles[i].weight = prob;
     if(DEBUG)
    		{
    cout<<i<<"\t Weight\t"<<particles[i].weight<<endl;
     }
    weights[i] = particles[i].weight;
 }
  
  //normalize the weights
  double sum_weight=0.0;
  for(int y=0;y<num_particles;y++)
  {
    sum_weight =particles[y].weight;
    
  }

  for(int y=0;y<num_particles;y++)
  {
     particles[y].weight = particles[y].weight/sum_weight;
     weights[y] = particles[y].weight;
    
  }
  
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  vector<double> wts;
  std::random_device rd;
  std::mt19937 gen(rd());
  for(int y=0;y<num_particles;y++)
  {
    wts.push_back(particles[y].weight);
    
  }
   vector<Particle> rp;
  
   discrete_distribution<int> distribution(wts.begin(),wts.end());
   for (int i=0;i<num_particles;i++)
   {
        rp.push_back(particles[distribution(gen)]);
   }
   particles=rp;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
