/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 *
 * Edited: Murilo Pinheiro
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    /*
    * Set the number of particles. 
    * Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties
    * from GPS and IMU) and all weights to 1. 
    * NOTE: Consult particle_filter.h for more information about this method 
    *   (and others in this file).
    */

    num_particles = 10;  // Set the number of particles

    std::default_random_engine gen;

    double std_x, std_y, std_theta;  // GPS and IMU Standard deviations for x, y, and theta

    // Set standard deviations for x, y, and theta
    std_x = std[0];
    std_y = std[1];
    std_theta = std[2];

    // Create normal distributions for x, y and theta
    std::normal_distribution<double> dist_x(x, std_x);  // (mean,std) -> (x,std_x)
    std::normal_distribution<double> dist_y(y, std_y);  
    std::normal_distribution<double> dist_theta(theta, std_theta);

    for (int i = 0; i < num_particles; i++)
    {
        // initialize i-th particle from these normal distributions.
        particles[i].id = i;
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
        particles[i].weight = 1;
    }

    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {

    /*
    * Predict particle location based on noisy position measurement and noiseless control inputs (velocity & yaw_rate)
    * (position, velocity and yaw_rate)
    */

    std::default_random_engine gen;

    // Set standard deviations for x, y, and theta
    double std_x = std[0];
    double std_y = std[1];
    double std_theta = std[2];


    for (int i = 0; i < num_particles; i++)
    {
        std::normal_distribution<double> dist_x(particles[i].x, std_x);  // (mean,std) -> (x,std_x)
        std::normal_distribution<double> dist_y(particles[i].y, std_y);
        std::normal_distribution<double> dist_theta(particles[i].theta, std_theta);

        double x_0 = dist_x(gen);
        double y_0 = dist_y(gen);
        double theta_0 = dist_theta(gen);

        particles[i].x = x_0 + (velocity / yaw_rate) * (sin(theta_0 + yaw_rate * delta_t) - sin(theta_0));
        particles[i].y = y_0 + (velocity / yaw_rate) * (cos(theta_0) - cos(theta_0 + yaw_rate * delta_t));
        particles[i].theta = theta_0 + yaw_rate * delta_t;
    }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /*
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const vector<LandmarkObs> &observations, const Map &map_landmarks) {

   /*
   * Update the weights of each particle using a mult-variate Gaussian distribution.
   * Observations are given in vehicle frame, so it requires transformation to map frame.
   * Once transformation is performed, each observation is assigned to a landmark.
   * In the multi-variate Gaussian, x,y are observations in map coordinates and mu_x,mu_y are the coordinates of nearest landmarks.
   */

    vector<LandmarkObs> transformedObs; // vector with transformed observations
    LandmarkObs transformedObs_i;   // to help constructing transformedObs vector

    // for every particles, transform observations from vehicle to map coordinates
    for (int p = 0; p < num_particles; p++)
    {
        double x_part = particles[p].x;
        double y_part = particles[p].y;
        double theta_part = particles[p].theta;

        for (int i = 0; i < observations.size(); i++)
        {
            double x_obs = observations[i].x;
            double y_obs = observations[i].y;

            // transform to map x coordinate
            double x_map;
            x_map = x_part + (cos(theta_part) * x_obs) - (sin(theta_part) * y_obs);

            // transform to map y coordinate
            double y_map;
            y_map = y_part + (sin(theta_part) * x_obs) + (cos(theta_part) * y_obs);
            
            transformedObs_i.id_i = i;
            transformedObs_i.x = x_map;
            transformedObs_i.y = y_map;

            transformedObs.push_back(transformedObs_i)
        }
    }

    double distance;
    double nearest_neighbor_dist = sensor_range;
    double nearest_neighbor_id;

    // associate observed landmarks to map landmarks
    for (int t = 0; t < transformedObs.size(); t++) // for each observation
    {
        for (int i = 0; i < map_landmarks.size(); i++)  // for each landmark
        {
            // compute distance between i-th landmark(map) and t-th observation
            distance = dist(map_landmarks[i].x_f, map_landmarks[i].y_f, transformedObs[t].x, transformedObs[t].y);

            //if (distance < sensor_range)    // ignore landmarks that are not within sensor range (?)
            //{
                if (distance < nearest_neighbor_dist)
                {
                    nearest_neighbor_dist = distance;
                    nearest_neighbor_id = i;
                }
            //}
        }
        transformedObs[t].id = i;   // assign matching landmark id to t-th observation
    }

    double mu_x;
    double mu_y;
    double sig_x = std_landmark[0];
    double sig_y = std_landmark[1];
    double w;
    for (int p = 0; p < num_particles; p++) // each particle
    {
        w = 1;
        for (int t = 0; t < transformedObs.size(); t++) // each observation
        {
            mu_x = map_landmarks[transformedObs[t].id].x;
            mu_y = map_landmarks[transformedObs[t].id].y;

            //if (dist(particles[p].x, particles[p].y, transformedObs[t].x, transformedObs[t].y) < sensor_range) // redundant (?)
            //{

                w *= (1 / (2 * M_PI * sig_x * sig_y)) * exp(-(pow((transformedObs[t].x - mu_x), 2) / (2 * sig_x * sig_x) + pow((transformedObs[t].y - mu_y), 2) / (2 * sig_y * sig_y)));
            //}
        }
        particles[p].weight = w;
    }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

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