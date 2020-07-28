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

    num_particles = 1;  // Set the number of particles

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
        Particle particles_i;
        particles_i.id = i;
        particles_i.x = dist_x(gen);
        particles_i.y = dist_y(gen);
        particles_i.theta = dist_theta(gen);
        particles_i.weight = 1;

        particles.push_back(particles_i);
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
    double std_x = std_pos[0];
    double std_y = std_pos[1];
    double std_theta = std_pos[2];

    std::cout << " --------------- prediction --------------- " << std::endl;
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

        std::cout << "x: " << particles[i].x << "\ty: " << particles[i].y << "\ttheta: " << particles[i].theta << std::endl;
    }
    std::cout << " ------------------------------------------------- " << std::endl;
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

    for (int i = 0; i < observations.size(); i++)
    {
        std::cout << "Obstacle id: " << observations[i].id << "\tx: " << observations[i].x << "\ty: " << observations[i].y << std::endl;
    }


    vector<vector<LandmarkObs>> transformedObs_all_part; // vector with transformed observations for all particles

    // for every particles, transform observations from vehicle to map coordinates
    for (int p = 0; p < num_particles; p++)
    {
        double x_part = particles[p].x;
        double y_part = particles[p].y;
        double theta_part = particles[p].theta;

        vector<LandmarkObs> transformedObs_part; // vector with transformed observations for one particle
        //std::cout << "Particle #" << p << "\t\tx: " << particles[p].x << "\ty: " << particles[p].y << "\ttheta: " << particles[p].theta << std::endl;
        for (int i = 0; i < observations.size(); i++)
        {
            LandmarkObs transformedObs_part_i;   // to help constructing transformedObs vector

            double x_obs = observations[i].x;
            double y_obs = observations[i].y;

            // transform to map x coordinate
            double x_map;
            x_map = x_part + (cos(theta_part) * x_obs) - (sin(theta_part) * y_obs);

            // transform to map y coordinate
            double y_map;
            y_map = y_part + (sin(theta_part) * x_obs) + (cos(theta_part) * y_obs);
            
            transformedObs_part_i.id = i;
            transformedObs_part_i.x = x_map;
            transformedObs_part_i.y = y_map;

            transformedObs_part.push_back(transformedObs_part_i);

            std::cout << "\tObservation #" << i << "\n\tx: " << observations[i].x << "\ty: " << observations[i].y << "\t--->\t" << "x: " << transformedObs_part[i].x << "\ty: " << transformedObs_part[i].y << std::endl;
        }
        transformedObs_all_part.push_back(transformedObs_part);
    }

    double distance;
    double nearest_neighbor_dist = sensor_range;
    double nearest_neighbor_id;
    
    // associate observed landmarks to map landmarks
    for (int p = 0; p < num_particles; p++)
    {
        std::cout << "Particle #" << p << "\t\tx: " << particles[p].x << "\ty: " << particles[p].y << "\ttheta: " << particles[p].theta << std::endl;
        for (int t = 0; t < transformedObs_all_part[p].size(); t++) // for each observation of p-th particle
        {
            std::cout << "Transformed Observation: " << t << std::endl;
            for (int i = 0; i < map_landmarks.landmark_list.size(); i++)  // for each landmark
            {
                // compute distance between i-th landmark(map) and t-th observation
                distance = dist(map_landmarks.landmark_list[i].x_f, map_landmarks.landmark_list[i].y_f, transformedObs_all_part[p][t].x, transformedObs_all_part[p][t].y);

                std::cout << "distance landmark i: " << distance << std::endl;

                //if (distance < sensor_range)    // ignore landmarks that are not within sensor range (?)
                //{
                if (distance < nearest_neighbor_dist)
                {
                    nearest_neighbor_dist = distance;
                    nearest_neighbor_id = i;
                }
                //}
            }
            transformedObs_all_part[p][t].id = nearest_neighbor_id;   // assign matching landmark id to t-th observation
            std::cout << "\tObservation #" << t << "\t" << transformedObs_all_part[p][t].id << std::endl;
        }
    }

    /*
    double mu_x;
    double mu_y;
    double sig_x = std_landmark[0];
    double sig_y = std_landmark[1];
    double w;
    std::cout << " --------------- update --------------- " << std::endl;
    for (int p = 0; p < num_particles; p++) // each particle
    {
        w = 1.0;
        for (int t = 0; t < transformedObs.size(); t++) // each observation
        {
            mu_x = map_landmarks.landmark_list[transformedObs[t].id].x_f;
            mu_y = map_landmarks.landmark_list[transformedObs[t].id].y_f;

            //if (dist(particles[p].x, particles[p].y, transformedObs[t].x, transformedObs[t].y) < sensor_range) // redundant (?)
            //{

                // calculate multi-variate gaussian
                w *= (1 / (2 * M_PI * sig_x * sig_y)) * exp(-(pow((transformedObs[t].x - mu_x), 2) / (2 * sig_x * sig_x) + pow((transformedObs[t].y - mu_y), 2) / (2 * sig_y * sig_y)));
            //}
        }
        particles[p].weight = w;
        std::cout << "w: " << particles[p].weight << std::endl;
    }
    std::cout << " ------------------------------------------------- " << std::endl;
    */
}

void ParticleFilter::resample() {

    /* Resample particles with replacement with probability proportional
    * to their weight.
    */

    std::vector<Particle> particlesCopy = particles;

    std::default_random_engine gen;

    double wSum = 0;
    double wMax = 0;
    // calculate max weight to be used to resample particles
    for (int i = 0; i < num_particles; i++) {
        if (particles[i].weight > wSum)
            wMax = particles[i].weight;
        wSum += particles[i].weight;
    }

    /*Resampling*/
    std::uniform_real_distribution<double> uniform_distribution_resample_real(0, 2 * wMax);
    std::uniform_int_distribution<int> uniform_distribution_resample_int(0, num_particles);
    int index = uniform_distribution_resample_int(gen);
    double beta = 0;
    for (int i = 0; i < num_particles; i++) {
        beta += uniform_distribution_resample_real(gen);
        while (particles[index].weight < beta) {
            beta -= particles[index].weight;
            index = (index + 1) % num_particles;
        }
        particlesCopy[i] = particles[index];
    }
    particles = particlesCopy;
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