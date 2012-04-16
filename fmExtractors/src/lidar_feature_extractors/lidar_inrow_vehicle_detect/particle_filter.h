#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "geometry_msgs/Point32.h"
#include "laser_geometry/laser_geometry.h"
#include "pcl/io/pcd_io.h"
#include "sensor_msgs/PointCloud.h"
#include "ros/ros.h"
#include "fmMsgs/Vector3.h"
#include "fmMsgs/vehicle_position.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/MarkerArray.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <vector>

struct Car
{
	Car()
	{
		x = 0; y = 0; theta = 0; w = 1;
	}
	Car(double X, double Y, double Theta)
	{
		x = X; y = Y; theta = Theta; w = 1;
	}
	Car(double X, double Y, double Theta, double W)
	{
		x = X; y = Y; theta = Theta; w = W;
	}
	double x;
	double y;
	double theta;
	double w;
};

class ParticleFilter
{
private:

  std::vector<Car*> particles;

  int noParticles;
  double offset_x;
  double length_x;
  double offset_y;
  double length_y;
  double max_angle;

  double noise_x;
  double noise_y;
  double noise_theta;

  double last_x;
  double last_y;
  double last_theta;

  double max_prob;

  visualization_msgs::MarkerArray particlesMarker;

  int print;

  void newParticles(double ratio);
  Car* getRandomParticle(double seed);
  void printParticles();
  void motionUpdate(const double& dx, const double& dy, const double& dtheta);
  void measurementUpdate(const sensor_msgs::PointCloud& pointCloud);
  void resampling();
  double gaussian(double mu, double sigma, double x);
  void addRandomGaussianNoise();
  fmMsgs::vehicle_position findVehicle();

public:

  ParticleFilter();
  ParticleFilter(int numberOfParticles,double len_x,double off_x,double len_y,double off_y,double max_ang, double x_noise, double y_noise, double theta_noise);

  void updateParticlesMarker(void);
  fmMsgs::vehicle_position update(const sensor_msgs::PointCloud& cloud, const double& dx, const double& dy, const double& dtheta);
  visualization_msgs::MarkerArray getParticlesMarker(void);
};

#endif /* PARTICLE_FILTER_H_ */
