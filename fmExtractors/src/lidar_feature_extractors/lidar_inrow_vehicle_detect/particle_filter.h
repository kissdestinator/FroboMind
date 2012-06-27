#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "geometry_msgs/Point32.h"
#include "laser_geometry/laser_geometry.h"
#include "pcl/io/pcd_io.h"
#include "sensor_msgs/PointCloud.h"
#include "ros/ros.h"
#include "fmMsgs/Vector3.h"
#include "fmMsgs/vehicle_position.h"
#include "fmMsgs/vehicle_coordinate.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/MarkerArray.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <vector>

struct Car
{
	Car()
	{
		x = 0; y = 0; theta = 0; w = 0;
	}
	Car(double X, double Y, double Theta)
	{
		x = X; y = Y; theta = Theta; w = 0;
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

  std::vector<Car> particles;

  int noParticles;

  double move_noise;
  double turn_noise;
  double measurement_noise;

  double offset_x;
  double length_x;
  double offset_y;
  double length_y;
  double max_angle;

  double max_prob;

  fmMsgs::vehicle_position last_pos;

  visualization_msgs::MarkerArray particlesMarker;

  int print;

  void newParticles(double ratio);
  void printParticles();
  void motionUpdate(const fmMsgs::vehicle_coordinate& delta_position);
  void measurementUpdate(const sensor_msgs::PointCloud& pointCloud, const nav_msgs::OccupancyGrid& map);
  void resampling();
  double gaussian(double mu, double sigma, double x);
  void addRandomGaussianNoise();
  fmMsgs::vehicle_position findVehicle();

public:

  ParticleFilter();
  ParticleFilter(int numberOfParticles,double len_x,double off_x,double len_y,double off_y,double max_ang, double measurements_noise, double movement_noise, double turning_noise);
  ~ParticleFilter();

  void updateParticlesMarker(void);
  fmMsgs::vehicle_position update(const sensor_msgs::PointCloud& pointCloud, const fmMsgs::vehicle_coordinate& delta_position, const nav_msgs::OccupancyGrid& map);
  visualization_msgs::MarkerArray getParticlesMarker(void);
  void resetParticleFilter(double off_x, double off_y);

};

#endif /* PARTICLE_FILTER_H_ */
