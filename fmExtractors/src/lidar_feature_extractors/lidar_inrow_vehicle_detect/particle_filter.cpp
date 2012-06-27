/****************************************************************************
 # Particle Filter
 # Copyright (c) 2012 Jeppe Pedersen <jepe009@student.sdu.dk>
 #
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 # THE SOFTWARE.
 #
 *****************************************************************************
 # File: particle_filter.cpp
 # Purpose: particle filter for placing the vehicle between rows from LIDAR scan.
 # Project: Field Robot Event 2012
 # Author: Jeppe Pedersen <jepe009@student.sdu.dk>
 # Created: 20 march, 2012, Source written
 ****************************************************************************/

#define X_RANGE 2.00
#define Y_RANGE 1.50
#define MIN_VALID_MEASUREMENTS 25
#define MAX_DISTANCE 0.10
#define MAX_ANGLE M_PI/12 // 15 degrees

#include "particle_filter.h"

ParticleFilter::ParticleFilter()
{

}

ParticleFilter::ParticleFilter(int numberOfParticles,double len_x,double off_x,double len_y,double off_y,double max_ang, double measurements_noise, double movement_noise, double turning_noise)
{
	noParticles = numberOfParticles;
	length_y = len_x;
	offset_y = off_x;
	length_x = len_y;
	offset_x = off_y;
	max_angle = max_ang;

	measurement_noise = measurements_noise;
	move_noise = movement_noise;
	turn_noise = turning_noise;

	print = 1;

	max_prob = 1;

	time_t sec;
	time(&sec);
	srand(uint(sec));
	for (int i = 0; i < noParticles; i++)
	{
		double x = (double)rand()/double(RAND_MAX) * length_x + offset_x - 0.5 * length_x;
		srand(uint(x*1000.0+i*1000));
		double y = (double)rand()/double(RAND_MAX) * length_y + offset_y - 0.5 * length_y;
		srand(uint(y*1000.0+i*2000));
		double theta = (double)rand()/double(RAND_MAX) * max_angle - 0.5 * max_angle;
		if (theta < 0)
			theta += 2*M_PI;
		srand(uint(theta*1000.0+i*3000));

		particles.push_back(Car(x,y,theta));
	}

	std::cout << "ParticleFilter created:" << std::endl;

	ROS_INFO("Particles: %d, len_x: %.3f, off_x: %.3f, len_y: %.3f, off_y: %.3f, max_ang: %.3f, meas_noise: %.3f, move_noise: %.3f, turn_noise: %.3f", numberOfParticles, len_x, off_x, len_y, off_y, max_ang, measurements_noise, movement_noise, turning_noise);



}

ParticleFilter::~ParticleFilter()
{
}

visualization_msgs::MarkerArray ParticleFilter::getParticlesMarker(void)
{
	return particlesMarker;
}

void ParticleFilter::updateParticlesMarker(void)
{
	particlesMarker.markers.clear();

	for (int i = 0; i < noParticles; i++)
	{
		double prob = (particles[i].w / max_prob);
		geometry_msgs::Quaternion pose =  tf::createQuaternionMsgFromYaw(particles[i].theta);
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.ns = "particles";
		marker.id = i;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = particles[i].x;
		marker.pose.position.y = particles[i].y;
		marker.pose.position.z = 0;
		marker.pose.orientation = pose;
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.1;
		marker.color.a = 0.7;
		marker.color.r = prob;
		marker.color.g = 0.0;
		marker.color.b = 1-prob;

		particlesMarker.markers.push_back(marker);
	}

	visualization_msgs::Marker marker;

	geometry_msgs::Quaternion pose = tf::createQuaternionMsgFromYaw(last_pos.position.th);
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = last_pos.position.x;
	marker.pose.position.y = last_pos.position.y;
	marker.pose.position.z = 0;
	marker.pose.orientation = pose;
	marker.scale.x = 0.4;
	marker.scale.y = 0.6;
	marker.scale.z = 0.5;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	particlesMarker.markers.push_back(marker);
}

double ParticleFilter::gaussian(double mu, double sigma, double x)
{
	return exp(-(pow(mu - x,2.0) / pow(sigma,2.0) / 2));
}

void ParticleFilter::addRandomGaussianNoise()
{
	  boost::mt19937 rng(time(0));

	  boost::normal_distribution<> nd_x(0.0, move_noise);
	  boost::normal_distribution<> nd_y(0.0, move_noise);
	  boost::normal_distribution<> nd_theta(0.0, turn_noise);

	  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_x(rng, nd_x);
	  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_y(rng, nd_y);
	  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_theta(rng, nd_theta);

	  for (int i = 0; i < noParticles; ++i)
	  {
		particles[i].x += var_x();
	    particles[i].y += var_y();
	    particles[i].theta += var_theta();
	  }
}

void ParticleFilter::printParticles()
{
	if (print)
	{
		std::cout << "Particles:" << std::endl;

		for (int i = 0; i < noParticles; i++)
			std::cout << "x: " << particles[i].x << " y: " << particles[i].y << " Theta: " << particles[i].theta << " W: " << particles[i].w << std::endl;

		std::cout << "End" << std::endl;
	}
}

void ParticleFilter::motionUpdate(const fmMsgs::vehicle_coordinate& delta_position)
{
	boost::mt19937 rng(time(0));

	boost::normal_distribution<> nd_x(0.0, move_noise);
	boost::normal_distribution<> nd_y(0.0, move_noise);
	boost::normal_distribution<> nd_theta(0.0, turn_noise);

	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_x(rng, nd_x);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_y(rng, nd_y);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_theta(rng, nd_theta);

	for (int i = 0; i < noParticles; i++)
	{

		particles[i].y += sqrt(pow(delta_position.x,2.0)+pow(delta_position.y,2.0))*sin(particles[i].theta) + var_y();
		particles[i].x += sqrt(pow(delta_position.x,2.0)+pow(delta_position.y,2.0))*cos(particles[i].theta) + var_x();

		particles[i].theta -= delta_position.th + var_theta();
		if (particles[i].theta < 0)
			particles[i].theta += 2*M_PI;
		else if (particles[i].theta > 2*M_PI)
			particles[i].theta -= 2*M_PI;

	}
}

void ParticleFilter::measurementUpdate(const sensor_msgs::PointCloud& pointCloud, const nav_msgs::OccupancyGrid& map)
{
	double prob;
	double temp_error = 0;
	int valid_measurements = 0;
	float res = map.info.resolution;
	int width = map.info.width;
	int height = map.info.height;

	for (int i = 0; i < noParticles; i++)
	{
		prob = 1;
		for (int j = 0; j < pointCloud.points.size(); j++)
		{
			// transponer laser målingerne ind omkring (0,0) for at muliggøre sortering
			geometry_msgs::Point32 t;
			t.x = pointCloud.points[j].x * cos(particles[i].theta) - pointCloud.points[j].y * sin(particles[i].theta);
			t.y = pointCloud.points[j].x * sin(particles[i].theta) + pointCloud.points[j].y * cos(particles[i].theta);

			// Beregn fejl hvis målingen er gyldig
			if ((t.x < X_RANGE/2 && t.x > -X_RANGE/2) && (t.y < Y_RANGE/2 && t.y > -Y_RANGE/2))
			{
				t.y += particles[i].y;
				t.x += particles[i].x;

				valid_measurements++;

				int y = (int)((float)(t.y)/res);
				int x = (int)((float)(t.x)/res);

				// Beregner fejlen
				if (map.data[y*width+x] == 0)
					temp_error = 0.25;
				else
					temp_error = (100 - map.data[y*width+x]) * 0.001;

				// bestemmer fejlen afhængig af om målingen ligger på den ene eller den anden række
				prob *= gaussian(0,measurement_noise,temp_error);
			}
		}
		// Kontroller at der har været nok målinger
		if (valid_measurements > MIN_VALID_MEASUREMENTS)
			particles[i].w = prob;
		else
			particles[i].w = 0;
	}
}

void ParticleFilter::resampling()
{
	srand(time(0));
	std::vector<Car> temp_particles;

	// initialize the start index to a random particle
	int index = (double)rand()/double(RAND_MAX)*noParticles;

	max_prob = 0;
	// Find the highest weight
	for (int i = 0; i < noParticles; i++)
		if (particles[i].w > max_prob)
			max_prob = particles[i].w;

	double beta = 0;
	// Perform the selection
	for (int i = 0; i < noParticles; i++)
	{
		srand(uint(beta+i));
		// Increase beta with a random number between 0 and 2 * max_prob
		beta += (double)rand()/double(RAND_MAX) * 2.0 * max_prob;
		while (beta > particles[index].w)
		{
			beta -= particles[index].w;
			index = (index + 1) % noParticles;
		}
		Car c = Car(particles[index].x,particles[index].y,particles[index].theta,particles[index].w);
		temp_particles.push_back(c);
	}
	particles.clear();
	for (int i = 0; i<noParticles; i++)
		particles.push_back(temp_particles[i]);
}
// Find Vehicle using the robust mean method
fmMsgs::vehicle_position ParticleFilter::findVehicle()
{
	double x(0),y(0),theta(0);
	double norm = 0;

	max_prob = 0;
	int index;
	for (int i = 0; i < noParticles; i++)
		if (particles[i].w > max_prob)
		{
			max_prob = particles[i].w;
			index = i;
		}
	Car c = Car(particles[index].x,particles[index].y,particles[index].theta,particles[index].w);

	for (int i = 0; i < noParticles; i++)
	{
		if (sqrt(pow(c.x-particles[i].x,2.0) + pow(c.y-particles[i].y,2.0)) < MAX_DISTANCE && (abs(c.theta - particles[i].theta < MAX_ANGLE) || abs(c.theta + 2*M_PI - particles[i].theta) < MAX_ANGLE || abs(c.theta - 2*M_PI - particles[i].theta) < MAX_ANGLE))
		{
			x += particles[i].x * particles[i].w;
			y += particles[i].y * particles[i].w;

			// orientation is tricky because it is cyclic. By normalizing
			// around the first particle we are somewhat more robust to
			// the 0=2pi problem
			double temp_theta = (particles[i].theta - particles[0].theta + M_PI);
			if (temp_theta < 0)
				temp_theta += 2*M_PI;
			else if (temp_theta > 2*M_PI)
				temp_theta -= 2*M_PI;
			temp_theta += particles[0].theta - M_PI;

			theta += temp_theta * particles[i].w;

			norm += particles[i].w;
		}
	}
	last_pos.position.x = x / norm;
	last_pos.position.y = y / norm;
	last_pos.position.th = theta / norm;
	last_pos.probability = max_prob;
	last_pos.header.stamp = ros::Time::now();

	fmMsgs::vehicle_position r;
	r.position.y = x / norm;
	r.position.x = y = y / norm;
	r.position.th = theta / norm;
	r.position.th = 2*M_PI - r.position.th;
	if (r.position.th > 2*M_PI)
		r.position.th -= 2*M_PI;
	else if (r.position.th < 0)
		r.position.th += 2*M_PI;

	r.probability = max_prob;
	r.header.stamp = ros::Time::now();

	return r;

}

void ParticleFilter::resetParticleFilter(double off_x, double off_y)
{

	max_prob = 1;

	offset_y = off_x;
	offset_x = off_y;

	last_pos.position.x = off_x;
	last_pos.position.y = off_y;
	last_pos.position.th = 0;
	last_pos.probability = max_prob;
	last_pos.header.stamp = ros::Time::now();

	time_t sec;
	time(&sec);
	srand(uint(sec));

	particles.clear();

	for (int i = 0; i < noParticles; i++)
	{
		double x = (double)rand()/double(RAND_MAX) * length_x + offset_x - 0.5 * length_x;
		srand(uint(x*1000.0+i*1000));
		double y = (double)rand()/double(RAND_MAX) * length_y + offset_y - 0.5 * length_y;
		srand(uint(y*1000.0+i*2000));
		double theta = (double)rand()/double(RAND_MAX) * max_angle - 0.5 * max_angle;
		if (theta < 0)
			theta += 2*M_PI;
		srand(uint(theta*1000.0+i*3000));

		particles.push_back(Car(x,y,theta));
	}
}

void ParticleFilter::newParticles(double ratio)
{
	std::vector<Car> temp;
	time_t sec;
	time(&sec);
	srand(uint(sec));

	for (int i = 0; i < noParticles; i++)
	{
		if (particles[i].w < max_prob * ratio)
		{
			double x = (double)rand()/double(RAND_MAX) * length_x + offset_x - 0.5 * length_x;
			srand(uint(x*1000.0+i*1000));
			double y = (double)rand()/double(RAND_MAX) * length_y + offset_y - 0.5 * length_y;
			srand(uint(y*1000.0+i*2000));
			double theta = (double)rand()/double(RAND_MAX) * max_angle - 0.5 * max_angle;
			if (theta < 0)
				theta += 2*M_PI;
			srand(uint(theta*1000.0+i*3000));

			temp.push_back(Car(x,y,theta));
		}
		else
			temp.push_back(particles[i]);
	}
	particles.clear();
	for (int i = 0; i < noParticles; i++)
		particles.push_back(temp[i]);

}

fmMsgs::vehicle_position ParticleFilter::update(const sensor_msgs::PointCloud& pointCloud, const fmMsgs::vehicle_coordinate& delta_position, const nav_msgs::OccupancyGrid& map)
{
	motionUpdate(delta_position);
	measurementUpdate(pointCloud,map);
	resampling();

	return findVehicle();
}

