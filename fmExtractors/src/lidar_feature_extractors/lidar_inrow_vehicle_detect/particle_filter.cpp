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

#include "particle_filter.h"

ParticleFilter::ParticleFilter()
{

}

ParticleFilter::ParticleFilter(int numberOfParticles,double len_x,double off_x,double len_y,double off_y,double max_ang, double x_noise, double y_noise, double theta_noise)
{
	noParticles = numberOfParticles;
	length_x = len_x;
	offset_x = off_x;
	length_y = len_y;
	offset_y = off_y;
	max_angle = max_ang;
	noise_x = x_noise;
	noise_y = y_noise;
	noise_theta = theta_noise;

	last_x = 0;
	last_y = 0;
	last_theta = 0;

	print = 0;

	max_prob = 1;

	for (int i = 0; i < noParticles; i++)
	{
		particles.push_back(getRandomParticle(i));
	}
	std::cout << "ParticleFilter created:" << std::endl;
}

Car* ParticleFilter::getRandomParticle(double seed)
{
	time_t sec;
	time(&sec);
	srand(uint(sec));

	double x = (double)rand()/double(RAND_MAX) * length_x + offset_x - 0.5 * length_x;
	srand(uint(x*1000.0+seed*1000));
	double y = (double)rand()/double(RAND_MAX) * length_y + offset_y - 0.5 * length_y;
	srand(uint(y*1000.0+seed*2000));
	double theta = (double)rand()/double(RAND_MAX) * max_angle - 0.5 * max_angle;
	if (theta < 0)
		theta += 2*M_PI;
	srand(uint(theta*1000.0+seed*3000));

	return new Car(x,y,theta);
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
		double prob = (particles[i]->w / max_prob);

		visualization_msgs::Marker marker;
		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time();
		marker.ns = "particles";
		marker.id = i;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = particles[i]->x;
		marker.pose.position.y = particles[i]->y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = cos(particles[i]->theta);
		marker.pose.orientation.y = sin(particles[i]->theta);
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 0.0;
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

	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = last_x;
	marker.pose.position.y = last_y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = cos(last_theta);
	marker.pose.orientation.y = sin(last_theta);
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 0.0;
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

	  boost::normal_distribution<> nd_x(0.0, noise_x);
	  boost::normal_distribution<> nd_y(0.0, 0.1*noise_y);
	  boost::normal_distribution<> nd_theta(0.0, 0.1*noise_theta);

	  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_x(rng, nd_x);
	  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_y(rng, nd_y);
	  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_theta(rng, nd_theta);

	  for (int i = 0; i < noParticles; ++i)
	  {
	 // particles[i]->x += var_x();
	    particles[i]->y += var_y();
	    particles[i]->theta += var_theta();
	  }
}

void ParticleFilter::printParticles()
{
	if (print)
	{
		std::cout << "Particles:" << std::endl;

		for (int i = 0; i < noParticles; i++)
			std::cout << "x: " << particles[i]->x << " y: " << particles[i]->y << " Theta: " << particles[i]->theta << " W: " << particles[i]->w << std::endl;

		std::cout << "End" << std::endl;
	}
}

void ParticleFilter::motionUpdate(const double& dx, const double& dy, const double& dtheta)
{
	double temp_y = sqrt(pow(dx,2.0)+pow(dy,2.0))*sin(last_theta);
//	double temp_x = sqrt(pow(dx,2.0)+pow(dy,2.0))*cos(last_theta);
	std::cout << "Temp_y: " << temp_y << std::endl;
	for (int i = 0; i < noParticles; i++)
	{
		particles[i]->theta += dtheta;
		if (particles[i]->theta < 0)
			particles[i]->theta += 2*M_PI;
		else if (particles[i]->theta > 2*M_PI)
			particles[i]->theta -= 2*M_PI;
		particles[i]->y += temp_y;
	//	particles[i]->x += temp_x;
	}
}

void ParticleFilter::measurementUpdate(const sensor_msgs::PointCloud& pointCloud)
{
	double error;
	double error_temp = 0;
	for (int i = 0; i < noParticles; i++)
	{
		error = 1;
		for (int j = 0; j < pointCloud.points.size(); j++)
		{
			error_temp = pointCloud.points[j].x * sin(particles[i]->theta) + pointCloud.points[j].y * cos(particles[i]->theta) + particles[i]->y;

			if (error_temp < 0.75/2)
				error *= gaussian(0,noise_y,error_temp);
			else
				error *= gaussian(0,noise_y,error_temp-0.75);
		}
		particles[i]->w = error;
	}
}

void ParticleFilter::resampling()
{
	srand(time(0));
	std::vector<Car*> temp;
	int count[noParticles];
	for (int i = 0; i < noParticles; i++)
		count[i] = 0;
	int index = (double)rand()/double(RAND_MAX)*noParticles;
	double beta = 0;
	max_prob = 0;
	for (int i = 0; i < noParticles; i++)
		if (particles[i]->w > max_prob)
			max_prob = particles[i]->w;

	std::cout << "Max prob: " << max_prob << std::endl;
	for (int i = 0; i < noParticles; i++)
	{
		srand(uint(beta+i));
		beta += (double)rand()/double(RAND_MAX) * 2.0 * max_prob;
		while (beta > particles[index]->w)
		{
			beta -= particles[index]->w;
			index = (index + 1) % noParticles;
		}
		Car* c = new Car(particles[index]->x,particles[index]->y,particles[index]->theta,particles[index]->w);
		temp.push_back(c);
	}
	particles.clear();
	for (int i = 0; i<noParticles; i++)
		particles.push_back(temp[i]);
}

fmMsgs::vehicle_position ParticleFilter::findVehicle()
{
	fmMsgs::vehicle_position r;
	double x(0),y(0),theta(0);
	for (int i = 0; i < noParticles; i++)
	{
		x += particles[i]->x;
		y += particles[i]->y;

		// orientation is tricky because it is cyclic. By normalizing
		// around the first particle we are somewhat more robust to
		// the 0=2pi problem
		double temp_theta = (particles[i]->theta - particles[0]->theta + M_PI);
		if (temp_theta < 0)
			temp_theta += 2*M_PI;
		else if (temp_theta > 2*M_PI)
			temp_theta -= 2*M_PI;
		temp_theta += particles[0]->theta - M_PI;

		theta += temp_theta;

	}
	r.position.x = last_x = x / noParticles;
	r.position.y = last_y = y / noParticles;
	r.position.th = last_theta = theta / noParticles;
	r.probability = max_prob;
	r.header.stamp = ros::Time::now();

	std::cout << "Row: x: " << last_x << " y: " << last_y << " theta: " << last_theta << std::endl;

	return r;

}

void ParticleFilter::newParticles(double ratio)
{
	std::vector<Car*> temp;
	for (int i = 0; i < noParticles; i++)
	{
		if (particles[i]->w < max_prob * ratio)
			temp.push_back(getRandomParticle(i));
		else
			temp.push_back(particles[i]);
	}
	particles.clear();
	for (int i = 0; i < noParticles; i++)
		particles.push_back(temp[i]);

}

fmMsgs::vehicle_position ParticleFilter::update(const sensor_msgs::PointCloud& pointCloud, const double& dx, const double& dy, const double& dtheta)
{
//	motionUpdate(dx,dy,dtheta);
	addRandomGaussianNoise();
	measurementUpdate(pointCloud);
	newParticles(0.5);
	measurementUpdate(pointCloud);
	resampling();

	return findVehicle();
}

