
#include <emc/io.h>
#include <emc/rate.h>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <unordered_map>

float get_angle(float increment, int i)
{
	return increment *i;
}
float get_angle_middle(float increment, int i , int j)
{
	return ((j-i)*increment);}
struct obstacle
{
	int begin, end;
	float angle_begin, angle_end;
	int id;
	static int total;

	obstacle() = default;
	obstacle(int begin_, int end_){begin = begin_; end = end_;total++; id = total;}
	obstacle(int begin_, int end_, float angle_begin_, float angle_end_){
		begin = begin_; end = end_;angle_begin = angle_begin_;angle_end = angle_end_;total++;id = total;
	}

	void updateAngle(const float angle_, int b)  // if b =0 -> begin else end
	{
		if(b ==0)
			{angle_begin = angle_;} 
		else
			{angle_end = angle_;}
	} 
	float get_angle(float increment, int i)const
	{
		return get_angle(increment,i);
	}
	
	int middle(){return (begin+end)/2;}
};

// init total static
int obstacle::total = 0;


// struct vector 3f
struct vector3f
{
public:
	float x,y,theta;
	float speed =0.5;

	vector3f()  =default;
	vector3f(float x_, float y_, float theta_){x =x_;y =y_; theta = theta_; }

// operators
	vector3f operator*(const vector3f vec2){return vector3f(vec2.x * x,vec2.y * y, theta * vec2.theta);}
	vector3f operator+(const vector3f vec2){return vector3f(vec2.x + x,vec2.y + y, theta + vec2.theta);}

// functions
	float get_theta(){return theta;}
	float get_dir(const float ref){if(ref>theta) return -1; else return 1;}
	float returnMod(const float ref){return abs(fmod(ref, theta));}
};

using namespace std;

int option = 1;


int main()
{
	vector3f dir_vector{0.5, 0, 0};
	vector<obstacle> obstacles = {};

	// Create IO object, which will initialize the io layer
	emc::IO io;

	// Create Rate object, which will help keep the loop at a fixed frequency
	emc::Rate r(10);

	int ref_theta = 0;

	// Loop while we are properly connected
	while (io.ok())
	{
		// Create an object / variable that will hold the laser data.
		emc::LaserData scan;

		// Send a reference to the base controller (vx, vy, vtheta)
		int middle_idx = scan.ranges.size() / 2;
		float minimum_threshold = 0.7;
		float middle_threshold = 1000; // Check again
		float min_distance = INFINITY;

		int amount_objects = 0;
		std::vector<int> idx_objects = {};
		std::vector<int> end_idx = {};

		int middle = -100; // Come back
		int begin = -1;

		if (io.readLaserData(scan))
		{
			// Constants for obstacle detection
			const double forward_angle = M_PI_4;		   // 45 degrees
			const double angle_increment = scan.angle_increment;
			const float minimum_threshold = 0.7;		   // Minimum threshold for obstacle detection

			// Calculate the range of indices to search for obstacles
			const int range_search = static_cast<int>(forward_angle / angle_increment);
			const int start_index = (scan.ranges.size() / 2) - range_search;
			const int end_index = (scan.ranges.size() / 2) + range_search;

			// Variables for obstacle detection
			int begin_idx = -1;
			bool detecting_obstacle = false;

			// Iterate through the range of indices and detect obstacles
			for (int i = start_index; i <= end_index; ++i)
			{
				const float cur_range = scan.ranges[i];

				if (cur_range < minimum_threshold)
				{
					min_distance = cur_range;
					if (!detecting_obstacle)
					{
						// Start of obstacle
						begin_idx = i;
						detecting_obstacle = true;
					}
				}
				else
				{
					if (detecting_obstacle)
					{
						// End of obstacle
						const int end_idx = i - 1;
						const int obstacle_length = end_idx - begin_idx + 1;

						if (obstacle_length > 1)
						{
							detecting_obstacle = false;

							// Create obstacle object
							obstacle new_obstacle(begin_idx, end_idx);
							obstacles.push_back(new_obstacle);

							// Update obstacle angle
							const float angle_increment = scan.angle_increment;
							const float obstacle_angle_begin = get_angle(angle_increment, begin_idx);
							const float obstacle_angle_end = get_angle(angle_increment, end_idx);
							new_obstacle.updateAngle(obstacle_angle_begin, 0);
							new_obstacle.updateAngle(obstacle_angle_end, 1);
						}
					}
				}
			}
			if(obstacles.size()==0)
			{
					cout<<" should move forward"<<endl;
					io.sendBaseReference(0.5, 0, 0); // forward
				
			}
			// Process detected obstacles
			for (auto &obstacle : obstacles)
			{
				const float obstacle_distance = min_distance;

				// Do something with the obstacle information (e.g., stop the robot)
				if (obstacle_distance < minimum_threshold)
				{
					cout << "Detected obstacle: " << obstacle_distance << " meters. Stopping the robot." << endl;
					// io.sendBaseReference(0, 0, 0); // Stop the robot
					// or
					io.sendBaseReference(0, 0, -0.3); // Change direction

					// Clear the obstacles vector and reset the obstacle detection
					obstacles.clear();
					begin_idx = -1;
					detecting_obstacle = false;
				}

			}

			// Print the detected obstacles
			
			cout << "Detected " << obstacles.size() << " obstacles." << endl;

			for (const auto& obstacle_ : obstacles)
			{
				cout << "Obstacle angle: " << obstacle_.get_angle(angle_increment, 0) << " to " << obstacle_.get_angle(angle_increment, 1) << " radians." << endl;
			}
		}
		else
		{
			cout << "Failed to read laser data." << endl;
		}

		// Sleep to maintain the loop rate
		r.sleep();
	}

	return 0;
}
