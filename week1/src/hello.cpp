#include <emc/io.h>
#include <emc/rate.h>
#include <iostream>
#include <algorithm>
#include <cmath>


int option = 1;

int main()
{
 // Create IO object, which will initialize the io layer
 emc::IO io;
 // Create Rate object, which will help keeping the loop at a fixed frequency
 emc::Rate r(10);
 // Loop while we are properly connected
 while(io.ok())
 {
  // create an object / variable that will hold the laser data.
 emc::LaserData scan;
 // Send a reference to the base controller (vx, vy, vtheta)

float minimum_treshold = 0.7;
float middle_treshold = scan.max_range; // check again
float min_distance = INFINITY;

if (io.readLaserData(scan))
{
	if(option ==0){
		int angle_check = 45; // 45 degrees to the left and right of the center
		int middle_idx = scan.ranges.size() / 2;
		double angle = scan.angle_increment;
		int amount = static_cast<int>(angle_check / angle);

		for (int i = -amount; i <= amount; ++i)
		{
			float cur_range = scan.ranges[middle_idx + i];
			if (cur_range < min_distance)
				min_distance = cur_range;
		}
	}
	else if(option==1)
	{
		double forward_angle = M_PI_4;  // 45 degrees
		double angle_increment = scan.angle_increment;

		int range_search = static_cast<int>(forward_angle / angle_increment);

		// Calculate the starting and ending indices of the forward angle range
		int start_index = (scan.ranges.size() / 2) - range_search;
		int end_index = (scan.ranges.size() / 2) + range_search;

		for (int i = start_index; i <= end_index; ++i)
		{
			float cur_range = scan.ranges[i];
			if (cur_range < min_distance)
				min_distance = cur_range;
		}
	} 

	std::cout << "min found: " << min_distance << std::endl;

	if (min_distance < minimum_treshold)
	{
		// Minimum distance below the threshold, stop the robot
		std::cout << "Detected obstacle: " << min_distance << " meters. Stopping the robot." << std::endl;
		io.sendBaseReference(0, 0, 0); // Stop the robot
	}
	else if (min_distance < middle_treshold)
	{
		// Additional condition to check
		std::cout << "detected max range--- WARNING" << std::endl;
	}
	else
	{
		// Distance above both thresholds, continue running
		// std::cout << "No obstacle detected. Running forward." << std::endl;
		io.sendBaseReference(0.5, 0, 0); // Move the robot forward
	}

}
else
{
   std::cout << "Failed to read laser data" << std::endl;
}
// Sleep remaining time
 r.sleep();
}
 return 0;
}
