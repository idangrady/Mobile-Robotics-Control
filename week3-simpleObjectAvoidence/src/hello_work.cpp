#include "utilis.h"


int maxDepth = 3;

// #include "config.h"
int main()
{
	// Create IO object, which will initialize the io layer
	emc::IO io;
	

	printingObject& printingObject = printingObject::getInstance();
	// Create Rate object, which will help keep the loop at a fixed frequency
	emc::Rate r(10);
	ObstacleDetector& detector = ObstacleDetector::getInstance(); // can not be copied
	signalOperations & signalOperator = signalOperations::getInstance();

	int ref_theta = 0;
	vector3f dir_vector{0.5, 0, 0};
	vector3f target(targetX, targetY, targettheta);
	
	vector<obstacle> obstacles;
	emc::OdometryData odomData;
	emc::OdometryData odomData_prev;
	
	vector<int> objectIdx;
	// Loop while we are properly connected
	while (io.ok())
	{
		dir_vector.max_angle =120;
		// Create an object / variable that will hold the laser data.
		emc::LaserData scan;
 		
		if (io.readLaserData(scan))
		{
			vector<float> filteredData = signalOperator.applyMovingAverage(scan.ranges, 5);
			scan.ranges = filteredData;
			dir_vector.theta = scan.angle_increment;
			// io.readOdometryData(odomData);
			obstacles.clear();
			// Constants for obstacle detection
			const double forward_angle = M_PI_2;		   // 45 degrees
			const double angle_increment = scan.angle_increment;
			const float minimum_threshold = 0.7;		   // Minimum threshold for obstacle detection

			// Calculate the range of indices to search for obstacles
			const int range_search = static_cast<int>(forward_angle / angle_increment);
			const int start_index = (scan.ranges.size() / 2) - range_search;
			const int end_index = (scan.ranges.size() / 2) + range_search;

			// Variables for obstacle detection
			bool detecting_obstacle = false;
 			// always try to go gorward
			vector<float> print_vec{};
 			int begin_idx = start_index-1;
			float start_beta_obstacle = start_index;
 			// Iterate through the range of indices and detect obstacles
										
		for (int i = start_index; i <= end_index; i++)
		{
			const float cur_range = scan.ranges[i];
			float angle_beta = detector.getBeta(scan.angle_increment, begin_idx, i, scan.ranges);
			const int end_idx = i - 1;
			const int obstacle_length = end_idx - begin_idx;

			if (cur_range >= maxDepth)
			{
				print_vec.push_back(0);
				scan.ranges[i] = maxDepth;
			}
			else if (cur_range < minimum_threshold && !detecting_obstacle)
			{
				print_vec.push_back(-1);
				detecting_obstacle = true;
				begin_idx = i;
				start_beta_obstacle = detector.getBeta(scan.angle_increment, i - 1, i, scan.ranges); // start beta of the obstacle
				cout << "Start Obstacle " << i << endl;
			}
			else if (cur_range > minimum_threshold && detecting_obstacle)
			{
				const float obstacle_angle_begin = detector.getAngle(scan.angle_increment, begin_idx);
				const float obstacle_angle_end = detector.getAngle(scan.angle_increment, end_idx);
				const float obstacle_angle = abs(obstacle_angle_end - obstacle_angle_begin);
				const float angle_beta = detector.getBeta(scan.angle_increment, start_index, i, scan.ranges);
				const float beta_diff = abs(angle_beta - start_beta_obstacle); // Difference in beta values
					// cout<<"beta_diff: "<<beta_diff<<endl;
					if ((beta_diff > beta_treshold && obstacle_length > minObstLength) || (i >= end_index))
					{
						detecting_obstacle = false;
						print_vec.push_back(-1);

						obstacle new_obstacle(begin_idx, end_idx);
						new_obstacle.updateAngles(obstacle_angle_begin, obstacle_angle_end);
						obstacles.push_back(new_obstacle);
						cout << "End Obstacle " << end_idx << endl;
					}
					else
					{
						print_vec.push_back(angle_beta);
					}
			}
			else
			{
				if(cur_range <= 2*minimum_threshold) print_vec.push_back(angle_beta);
				else{print_vec.push_back(0);}

 			}
		if(detecting_obstacle&& i==end_index)
		{
			// close opened obstacle at the end
			print_vec.push_back(-1);
			detecting_obstacle = false;
			print_vec.push_back(-1);
			obstacle new_obstacle(begin_idx, end_idx);
			new_obstacle.updateAngles(detector.getAngle(scan.angle_increment, begin_idx), detector.getAngle(scan.angle_increment, end_idx));
			obstacles.push_back(new_obstacle);
			cout << "End Obstacle " << end_idx << endl;
		}			
 		}
 			if(begin_idx>start_index-1)printingObject.findCornersAndPlot(print_vec, 100, "Moving Average Data");
			float maxAngle = detector.getMAxDegree(scan.angle_increment, obstacles);
			cout<<"maxAngle: " << maxAngle<<endl;
			// float idxMaxAngle = detector.getIdxAngle(scan.angle_increment, maxAngle);

			float curOdomData = odomData.a;
			float difference = maxAngle - curOdomData;

			cout << "Num Detected Obstacle: " << obstacles.size() << endl;

		    double targetTheta = std::atan2(targetY - odomData.y, targetX - odomData.x);
		    double distanceToTarget = std::hypot(targetX - odomData.x, targetY - odomData.y);


   			std::cout << "Target Theta: " << targetTheta << std::endl;
	    	std::cout << "Distance to Target: " << distanceToTarget << std::endl;

			const float targetThreshold= 0.4;
			double thetaError = targetTheta - odomData.a;

			for (int i =0; i<3; i++)
			{
            	if (io.readOdometryData(odomData))
			{
 				io.sendBaseReference(begin_idx >= start_index ? 0 : 0.5, 0, 0.1 * (begin_idx >= start_index ? 1 : 0)); 		
				targetTheta = std::atan2(targetY - odomData.y, targetX - odomData.x);
				distanceToTarget = std::hypot(targetX - odomData.x, targetY - odomData.y);
				std::cout << "Target Theta: " << targetTheta << std::endl;
	    		std::cout << "Distance to Target: " << distanceToTarget << std::endl;

				}
				else{				
					std::cout << "Error to read Odometry Data" << std::endl;
				}
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
