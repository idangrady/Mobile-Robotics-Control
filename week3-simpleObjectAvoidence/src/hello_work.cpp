#include "utilis.h"


int maxDepth = 3;
float lastThetaMax =-1;
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
			io.readOdometryData(odomData);
			// obstacles.clear();
			// Constants for obstacle detection
			const double forward_angle = M_PI_4;		   // 45 degrees
			const double angle_increment = scan.angle_increment;
			const float minimum_threshold = 1.2;		   // Minimum threshold for obstacle detection

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
			print_vec.push_back(-1);				
			bool stright = true;	

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
				//cout << "Start Obstacle " << i << endl;
			}
			else if (cur_range > minimum_threshold && detecting_obstacle)
			{
				const float obstacle_angle_begin = detector.getAngle(scan.angle_increment, begin_idx);
				const float obstacle_angle_end = detector.getAngle(scan.angle_increment, end_idx);
				const float obstacle_angle = abs(obstacle_angle_end - obstacle_angle_begin);
				const float angle_beta = detector.getBeta(scan.angle_increment, start_index, i, scan.ranges);
				const float beta_diff = abs(angle_beta - start_beta_obstacle); // Difference in beta values
 
 					if ((beta_diff > beta_treshold && obstacle_length > minObstLength) || (i >= end_index))
					{
						detecting_obstacle = false;
						print_vec.push_back(-1);
 						stright = false;
					}
					else
					{
						print_vec.push_back(angle_beta);
					}
			}
			else
			{
				if(cur_range <=  minimum_threshold) print_vec.push_back(angle_beta);
				else{print_vec.push_back(0);}
 			}
		if(detecting_obstacle&& i==end_index)
		{
			// close opened obstacle at the end
 			detecting_obstacle = false;
			print_vec.push_back(-1);
 		}			
 		}

			tuple<int, int> result  = printingObject.findCornersAndPlot(print_vec, 100, "PostProcessed");
			int moveforward, maxIdx;
			std::tie(moveforward, maxIdx) = result;

			// bool narrowedViewForwardCheck = detector.canMoveForwardNarrowedBand(print_vec, ) 
			float maxAngle = detector.getAngle(scan.angle_increment, maxIdx+ start_index); //
			double targetTheta = std::atan2(targetY - odomData.y, targetX - odomData.x);
			double distanceToTarget = std::hypot(targetX - odomData.x, targetY - odomData.y);


			lastThetaMax =maxAngle;
			// tresholds
			const float targetThreshold = 0.11;
			const float ObstacleThreshold = 0.11;
			// calculate theta
			float curTheta =  odomData.a; // detector.getAngle(scan.angle_increment, (scan.ranges.size())/2) +
			int middleIdx = detector.getIdxAngle(scan.angle_increment,curTheta);


			
			// errors
			double thetaErrorTarget = (targetTheta - curTheta);
			double thetaErrorObstacle = (maxAngle - curTheta);

 			float curDistance = 1.2 * minimum_threshold;
			bool rotateTowardTarget = false;
			int size_vec = print_vec.size();

			float moveForwardSpeed =0.1;
			float rotationSpeed = 0.1;
			int middleIdxCheck =30;

			std::cout<<"targetTheta "<< targetTheta<<", curTheta: "<<curTheta<<", distanceToTarget: "<<distanceToTarget<<endl;
 			std::cout<<"x "<< odomData.x<<", y"<< odomData.y<<", "<< odomData.a<<endl; 
 			std::cout<<"maxAngle "<< maxAngle<<", thetaErrorObstacle"<< thetaErrorObstacle<<endl; 


// THESE ARE THE CORRECT
  		//	if(std::abs((thetaErrorTarget) < targetThreshold)&& (std::abs(thetaErrorObstacle)< ObstacleThreshold))io.sendBaseReference(moveForwardSpeed,0,0);
			// Target
 			// while (std::abs(thetaErrorTarget) >= targetThreshold)
			// 	{
 			// 		io.sendBaseReference(0, 0, (thetaErrorTarget > 0 ?  rotationSpeed : -rotationSpeed)); // abs(size_vec- 2*middleIdx)<middleIdxCheck?moveForwardSpeed:0
			// 		curTheta  +=(thetaErrorTarget < 0 ?  rotationSpeed : -rotationSpeed);
			// 		thetaErrorTarget +=(thetaErrorTarget>0 ? -rotationSpeed: rotationSpeed);// targetTheta - curTheta;
			// 		cout << "Rotate-Target " << (thetaErrorTarget > 0 ? "Positive:.1" : "Negative:-.1") << " | Error = " << thetaErrorTarget << endl;
  			// 	}

			cout<<"Difference before OB:"<<thetaErrorObstacle<<endl;

// MOFIDIED FOR EXPERIMENT BEGIN
  			if(std::abs((thetaErrorTarget) < targetThreshold)|| (std::abs(thetaErrorObstacle)< ObstacleThreshold))io.sendBaseReference(moveForwardSpeed,0,0);
			while (std::abs(thetaErrorObstacle) >= ObstacleThreshold)
				{
 					io.sendBaseReference(0, 0, (thetaErrorObstacle > 0 ?  rotationSpeed : -rotationSpeed)); // abs(size_vec- 2*middleIdx)<middleIdxCheck?moveForwardSpeed:0
					curTheta  +=(thetaErrorObstacle < 0 ?  rotationSpeed : -rotationSpeed);
					thetaErrorObstacle +=(thetaErrorTarget>0 ? -rotationSpeed: rotationSpeed);// targetTheta - curTheta;
					cout << "Rotate-ObstacleThreshold " << (thetaErrorObstacle > 0 ? "Positive:.1" : "Negative:-.1") << " | Error = " << thetaErrorObstacle << endl;
  				}
// MOFIDIED FOR EXPERIMENT END

 			// while (std::abs(thetaErrorObstacle) >= ObstacleThreshold)
			// 	{
 			// 		io.sendBaseReference(0, 0, (thetaErrorObstacle > 0 ?  rotationSpeed : -rotationSpeed)); // abs(size_vec- 2*middleIdx)<middleIdxCheck?moveForwardSpeed:0
			// 		curTheta  +=(thetaErrorObstacle < 0 ?  rotationSpeed : -rotationSpeed);
			// 		thetaErrorObstacle +=(thetaErrorTarget>0 ? -rotationSpeed: rotationSpeed);// targetTheta - curTheta;
			// 		cout << "Rotate-ObstacleThreshold " << (thetaErrorObstacle > 0 ? "Positive:.1" : "Negative:-.1") << " | Error = " << thetaErrorObstacle << endl;
  			// 	}



			cout<<"Difference after OB:"<<thetaErrorObstacle<<endl;
			// }
			
			if(distanceToTarget<0.5){io.sendBaseReference(0, 0, 0);}
 		}		
		else
		{
			//cout << "Failed to read laser data." << endl;
		}

		// Sleep to maintain the loop rate
		r.sleep();
	}

	return 0;
}
