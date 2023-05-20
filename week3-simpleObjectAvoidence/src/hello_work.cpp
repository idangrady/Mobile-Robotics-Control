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

	int ref_theta = 0;
	vector3f dir_vector{0.5, 0, 0};
	vector3f target(targetx, targety, targettheta);
	
	vector<obstacle> obstacles;
	emc::OdometryData odomData;
	emc::OdometryData odomData_prev;
	

	// Loop while we are properly connected
	while (io.ok())
	{
		dir_vector.max_angle =120;

		// Create an object / variable that will hold the laser data.
		emc::LaserData scan;
		// Send a reference to the base controller (vx, vy, vtheta)
		float minimum_threshold = 0.7;
		
		if (io.readLaserData(scan))
		{
			
			dir_vector.theta = scan.angle_increment;
			io.readOdometryData(odomData);
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
			int begin_idx = -1;
			bool detecting_obstacle = false;
			bool ShouldMove = true;
			// always try to go gorward

			float start_beta_obstacle = start_index;
			// Iterate through the range of indices and detect obstacles
			for (int i = start_index; i <= end_index; i++)
			{

				const float cur_range = scan.ranges[i];
				float angle_beta = detector.getBeta(scan.angle_increment,start_index-1, i,scan.ranges);
 
				if(cur_range>maxDepth) scan.ranges[i] = maxDepth; //clapping
				else if (cur_range < minimum_threshold && !detecting_obstacle)
				{
					detecting_obstacle = true;
					begin_idx = i;
					start_beta_obstacle = detector.getBeta(scan.angle_increment, i-1, i, scan.ranges); // start beta of the obstacle
					// cout<<"Start!. start_beta_obstacle: "<<start_beta_obstacle<<endl;

				}
				else if (detecting_obstacle)
				{
					const int end_idx = i - 1;
					const int obstacle_length = end_idx - begin_idx + 1;

					const float obstacle_angle_begin = detector.getAngle(angle_increment, begin_idx);
					const float obstacle_angle_end = detector.getAngle(scan.angle_increment, end_idx);
					const float obstacle_angle = abs(obstacle_angle_end - obstacle_angle_begin);
					const float angle_beta = detector.getBeta(scan.angle_increment, start_beta_obstacle, i, scan.ranges);
					const float beta_diff = abs(angle_beta - start_beta_obstacle); // Difference in beta values
					// cout<<beta_diff<<endl;
					if (obstacle_length >= 20 && beta_diff>0.25)
					{
						// cout<<"Added: " <<obstacle_length<< ", "<<beta_diff<<endl;
						detecting_obstacle = false;
						// Create obstacle object
						obstacle new_obstacle(begin_idx, end_idx);
						// Update obstacle angle
						new_obstacle.updateAngles(obstacle_angle_begin, obstacle_angle_end);
						// Push to the obstacles list
						obstacles.push_back(new_obstacle);
					}
			}
    

			}

			float maxAngle = detector.getMAxDegree(scan.angle_increment, obstacles);
			float idxMaxAngle = detector.getIdxAngle(scan.angle_increment,maxAngle);

			float curOdomData  =odomData.a;
			float difference = maxAngle -curOdomData;
			if(printAngles==true&& obstacles.size()>0)
			{
				cout<<"distance to max middle: " <<scan.ranges[idxMaxAngle]<<  idxMaxAngle<<endl;
				cout<<"difference: "<<difference<<endl;
				cout<<detector.sign(difference)<<endl;
				cout<<"Max Angle: " <<maxAngle <<" , idxMax Angle: " <<  idxMaxAngle<<endl;
				cout<<"Odom: " <<curOdomData <<endl;
				cout<<"difference: "<<difference<<endl;
				cout<<"Amount of steps needed-> " << difference/0.1<<endl;
			}

			if(printDetectedObstacles)
			{
				cout<<"detected Obstacles: "<<obstacles.size()<<endl;
				for(int i =0; i<obstacles.size(); i++)
				{
					cout<<"Obstcle " <<i <<"begin: "<< obstacles[i].begin <<" " << "-End:"<<obstacles[i].end<<endl;
				}
			}
			if(obstacles.size()){
				cout << "sign: "<<detector.sign(difference)<<endl;
				for(int i =0; i<(int)(difference/0.1);i++)
				{
					io.sendBaseReference(0,0, -detector.sign(difference)*0.1);
				}
			}
			else io.sendBaseReference(0.5,0, 0);	
			// Questoion 1+2
			// printingObject.printingOdomData(odomData);
			// printingObject.printingOdomDataDifference(odomData, odomData_prev, true);

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
