

#include <vector>
#include <emc/io.h>
#include <emc/rate.h>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <string>
#include <fstream>


// opencv
#include<opencv4/opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


using namespace std;


// Values
int option = 1;
bool debug = false;
bool printAngles = false;
bool debugObstacleDetector = false;
bool printDetectedObstacles = false;


int minObstLength = 15 ;
float minBetaDiff = 0.9;
int GaussianFilterLength = 10;
float sigmaFilter = 2;
int targetX =2;
int targetY =2;
int targettheta =0;

int beta_treshold = 0.001;

bool wheelsleep = false; // turning uncertaint odometry


// init total static

