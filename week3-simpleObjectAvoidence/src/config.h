

#include <vector>
#include <emc/io.h>
#include <emc/rate.h>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <unordered_map>


using namespace std;


// Values
int option = 1;
bool debug = false;
bool printAngles = true;
bool debugObstacleDetector = false;
bool printDetectedObstacles = true;


int targetx =0;
int targety =0;
int targettheta =0;

int beta_treshold = 40;

bool wheelsleep = false; // turning uncertaint odometry


// init total static

