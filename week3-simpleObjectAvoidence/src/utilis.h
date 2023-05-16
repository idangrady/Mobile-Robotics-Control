#include <vector>
#include <emc/io.h>
#include <emc/rate.h>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <unordered_map>

using namespace std;

// SingleTom
class ObstacleDetector {
public:
    static ObstacleDetector& getInstance() {
        static ObstacleDetector instance;
        return instance;
    }

    ObstacleDetector(const ObstacleDetector&) = delete; // Disable copy constructor
    ObstacleDetector& operator=(const ObstacleDetector&) = delete; // Disable assignment operator

    float getAngle(float increment, int i) {
        return increment * i;
    }

    float getAngleMiddle(float increment, int i, int j) {
        return ((j - i) * increment);
    }

    float getBeta(float angleIncrement, int beginIdx, int endIdx, const std::vector<float>& ranges) {
        const float obstacleAngleBegin = getAngle(angleIncrement, beginIdx);
        const float obstacleAngleEnd = getAngle(angleIncrement, endIdx);
        const float obstacleAngle = std::abs(obstacleAngleEnd - obstacleAngleBegin);

        const float distanceBC = std::sqrt(
            ranges[beginIdx + 1] * ranges[beginIdx + 1] +
            ranges[endIdx] * ranges[endIdx] -
            2 * ranges[endIdx] * ranges[beginIdx + 1] * std::cos(obstacleAngle)
        );

        const double beta = std::acos(
            (distanceBC * distanceBC + ranges[beginIdx + 1] * ranges[beginIdx + 1] - ranges[endIdx] * ranges[endIdx]) /
            (2 * ranges[beginIdx + 1] * distanceBC)
        );

        return static_cast<float>(beta * (180.0 / M_PI)); // return in degrees
    }
private:
    ObstacleDetector() = default; // Private constructor to prevent instantiation
};

struct obstacle
{
	int begin, end;
	float angle_begin, angle_end;
	int id;
 	static int total;
	float beta; 

	obstacle() = default;
	obstacle(int begin_, int end_){begin = begin_; end = end_;total++; id = total;}
	obstacle(int begin_, int end_, float angle_begin_, float angle_end_){
		begin = begin_; end = end_;angle_begin = angle_begin_;angle_end = angle_end_;total++;id = total;
	}

	void updateAngle(const float angle_, int binary_cond)  // if b =0 -> begin else end
	{
		if(binary_cond ==0)
			{angle_begin = angle_;} 
		else
			{angle_end = angle_;}
	} 

	void updateAngles(const float angle_begin_, const float angle_end_)  // if b =0 -> begin else end
	{
		angle_begin =angle_begin_;
		angle_end = angle_end_;
	} 

	float get_angle(float increment, int i)const
	{
		return get_angle(increment,i);
	}
	
	int middle(){return (begin+end)/2;}
};



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


// Values

int option = 1;
bool debug = false;

// init total static
int obstacle::total = 0;



