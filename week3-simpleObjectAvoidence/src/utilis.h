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


// struct vector3f
struct vector3f {
public:
	float x, y, theta;
	float speed = 0.5;
	float Odom = 0;
	bool right = false;
	float max_angle;

	vector3f() = default;
	vector3f(float x_, float y_, float theta_) : x(x_), y(y_), theta(theta_) {}

	// operators
	vector3f operator*(const vector3f& vec2) {
		return vector3f(vec2.x * x, vec2.y * y, theta * vec2.theta);
	}

	vector3f operator+(const vector3f& vec2) {
		return vector3f(vec2.x + x, vec2.y + y, theta + vec2.theta);
	}

	std::tuple<float, float> get_X_theta(bool obstacle_detect_) {
		if (obstacle_detect_) {
			updateTheta();
			return std::make_tuple(0, theta);
		}
		else {
			return std::make_tuple(x, 0);
		}
	}

	// functions
	float get_theta() {
		return theta;
	}

	float get_dir(const float ref) {
		if (ref > theta) {
			return -1;
		}
		else {
			return 1;
		}
	}

	float returnMod(const float ref) {
		return std::abs(std::fmod(ref, theta));
	}

	void checkOdomMax() {
		if (std::abs(Odom) > 2) {
			right = !right; // Toggle the value of right using the negation operator
		}
	}

	void checkOdom() {
		checkOdomMax();
		if (right) {
			Odom += 0.1;
		}
		else {
			Odom -= 0.1;
		}
	}

	void updateTheta() {
		checkOdom(); // Decide the direction
		std::cout << "Odom: " << Odom << std::endl;
		if (right) {
			theta = -0.2;
		}
		else {
			theta = 0.2;
		}
	}

	void updateThetaObstacle(bool found) {
		if (found) {
			updateTheta();
			x = 0;
		}
		else {
			theta = 0;
			x = 0.5;
		}
	}
};



// // struct vector 3f
// struct vector3f
// {
// public:
// 	float x,y,theta;
	
// 	float speed =0.5;

// 	float Odom=0;
// 	bool right = false;
// 	float max_angle;


// 	vector3f()  =default;
// 	vector3f(float x_, float y_, float theta_){x =x_;y =y_; theta = theta_; }

// // operators
// 	vector3f operator*(const vector3f vec2){return vector3f(vec2.x * x,vec2.y * y, theta * vec2.theta);}
// 	vector3f operator+(const vector3f vec2){return vector3f(vec2.x + x,vec2.y + y, theta + vec2.theta);}

// 	std::tuple<float, float> get_X_theta(bool obstacle_detect_) {
// 		if (obstacle_detect_) {
// 			updateTheta();
// 			return std::make_tuple(0, theta);
// 		}
// 		else{
			
// 			return std::make_tuple(x, 0);
// 			}
// 	}
	
	

// // functions
// 	float get_theta(){return theta;}
// 	float get_dir(const float ref){if(ref>theta) return -1; else return 1;}
// 	float returnMod(const float ref){return abs(fmod(ref, theta));}

// 	//
// 	void checkOdemMax()
// 	{
// 		if(abs(Odom)>2) {~right;}  // *(180/M_PI))>
// 	}
// 	void checkOdom()
// 	{
// 		checkOdemMax();
// 		if(right){Odom= Odom+ 0.1;}
// 		else{Odom= Odom- 0.1;}	
// 	}
// 	void updateTheta()
// 	{
// 		checkOdom(); //decide the direction
// 		cout<<"Odom: "<< Odom<<endl;
// 		if(right){theta = -0.2;}
// 		else{theta =0.2;}
// 	}
// 		void updateThetaObstacle(bool found)
// 	{
// 		if(found) {updateTheta();x=0; }  //decide the direction
// 		else{
// 			theta =0;
// 			x = 0.5;
// 			}
// 	}
// };


// Values

int option = 1;
bool debug = false;

// init total static
int obstacle::total = 0;



