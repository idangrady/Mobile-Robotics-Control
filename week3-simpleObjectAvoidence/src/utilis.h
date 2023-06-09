
#include "config.h"

class obstacle;  // forward declaration

struct obstacle
{
	int begin, end;
	float angle_begin, angle_end;
	int id;
 	static int total;  				// track how many obstacle we have
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
	float x, y, theta; // theta is either 0.1 or -0.1 -> convert to radiance: - > 180/M_PI*rad
	float speed = 0.5;
	float Odom = 0;
	float totalAngle;
	bool right = false;
	float max_angle;

	vector3f() = default;
	vector3f(float x_, float y_, float theta_) : x(x_), y(y_), theta(theta_) {}
	vector3f(float x_, float y_, float theta_, float totalAngle_) : x(x_), y(y_), theta(theta_), totalAngle(totalAngle_) {}

	// operators
	vector3f operator*(const vector3f& vec2) {
		return vector3f(vec2.x * x, vec2.y * y, theta * vec2.theta);
	}
	float getLength() const {return sqrt(x*x+ y*y);}
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
		Odom += (right ? 0.1 : -0.1);
	}

	void updateTheta() {
		checkOdom();
		theta = (right ? -0.2 : 0.2);
		if(debug) std::cout << "Odom: " << Odom << std::endl;
	}
	void updateThetaObstacle(bool found) {
		if (found) {
			updateTheta();
 		} else {
			theta = 0.2;
		}
		x = (found ? 0.0 : 0.5);
	}
	void updateOdom(float odom_){Odom =odom_;}
};

// SingleTom
class ObstacleDetector {
public:
    static ObstacleDetector& getInstance() {
        static ObstacleDetector instance;
        return instance;
    }
    ObstacleDetector(const ObstacleDetector&) = delete; // Disable copy constructor
    ObstacleDetector& operator=(const ObstacleDetector&) = delete; // Disable assignment operator

    inline float getAngle(float increment, int i) {
        return increment * i;
    }
	inline int getIdxAngle(float increment, float angle)
	{
		return (int)(angle/increment);
	}
    inline float getAngleMiddle(float increment, int i, int j) {
        return ((j - i) * increment);
    }
	inline int getOpeningSize(obstacle & obs1, obstacle & obs2 )
	{
		return obs2.begin - obs1.end;
	}
	// inline getLocNode(float theta, ){}
	inline float square(float num){return num*num;}
    float getBeta(float angleIncrement, int beginIdx, int endIdx, const std::vector<float>& ranges, bool degree =false) {
        const float obstacleAngleBegin = getAngle(angleIncrement, beginIdx);
        const float obstacleAngleEnd = getAngle(angleIncrement, endIdx);
        const float obstacleAngle = std::abs(obstacleAngleEnd - obstacleAngleBegin);

        const float distanceBC = std::sqrt(
			square(ranges[beginIdx])+ square(ranges[endIdx])+
            2 * ranges[endIdx] * ranges[beginIdx] * std::cos(obstacleAngle)
        );
        const double beta = std::acos(
            (square(distanceBC) +  square(ranges[beginIdx]) -square(ranges[endIdx])) /
            (2 * ranges[beginIdx] * distanceBC)
        );
		// bc = num
		// AB, BC, AC     
		//  AB = range[begin], AC = range[end] , BC^2 = AB^2 +AC^2 + AC*AB*2COS(theta)
		// Beta -> BC^2 + AB^2 - 2cos(beta) AB*BC   = AC^2
		//  cos(beta) = (BC^2 + AB^2 -AC^2) /AB*BC*2
		
		if(degree) return static_cast<float>(beta * (180.0 / M_PI));  // return in degrees
        return beta; 
	}
	
	int sign(float num){
		int out (num>0 ? 1 : -1);
		out = (num==0?0:out);
		return out;
		}
	float getMAxDegree(const float increment, std::vector<obstacle>& detectedObstacles) 
	{
		int amountObstacles = detectedObstacles.size();	
		if(debug) cout<<"Amount of detected Obstacle "<< amountObstacles <<endl;
		float maxDegree=-1;
		float maxOpeningSize = 0;
		if (amountObstacles == 1) {
			const obstacle& onlyObstacle = detectedObstacles[0];
			int currentSize = onlyObstacle.end - onlyObstacle.begin;
			cout << "currentSize: " << currentSize << endl;

			if (currentSize > maxOpeningSize) {
				maxOpeningSize = currentSize;
				maxDegree = (getAngle(increment, onlyObstacle.begin) + getAngle(increment, onlyObstacle.end)) / 2;
			}
		} else if (amountObstacles > 1) {
			for (int i = 0; i < amountObstacles - 1; i++) {
				int currentSize = getOpeningSize(detectedObstacles[i], detectedObstacles[i+1]);
				cout << "currentSize: " << currentSize << endl;
				
				if (currentSize > maxOpeningSize) {
					maxOpeningSize = currentSize;
					maxDegree = getAngle(increment, (detectedObstacles[i+1].begin + detectedObstacles[i].end) / 2);
				}
			}
		}

		if (maxDegree == -1) {
			maxDegree = 0.0; // Set maxDegree to a default value if no valid opening size is found
		}
		if(debugObstacleDetector==true) cout<<"Max Degree size: "<<maxOpeningSize<<endl;
		// we should adapt the steering so that go toward the max degree. 
		return maxDegree;
	}	

	int findMiddleIndexOfLongestConsecutiveZeros(const std::vector<float>& vec)
	{
		int maxCount = 0;      // Maximum count of consecutive zeros
		int maxIndex = -1;     // Index of the middle element of the maximum consecutive zeros
		int currentCount = 0;  // Current count of consecutive zeros
		int currentIndex = -1; // Index of the current element

		for (int i = 0; i < vec.size(); ++i)
		{
			if (vec[i] == 0)
			{
				// If the current element is zero
				if (currentCount == 0)
					currentIndex = i;  // Set the current index as the starting index

				++currentCount;  // Increment the current count of consecutive zeros

				if (currentCount > maxCount)
				{
					// If the current count exceeds the maximum count
					maxCount = currentCount;                          // Update the maximum count
					maxIndex = currentIndex + (currentCount / 2);     // Update the middle index
				}
			}
			else
			{
				// If the current element is not zero
				currentCount = 0;  // Reset the current count of consecutive zeros
			}
		}

    return maxIndex;
}

template<typename T, typename U>
bool canMoveForwardNarrowedBand(vector<T>& values, int range, U valueToExpect)
{
    int middle = values.size() / 2;
    bool out = true;
    for (int i = middle - range / 2; i < range / 2 + middle; i++)
    {
        if (static_cast<U>(values[i]) == valueToExpect)
        {
            out = false;
            break;
        }
    }
    return out;
}

private:
    ObstacleDetector() = default; // Private constructor to prevent instantiation
};

// init obstacle static var to 0
int obstacle::total = 0;

class signalOperations
{
	public:
	static signalOperations & getInstance(){
		 static signalOperations instance;
		return instance;
	}
	signalOperations(const signalOperations&) = delete; // Disable copy constructor
    signalOperations& operator=(const signalOperations&) = delete; // Disable assignment operator
	std::vector<float> applyMovingAverage(const std::vector<float>& signal, int numSamples)
	{
		std::vector<float> result;
		int signalSize = signal.size();

		for (int i = 0; i < signalSize; ++i) {
			float sum = 0.0;
			int count = 0;

			// Calculate the sum of 'numSamples' values around the current index
			for (int j = i - numSamples / 2; j <= i + numSamples / 2; ++j) {
				if (j >= 0 && j < signalSize) {
					sum += signal[j];
					++count;
				}
			}

			// Calculate the average and add it to the result vector
			if (count > 0) {
				float average = sum / count;
				result.push_back(average);
			}
		}

		return result;
	}

 	std::vector<float> applyGaussianFilter(const std::vector<float>& signal, int length, float sigma) {
		std::vector<float> kernel(length);
		std::vector<float> smoothed(signal.size());

		// Calculate the kernel
		float sum = 0.0;
		for (int i = 0; i < length; ++i) {
			float x = i - length / 2;
			kernel[i] = std::exp(-(x * x) / (2 * sigma * sigma));
			sum += kernel[i];
		}

		// Normalize the kernel
		for (int i = 0; i < length; ++i) {
			kernel[i] /= sum;
		}

		// Apply the Gaussian filter
		for (int i = 0; i < signal.size(); ++i) {
			float value = 0.0;
			for (int j = 0; j < length; ++j) {
				int index = i + j - length / 2;
				if (index >= 0 && index < signal.size()) {
					value += signal[index] * kernel[j];
				}
			}
			smoothed[i] = value;
		}

		return smoothed;
	}


	private:
	signalOperations() = default;
};
// Singeltom printing
class printingObject
{
public:
	static int currentPrintingVal;
	static printingObject& getInstance()
	{
		static printingObject instance;
		return instance;
	}

	void printingOdomData(emc::OdometryData odomData_)
	{
		cout<<"OdomData----x: " << odomData_.x<< ". y:" <<odomData_.y << ". z: "  <<odomData_.a <<endl; 
	}
	void printingOdomDataDifference(emc::OdometryData & odomData_current, emc::OdometryData & odomData_previous , bool transferOdom =false)
	{
		cout<<"Difference Odom----x: " << odomData_current.x -odomData_previous.x << ". y:" <<odomData_current.y- odomData_previous.y << ". theta: "  <<odomData_current.a- odomData_previous.a <<endl;
		if(transferOdom==true){
		odomData_previous.x = odomData_current.x;
		odomData_previous.y = odomData_current.y;
		odomData_previous.a =odomData_current.a;
		}
	}
	void printCurrent()
	{
		cout<<currentPrintingVal<<endl;
		currentPrintingVal++;
	}

	cv::Mat findCorners(const std::vector<float>& values, double threshold=0.005)
	{
		// Convert the values vector to a grayscale image
		cv::Mat image(values.size(), 1, CV_32F);
		for (int i = 0; i < values.size(); ++i)
		{
			image.at<float>(i, 0) = values[i];
		}
 		// Convert the grayscale image to a 32-bit floating-point format
	    image.convertTo(image, CV_32F);

		// Apply Harris corner detection
		cv::Mat cornerResponse;
		cv::cornerHarris(image, cornerResponse, 3, 3, 0.04);

		// Threshold the corner response to find corners
		cv::Mat cornerMask;
		cv::threshold(cornerResponse, cornerMask, threshold, 255, cv::THRESH_BINARY);

  
		// for(int i=0; i<values.size(); i++){
		// 	cout<<", " <<image.at<float>(i, 0);
		// }
		// for(int i=0; i<values.size(); i++){
		// 	cout<<", " <<cornerMask.at<float>(i, 0);
		// }
		// cout<<"cornerResponse: "<< cornerResponse.size()<<endl;
   	 // Find corner indices
  		return cornerMask;
	}


tuple<int, int> findCornersAndPlot(const std::vector<float>& values, double threshold = 0.005, const std::string& windowName = "Scatter Plot")
{

	cv::Mat corners = findCorners(values,threshold);
	//cout<<corners.size()<<endl;
 
	// vector<int> outCorner;
    double minVal = *std::min_element(values.begin(), values.end());
    double maxVal = *std::max_element(values.begin(), values.end());
	double range = maxVal - minVal;
	int8_t moveForward=1; 
	int8_t amountToCheck = 20; 
	int middleIdx = values.size()/2;
	
	int biggestMiddle = -1; 
	int start =-1; 
 	cv::Mat scatterPlot(values.size(), 600, CV_8UC3, cv::Scalar(255, 255, 255));
    int x = 0;
	
     for (const double& value : values) {
	// 	cout<<"corners[x] "<<corners.at<float>(x, 0)<<endl;
        if (corners.at<float>(x, 0)==255 ||value==-1) {
 			cv::line(scatterPlot, cv::Point(x, 0), cv::Point(x, scatterPlot.rows - 1), cv::Scalar(20, 100, 20), 5);
			if(abs(x-middleIdx)<amountToCheck)
			{
				// we have corner in the middle
				moveForward =0;
			};
		}
		else {
			// corners.push_back(0);
            // Calculate the normalized position of the data point
            int y = static_cast<int>((scatterPlot.rows - 1) - ((value - minVal) / range * (scatterPlot.rows - 1))) + 100;
             // Set the color based on the range index
            cv::Scalar color(100, 0, 100);

			if(start>-1 && value!=0)
			{
				biggestMiddle = biggestMiddle <(start+ x)/2? (start+ x)/2 : biggestMiddle;
				start =-1;
			}
			else if(value==0) 
				{color=cv::Scalar(0,0,0);
				if(start ==-1){start=x;}
			};
            // Draw a circle at the data point position with the determined color
            cv::circle(scatterPlot, cv::Point(x, y), 3, color, cv::FILLED);
        }
        x++;
    }
	//int max = findMiddleIndexOfLongestConsecutiveZeros(values);
	cv::line(scatterPlot, cv::Point(biggestMiddle, 0), cv::Point(biggestMiddle, scatterPlot.rows - 1), cv::Scalar(100, 100, 100), 5);

 //	cout<<"biggestMiddle: "<<biggestMiddle<<endl;

    // Display the scatter plot with vertical lines
	// cv::flip(scatterPlot, scatterPlot, 1);

    cv::imshow(windowName, scatterPlot);
    cv::waitKey(1);
	return make_tuple(moveForward, biggestMiddle);
}

void plotScatter(const std::vector<float>& data, std::vector<int>& obsIdx, std::string windowName_)
{
    if (data.empty()) {
        std::cout << "Data vector is empty." << std::endl;
        return;
    }

	// std::vector<int> xxx = findCorners(data);
    // Create a black image with a white background
    cv::Mat image(data.size(), 600, CV_8UC3, cv::Scalar(255, 255, 255));

    // Determine the minimum and maximum values in the data
    float minVal = *std::min_element(data.begin(), data.end());
    float maxVal = *std::max_element(data.begin(), data.end());

    // Calculate the range of data values
    float range = maxVal - minVal;

    // Calculate the color step for each range
    int colorStep = 0;
    if (!obsIdx.empty()) {
        int sizeObjects = obsIdx.size() / 2;
        colorStep = 255 / sizeObjects;
    }

    // Plot the data points as circles on the image
    int x = 0;
	bool stp = false;
    cv::Scalar color(0, 255, 255);
    for (const float& value : data) {
        if (value == -1) {
            // Draw a straight line from the bottom to the top
			// cout<<"Line Drawn: x: "<<x<<endl;
			stp = true;
	        cv::line(image, cv::Point(x, 0), cv::Point(x, 500 - 1), cv::Scalar(20, 100, 20), 5);

        } else if (value == 0) {
            // Set color to white
            color = cv::Scalar(255, 255, 255);
        } else {
            // Calculate the normalized position of the data point
            int y = static_cast<int>(image.rows - 1 - ((value - minVal) / range * (image.rows - 1))) + 100;
            // Determine the color based on the range index
            int rangeIndex = -1; 
            // Set the color based on the range index
            color = cv::Scalar(rangeIndex * colorStep, 0, 255 - rangeIndex * colorStep);
            // Draw a circle at the data point position with the determined color
           cv::circle(image, cv::Point(x, y), 3, color, cv::FILLED);
        }
        x++;
    }

    // Display the scatter plot
    cv::imshow(windowName_, image);
	// if(stp) cv::waitKey(10000); 
    cv::waitKey(1);
}


void printDetectedObstacles(const std::vector<obstacle>& obstacles)
{
    std::cout << "Detected Obstacles: " << obstacles.size() << std::endl;
    for (int i = 0; i < obstacles.size(); i++)
    {
        std::cout << "Obstacle " << i << ": Begin = " << obstacles[i].begin << ", End = " << obstacles[i].end << std::endl;
    }
}


private:
	printingObject() = default;
};
int printingObject::currentPrintingVal= 0;