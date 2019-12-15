#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

// ************** CHANGABLES****
// distance from circle in pixles, inside of which other points are considered inliers
const double thresh = 5.0;
// what confidence does the algorithm need to reach to be satisfied with the found circle
const double confidence = 0.99;
// *****************************


size_t getIterationNumber(const double confidence_,
	const size_t inlier_number_,
	const size_t point_number_,
	const size_t sample_size_)
{
	const double one_minus_confidence = 1.0 - confidence_;
	const double log_confidence = log(one_minus_confidence);
	const double inlier_ratio = static_cast<double>(inlier_number_) / point_number_;
	const double pow_inlier_ratio = std::pow(inlier_ratio, sample_size_);

	return log_confidence / log(1.0 - pow_inlier_ratio);
}

// Function to find the circle on 
// which the given three points lie 
void findCircle(const Point2d& p1, const Point2d& p2, const Point2d& p3,
                Point2d& out_center, double& out_radius)
{ 
    int x12 = p1.x - p2.x; 
    int x13 = p1.x - p3.x; 
  
    int y12 = p1.y - p2.y; 
    int y13 = p1.y - p3.y; 
  
    int y31 = p3.y - p1.y; 
    int y21 = p2.y - p1.y; 
  
    int x31 = p3.x - p1.x; 
    int x21 = p2.x - p1.x; 
  
    // p1.x^2 - p3.x^2 
    int sx13 = pow(p1.x, 2) - pow(p3.x, 2); 
  
    // p1.y^2 - p3.y^2 
    int sy13 = pow(p1.y, 2) - pow(p3.y, 2); 
  
    int sx21 = pow(p2.x, 2) - pow(p1.x, 2); 
    int sy21 = pow(p2.y, 2) - pow(p1.y, 2); 
  
    int f = ((sx13) * (x12) 
             + (sy13) * (x12) 
             + (sx21) * (x13) 
             + (sy21) * (x13)) 
            / (2 * ((y31) * (x12) - (y21) * (x13))); 
    int g = ((sx13) * (y12) 
             + (sy13) * (y12) 
             + (sx21) * (y13) 
             + (sy21) * (y13)) 
            / (2 * ((x31) * (y12) - (x21) * (y13))); 
  
    int c = -pow(p1.x, 2) - pow(p1.y, 2) - 2 * g * p1.x - 2 * f * p1.y; 
  
    // eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0 
    // where centre is (h = -g, k = -f) and radius r 
    // as r^2 = h^2 + k^2 - c 
    int h = -g; 
    int k = -f; 
    int sqr_of_r = h * h + k * k - c; 
  
    // r is the radius 
    double r = sqrt(sqr_of_r); 
    
    // cout << "Centre = (" << h << ", " << k << ")" << endl; 
    // cout << "Radius = " << r; 

    out_center.x = h;
    out_center.y = k;
    out_radius = r;
} 

double getRandomNumber()
{
	return static_cast<double>(rand()) / RAND_MAX;
}

double dist2d(const Point2d& p1, const Point& p2){
  return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}


// Apply RANSAC to fit points to a circle
int fitCircleRANSAC(
	const std::vector<cv::Point2d> * const points_, // The points used for the circle fitting
	std::vector<int> &inliers_, // The inliers of the obtained circle
	cv::Mat &found_circle_, // The parameters of the obtained circle
	const double &threshold_, // The inlier-outlier threshold used for determining which points are inliers
	const double &confidence_) // The required iteration number
	{

  // check if given points vector is not empty
  if(points_->size()==0){
    cout << "Points vector is empty." << endl;
    return 0;
  }

  // initialisation of vars
  const auto sample_size = 3;  // Sample size
  int *sample = new int[sample_size];
  int iteration_number = INT_MAX;  
  std::vector<int> tmp_inliers;
  tmp_inliers.reserve(points_->size());
  found_circle_.create(3, 1, CV_16U);

  for (auto iteration = 0; iteration < iteration_number; ++iteration){
		// Select a random sample of size sample_size
		for (auto sample_idx = 0; sample_idx < sample_size; ++sample_idx){
			// Select a points via its index randomly
			int idx = round(getRandomNumber() * (points_->size() - 1));
			sample[sample_idx] = idx;

			// Check if the selected index has been already selected
			for (auto prev_sample_idx = 0; prev_sample_idx < sample_idx; ++prev_sample_idx){
				if (sample[prev_sample_idx] == sample[sample_idx]){
					--sample_idx;
					continue;
				}
			}
		}

    // fit a circle to the selecte points
    Point2d pt1 = points_->at(sample[0]); // First point
    Point2d pt2 = points_->at(sample[1]); // Second point
    Point2d pt3 = points_->at(sample[2]); // Third point

    Point2d c; //found center
    double r; // found radius
    findCircle(pt1,pt2,pt3,c,r);

    // Iterate through all the points and count the inliers
    tmp_inliers.resize(0);
    for (auto point_idx = 0; point_idx < points_->size(); ++point_idx){
      const double &x = points_->at(point_idx).x,
          &y = points_->at(point_idx).y;
      Point2d f;
      f.x = x;
      f.y = y;

      // distance of the point f from circle with radius r and center c
      const double dist = fabs(dist2d(c,f) - r);

      // if distance is whitin thresh, save it into inliers list
      if(dist<threshold_){
        tmp_inliers.push_back(point_idx);
      }
    }

    // If the current circle has more inliers than the previous so-far-the-best, update
		// the best parameters
		if (tmp_inliers.size() > inliers_.size()){
			tmp_inliers.swap(inliers_);
  
      // save the current parameters
			found_circle_.at<int>(0) = c.x;
			found_circle_.at<int>(1) = c.y;
			found_circle_.at<int>(2) = (int)r;

			iteration_number = getIterationNumber(confidence_,
					inliers_.size(),
					points_->size(),
					2);
		}
  }
  // Clean up the memory
	delete[] sample;
  // return
  return 1;
}




int main(int argc, char* argv[]){
	if (argc != 2){
		cout << "No arguments found" << endl;
		return -1;
	}

  // import circle points
  ifstream myfile(argv[1]);
  string data;
  
  // the container in which we will store all the points
  vector<Point2d> points;

  // a temporary point to hold the three coords in while we read them all
  Point tmp;

  int maxX = 0;
  int maxY = 0;
  // read in two doubles from infile into the two coords in tmp
  while (myfile >> tmp.x && myfile >> tmp.y){
    // add a copy of tmp to points
    points.push_back(tmp);

    if(tmp.x>maxX) maxX = tmp.x;
    if(tmp.y>maxY) maxY = tmp.y;
  }
  myfile.close();

  // create an image from the vector of points
  Mat img = Mat(Size(maxX+20, maxY+20), CV_8UC3);
  for(auto i:points){
    circle(img, Point(i.x, i.y), 1, Scalar(255,255,255));
  }
	
  // create a vector to save inliers to 
  std::vector<int> inliers; // The  found inliers
  std::vector<cv::Point2d> tmp_points;

  // create a mat to save found circle parameters to (center.x, center.y, radius)
  cv::Mat found_circle;
  if(!fitCircleRANSAC(&points, inliers, found_circle, thresh, confidence)){
    cout << "Circle RANSAC failed" << endl;
    return -1;
  }
  
  // draw the detected circle
  circle(img, Point(found_circle.at<int>(0), found_circle.at<int>(1)), found_circle.at<int>(2), Scalar(0,0,255));

	int key;
  while (1)
  {
    key = waitKey(30);

    if (key == 27)
    {
      // if "esc" is pressed end the program
      std::cout << "Closing the program because esc pressed \n";
      break;
    }

    // show the resulting image
    imshow("original", img);
    
  }

  return 0;
}
