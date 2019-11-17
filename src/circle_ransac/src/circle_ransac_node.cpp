#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;


// A function to generate synthetic data
void generateData(
	std::vector<cv::Point2d> &points_, // The vector where the generated points should be stored
	const double &noise_, // The noise parameter added to the point coordinates
	const size_t &number_of_inliers_, // The number of inliers (i.e. points on the line) to generate
	const size_t &number_of_outliers_, // The number of outliers (i.e. random points) to generate
	const size_t &number_of_lines_,
	const cv::Size &image_size_); // The size of the image 

// A function to draw points into an image
void drawPoints(
	const std::vector<cv::Point2d> &points_, // The points to be drawn
	cv::Mat &image_, // The image where the points are supposed to be drawn
 	const cv::Scalar &color_, // The color used for the drawing
	const double &size_ = 3.0, // The radius of circles drawn as points
	const std::vector<int> * inliers_ = nullptr); // A subset of the points

// The function fitting a 2D line by applying RANSAC
void fitLineRANSAC(
	const std::vector<cv::Point2d> * const points_, // The points used for the line fitting
	std::vector<int> &inliers_, // The inliers of the obtained line
	cv::Mat &found_line_, // The parameters of the obtained line
	const double &threshold_, // The inlier-outlier threshold used for determining which points are inliers
	const double &confidence_); // The required iteration number 

size_t getIterationNumber(const double confidence_,
	const size_t inlier_number_,
	const size_t point_number_,
	const size_t sample_size_);

// Draw a 2D line to an image
void draw2DLine(
	cv::Mat &image_, // The image where the line is supposed to be drawn
	const cv::Mat &line_, // The line parameters
	const cv::Scalar &color_, // The color of the drawing
	const double &size_); // The line weight

// Return a random number in-between 0 and 1.
double getRandomNumber();

// Fit a 2D line to a set of 2D points by least-squares fitting
void fitLineLSQ(
	const std::vector<cv::Point2d> * const points_, // All points
	const std::vector<int> &inliers_, // The subset of points which are used for the fitting
	cv::Mat &line_); // The estimated line parameters

int main(int argc, char* argv[])
{
	std::vector<cv::Point2d> points; // The vector where the generated points are stored
	cv::Mat image = cv::Mat::zeros(600, 600, CV_8UC3); // The generated image
	const double noise = 5., // The noise (in pixels) added to the point coordinates
		threshold = 5.; // The inlier-outlier threshold for RANSAC
	const size_t number_of_inliers = 100, // The number inliers to be generated
		number_of_outliers = 100; // The number of outlier to be generated

	// Generating a synthetic scene to have points on which RANSAC
	// can be tested.
	generateData(points, // Generated 2D points
		noise, // Noise added to the point coordinates
		number_of_inliers, // Number of inliers
		number_of_outliers, // Number of outliers
		4,
		cv::Size(image.cols, image.rows)); // Size of the image

	// Draw the points to the image
	drawPoints(points,  // Input 2D points
		image,// The image to draw
		cv::Scalar(255,255,255)); // Color of the points

	// Show the image with the points
	cv::imshow("Input image", image);

	std::vector<int> inliers; // The  found inliers
	std::vector<cv::Point2d> tmp_points;
	for(int i = 0; i<4; i++){
		for(int p = 0; p<points.size(); p++){
			bool write = 1;
			for(int a = 0; a<inliers.size(); a++){
				if(p == inliers[a]){
					write = 0;
					break;
				}
			}

			if(write){
				tmp_points.push_back(points[p]);
			}
		}	

		cv::Mat found_line; // The found line parameters
		// Find a line by RANSAC
		fitLineRANSAC(&tmp_points, // Input 2D points
			inliers, // Obtained inliers
			found_line, // Obtained line
			threshold, // Threshold
			0.99); // The image

		cv::Mat polished_line = found_line.clone(); // The polished line parameters
		// Re-calculate the line parameters by applying least-squared fitting to all found inliers
		fitLineLSQ(&tmp_points,  // Input 2D points
			inliers,  // The found inliers
			polished_line); // The refined model parameters

	points = tmp_points;

	
		/*// Draw the inliers and the found line
		drawPoints(points,  // Input 2D points
			image, // The image to draw
			cv::Scalar(0,255,0), // Color of the points
			3, // Size of the drawn points
			&inliers); // Inliers*/

		// Draw the found line
		draw2DLine(image,
			polished_line,
			cv::Scalar(0, 0, 255),
			2);

	}
	// Show the image with the points
	cv::imshow("Output image", image);
	// Wait for keypress
	cv::waitKey(0);

	return 0;
}

void draw2DLine(
	cv::Mat &image_, // The image where the line is supposed to be drawn
	const cv::Mat &line_, // The line parameters
	const cv::Scalar &color_, // The color of the drawing
	const double &size_) // The line weight
{
	const double &a = line_.at<double>(0);
	const double &b = line_.at<double>(1);
	const double &c = line_.at<double>(2);

	const double x1 = 0;
	const double y1 = (-x1 * a - c) / b;

	const double x2 = image_.cols;
	const double y2 = (-x2 * a - c) / b;

	cv::line(image_,
		cv::Point2d(x1, y1),
		cv::Point2d(x2, y2),
		color_,
		size_);
}

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


// Draw points to the image
void drawPoints(
	const std::vector<cv::Point2d> &points_, // The points to be drawn
	cv::Mat &image_, // The image where the points are supposed to be drawn
	const cv::Scalar &color_, // The color used for the drawing
	const double &size_, // The radius of circles drawn as points
	const std::vector<int> * inliers_) // A subset of the points
{
	if (inliers_ == nullptr)
		for (const auto &point : points_)
			circle(image_, point, size_, color_, -1);
	else
		for (const auto &point_idx : *inliers_)
			circle(image_, points_[point_idx], size_, color_, -1);
}

// Generate a synthetic line and sample that. Then add outliers to the data.
void generateData(
	std::vector<cv::Point2d> &points_, // The vector where the generated points should be stored
	const double &noise_, // The noise parameter added to the point coordinates
	const size_t &number_of_inliers_, // The number of inliers (i.e. points on the line) to generate
	const size_t &number_of_outliers_, // The number of outliers (i.e. random points) to generate
	const size_t &number_of_lines_, // The number of outliers (i.e. random points) to generate
	const cv::Size &image_size_) // The size of the image 
{

	double a, b, c;
	// Generate random points on that line
	double x, y;
	points_.reserve(number_of_lines_* number_of_inliers_+number_of_outliers_);
	points_.reserve(number_of_inliers_ + number_of_outliers_);
	for (auto j = 0; j<number_of_lines_; ++j){
		const double alpha = getRandomNumber() * 3.14; // A random angle determining the line direction
		// Generate random line by its normal direction and a center point
		cv::Point2d center(getRandomNumber() * image_size_.width,
			getRandomNumber() * image_size_.height); // A point of the line
		a = sin(alpha); // The x coordinate of the line normal
		b = cos(alpha); // The y coordinate of the line normal
		c = -a * center.x - b * center.y; // The offset of the line coming from equation "a x + b y + c = 0"

		for (auto i = 0; i < number_of_inliers_; ++i)
		{
			x = getRandomNumber() * image_size_.width; // Generate a random x coordinate in the window
			y = -(a * x + c) / b; // Calculate the corresponding y coordinate

			// Add the point to the vector after adding random noise
			points_.emplace_back(
				cv::Point2d(x + noise_ * getRandomNumber(), y + noise_ * getRandomNumber()));
		}
	}

	// Add outliers
	for (auto i = 0; i < number_of_outliers_; ++i)
	{
		x = getRandomNumber() * image_size_.width; // Generate a random x coordinate in the window
		y = getRandomNumber() * image_size_.height; // Generate a random y coordinate in the window

		// Add outliers, i.e., random points in the image
		points_.emplace_back(cv::Point2d(x, y));
	}
}



double getRandomNumber()
{
	return static_cast<double>(rand()) / RAND_MAX;
}

void MultiLineFitting(){

}

// Apply RANSAC to fit points to a 2D line
void fitLineRANSAC(
	const std::vector<cv::Point2d> * const points_, // The points used for the line fitting
	std::vector<int> &inliers_, // The inliers of the obtained line
	cv::Mat &found_line_, // The parameters of the obtained line
	const double &threshold_, // The inlier-outlier threshold used for determining which points are inliers
	const double &confidence_) // The required iteration number
	{
	constexpr size_t sample_size = 2; // Sample size
	size_t * const sample = new size_t[sample_size];
	std::vector<int> tmp_inliers;
	tmp_inliers.reserve(points_->size());
	found_line_.create(3, 1, CV_64F);
	size_t iteration_number = std::numeric_limits<size_t>::max();
	
	for (size_t iteration = 0; iteration < iteration_number; ++iteration)
	{
		// Select a random sample of size two
		for (size_t sample_idx = 0; sample_idx < sample_size; ++sample_idx)
		{
			// Select a points via its index randomly
			size_t idx = round(getRandomNumber() * (points_->size() - 1));
			sample[sample_idx] = idx;

			// Check if the selected index has been already selected
			for (size_t prev_sample_idx = 0; prev_sample_idx < sample_idx; ++prev_sample_idx)
			{
				if (sample[prev_sample_idx] == sample[sample_idx])
				{
					--sample_idx;
					continue;
				}
			}
		}

		// Fit a line to the selected points
		cv::Point2d pt1 = points_->at(sample[0]); // The first point of the line
		cv::Point2d pt2 = points_->at(sample[1]); // The second point of the line
 		cv::Point2d v = pt2 - pt1;  // The direction of the line
		v = v / norm(v); // Normalize the direction since the length does not matter
		cv::Point2d n(-v.y, v.x); // The normal of the line, i.e., the direction rotated by 90�.
		double a = n.x, // The x coordinate of the normal
			b = n.y; // The y coordinate of the normal
		double c = -a * pt1.x - b * pt1.y; // The offset coming from equation "a x + b y + c = 0"
		
		// Iterate through all the points and count the inliers
		tmp_inliers.resize(0);
		for (size_t point_idx = 0; point_idx < points_->size(); ++point_idx)
		{
			const double &x = points_->at(point_idx).x,
				&y = points_->at(point_idx).y;
			const double distance = abs(a * x + b * y + c);

			if (distance < threshold_)
				tmp_inliers.emplace_back(point_idx);
		}

		// If the current line has more inliers than the previous so-far-the-best, update
		// the best parameters
		if (tmp_inliers.size() > inliers_.size())
		{
			tmp_inliers.swap(inliers_);
		
			found_line_.at<double>(0) = a;
			found_line_.at<double>(1) = b;
			found_line_.at<double>(2) = c;

			iteration_number = getIterationNumber(confidence_,
					inliers_.size(),
					points_->size(),
					2);
		}
	}

	// Clean up the memory
	delete[] sample;
}

// Apply Least-Squares line fitting
void fitLineLSQ(
	const std::vector<cv::Point2d> * const points_, // All points
	const std::vector<int> &inliers_, // The subset of points which are used for the fitting
	cv::Mat &line_) // The estimated line parameters
{
	const size_t n = inliers_.size();
	cv::Mat A(n, 3, CV_64F);
	double *A_ptr = reinterpret_cast<double*>(A.data);
	
	for(size_t i = 0; i<n; ++i){
		double x = points_->at(inliers_[i]).x;
		double y = points_->at(inliers_[i]).y;
		*(A_ptr++) = x;
		*(A_ptr++) = y;
		*(A_ptr++) = 1;
	}
	
	cv::Mat AtA = A.t() * A;
	cv::Mat evals, evecs;
	cv::eigen(AtA, evals, evecs);

	line_ = evecs.row(2);



	// 3[A_ptr] = 0;
}