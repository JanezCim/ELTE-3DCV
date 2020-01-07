#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <chrono> 

using namespace cv;
using namespace std;

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

// Detecting point correspondences in two images
void detectFeatures(
	const cv::Mat &image1_, // The source images
	const cv::Mat &image2_, // The destination image
	std::vector<cv::Point2d> &source_points_, // Points in the source image
	std::vector<cv::Point2d> &destination_points_); // Points in the destination image

// A function estimating the fundamental matrix from point correspondences
// by RANSAC.
void ransacFundamentalMatrix(
	const std::vector<cv::Point2d> &input_source_points_, // Points in the source image
	const std::vector<cv::Point2d> &input_destination_points_, // Points in the destination image
	const std::vector<cv::Point2d> &normalized_input_src_points_, // Normalized points in the source image
	const std::vector<cv::Point2d> &normalized_input_destination_points_, // Normalized points in the destination image
	const cv::Mat &T1_, // Normalizing transformation in the source image
	const cv::Mat &T2_, // Normalizing transformation in the destination image
	cv::Mat &fundamental_matrix_, // The estimated fundamental matrix
	std::vector<size_t> &inliers_, // The inliers of the fundamental matrix
	double confidence_, // The required confidence of RANSAC
	double threshold_); // The inlier-outlier threshold

// A function estimating the fundamental matrix from point correspondences
// by least-squares fitting.
void getFundamentalMatrixLSQ(
	const std::vector<cv::Point2d> &source_points_, // Points in the source image
	const std::vector<cv::Point2d> &destination_points_, // Points in the destination image
	cv::Mat &fundamental_matrix_); // The estimated fundamental matrix

// A function decomposing the essential matrix to the projection matrices
// of the two cameras.
void getProjectionMatrices(
	const cv::Mat &essential_matrix_, // The parameters of the essential matrix
	const cv::Mat &K1_, // The intrinsic camera parameters of the source image
	const cv::Mat &K2_, // The intrinsic camera parameters of the destination image
	const cv::Mat &src_point_, // A point in the source image
	const cv::Mat &dst_point_, // A point in the destination image
	cv::Mat &projection_1_, // The projection matrix of the source image
	cv::Mat &projection_2_); // The projection matrix of the destination image

// A function estimating the 3D point coordinates from a point correspondences
// from the projection matrices of the two observing cameras.
void linearTriangulation(
	const cv::Mat &projection_1_, // The projection matrix of the source image
	const cv::Mat &projection_2_, // The projection matrix of the destination image
	const cv::Mat &src_point_, // A point in the source image
	const cv::Mat &dst_point_, // A point in the destination image
	cv::Mat &point3d_); // The estimated 3D coordinates

// Normalizing the point coordinates for the fundamental matrix estimation
void normalizePoints(
	const std::vector<cv::Point2d> &input_source_points_, // Points in the source image
	const std::vector<cv::Point2d> &input_destination_points_, // Points in the destination image
	std::vector<cv::Point2d> &output_source_points_, // Normalized points in the source image
	std::vector<cv::Point2d> &output_destination_points_, // Normalized points in the destination image
	cv::Mat &T1_, // Normalizing transformation in the source image
	cv::Mat &T2_); // Normalizing transformation in the destination image

// Return the iteration number of RANSAC given the inlier ratio and
// a user-defined confidence value.
int getIterationNumber(int point_number_, // The number of points
	int inlier_number_, // The number of inliers
	int sample_size_, // The sample size
	double confidence_); // The required confidence


/******************************* MAIN *******************************************************/


int main(int argc, char* argv[]){
  if(argc != 3){
    cout << "num of added parameters must be eather 2. See README" << endl;
		return -1;
  }

  Mat src_img;
  Mat dst_img;
  Mat gs_src_img;
  Mat gs_dst_img;

  src_img = imread(argv[1]);
  dst_img = imread(argv[2]);

  if (!src_img.data){
    cout << "Could not open or find the source image" << endl;
    return -1;
  }
  else{
    cout << "Imported the source image :)" << endl;
  }
  
  if (!dst_img.data){
    cout << "Could not open or find the destination image" << endl;
    return -1;
  }
  else{
    cout << "Imported the destination image :)" << endl;
  }

  vector<Point2d> src_points, dst_points;
  detectFeatures(src_img, dst_img, src_points, dst_points);

	cv::Mat T1, T2; // Normalizing transforcv::Mations
	std::vector<cv::Point2d> norm_src_points, norm_dst_points; // Normalized point correspondences
	normalizePoints(src_points, // Points in the first image 
		dst_points,  // Points in the second image
		norm_src_points,  // Normalized points in the first image
		norm_dst_points, // Normalized points in the second image
		T1, // Normalizing transforcv::Mation in the first image
		T2); // Normalizing transforcv::Mation in the second image

  cv::Mat F; // The fundamental matrix
	std::vector<size_t> inliers; // The inliers of the fundamental matrix
	ransacFundamentalMatrix(src_points,  // Points in the first image 
		dst_points,   // Points in the second image
		norm_src_points,  // Normalized points in the first image 
		norm_dst_points, // Normalized points in the second image
		T1, // Normalizing transforcv::Mation in the first image
		T2, // Normalizing transforcv::Mation in the second image
		F, // The fundamental matrix
		inliers, // The inliers of the fundamental matrix
		0.99, // The required confidence in the results 
		2.0); // The inlier-outlier threshold
  

  // Draw the points and the corresponding epipolar lines
	constexpr double resize_by = 3.0;
	cv::Mat tmp_src_img, tmp_dst_img;
	// resize(src_img, tmp_src_img, cv::Size(), tmp_src_img.cols / resize_by, tmp_src_img.rows / resize_by);
	// resize(dst_img, tmp_dst_img, cv::Size(), tmp_dst_img.cols / resize_by, tmp_dst_img.rows / resize_by);

	std::vector<Point2d> src_inliers(inliers.size()), dst_inliers(inliers.size());
	std::vector<Point2d> norm_src_inliers(inliers.size()), norm_dst_inliers(inliers.size());
	std::vector<cv::DMatch> inlier_matches(inliers.size());
	for (auto inl_idx = 0; inl_idx < inliers.size(); ++inl_idx)
	{
		// Construct the cv::Matches std::vector for the drawing
		src_inliers[inl_idx] = src_points[inliers[inl_idx]];// / resize_by;
		dst_inliers[inl_idx] = dst_points[inliers[inl_idx]];// / resize_by;
		norm_src_inliers[inl_idx] = norm_src_points[inliers[inl_idx]];// / resize_by;
		norm_dst_inliers[inl_idx] = norm_dst_points[inliers[inl_idx]];// / resize_by;

		inlier_matches[inl_idx].queryIdx = inl_idx;
		inlier_matches[inl_idx].trainIdx = inl_idx;
	}

	// setup A matrix according to the slide 45 in http://cg.elte.hu/~hajder/vision/slides/lec01_camera.pdf
	int sample_number = norm_src_inliers.size();
	cv::Mat A(sample_number*2, 9, CV_32F);
	for (int i = 0; i < sample_number; i++){
		const float u = norm_src_inliers[i].x, v = norm_src_inliers[i].y;
		const float ud = norm_dst_inliers[i].x, vd = norm_dst_inliers[i].y;

		A.at<float>(i*2,0)=u;
		A.at<float>(i*2,1)=v;
		A.at<float>(i*2,2)=1;
		A.at<float>(i*2,3)=0;
		A.at<float>(i*2,4)=0;
		A.at<float>(i*2,5)=0;
		A.at<float>(i*2,6)=-u*ud;
		A.at<float>(i*2,7)=-v*ud;
		A.at<float>(i*2,8)=-ud;

		A.at<float>(i*2+1,0)=0;
		A.at<float>(i*2+1,1)=0;
		A.at<float>(i*2+1,2)=0;
		A.at<float>(i*2+1,3)=u;
		A.at<float>(i*2+1,4)=v;
		A.at<float>(i*2+1,5)=1;
		A.at<float>(i*2+1,6)=-u*vd;
		A.at<float>(i*2+1,7)=-v*vd;
		A.at<float>(i*2+1,8)=-v;
	}

	// apply the SVD to A
	Mat w, u, vt;
	cv::SVDecomp(A, w, u, vt);

	// the last (9th) value of vt is the optimal homography
	// here we convert it from 9x1 to 3x3
	cv::Mat h(3, 3, CV_32F);
	h.at<float>(0,0)=vt.at<float>(8,0);
	h.at<float>(0,1)=vt.at<float>(8,1);
	h.at<float>(0,2)=vt.at<float>(8,2);
	h.at<float>(1,0)=vt.at<float>(8,3);
	h.at<float>(1,1)=vt.at<float>(8,4);
	h.at<float>(1,2)=vt.at<float>(8,5);
	h.at<float>(2,0)=vt.at<float>(8,6);
	h.at<float>(2,1)=vt.at<float>(8,7);
	h.at<float>(2,2)=vt.at<float>(8,8);

	// convert the homography to double
	h.convertTo(h, CV_64F);

	// denormalize homography
	h = T2.inv() * h * T1;

	// wrap the prespective with new homography
	Mat trans_img;
	warpPerspective(src_img, trans_img, h, dst_img.size());

	// resize the image before showing it
	float resize_factor = 0.25;
	resize(src_img, src_img, Size(),resize_factor,resize_factor);
  resize(dst_img, dst_img, Size(),resize_factor,resize_factor);
	resize(trans_img, trans_img, Size(),resize_factor,resize_factor);

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
    imshow("source", src_img);
    imshow("destination", dst_img);
    imshow("warped image", trans_img);
  }

  return 0;
}


void detectFeatures(
	const cv::Mat &image1_,
	const cv::Mat &image2_,
	std::vector<cv::Point2d> &source_points_,
	std::vector<cv::Point2d> &destination_points_)
{
	printf("Detect SIFT features\n");
	cv::Mat descriptors1, descriptors2; // The descriptors of the found keypoints in the two images
	std::vector<cv::KeyPoint> keypoints1, keypoints2; // The keypoints in the two images

	cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create(); // The SIFT detector
	detector->detect(image1_, keypoints1); // Detecting keypoints in the first image
	detector->compute(image1_, keypoints1, descriptors1); // Computing the descriptors of the keypoints in the first image
	printf("Features found in the first image: %d\n", keypoints1.size());

	detector->detect(image2_, keypoints2); // Detecting keypoints in the second image
	detector->compute(image2_, keypoints2, descriptors2); // Computing the descriptors of the keypoints in the second image
	printf("Features found in the second image: %d\n", keypoints2.size());

	// Do the descriptor matching by an approximated k-nearest-neighbors algorithm (FLANN) with k = 2.
	std::vector<std::vector< cv::DMatch >> matches_vector;
	cv::FlannBasedMatcher matcher(new cv::flann::KDTreeIndexParams(5), new cv::flann::SearchParams(50));
	matcher.knnMatch(descriptors1, descriptors2, matches_vector, 2);

	// Iterate through the matches, apply the SIFT ratio test and if the match passes,
	// add it to the vector of found correspondences
	for (auto m : matches_vector)
	{
		if (m.size() == 2 && m[0].distance < m[1].distance * 0.80)
		{
			auto& kp1 = keypoints1[m[0].queryIdx];
			auto& kp2 = keypoints2[m[0].trainIdx];
			source_points_.push_back(kp1.pt);
			destination_points_.push_back(kp2.pt);
		}
	}

	printf("Detected correspondence number: %d\n", source_points_.size());
}

void normalizePoints(
	const std::vector<cv::Point2d> &input_source_points_,
	const std::vector<cv::Point2d> &input_destination_points_,
	std::vector<cv::Point2d> &output_source_points_,
	std::vector<cv::Point2d> &output_destination_points_,
	cv::Mat &T1_, 
	cv::Mat &T2_)
{
	T1_ = cv::Mat::eye(3, 3, CV_64F);
	T2_ = cv::Mat::eye(3, 3, CV_64F);

	auto N = input_source_points_.size();
	output_source_points_.resize(N);
	output_destination_points_.resize(N);

	cv::Point2d center_1(0, 0),
		center_2(0, 0);
	for (auto i = 0; i < N; ++i)
	{
		center_1 += input_source_points_[i];
		center_2 += input_destination_points_[i];
	}
	center_1 = center_1 * (1.0 / N);
	center_2 = center_2 * (1.0 / N);

	double mean_1 = 0,
		mean_2 = 0;
	for (auto i = 0; i < N; ++i)
	{
		output_source_points_[i] = input_source_points_[i] - center_1;
		output_destination_points_[i] = input_destination_points_[i] - center_2;

		mean_1 += norm(output_source_points_[i]);
		mean_2 += norm(output_destination_points_[i]);
	}

	mean_1 /= N;
	mean_2 /= N;

	double ratio_1 = sqrt(2) / mean_1,
		ratio_2 = sqrt(2) / mean_2;

	for (auto i = 0; i < N; ++i)
	{
		output_source_points_[i] = output_source_points_[i] * ratio_1;
		output_destination_points_[i] = output_destination_points_[i] * ratio_2;
	}

	T1_ = (cv::Mat_<double>(3, 3) << ratio_1, 0, 0,
		0, ratio_1, 0,
		0, 0, 1) *
		(cv::Mat_<double>(3, 3) << 1, 0, -center_1.x,
			0, 1, -center_1.y,
			0, 0, 1);

	T2_ = (cv::Mat_<double>(3, 3) << ratio_2, 0, 0,
		0, ratio_2, 0,
		0, 0, 1) *
		(cv::Mat_<double>(3, 3) << 1, 0, -center_2.x,
			0, 1, -center_2.y,
			0, 0, 1);
}

void ransacFundamentalMatrix(
	const std::vector<cv::Point2d> &input_src_points_,
	const std::vector<cv::Point2d> &input_destination_points_,
	const std::vector<cv::Point2d> &normalized_input_src_points_,
	const std::vector<cv::Point2d> &normalized_input_destination_points_,
	const cv::Mat &T1_,
	const cv::Mat &T2_,
	cv::Mat &fundamental_matrix_, 
	std::vector<size_t> &inliers_,
	double confidence_, 
	double threshold_)
{
	// The so-far-the-best fundamental matrix
	cv::Mat best_fundamental_matrix;
	// The number of correspondences
	const size_t point_number = input_src_points_.size();
	
	// Initializing the index pool from which the minimal samples are selected
	std::vector<size_t> index_pool(point_number);
	for (size_t i = 0; i < point_number; ++i)
		index_pool[i] = i;

	// The size of a minimal sample
	constexpr size_t sample_size = 8;
	// The minimal sample
	size_t *mss = new size_t[sample_size];

	size_t maximum_iterations = std::numeric_limits<int>::max(), // The maximum number of iterations set adaptively when a new best model is found
		iteration_limit = 5000, // A strict iteration limit which mustn't be exceeded
		iteration = 0; // The current iteration number

	std::vector<cv::Point2d> source_points(sample_size), 
		destination_points(sample_size);

	while (iteration++ < MIN(iteration_limit, maximum_iterations))
	{

		for (auto sample_idx = 0; sample_idx < sample_size; ++sample_idx)
		{
			// Select a random index from the pool
			const size_t idx = round((rand() / (double)RAND_MAX) * (index_pool.size() - 1));
			mss[sample_idx] = index_pool[idx];
			index_pool.erase(index_pool.begin() + idx);

			// Put the selected correspondences into the point containers
			const size_t point_idx = mss[sample_idx];
			source_points[sample_idx] = normalized_input_src_points_[point_idx];
			destination_points[sample_idx] = normalized_input_destination_points_[point_idx];
		}

		// Estimate fundamental matrix
		cv::Mat fundamental_matrix(3, 3, CV_64F);
		getFundamentalMatrixLSQ(source_points, destination_points, fundamental_matrix);
		fundamental_matrix = T2_.t() * fundamental_matrix * T1_; // Denormalize the fundamental matrix

		// Count the inliers
		std::vector<size_t> inliers;
		const double* p = (double *)fundamental_matrix.data;
		for (int i = 0; i < input_src_points_.size(); ++i)
		{
			// Symmetric epipolar distance   
			cv::Mat pt1 = (cv::Mat_<double>(3, 1) << input_src_points_[i].x, input_src_points_[i].y, 1);
			cv::Mat pt2 = (cv::Mat_<double>(3, 1) << input_destination_points_[i].x, input_destination_points_[i].y, 1);

			cv::Mat a = fundamental_matrix * pt1;
			a = a * (1.0 / sqrt(a.at<double>(0)*a.at<double>(0) + a.at<double>(1)*a.at<double>(1)));
			a = pt2.t() * a;

			cv::Mat b = pt2.t() * fundamental_matrix;
			b = b * (1.0 / sqrt(b.at<double>(0)*b.at<double>(0) + b.at<double>(1)*b.at<double>(1)));
			b = b * pt1;

			double dist = 0.5 * (abs(a.at<double>(0)) + abs(b.at<double>(0)));

			if (dist < threshold_)
				inliers.push_back(i);
		}

		// Update if the new model is better than the previous so-far-the-best.
		if (inliers_.size() < inliers.size())
		{
			// Update the set of inliers
			inliers_.swap(inliers);
			inliers.clear();
			inliers.resize(0);
			// Update the model parameters
			best_fundamental_matrix = fundamental_matrix;
			// Update the iteration number
			maximum_iterations = getIterationNumber(point_number,
				inliers_.size(),
				sample_size,
				confidence_);
		}

		// Put back the selected points to the pool
		for (size_t i = 0; i < sample_size; ++i)
			index_pool.push_back(mss[i]);
	}

	delete mss;

	fundamental_matrix_ = best_fundamental_matrix;
}

void getFundamentalMatrixLSQ(
	const std::vector<cv::Point2d> &source_points_,
	const std::vector<cv::Point2d> &destination_points_,
	cv::Mat &fundamental_matrix_)
{
	const int sample_number = source_points_.size();
	cv::Mat evals(1, 9, CV_64F), evecs(9, 9, CV_64F);
	cv::Mat A(sample_number, 9, CV_64F);
	double *A_ptr = reinterpret_cast<double *>(A.data);

	// form a linear system: i-th row of A(=a) represents
	// the equation: (m2[i], 1)'*F*(m1[i], 1) = 0
	for (auto i = 0; i < sample_number; i++)
	{
		const double x0 = source_points_[i].x, y0 = source_points_[i].y;
		const double x1 = destination_points_[i].x, y1 = destination_points_[i].y;

		*(A_ptr++) = x1 * x0;
		*(A_ptr++) = x1 * y0;
		*(A_ptr++) = x1;
		*(A_ptr++) = y1 * x0;
		*(A_ptr++) = y1 * y0;
		*(A_ptr++) = y1;
		*(A_ptr++) = x0;
		*(A_ptr++) = y0;
		*(A_ptr++) = 1;
	}

	// A*(f11 f12 ... f33)' = 0 is singular (7 equations for 9 variables), so
	// the solution is linear subspace of dimensionality 2.
	// => use the last two singular std::vectors as a basis of the space
	// (according to SVD properties)
	cv::Mat cov = A.t() * A;
	eigen(cov, evals, evecs);

	double *F_ptr = reinterpret_cast<double*>(fundamental_matrix_.data);
	memcpy(fundamental_matrix_.data, evecs.row(8).data, 9 * sizeof(double));
}

int getIterationNumber(int point_number_,
	int inlier_number_,
	int sample_size_,
	double confidence_)
{
	const double inlier_ratio = static_cast<float>(inlier_number_) / point_number_;

	static const double log1 = log(1.0 - confidence_);
	const double log2 = log(1.0 - pow(inlier_ratio, sample_size_));
	
	const int k = log1 / log2;
	if (k < 0)
		return INT_MAX;
	return k;
}