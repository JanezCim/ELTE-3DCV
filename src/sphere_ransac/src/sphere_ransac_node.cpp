#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
// #include "sphere_ransac/vector3d.hpp"

using namespace cv;
using namespace std;
typedef float f;

// ************** CHANGABLES****
// distance from sphere in pixles, inside of which other points are considered inliers
const double thresh = 5.0;
// what confidence does the algorithm need to reach to be satisfied with the found sphere
const double confidence = 0.99;

// file to output the xyz file to
std::string file = "output.xyz";
// *****************************

class vector3d
{
public:
    f x,y,z;
    vector3d()  //constructor
    {
        x=0;
        y=0;
        z=0;
    }

    vector3d(f x1,f y1,f z1=0)     //initializing object with values.
    {
        x=x1;
        y=y1;
        z=z1;
    }
    vector3d(const vector3d &vec);    //copy constructor
    vector3d operator+(const vector3d &vec);    //addition
    vector3d &operator+=(const vector3d &vec);  ////assigning new result to the vector
    vector3d operator-(const vector3d &vec);    //substraction
    vector3d &operator-=(const vector3d &vec);  //assigning new result to the vector
    vector3d operator*(f value);    //multiplication
    vector3d &operator*=(f value);  //assigning new result to the vector.
    vector3d operator/(f value);    //division
    vector3d &operator/=(f value);  //assigning new result to the vector
    vector3d &operator=(const vector3d &vec);
    f dot_product(const vector3d &vec); //scalar dot_product
    vector3d cross_product(const vector3d &vec);    //cross_product
    f magnitude();  //magnitude of the vector
    vector3d normalization();   //nor,malized vector
    f square(); //gives square of the vector

    f distance(const vector3d &vec);    //gives distance between two vectors
    f show_X(); //return x
    f show_Y(); //return y
    f show_Z(); //return z
    void disp();    //display value of vectors
};

vector3d::vector3d(const vector3d &vec)
{
    x=vec.x;
    y=vec.y;
    z=vec.z;
}

//addition

vector3d vector3d ::operator+(const vector3d &vec)
{
    return vector3d(x+vec.x,y+vec.y,z+vec.z);
}

vector3d &vector3d ::operator+=(const vector3d &vec)
{
    x+=vec.x;
    y+=vec.y;
    z+=vec.z;
    return *this;
}
//substraction//
vector3d vector3d ::operator-(const vector3d &vec)
{
    return vector3d(x-vec.x,y-vec.y,z-vec.z);
}

vector3d &vector3d::operator-=(const vector3d &vec)
{
    x-=vec.x;
    y-=vec.y;
    z-=vec.z;
    return *this;
}

//scalar multiplication

vector3d vector3d ::operator*(f value)
{
    return vector3d(x*value,y*value,z*value);
}

vector3d &vector3d::operator*=(f value)
{
    x*=value;
    y*=value;
    z*=value;
    return *this;
}

//scalar division
vector3d vector3d ::operator/(f value)
{
    assert(value!=0);
    return vector3d(x/value,y/value,z/value);
}

vector3d &vector3d ::operator/=(f value)
{
    assert(value!=0);
    x/=value;
    y/=value;
    z/=value;
    return *this;
}


vector3d &vector3d::operator=(const vector3d &vec)
{
    x=vec.x;
    y=vec.y;
    z=vec.z;
    return *this;
}

//Dot product
f vector3d::dot_product(const vector3d &vec)
{
    return x*vec.x+vec.y*y+vec.z*z;
}

//cross product
vector3d vector3d ::cross_product(const vector3d &vec)
{
    f ni=y*vec.z-z*vec.y;
    f nj=z*vec.x-x*vec.z;
    f nk=x*vec.y-y*vec.x;
    return vector3d(ni,nj,nk);
}

f vector3d::magnitude()
{
    return sqrt(square());
}

f vector3d::square()
{
    return x*x+y*y+z*z;
}

vector3d vector3d:: normalization()
{
    assert(magnitude()!=0);
    *this/=magnitude();
    return *this;
}

f vector3d::distance(const vector3d &vec)
{
    vector3d dist=*this-vec;
    return dist.magnitude();
}

f vector3d::show_X()
{
    return x;
}

f vector3d::show_Y()
{
    return y;
}

f vector3d::show_Z()
{
    return z;
}

void vector3d::disp()
{
    cout<<x<<" "<<y<<" "<<z<<endl;
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


double determinant(vector<vector<double > >& a , int n)
{
  int i, j, j1, j2;
  double d = 0;
  vector<vector<double > > m(4, vector<double>(4,0));

  if (n == 2)
  {
    // Terminate recursion.
    d = a[0][0] * a[1][1] - a[1][0] * a[0][1];
  }
  else
  {
    d = 0;
    for (j1 = 0; j1 < n; j1++)  // Do each column.
    {
      for (i = 1; i < n; i++)  // Create minor.
      {
        j2 = 0;
        for (j = 0; j < n; j++)
        {
          if (j == j1)
            continue;
          m[i - 1, j2] = a[i, j];
          j2++;
        }
      }

      // Sum (+/-)cofactor * minor.
      d = d + pow(-1.0, j1) * a[0][j1] * determinant(m, n - 1);
    }
  }
  return d;
}

void findSphere1(const Point3d& p1, const Point3d& p2, const Point3d& p3, const Point3d& p4,
                Point3d& out_center, double& out_radius){ 
  // inti a 4x3 vector filled with zeros
  vector<vector<double > > P(4, vector<double>(3,0));

  double m_X0 = 0;
  double m_Y0 = 0;
  double m_Z0 = 0;
  double m_Radius = 0;

  P[0][0] = p1.x;
  P[0][1] = p1.y;
  P[0][2] = p1.z;
  P[1][0] = p2.x;
  P[1][1] = p2.y;
  P[1][2] = p2.z;
  P[2][0] = p3.x;
  P[2][1] = p3.y;
  P[2][2] = p3.z;
  P[3][0] = p4.x;
  P[3][1] = p4.y;
  P[3][2] = p4.z;

  double r, m11, m12, m13, m14, m15;

  vector<vector<double > > a (4, vector<double>(4,0));

  // Find minor 1, 1.
  for (int i = 0; i < 4; i++)
  {
    a[i][0] = P[i][0];
    a[i][1] = P[i][1];
    a[i][2] = P[i][2];
    a[i][3] = 1;
  }
  m11 = determinant(a, 4);

  // Find minor 1, 2.
  for (int i = 0; i < 4; i++)
  {
    a[i][0] = P[i][0] * P[i][0] + P[i][1] * P[i][1] + P[i][2] * P[i][2];
    a[i][1] = P[i][1];
    a[i][2] = P[i][2];
    a[i][3] = 1;
  }
  m12 = determinant(a, 4);

  // Find minor 1, 3.
  for (int i = 0; i < 4; i++)
  {
    a[i][0] = P[i][0] * P[i][0] + P[i][1] * P[i][1] + P[i][2] * P[i][2];
    a[i][1] = P[i][0];
    a[i][2] = P[i][2];
    a[i][3] = 1;
  }
  m13 = determinant(a, 4);

  // Find minor 1, 4.
  for (int i = 0; i < 4; i++)
  {
    a[i][0] = P[i][0] * P[i][0] + P[i][1] * P[i][1] + P[i][2] * P[i][2];
    a[i][1] = P[i][0];
    a[i][2] = P[i][1];
    a[i][3] = 1;
  }
  m14 = determinant(a, 4);

  // Find minor 1, 5.
  for (int i = 0; i < 4; i++)
  {
    a[i][0] = P[i][0] * P[i][0] + P[i][1] * P[i][1] + P[i][2] * P[i][2];;
    a[i][1] = P[i][0];
    a[i][2] = P[i][1];
    a[i][3] = P[i][2];
  }
  m15 = determinant(a, 4);

  // Calculate result.
  if (m11 == 0)
  {
    m_X0 = 0;
    m_Y0 = 0;
    m_Z0 = 0;
    m_Radius = 0;
  }
  else
  {
    m_X0 = 0.5 * m12 / m11;
    m_Y0 = -0.5 * m13 / m11;
    m_Z0 = 0.5 * m14 / m11;
    m_Radius = sqrt(m_X0 * m_X0 + m_Y0 * m_Y0 + m_Z0 * m_Z0 - m15 / m11);
  }

  out_center.x = m_X0;
  out_center.y = m_Y0;
  out_center.z = m_Z0;
  out_radius = m_Radius;
} 

void findSphere(const Point3d& pt1, const Point3d& pt2, const Point3d& pt3, const Point3d& pt4,
                Point3d& out_center, double& out_radius){            
   
  vector3d p1 = vector3d(pt1.x, pt1.y, pt1.z);
  vector3d p2 = vector3d(pt2.x, pt2.y, pt2.z);
  vector3d p3 = vector3d(pt3.x, pt3.y, pt3.z);

  // triangle "edges"
  vector3d t = p2-p1;
  vector3d u = p3-p1;
  vector3d v = p3-p2;

  // triangle normal
  vector3d w = t.cross_product(u);
  double wsl = w.square();
  if (wsl<10e-14) return; // area of the triangle is too small (you may additionally check the points for colinearity if you are paranoid)

  // helpers
  double iwsl2 = 1.0 / (2.0*wsl);
  double tt = t.dot_product(t);
  double uu = u.dot_product(u);
  double uv = u.dot_product(v);
  double tv = t.dot_product(v);
  double vv = v.dot_product(v);

  // result circle
  vector3d circCenter = p1 + (u*tt*uv - t*uu*tv) * iwsl2;
  double   circRadius = sqrt(tt * uu * vv * iwsl2*0.5);
  vector3d circAxis   = w / sqrt(wsl);

  out_center.x = circCenter.x;
  out_center.y = circCenter.y;
  out_center.z = circCenter.z;
  out_radius = circRadius;

}

double getRandomNumber()
{
	return static_cast<double>(rand()) / RAND_MAX;
}

double dist3d(const Point3d& p1, const Point3d& p2){
  return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y)+ (p1.z-p2.z)*(p1.z-p2.z));
}

// Apply RANSAC to fit points to a sphere
int fitSphereRANSAC(
	const std::vector<cv::Point3d> * const points_, // The points used for the sphere fitting
	std::vector<int> &inliers_, // The inliers of the obtained sphere
	cv::Mat &found_sphere_, // The parameters of the obtained sphere
	const double &threshold_, // The inlier-outlier threshold used for determining which points are inliers
	const double &confidence_) // The required iteration number
	{

  // check if given points vector is not empty
  if(points_->size()==0){
    cout << "Points vector is empty." << endl;
    return 0;
  }

  // initialisation of vars
  const auto sample_size = 4;  // Sample size
  int *sample = new int[sample_size];
  int iteration_number = INT_MAX;  
  std::vector<int> tmp_inliers;
  tmp_inliers.reserve(points_->size());
  found_sphere_.create(sample_size, 1, CV_64F);

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

    // fit a sphere to the selected points
    Point3d pt1 = points_->at(sample[0]); // First point
    Point3d pt2 = points_->at(sample[1]); // Second point
    Point3d pt3 = points_->at(sample[2]); // Third point
    Point3d pt4 = points_->at(sample[3]); // Fourth point


    Point3d c; //found center
    double r; // found radius
    findSphere(pt1,pt2,pt3,pt4,c,r);

    // Iterate through all the points and count the inliers
    tmp_inliers.resize(0);
    for (auto point_idx = 0; point_idx < points_->size(); ++point_idx){
      const double &x = points_->at(point_idx).x,
                   &y = points_->at(point_idx).y,
                   &z = points_->at(point_idx).z;
      Point3d f;
      f.x = x;
      f.y = y;
      f.z = z;

      // distance of the point f from sphere_ with radius r and center c
      const double dist = fabs(dist3d(c,f) - r);

      // if distance is whitin thresh, save it into inliers list
      if(dist<threshold_){
        tmp_inliers.push_back(point_idx);
      }
    }

    // If the current sphere_ has more inliers than the previous so-far-the-best, update
		// the best parameters
		if (tmp_inliers.size() > inliers_.size()){
			tmp_inliers.swap(inliers_);
  
      // save the current parameters
			found_sphere_.at<double>(0) = c.x;
			found_sphere_.at<double>(1) = c.y;
			found_sphere_.at<double>(2) = c.z;
			found_sphere_.at<double>(3) = r;

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
  
  // import sphere_ points
  ifstream myfile(argv[1]);
  string data;
  
  // the container in which we will store all the points
  vector<Point3d> points;

  // a temporary point to hold the three coords in while we read them all
  Point3d tmp;

  int maxX = 0;
  int maxY = 0;
  int maxZ = 0;
  double xCntr, yCntr, zCntr = 0;
  // read from infile into the two coords in tmp
  while (myfile >> tmp.x && myfile >> tmp.y && myfile >> tmp.z){
    // add a copy of tmp to points
    points.push_back(tmp);

    // if(tmp.x>maxX) maxX = tmp.x;
    // if(tmp.y>maxY) maxY = tmp.y;
    // if(tmp.z>maxZ) maxZ = tmp.z;

    xCntr+=tmp.x;
    yCntr+=tmp.y;
    zCntr+=tmp.z;
  }

  xCntr = (xCntr/points.size());
  yCntr = (yCntr/points.size());
  zCntr = (zCntr/points.size());

  myfile.close();
	
  // create a vector to save inliers to 
  std::vector<int> inliers; // The  found inliers
  std::vector<cv::Point3d> tmp_points;

  // create a mat to save found sphere_ parameters to (center.x, center.y, center.z, radius)
  cv::Mat found_sphere;
  if(!fitSphereRANSAC(&points, inliers, found_sphere, thresh, confidence)){
    cout << "Sphere RANSAC failed" << endl;
    return -1;
  }
  
  cout << "points center of mass: " + to_string(xCntr) + " " + to_string(yCntr) + " " + to_string(zCntr) << endl;
  cout << "sphere: " + to_string(found_sphere.at<double>(0))+" "+to_string(found_sphere.at<double>(1))+" "+to_string(found_sphere.at<double>(2))+" "+to_string(found_sphere.at<double>(3)) << endl;

  return 0;
}
