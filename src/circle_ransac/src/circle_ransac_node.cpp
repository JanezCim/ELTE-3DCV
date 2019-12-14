#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;


int main(int argc, char* argv[]){
	if (argc != 2){
		cout << " Usage: display_image ImageToLoadAndDisplay" << endl;
		return -1;
	}

  // Mat image;
	// image = imread(argv[1]);

  // import circle points
  ifstream myfile(argv[1]);
  string data;
  
  // the container in which we will store all the points
  vector<Point> points;

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

  Mat img = Mat(Size(maxX+20, maxY+20), CV_8UC1);
  for(auto i:points){
    img.at<uchar>(i.y, i.x) = 255;
  }
	
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
    imshow("original", img);
    
  }

  return 0;
}
