#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

// ************** CHANGABLES****

// *****************************



int main(int argc, char* argv[]){
	if (argc != 2){
		cout << "No arguments found" << endl;
		return -1;
	}
  
  // import sphere_ points
  ifstream myfile(argv[1]);
  string data;
  
  return 0;
}
