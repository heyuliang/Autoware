#include <iostream>
#include <string>
#include <map>
#include <exception>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "INIReader.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "optimizer.h"



//#include "DataLoader.h"

using namespace std;
using namespace Eigen;


int main (int argc, char *argv[])
{
	VMap myMap;
	myMap.load(string(argv[1]));

	cv::Mat frmImg = cv::imread(string(argv[2]), cv::IMREAD_GRAYSCALE);
	Frame cf (frmImg, &myMap);

	// Rescale

	// Mask

	cout << "Done" << endl;
	return 0;
}
