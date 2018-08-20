#include <iostream>
#include <string>
#include <map>
#include <exception>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "INIReader.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Localizer.h"
#include "MapBuilder.h"
#include "optimizer.h"



//#include "DataLoader.h"

using namespace std;
using namespace Eigen;




int main (int argc, char *argv[])
{
	VMap myMap;
	myMap.load("/home/sujiwo/maptest.map");
	Localizer loc(&myMap);
	loc.setMask(cv::imread("/home/sujiwo/Works/Photogrammetry/test/mask.png"));
	loc.setCameraParameter(
		CameraPinholeParams::loadCameraParamsFromFile(
		"/home/sujiwo/Works/Photogrammetry/test/camera.txt"));
	cv::Mat image = cv::imread("/home/sujiwo/Works/Photogrammetry/test/test2.png", cv::IMREAD_GRAYSCALE);
	kfid k = loc.detect(image);

	cout << "Done: " << k << endl;
	return 0;
}
