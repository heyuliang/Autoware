#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include "utilities.h"


using namespace std;
using namespace Eigen;


int main (int argc, char *argv[])
{
	Pose P = Pose::from_Pos_Quat(Vector3d(1,2,3), TQuaternion(0.04, 0.04, 0.05));
	Pose X;

//	const float Arr[5] = {1,2,3,4,5};
//	vector<float> ArrV(Arr, Arr+5);

	fstream storeFd;
	storeFd.open("/tmp/inst.bin", fstream::out);
	boost::archive::binary_oarchive storeOArc (storeFd);
	storeOArc << P;
	storeFd.close();

	fstream readFd;
	readFd.open("/tmp/inst.bin", fstream::in);
	boost::archive::binary_iarchive storeIArc (readFd);
	storeIArc >> X;
	readFd.close();

	return 0;
}
