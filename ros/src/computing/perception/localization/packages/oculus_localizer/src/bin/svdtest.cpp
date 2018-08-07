#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <iostream>


using namespace Eigen;
using namespace std;


int main (int argc, char *argv[])
{
	Matrix4d A;
	A << 1, 6, 1, 2,
		9, 5, 3, 7,
		8, 6, 4, 1,
		3, 2, 0, 6;
	JacobiSVD<Matrix4d,false> svd(A, ComputeFullV|ComputeFullU);
	Matrix4d Vh = svd.matrixV().transpose();
	Matrix4d U = svd.matrixU();
	Vector4d S = svd.singularValues();
	Matrix4d Sx = Matrix4d::Zero();
	for (int i=0; i<4; i++)
		Sx(i,i) = S[i];
	Matrix4d Ax = U*Sx*Vh;

	cout << Ax << endl << endl << Vh << endl;

	return 0;
}
