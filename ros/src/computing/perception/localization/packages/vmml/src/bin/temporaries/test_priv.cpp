/*
 * test_priv.cpp
 *
 *  Created on: Aug 12, 2018
 *      Author: sujiwo
 */


#include <iostream>
#include <string>
#include <utility>
#include <Eigen/Eigen>


using namespace std;
using namespace Eigen;


template <typename Derived>
double medianz (const Eigen::MatrixBase<Derived> &v)
{
	int n = v.rows() * v.cols();

	vector<typename Derived::Scalar> vs(n);

	int i=0;
	for (typename Eigen::MatrixBase<Derived>::InnerIterator it(v, n); it; ++it) {
		vs.at(i) = it.value();
		++i;
	}

	sort(vs.begin(), vs.end());
	if (n%2==1)
		return (static_cast<double> (vs[(n-1)/2]) );
	else
		return static_cast<double>( (vs[n/2]) + (vs[(n/2)-1]) ) / 2;
}


template <typename Derived>
void useless (Eigen::MatrixBase<Derived> &v)
{
	v(0,0) = -v(0,0);
	return;
}


int main()
{
	Matrix3d M = Matrix3d::Identity();
	M(2,2) = 2.0;
	useless(M);

	cout << M << endl;
//	double med = medianz(M.col(2));

	return 0;
}
