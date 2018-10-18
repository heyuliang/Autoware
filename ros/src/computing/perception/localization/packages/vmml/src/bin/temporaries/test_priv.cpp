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
	int n = v.size();

	vector<typename Derived::Scalar> vs(n);

	if (v.Flags & Eigen::MatrixBase<Derived>::IsRowMajor) {
		for (int i=0; i<v.rows(); ++i)
			for (int j=0; j<v.cols(); ++j)
				vs[i*v.cols() + j] = v(i,j);
	}

	else {
		for (int j=0; j<v.cols(); ++j)
			for (int i=0; i<v.rows(); ++i)
				vs[j*v.rows() + i] = v(i,j);
	}

	sort(vs.begin(), vs.end());

	if (n%2==1)
		return (static_cast<double> (vs[(n-1)/2]) );
	else
		return static_cast<double>( (vs[n/2]) + (vs[(n/2)-1]) ) / 2;
}


int main()
{
	Matrix4d M = Matrix4d::Random();
	double d = medianz(M.diagonal());

//	cout << M << endl;
//	double med = medianz(M.col(2));

	return 0;
}
