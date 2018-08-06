/*
 * utilities.h
 *
 *  Created on: Jul 7, 2018
 *      Author: sujiwo
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_


#include <set>
#include <map>
#include <vector>
#include <utility>
#include <limits>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>


using std::pair;
using std::set;
using std::map;
using std::vector;
using Eigen::Quaterniond;
using Eigen::Vector3d;


//struct DataItem {
//	std::string imagePath;
//	Eigen::Vector3d position;
//	Eigen::Quaterniond orientation;
//};
//
//
//class DataItemx
//{
//public:
//	std::string getImagePath() const;
//	Eigen::Vector3d getPosition() const;
//	Eigen::Quaterniond getOrientation() const;
//};


typedef Eigen::Affine3d Transform3d;


template<typename P, typename Q>
map<Q,P> reverseMap (const map<P,Q> &smap)
{
	map<Q,P> nmap;
	for (auto &pq: smap) {
		P p = pq.first;
		Q q = pq.second;
		nmap[q] = p;
	}
	return nmap;
}


template<typename K, typename V>
set<K> allKeys (const map<K,V> &smap)
{
	set<K> retval;
	for (auto &p: smap) {
		retval.insert(p.first);
	}
	return retval;
}


template<typename T>
set<T> toSet (const vector<T> &vect)
{
	set<T> Res;
	for (T &v: vect) {
		Res.insert(v);
	}
	return Res;
}


template<typename T>
bool inSet (const set<T> &S, const T &val)
{
	return (S.find(val) != S.end());
}


// Result = A - B
template<typename T>
set<T> subtract (const set<T> &A, const set<T> &B)
{
	set<T> Res;
	for (auto &p: A) {
		if (!inSet(B, p)) {
			Res.insert(p);
		}
	}
	return Res;
}


template<typename K, typename V>
vector<V> allValues (const map<K,V> &sMap)
{
	vector<V> allV;
	for (auto &kv: sMap) {
		allV.push_back(kv.second);
	}
	return allV;
}


template<typename Scalar>
using VectorXx = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;


template <typename Scalar>
double median(const VectorXx<Scalar> &v)
{
	int n = v.rows();
	vector<Scalar> vs (v.data(), v.data()+v.rows());
	sort(vs.begin(), vs.end());
	if (n%2==1)
		return (double)vs[(n-1)/2];
	else
		return (double(vs[n/2])+double(vs[(n/2)-1])) / 2;
}


inline
int medianx (const Eigen::VectorXi &v)
{
	return median(v);
}


template <typename k, typename v>
pair<const k,v>
maximumMapElement(const map<k,v> &maptg)
{
	auto p = max_element(maptg.begin(), maptg.end(),
		[](const pair<k,v> &element1, const pair<k,v> &element2)
			{return element1.second < element2.second;}
	);
	return *p;
}


/*
 * All angles are in Radian
 */
Quaterniond fromRPY (double roll, double pitch, double yaw);

Vector3d quaternionToRPY (const Quaterniond &q);


class TQuaternion : public Eigen::Quaterniond
{
public:
	inline TQuaternion(const double &x, const double &y, const double &z, const double &w):
		Quaterniond(w, x, y, z)
	{}

	inline TQuaternion(double roll, double pitch, double yaw)
	{
		Quaterniond q = fromRPY(roll, pitch, yaw);
		coeffs() = q.coeffs();
	}

	inline TQuaternion& operator=(const Quaterniond &q)
	{
		coeffs() = q.coeffs();
		return *this;
	}
};


struct TTransform : public Eigen::Affine3d
{
	TTransform()
	{ m_matrix = Eigen::Matrix4d::Identity(); }

	TTransform(const Eigen::Affine3d &a)
	{ m_matrix = a.matrix(); }

	static TTransform from_XYZ_RPY
		(const Eigen::Vector3d &pos,
		double roll=0, double pitch=0, double yaw=0);

	static TTransform from_Pos_Quat
		(const Eigen::Vector3d &pos,
			const Eigen::Quaterniond &ori
				=Eigen::Quaterniond::Identity());

	inline const Vector3d position() const
	{ return this->translation(); }

	inline const Quaterniond orientation() const
	{ return Eigen::Quaterniond(this->rotation()); }

	std::string
	str () const;

	void
	displacement (const TTransform &other, double &linear, double &angular) const;

//	inline void setPosition(const double &x=0, const double &y=0, const double &z=0)
//	{
//		translation() = Eigen::Translation3d(x, y, z);
//	}
};


typedef TTransform Pose;


#endif /* UTILITIES_H_ */
