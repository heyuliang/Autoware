/*
 * python_mod.cpp
 *
 *  Created on: Jul 17, 2018
 *      Author: sujiwo
 */


#include <boost/python.hpp>
#include <iostream>
#include <Eigen/Eigen>
#include "VMap.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Localizer.h"
#include "MapBuilder2.h"
#include "pymat.h"


using namespace boost::python;
using namespace std;
using namespace Eigen;


static NDArrayConverter matcvt;


template<typename _Tp, int _rows>
void copyEigenVector2cv (const Eigen::Matrix<_Tp,_rows,1> &v, cv::Mat &m)
{
	assert(m.type() == cv::DataType<_Tp>::type);
	cv::Mat __(v.rows(), 1, cv::DataType<_Tp>::type, (void*)v.data(), v.stride()*sizeof(_Tp));
	__.copyTo(m);
}


Vector3d toVector3(const boost::python::list &p)
{
	return Vector3d (extract<double>(p[0]), extract<double>(p[1]), extract<double>(p[2]));
}


//Vector3d toVector3(const PyArrayObject *v)
//{
//	Vector3d vr;
//	vr.x() = extract<double>(v[0]);
//}


Quaterniond toQuaternion(const boost::python::list &qs)
{
	Quaterniond q;
	q.x() = extract<double>(qs[0]);
	q.y() = extract<double>(qs[1]);
	q.z() = extract<double>(qs[2]);
	q.w() = extract<double>(qs[3]);
	return q;
}


class VMapPy : public VMap
{
public:

	kfid find (PyObject *inp)
	{
		cv::Mat minp = matcvt.toMat(inp);
		kfid k = localizer->detect(minp);
		return k;
	}


	bool setCameraParams (const string &filename)
	{
		try {
			CameraPinholeParams par
				= CameraPinholeParams::loadCameraParamsFromFile(filename);
			if (localizer)
				localizer->setCameraParameter(par);
			return true;
		} catch (exception &e) {
			return false;
		}
	}


	dict info () const
	{
		dict mapinfo;
		mapinfo["numKeyframes"] = this->numOfKeyFrames();
		mapinfo["numMappoints"] = this->numOfMapPoints();
		return mapinfo;
	}


	dict camera() const
	{
		const CameraPinholeParams &cm = localizer->getCamera();
		dict camera;
		camera["fx"] = cm.fx;
		camera["fy"] = cm.fy;
		camera["cx"] = cm.cx;
		camera["cy"] = cm.cy;
		return camera;
	}


	PyObject*
	allMapPoints ()
	{
		uint N = this->numOfMapPoints();
		cv::Mat allPts(N, 3, CV_64F);

		for (mpid i: this->getMapPointList()) {
			Vector3d p = this->mappoint(i)->getPosition();
//			cv::Mat r = allPts.row(i);
//			copyEigenVector2cv(p, r);
			allPts.row(i).at<double>(0) = p.x();
			allPts.row(i).at<double>(1) = p.y();
			allPts.row(i).at<double>(2) = p.z();
		}

		return matcvt.toNDArray(allPts);
	}


	PyObject*
	allKeyFrames ()
	{
		uint N = this->numOfKeyFrames();
		cv::Mat allKfs(N, 7, CV_64F);

		for (kfid k: this->getKeyFrameList()) {
			Vector3d p = this->keyframe(k)->position();
			Quaterniond q = this->keyframe(k)->orientation();
			allKfs.row(k).col(0) = p.x();
			allKfs.row(k).col(1) = p.y();
			allKfs.row(k).col(2) = p.z();
			allKfs.row(k).col(3) = q.x();
			allKfs.row(k).col(4) = q.y();
			allKfs.row(k).col(5) = q.z();
			allKfs.row(k).col(6) = q.w();
		}

		return matcvt.toNDArray(allKfs);
	}


	bool load (const std::string &p)
	{
		try {
			VMap::load (p);
			localizer = new Localizer(this);
			return true;
		} catch(exception &e) {
			cerr << e.what() << endl;
			return false;
		}
	}


protected:
	Localizer *localizer;
};


class InputFramePy : public InputFrame
{
public:
	InputFramePy() {}

	InputFramePy(PyObject *_img, boost::python::list &posl, boost::python::list &ql)
	{
		position = toVector3(posl);
		orientation = toQuaternion(ql);
		image = matcvt.toMat(_img);
	}

	boost::python::list
	getpos()
	{
		boost::python::list l;
		l.append(position[0]);
		l.append(position[1]);
		l.append(position[2]);
		return l;
	}
};


class MapBuilderPy : public MapBuilder2
{
public:
	void initialize (const InputFramePy &f1, const InputFramePy &f2)
	{
		MapBuilder2::initialize(f1, f2);
	}

//	void initialize (PyObject *f1, PyObject *f2)
//	{
//
//	}

	void track (const InputFrame &f)
	{

	}

	VMap* getMap()
	{ return cMap; }

	void addCameraParam (const CameraPinholeParams &c)
	{ cMap->addCameraParameter(c); }

};


void
test_read (PyObject *arr)
{
	cv::Mat x = matcvt.toMat(arr);
	cerr << x << endl;
}


PyObject*
test_eye()
{
	cv::Mat M = cv::Mat::eye(3,3,CV_64F);
	cerr << M << endl;
	return matcvt.toNDArray(M);
}


BOOST_PYTHON_MODULE(photogrampy)
{
	Py_Initialize();
	import_array();
//	import_ufunc();

	class_ <VMapPy> ("VMap")
		.def("load", &VMapPy::load)
		.add_property("info", &VMapPy::info)
		.add_property("camera", &VMapPy::camera)
		.def("allMapPoints", &VMapPy::allMapPoints)
		.def("allKeyFrames", &VMapPy::allKeyFrames)
		.def("setCameraParams", &VMapPy::setCameraParams)
		.def("find", &VMapPy::find);


	class_ <InputFramePy> ("InputFrame",
		init<PyObject*,boost::python::list&,boost::python::list&>())
		.def("getpos", &InputFramePy::getpos);


	class_ <MapBuilderPy> ("MapBuilder")
		.def("initialize", &MapBuilderPy::initialize);


	def("test_read", &test_read);

	def("test_eye", &test_eye);
}

