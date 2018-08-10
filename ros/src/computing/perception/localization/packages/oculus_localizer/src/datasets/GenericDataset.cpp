/*
 * GenericDataset.cpp
 *
 *  Created on: Aug 3, 2018
 *      Author: sujiwo
 */

#include <fstream>
#include "datasets/GenericDataset.h"

using namespace std;
using namespace Eigen;


string GenericDataset::dSetName = "Generic";


void
GenericDataset::dump(const std::string &filename)
{
	const char dumpSep = ',';
	ostream *fd;
	ofstream fdr;
	if (filename.size()==0)
		fd = &cout;
	else {
		fdr.open(filename);
		if (fdr.good())
			throw runtime_error("Unable to open file");
		fd = &fdr;
	}

	*fd << fixed;
	*fd << setprecision(4);

	for (int i=0; i<this->size(); i++) {
		auto &di = this->at(i);
		Vector3d pos = di.getPosition();
		Quaterniond orn = di.getOrientation();
		*fd << di.getId()
			<< dumpSep << pos.x()
			<< dumpSep << pos.y()
			<< dumpSep << pos.z()
			<< dumpSep << orn.x()
			<< dumpSep << orn.y()
			<< dumpSep << orn.z()
			<< dumpSep << orn.w()
			<< endl;
	}

	if (fdr.is_open())
		fdr.close();
}


GenericDataItem::~GenericDataItem()
{}


string
GenericDataset::getName() const
{
	return dSetName;
}
