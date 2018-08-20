/*
 * DataLoader.cpp
 *
 *  Created on: Apr 5, 2018
 *      Author: sujiwo
 */

#include <iostream>
#include <fstream>
#include <vector>
#include "DataLoader.h"
#include <boost/tokenizer.hpp>
#include <opencv2/highgui/highgui.hpp>


using std::string;
using std::ifstream;
using boost::tokenizer;


DataLoader::DataLoader(const string &path)
{
	parentPath = path;

	string timestamplist = path + "/timestamp.txt";
	ifstream inputfd (timestamplist.c_str());
	if (!inputfd.is_open())
		throw 1;

	string line;
	while (getline(inputfd, line)) {
		tokenizer<> tok(line);
		string ts = *(tok.begin());
		timestamps.push_back(ts);
	}
}


cv::Mat DataLoader::load(int i)
{
	string tsimg = parentPath + "/images/" + timestamps[i] + ".png";
	return cv::imread(tsimg, 1);
}


DataLoader::~DataLoader()
{
	// TODO Auto-generated destructor stub
}

