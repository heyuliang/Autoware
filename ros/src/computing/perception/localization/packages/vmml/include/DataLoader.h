/*
 * DataLoader.h
 *
 *  Created on: Apr 5, 2018
 *      Author: sujiwo
 */

#ifndef DATALOADER_H_
#define DATALOADER_H_


#include <string>
#include <vector>
#include <opencv2/core/core.hpp>


class DataLoader {
public:
	DataLoader(const std::string &);
	virtual ~DataLoader();

	inline int size ()
	{ return timestamps.size(); }

//	std::strget (const std::string &ts);
	cv::Mat load (int i);

	std::vector<std::string> timestamps;

private:
	std::string parentPath;
};

#endif /* DATALOADER_H_ */
