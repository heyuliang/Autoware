/*
 * test_tf.cpp
 *
 *  Created on: Jul 31, 2018
 *      Author: sujiwo
 */

#include <iostream>

#include "datasets/GenericDataset.h"
#include "datasets/MeidaiBagDataset.h"


int main (int argc, char *argv[])
{
	GenericDataset::Ptr loadedDataset = MeidaiBagDataset::load("/home/sujiwo/Data/sample-mapping.bag");

	auto item100 = loadedDataset->get(100);
	auto pose100 = item100->getPose();

	return 0;
}
