#include <iostream>
#include <string>
#include "datasets/OxfordDataset.h"


using namespace std;


int main (int argc, char *argv[])
{
	string datasetPath("/home/sujiwo/Data/Oxford/2014-12-02-15-30-08");
	string modelPath("/home/sujiwo/Sources/robotcar-dataset-sdk/models");
	OxfordDataset DS(datasetPath, modelPath);
	return 0;
}
