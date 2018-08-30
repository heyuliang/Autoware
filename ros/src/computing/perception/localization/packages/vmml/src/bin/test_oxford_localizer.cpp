/*
 * test_localizer.cpp
 *
 *  Created on: Aug 29, 2018
 *      Author: sujiwo
 */



#include <vector>
#include <string>
#include <iostream>
#include <histedit.h>
#include <editline/readline.h>
#include <opencv2/highgui.hpp>

#include "VMap.h"
#include "ImageDatabase.h"
#include "SequenceSLAM.h"
#include "datasets/OxfordDataset.h"


using namespace std;


typedef vector<string> stringTokens;


const string TestPrompt = "OX>";


class LineEditor
{
public:
LineEditor(const char *argv0, const string &askPrompt="")
{
	prompt = askPrompt + ' ';
	el = el_init("test_localizer", stdin, stdout, stderr);
	rl_initialize();
	tokenizer = tok_init(NULL);
}

~LineEditor()
{
	tok_end(tokenizer);
	el_end(el);
}

stringTokens getLine ()
{
	int nt;
	const char **stoklist;
	char *sline = readline(prompt.c_str());
	tok_str(tokenizer, sline, &nt, &stoklist);

	stringTokens st;
	for (int i=0; i<nt; i++) {
		string s(stoklist[i]);
		st.push_back(s);
	}
	free(sline);
	tok_reset(tokenizer);
	return st;
}

protected:
	EditLine *el;
	Tokenizer *tokenizer;
	string prompt;
};


int localize_seq_slam (SequenceSLAM *seqSl, OxfordImagePreprocessor &proc, const string &imgPath)
{
	cv::Mat image = proc.load(imgPath);
	cv::imwrite("/tmp/1.png", image);
	exit(-1);

	vector<cv::Mat> imgList;
	imgList.push_back(image);

	seqSl->find(imgList, 10);
}






class LocalizerApp
{
public:
LocalizerApp (int argc, char *argv[]):
	mLineEditor(argv[0], TestPrompt)
{

}


~LocalizerApp ()
{
	if (mapSrc)
		delete(mapSrc);
	if (localizTestDataSrc)
		delete(localizTestDataSrc);
}


void loop()
{
	bool doExit=false;
	while (doExit==false) {

		stringTokens command = mLineEditor.getLine();

		if (command[0]=="quit")
			doExit = true;

		else if (command[0]=="map")
			map_open_cmd(command[1]);

		else if (command[0]=="dataset")
			dataset_open_cmd(command[1], command[2]);

		else if (command[0]=="trajectory")
			map_trajectory_dump(command[1]);

		else if (command[0]=="find")
			map_find_cmd(command[1]);

		else if (command[0]=="save")
			dataset_save_dsecond(command[1]);

		else if (command[0]=="zoom")
			dataset_set_zoom(command[1]);
	}
}


protected:
	LineEditor mLineEditor;

	VMap *mapSrc = NULL;
	ImageDatabase *imgDb = NULL;
	SequenceSLAM *seqSlProv = NULL;

	OxfordDataset *localizTestDataSrc = NULL;


private:

	void map_open_cmd(const string &mapPath)
	{
		mapSrc = new VMap();
		mapSrc->load(mapPath);
		debug("Map loaded");
		imgDb = mapSrc->getImageDB();
		seqSlProv = imgDb->getSequence();
	}

	void map_trajectory_dump(const string &dumpPath)
	{

	}

	void dataset_open_cmd(const string &dsPath, const string &modelDir)
	{
		localizTestDataSrc = new OxfordDataset(dsPath, modelDir);
		debug("Dataset loaded");
	}

	void map_find_cmd(const string &durationSecStr)
	{
		double d = std::stod(durationSecStr);
		auto di = localizTestDataSrc->atDurationSecond(d);
		cv::Mat img = di.getImage();
		cv::cvtColor(img, img, CV_RGB2GRAY);
		seqSlProv->find(img, 10);
	}

	const string dumpImagePath = "/tmp/dump_image.png";

	void dataset_save_dsecond(const string &durationSecStr)
	{
		double d = std::stod(durationSecStr);
		auto di = localizTestDataSrc->atDurationSecond(d);
		cv::Mat img = di.getImage();
		cv::imwrite(dumpImagePath, img);
		debug("Dumped to " + dumpImagePath);
	}

	void debug(const string &s)
	{
		cerr << s << endl;
	}

	void dataset_set_zoom(const string &zstr)
	{
		float z = std::stof(zstr);
		localizTestDataSrc->setZoomRatio(z);
	}

};





int main (int argc, char *argv[])
{
	LocalizerApp mainApp(argc, argv);
	mainApp.loop();

	return 0;
}
