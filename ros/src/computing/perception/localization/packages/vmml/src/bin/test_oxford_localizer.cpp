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


class LineEditor
{
public:
LineEditor(const char *argv0)
{
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
	char *sline = readline("L> ");
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


int main (int argc, char *argv[])
{
	LineEditor mLineEditor (argv[0]);
	bool doExit = false;

	VMap maptest;
	maptest.load(argv[1]);
	ImageDatabase *imgDb = maptest.getImageDB();
	SequenceSLAM *seqSlProv = imgDb->getSequence();

	OxfordImagePreprocessor imgLoader("/home/sujiwo/Sources/robotcar-dataset-sdk/models");

	while (doExit==false) {
		stringTokens command = mLineEditor.getLine();

		if (command[0]=="loc") {
			int k = localize_seq_slam(seqSlProv, imgLoader, command[1]);
			cout << k << endl;
		}

		else if (command[0]=="print") {

		}

		else if (command[0]=="quit") {
			doExit = true;
			break;
		}
	}
}
