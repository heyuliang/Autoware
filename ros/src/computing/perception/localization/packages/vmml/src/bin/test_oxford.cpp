/*
 * test_localizer.cpp
 *
 *  Created on: Aug 29, 2018
 *      Author: sujiwo
 */



#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <histedit.h>
#include <editline/readline.h>
#include <opencv2/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <boost/filesystem.hpp>

#include "VMap.h"
#include "ImageDatabase.h"
#include "SequenceSLAM.h"
#include "Localizer.h"
#include "MapBuilder2.h"
#include "Viewer.h"
#include "datasets/OxfordDataset.h"


using namespace std;
namespace fs = boost::filesystem;


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


#define Precision 10

string dumpVector(const Vector3d &v)
{
	stringstream s;
	s.precision(Precision);
	s << v.x() << " " << v.y() << " " << v.z();
	return s.str();
}


string dumpVector(const Quaterniond &v)
{
	stringstream s;
	s.precision(Precision);
	s << v.x() << " " << v.y() << " " << v.z() << ' ' << v.w();
	return s.str();
}


InputFrame createInputFrame(const OxfordDataItem &d)
{
	cv::Mat i=d.getImage();
	// Oxford datasets output is RGB
	cv::cvtColor(i, i, CV_BGR2GRAY, 1);
	InputFrame f(
		i,
		d.groundTruth.position(),
		d.groundTruth.orientation(),
		// Force Keyframe ID using timestamp. This way, we can refer to original
		// image for display purpose (which is the case for Oxford Dataset)
		static_cast<kfid>(d.timestamp));
	f.tm = d.getTimestamp();

	return f;
}


void buildMap2
(OxfordDataset &dataset, MapBuilder2 &builder)
{
	builder.addCameraParam(dataset.getCameraParameter());

	Viewer *imgViewer = new Viewer (dataset);
	imgViewer->setMap(builder.getMap());
	dataItemId currentItemId;

	MapBuilder2::frameCallback frmCallback =
	[&] (const InputFrame &f)
	{
		imgViewer->update(currentItemId, builder.getCurrentKeyFrameId());
		cout << currentItemId << " / " << dataset.size() << endl;
	};
	builder.registerFrameCallback(frmCallback);

	for (int framePtr=0; framePtr<dataset.size(); framePtr++) {
		const OxfordDataItem &dx = dataset.at(framePtr);
		currentItemId = dx.getId();
		InputFrame frame = createInputFrame(dx);
		builder.input(frame);
	}

	builder.build();
	delete(imgViewer);
}


class LocalizerApp
{
public:
	LocalizerApp (int argc, char *argv[]):
		mLineEditor(argv[0], TestPrompt)
	{
	//	localizer = new Localizer()
	}


	~LocalizerApp ()
	{
		if (mapSrc)
			delete(mapSrc);
		if (localizTestDataSrc)
			delete(localizTestDataSrc);
		if (localizer)
			delete(localizer);
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

			else if (command[0]=="map_pcl")
				map_dump_pcl();

			else if (command[0]=="dataset_trajectory")
				dataset_trajectory_dump();

			else if (command[0]=="map_trajectory")
				map_trajectory_dump();

			else if (command[0]=="find")
				map_find_cmd(command[1]);

			else if (command[0]=="save")
				dataset_save_dsecond(command[1]);

			else if (command[0]=="zoom")
				dataset_set_zoom(command[1]);

			else if (command[0]=="dataset_simulate_seqslam")
				dataset_simulate_seqslam(command[1]);

			else if (command[0]=="dataset_view")
				dataset_view(command[1]);

			else if (command[0]=="detect")
				map_detect_cmd(command[1]);

			else if (command[0]=="map_create")
				map_create_cmd(stringTokens(command.begin()+1, command.end()));

			else if (command[0]=="map_info")
				map_info_cmd();
		}
	}


protected:
	LineEditor mLineEditor;

	VMap *mapSrc = NULL;
	ImageDatabase *imgDb = NULL;
	SequenceSLAM *seqSlProv = NULL;
	Localizer *localizer = NULL;

	OxfordDataset *localizTestDataSrc = NULL;


private:

	void map_open_cmd(const string &mapPath)
	{
		try {
			mapSrc = new VMap();
			mapSrc->load(mapPath);
			localizer = new Localizer(mapSrc);
			localizer->setCameraParameterFromId(0);

			debug("Map loaded");
			imgDb = mapSrc->getImageDB();
			seqSlProv = imgDb->getSequence();
		} catch (exception &e) {
			debug ("Unable to load map");
		}
	}

	const string imageDumpSeqSlam = "/tmp/seqslam.png";
	void dataset_simulate_seqslam(const string &cs)
	{
		double dt = std::stod(cs);
		cv::Mat img = localizTestDataSrc->atDurationSecond(dt).getImage();
		cv::cvtColor(img, img, CV_BGR2GRAY);
		img = seqSlProv->normalizePatch(img, 8);
		cv::imwrite(imageDumpSeqSlam, img);
		debug("Dumped image to "+imageDumpSeqSlam);
	}

	const string viewerWindowName="Dataset Viewer";
	void dataset_view(const string &durationSecStr)
	{
		cv::namedWindow(viewerWindowName);
		double d = std::stod(durationSecStr);
		auto di = localizTestDataSrc->atDurationSecond(d);
		cv::Mat img = di.getImage();
		cv::cvtColor(img, img, CV_RGB2GRAY);
		cv::imshow(viewerWindowName, img);
		cv::waitKey(1);
	}

	const string mapDumpPcl = "/tmp/map.pcl";
	void map_dump_pcl()
	{
		auto pclDump = mapSrc->dumpPointCloudFromMapPoints();
		pcl::io::savePCDFileBinary(mapDumpPcl, *pclDump);
		debug("Point Cloud Map dumped to "+mapDumpPcl);
	}

	void map_info_cmd()
	{
		if (mapSrc==NULL) {
			debug("Map not loaded");
			return;
		}
		debug("# of keyframe(s): "+to_string(mapSrc->numOfKeyFrames()));
		debug("# of map point(s): " +to_string(mapSrc->numOfMapPoints()));
	}

	const string dumpMapTrajectoryPath = "/tmp/dump_map_trajectory.csv";
	void map_trajectory_dump()
	{
		fstream mapTrFd (dumpMapTrajectoryPath, ios_base::out|ios_base::trunc);
		if (!mapTrFd.is_open()) {
			debug("Unable to create "+dumpMapTrajectoryPath);
			return;
		}

		auto mapPoses = mapSrc->dumpCameraPoses();
		for (auto ps: mapPoses) {
			mapTrFd << dumpVector(ps.first) << " " << dumpVector(ps.second) << endl;
		}

		mapTrFd.close();
		debug("Map trajectory dumped to "+dumpMapTrajectoryPath);
	}

	const string dumpDatasetTrajectoryPath = "/tmp/dump_dataset_trajectory";
	void dataset_trajectory_dump()
	{
		const string dsDumpPath = dumpDatasetTrajectoryPath + '-' + fs::basename(localizTestDataSrc->getPath());
		fstream dsTrFd (dsDumpPath, ios_base::out|ios_base::trunc);
		if (!dsTrFd.is_open()) {
			debug("Unable to create "+dumpMapTrajectoryPath);
			return;
		}

		for (int i=0; i<localizTestDataSrc->size(); i++) {
			const OxfordDataItem di = localizTestDataSrc->at(i);
			dsTrFd << di.timestamp << " "
					<< dumpVector(di.getPosition()) << " "
					<< dumpVector(di.getOrientation())
					<< endl;
		}

		dsTrFd.close();
		debug("Dataset trajectory dumped to "+dsDumpPath);
	}

	void dataset_open_cmd(const string &dsPath, const string &modelDir)
	{
		try {
			localizTestDataSrc = new OxfordDataset(dsPath, modelDir);
			debug("Dataset loaded");
		} catch (exception &e) {
			debug("Unable to load dataset");
		}
	}

	void map_find_cmd(const string &durationSecStr)
	{
		double d = std::stod(durationSecStr);
		auto di = localizTestDataSrc->atDurationSecond(d);
		cv::Mat img = di.getImage();
		cv::cvtColor(img, img, CV_RGB2GRAY);
		seqSlProv->find(img, 10);
	}

	void map_detect_cmd(const string &durationSecStr)
	{
		if (localizer==NULL) {
			debug("Map not loaded");
			return;
		}
		double d = std::stod(durationSecStr);
		auto di = localizTestDataSrc->atDurationSecond(d);
		cv::Mat img = di.getImage();
		cv::cvtColor(img, img, CV_RGB2GRAY);
		kfid k = localizer->detect(img);
		debug("Max.: "+to_string(k));
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

	/*
	 * XXX: We should not use naked pointer here
	 */
	void map_create_cmd (const stringTokens &cmd)
	{
		OxfordDataset* oxfSubset;
		bool isSubset;

		double start, duration;
		if (cmd.size() >= 2) {
			start = stod(cmd[0]);
			duration = stod(cmd[1]);
			oxfSubset = localizTestDataSrc->timeSubset(start, duration);
			isSubset = true;
		}
		else {
			oxfSubset = localizTestDataSrc;
			start = 0;
			duration = double(oxfSubset->getTimeLength().total_microseconds())/1e6;
			isSubset = false;
		}

		debug ("About to run mapping with duration "+to_string(duration) +" seconds, " +to_string(oxfSubset->size()) + " frames");

		MapBuilder2 mapBld;
		buildMap2(*oxfSubset, mapBld);

		const string mapFilePath = oxfSubset->getPath() + "/vmml.map";
		mapBld.getMap()->save(mapFilePath);

		debug ("Mapping done");
		debug ("Duration " + to_string(duration) + " seconds");
		debug ("Path: " + mapFilePath);

		if (isSubset)
			delete(oxfSubset);
	}
};





int main (int argc, char *argv[])
{
	LocalizerApp mainApp(argc, argv);
	mainApp.loop();

	return 0;
}
