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
#include "datasets/MeidaiBagDataset.h"


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


InputFrame createInputFrame(OxfordDataItem::ConstPtr &DI)
{
	cv::Mat img=DI->getImage();
	cv::cvtColor(img, img, CV_BGR2GRAY, 1);

	InputFrame f(
		img,
		DI->getPosition(),
		DI->getOrientation(),
		// Force Keyframe ID using timestamp. This way, we can refer to original
		// image for display purpose (which is the case for Oxford Dataset)
		static_cast<kfid>(DI->timestamp));
	f.tm = DI->getTimestamp();

	return f;
}


InputFrame createInputFrame(MeidaiDataItem::ConstPtr &DI)
{
	cv::Mat img=DI->getImage();
	cv::cvtColor(img, img, CV_BGR2GRAY, 1);

	/*
	 * Contrarily, we let VMap use autonumbering for keymap ID
	 */
	InputFrame f(
		img,
		DI->getPosition(),
		DI->getOrientation());
	f.tm = DI->getTimestamp();

	return f;
}


//void buildMap2
//(OxfordDataset &dataset, MapBuilder2 &builder)
//{
//	builder.addCameraParam(dataset.getCameraParameter());
//
//	Viewer *imgViewer = new Viewer (dataset);
//	imgViewer->setMap(builder.getMap());
//	dataItemId currentItemId;
//
//	MapBuilder2::frameCallback frmCallback =
//	[&] (const InputFrame &f)
//	{
//		imgViewer->update(currentItemId, builder.getCurrentKeyFrameId());
//		cout << currentItemId << " / " << dataset.size() << endl;
//	};
//	builder.registerFrameCallback(frmCallback);
//
//	for (int framePtr=0; framePtr<dataset.size(); framePtr++) {
//		const OxfordDataItem &dx = dataset.at(framePtr);
//		currentItemId = dx.getId();
//		InputFrame frame = createInputFrame(dx);
//		builder.input(frame);
//	}
//
//	builder.build();
//	delete(imgViewer);
//}


class LocalizerApp
{
public:
	enum datasetType {
		OXFORD_DATASET_TYPE,
		MEIDAI_DATASET_TYPE
	} ;

	LocalizerApp (int argc, char *argv[]):
		mLineEditor(argv[0], TestPrompt)
	{
	//	localizer = new Localizer()
	}


	~LocalizerApp ()
	{
		if (mapSrc)
			delete(mapSrc);
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

//			else if (command[0]=="find")
//				map_find_cmd(command[1]);

			else if (command[0]=="save")
				dataset_save_dsecond(command[1]);

			else if (command[0]=="zoom")
				dataset_set_zoom(command[1]);

//			else if (command[0]=="dataset_simulate_seqslam")
//				dataset_simulate_seqslam(command[1]);

			else if (command[0]=="dataset_view")
				dataset_view(command[1]);

			else if (command[0]=="detect")
				map_detect_cmd(command[1]);

			else if (command[0]=="map_create")
				map_create_cmd(stringTokens(command.begin()+1, command.end()));

			else if (command[0]=="map_info")
				map_info_cmd();

			else if (command[0]=="mask")
				mask_set_cmd(command[1]);
		}
	}


	void buildMap
	(GenericDataset::Ptr datasetSrc, MapBuilder2 &builder)
	{
		builder.addCameraParam(datasetSrc->getCameraParameter());

		Viewer *imgViewer = new Viewer (datasetSrc);
		imgViewer->setMap(builder.getMap());
		dataItemId currentItemId;

		MapBuilder2::frameCallback frmCallback =
		[&] (const InputFrame &f)
		{
			imgViewer->update(currentItemId, builder.getCurrentKeyFrameId());
			cout << currentItemId << " / " << datasetSrc->size() << endl;
		};
		builder.registerFrameCallback(frmCallback);

		for (int framePtr=0; framePtr<datasetSrc->size(); framePtr++) {

			InputFrame frame;
			if (slDatasourceType==OXFORD_DATASET_TYPE) {
				OxfordDataItem::ConstPtr dx = static_pointer_cast<OxfordDataItem const> (datasetSrc->get(framePtr));
				frame = createInputFrame(dx);
				currentItemId = dx->getId();
			}
			else if (slDatasourceType==MEIDAI_DATASET_TYPE) {
				MeidaiDataItem::ConstPtr dx = static_pointer_cast<MeidaiDataItem const> (datasetSrc->get(framePtr));
				frame = createInputFrame(dx);
				currentItemId = dx->getId();
			}
			builder.input(frame);
		}

		builder.build();
		delete(imgViewer);
	}


protected:
	LineEditor mLineEditor;

	VMap *mapSrc = NULL;
	ImageDatabase *imgDb = NULL;
	SequenceSLAM *seqSlProv = NULL;
	Localizer *localizer = NULL;

//	OxfordDataset *localizTestDataSrc = NULL;
	datasetType slDatasourceType;
	GenericDataset::Ptr loadedDataset;


	cv::Mat mask;


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

			mask = mapSrc->getMask();
			if (mask.empty()==false) {
				debug ("Map contains mask image file");
				localizer->setMask(mask);
			}

		} catch (exception &e) {
			debug ("Unable to load map");
		}
	}

//	const string imageDumpSeqSlam = "/tmp/seqslam.png";
//	void dataset_simulate_seqslam(const string &cs)
//	{
//		double dt = std::stod(cs);
//		cv::Mat img = localizTestDataSrc->atDurationSecond(dt).getImage();
//		cv::cvtColor(img, img, CV_BGR2GRAY);
//		img = seqSlProv->normalizePatch(img, 8);
//		cv::imwrite(imageDumpSeqSlam, img);
//		debug("Dumped image to "+imageDumpSeqSlam);
//	}

	const string viewerWindowName="Dataset Viewer";
	void dataset_view(const string &durationSecStr)
	{
		cv::namedWindow(viewerWindowName);
		double d = std::stod(durationSecStr);
		auto di = loadedDataset->atDurationSecond(d);
		cv::Mat img = di->getImage();
		cv::cvtColor(img, img, CV_RGB2GRAY);
		cv::imshow(viewerWindowName, img);
		cv::waitKey(1);
	}

	const string mapDumpPcl = "/tmp/map.pcd";
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
		if (slDatasourceType==OXFORD_DATASET_TYPE) {
			OxfordDataset::Ptr oxfDataset = static_pointer_cast<OxfordDataset>(loadedDataset);
			const string dsDumpPath = dumpDatasetTrajectoryPath + '-' + fs::basename(oxfDataset->getPath());
			fstream dsTrFd (dsDumpPath, ios_base::out|ios_base::trunc);
			if (!dsTrFd.is_open()) {
				debug("Unable to create "+dumpMapTrajectoryPath);
				return;
			}

			for (int i=0; i<oxfDataset->size(); i++) {
				OxfordDataItem::ConstPtr di = static_pointer_cast<OxfordDataItem const> (oxfDataset->get(i));
				dsTrFd << di->timestamp << " "
						<< dumpVector(di->getPosition()) << " "
						<< dumpVector(di->getOrientation())
						<< endl;
			}

			dsTrFd.close();
			debug("Dataset trajectory dumped to "+dsDumpPath);
			return;
		}

		else if (slDatasourceType==MEIDAI_DATASET_TYPE) {
			MeidaiBagDataset::Ptr meidaiDs = static_pointer_cast<MeidaiBagDataset>(loadedDataset);
			const Trajectory &Tr = meidaiDs->getGnssTrajectory();
			fstream dsTrFd (dumpDatasetTrajectoryPath, ios_base::out|ios_base::trunc);
		}
	}

	void dataset_open_cmd(const string &dsPath, const string &modelDir)
	{
		boost::filesystem::path datasetPath(dsPath);
		if (boost::filesystem::is_directory(datasetPath)) {
			loadedDataset = OxfordDataset::create(datasetPath.string(), modelDir);
			debug ("Oxford-type Dataset Loaded");
		}
		else if (datasetPath.extension()==".bag") {
			loadedDataset = MeidaiBagDataset::create(datasetPath.string(), 0, 1, "", false);
			debug ("Nagoya University Dataset Loaded");
		}

		else {
			debug("Unsupported dataset type");
			return;
		}
	}

//	void map_find_cmd(const string &durationSecStr)
//	{
//		double d = std::stod(durationSecStr);
//		auto di = localizTestDataSrc->atDurationSecond(d);
//		cv::Mat img = di.getImage();
//		cv::cvtColor(img, img, CV_RGB2GRAY);
//		seqSlProv->find(img, 10);
//	}

	void map_detect_cmd(const string &durationSecStr)
	{
		if (localizer==NULL) {
			debug("Map not loaded");
			return;
		}
		double d = std::stod(durationSecStr);
		auto di = loadedDataset->atDurationSecond(d);
		cv::Mat img = di->getImage();
		cv::cvtColor(img, img, CV_RGB2GRAY);
		kfid k = localizer->detect(img);
		debug("Max.: "+to_string(k));
	}

	const string dumpImagePath = "/tmp/dump_image.png";

	void dataset_save_dsecond(const string &durationSecStr)
	{
		double d = std::stod(durationSecStr);
		auto di = loadedDataset->atDurationSecond(d);
		cv::Mat img = di->getImage();
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
		loadedDataset->setZoomRatio(z);
	}

	string createMapFilename ()
	{
		string mapResName;

		if (slDatasourceType==OXFORD_DATASET_TYPE) {
			OxfordDataset::Ptr oxfDs = static_pointer_cast<OxfordDataset> (loadedDataset);
			mapResName = oxfDs->getPath() + "/vmml.map";
		}
		else if(slDatasourceType==MEIDAI_DATASET_TYPE) {
			MeidaiBagDataset::Ptr mdiDs = static_pointer_cast<MeidaiBagDataset> (loadedDataset);
			mapResName = mdiDs->getPath() + ".map";
		}
		return mapResName;
	}

	/*
	 * XXX: We should not use naked pointer here
	 */
	void map_create_cmd (const stringTokens &cmd)
	{
		if (slDatasourceType==OXFORD_DATASET_TYPE) {
			debug ("Error: Mapping (still) not supported on NU Dataset");
			return;
		}

		OxfordDataset::Ptr oxfSubset;
		OxfordDataset::Ptr oxfAll = static_pointer_cast<OxfordDataset>(loadedDataset);
		bool isSubset;

		double start, duration;
		if (cmd.size() >= 2) {
			start = stod(cmd[0]);
			duration = stod(cmd[1]);
			oxfSubset = oxfAll->timeSubset(start, duration);
			isSubset = true;
		}
		else {
			oxfSubset = oxfAll;
			start = 0;
			duration = double(oxfSubset->getTimeLength().total_microseconds())/1e6;
			isSubset = false;
		}

		debug ("About to run mapping with duration "+to_string(duration) +" seconds, " +to_string(oxfSubset->size()) + " frames");

		MapBuilder2 mapBld;
		if (mask.empty()==false) {
			float zr = loadedDataset->getZoomRatio();
			cv::Mat mapMask;
			cv::resize(mask, mapMask, cv::Size(), zr, zr, cv::INTER_CUBIC);
			mapBld.setMask(mapMask);
		}

		buildMap(oxfSubset, mapBld);

		const string mapFilePath = createMapFilename();
		mapBld.getMap()->save(mapFilePath);

		debug ("Mapping done");
		debug ("Duration " + to_string(duration) + " seconds");
		debug ("Path: " + mapFilePath);

	}

	void mask_set_cmd(const string &maskpath)
	{
		mask = cv::imread(maskpath, cv::IMREAD_GRAYSCALE);
		if (mask.empty()) {
			debug("Unable to fetch mask image file");
			return;
		}
		else
			debug("Mask read; size is "+to_string(mask.cols)+'x'+to_string(mask.rows));

		if (loadedDataset) {
			cv::Mat localizerMask;
			float zr = loadedDataset->getZoomRatio();
			cv::resize(mask, localizerMask, cv::Size(), zr, zr, cv::INTER_CUBIC);
		}
	}
};





int main (int argc, char *argv[])
{
	LocalizerApp mainApp(argc, argv);
	mainApp.loop();

	return 0;
}
