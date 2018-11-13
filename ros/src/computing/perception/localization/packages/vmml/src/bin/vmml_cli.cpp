/*
 * test_localizer.cpp
 *
 *  Created on: Aug 29, 2018
 *      Author: sujiwo
 */



#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <histedit.h>
#include <editline/readline.h>
#include <opencv2/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <boost/filesystem.hpp>

#include "VMap.h"
#include "KeyFrame.h"
#include "ImageDatabase.h"
#include "SequenceSLAM.h"
#include "Localizer.h"
#include "MapBuilder2.h"
#include "Viewer.h"
#include "datasets/OxfordDataset.h"
#include "datasets/MeidaiBagDataset.h"
#include "utilities.h"


using namespace std;
namespace fs = boost::filesystem;


struct {
	std::string velodyneCalibrationPath;
	std::string pcdMapPath;

	// XXX: Find a way to specify these values from external input
	TTransform lidarToCamera = TTransform::from_XYZ_RPY(
		Vector3d(0.9, 0.3, -0.6),
		-1.520777, -0.015, -1.5488);
} meidaiNdtParameters;


// XXX: Find a way to specify these values from external input
CameraPinholeParams meidaiCamera1Params(
	1150.96938467,	// fx
	1150.96938467,	// fy
	988.511326762,	// cx
	692.803953253,	// cy
	1920,			// width
	1440			// height
);


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
		static_cast<dataItemId>(DI->timestamp));
	f.tm = DI->getTimestamp();

	return f;
}


InputFrame createInputFrame(MeidaiDataItem::ConstPtr &DI)
{
	cv::Mat img=DI->getImage();
	cv::cvtColor(img, img, CV_BGR2GRAY, 1);

	InputFrame f(
		img,
		DI->getPosition(),
		DI->getOrientation(),
		// Force Keyframe ID using data Item ID
		DI->getId()
	);
	f.tm = DI->getTimestamp();

	return f;
}


class LocalizerApp
{
public:
	enum datasetType {
		OXFORD_DATASET_TYPE,
		MEIDAI_DATASET_TYPE
	} ;


	LocalizerApp (int argc, char *argv[]):
		mLineEditor(argv[0], TestPrompt),
		mapPath("")

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

			if (command[0][0]=='#')
				continue;

			if (command[0]=="quit")
				doExit = true;

			else if (command[0]=="map")
				{ RecordRuntime("MapOpen", map_open_cmd(command[1]) ); }

			else if (command[0]=="dataset")
				{ RecordRuntime("DatasetOpen", dataset_open_cmd(command[1], command[2])); }

			else if (command[0]=="map_pcl")
				map_dump_pcl();

			else if (command[0]=="dataset_trajectory")
				dataset_trajectory_dump(command[1]);

			else if (command[0]=="map_trajectory")
				map_trajectory_dump();

//			else if (command[0]=="find")
//				map_find_cmd(command[1]);

			else if (command[0]=="save")
				dataset_save_dsecond(command[1]);

			else if (command[0]=="savei")
				dataset_save_id(command[1]);

			else if (command[0]=="zoom")
				dataset_set_zoom(command[1]);

//			else if (command[0]=="dataset_simulate_seqslam")
//				dataset_simulate_seqslam(command[1]);

			else if (command[0]=="dataset_view")
				dataset_view(command[1]);

			else if (command[0]=="detect")
				{ RecordRuntime("PlaceDetection", map_detect_cmd(command[1]) ); }

			// To ask a subset, specify `start' and `stop' offset from beginning
			// as optional parameters
			else if (command[0]=="map_create")
				{ RecordRuntime("MapCreate", map_create_cmd(stringTokens(command.begin()+1, command.end())) ); }

			else if (command[0]=="map_info")
				map_info_cmd();

			else if (command[0]=="dataset_info")
				dataset_info_cmd();

			else if (command[0]=="mask")
				mask_set_cmd(command[1]);

			else if (command[0]=="velodyne" or command[0]=="pcdmap")
				dataset_set_param(command);

			else if (command[0]=="build")
				{ RecordRuntime("DatasetBuild", dataset_build(command) ); }

			else if (command[0]=="map_images")
				map_dump_images();
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

		int N = datasetSrc->size();
		for (int framePtr=0; framePtr<N; framePtr++) {

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
	boost::filesystem::path mapPath;

	ImageDatabase *imgDb = NULL;
	SequenceSLAM *seqSlProv = NULL;
	Localizer *localizer = NULL;

	datasetType slDatasourceType;
	GenericDataset::Ptr loadedDataset;


	cv::Mat mask;


private:


	void map_open_cmd(const string &mapPathInput)
	{
		try {
			mapSrc = new VMap();
			mapSrc->load(mapPathInput);
			localizer = new Localizer(mapSrc);
			localizer->setCameraParameterFromId(0);

			debug("Map loaded");
			imgDb = mapSrc->getImageDB();
			seqSlProv = imgDb->getSequence();
			mapPath = boost::filesystem::path(mapPathInput);

			mask = mapSrc->getMask();
			if (mask.empty()==false) {
				debug ("Map contains mask image file");
				localizer->setMask(mask);
			}

		} catch (exception &e) {
			debug ("Unable to load map");
		}
	}


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


	void dataset_build(stringTokens &cmd)
	{
		if (slDatasourceType==MEIDAI_DATASET_TYPE) {
			if (meidaiNdtParameters.pcdMapPath.empty() or meidaiNdtParameters.velodyneCalibrationPath.empty()) {
				debug ("Parameters must be set with commands `velodyne' and `pcdmap'");
				return;
			}

			MeidaiBagDataset::Ptr nuDataset;
			bool resetSubset;
			bool useNdt = true;

			if (cmd.size()<=2) {
				nuDataset = static_pointer_cast<MeidaiBagDataset>(loadedDataset);
				resetSubset = true;
				if (cmd.size()==2 and cmd[1]=="gnss")
					useNdt = false;
			}

			else {

				double startPos = stod(cmd[1]),
					stopPos = stod(cmd[2]);

				if (cmd.size()==4 and cmd[3]=="gnss")
					useNdt = false;

				debug ("Building from "+to_string(startPos) + " to " + to_string(stopPos));
				MeidaiBagDataset::Ptr nTmp = static_pointer_cast<MeidaiBagDataset>(loadedDataset);
				nuDataset = nTmp->subset(startPos, stopPos);
				resetSubset = false;
			}

			if (useNdt==false)
				debug ("Not using NDT; camera positions are estimated from GNSS");

			nuDataset->setLidarParameters(meidaiNdtParameters.velodyneCalibrationPath, meidaiNdtParameters.pcdMapPath, meidaiNdtParameters.lidarToCamera);
			nuDataset->forceCreateCache(resetSubset, useNdt);
		}

		else {
			debug("Oxford datasets need not to be built");
		}
	}


	void map_info_cmd()
	{
		if (mapSrc==NULL) {
			debug("Map not loaded");
			return;
		}
		debug("# of keyframe(s): "+to_string(mapSrc->numOfKeyFrames()));
		debug("# of map point(s): " +to_string(mapSrc->numOfMapPoints()));

		auto camInfo = mapSrc->getCameraParameter(0);
		debug("Horizontal FieldOfView (rad): " + to_string(camInfo.getHorizontalFoV()));
		debug("Vertical FieldOfView (rad): " + to_string(camInfo.getVerticalFoV()));

		auto &mapInfo = mapSrc->getAllInfo();
		for (auto rInfo: mapInfo) {
			const string
				&k = rInfo.first,
				&v = rInfo.second;
			stringstream ss;
			ss << k << " : " << v;
			debug(ss.str());
		}
	}


	void dataset_info_cmd()
	{
		if (slDatasourceType==MEIDAI_DATASET_TYPE) {
			MeidaiBagDataset::Ptr meidaiDs = static_pointer_cast<MeidaiBagDataset>(loadedDataset);
			auto cameraTrack = meidaiDs->getCameraTrajectory();

			debug("# of images:" + to_string(meidaiDs->size()));

			if (cameraTrack.size() != 0) {
				debug ("Dataset contains camera trajectory");
				if (meidaiDs->isSubset()) {
					ptime startt, stopt;
					meidaiDs->getSubsetRange(startt, stopt);
					debug ("Dataset is a subset");
				}
			}

			else {
				debug ("Dataset does not contain camera trajectory; must be built");
			}
		}
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
		uint32_t ix = 0;
		for (auto ps: mapPoses) {
			mapTrFd << ix << " ";
			mapTrFd << dumpVector(ps.first) << " " << dumpVector(ps.second) << endl;
			ix += 1;
		}

		mapTrFd.close();
		debug("Map trajectory dumped to "+dumpMapTrajectoryPath);
	}

	const string dumpDatasetTrajectoryPath = "/tmp/dump_dataset_trajectory";
	void dataset_trajectory_dump(const string &type="camera")
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

			Trajectory nuTrack;
			if (type=="gnss")
				nuTrack = meidaiDs->getGnssTrajectory();
			else if (type=="ndt")
				nuTrack = meidaiDs->getNdtTrajectory();
			else if (type=="camera")
				nuTrack = meidaiDs->getCameraTrajectory();
			else {
				debug ("Unknown trajectory type for Meidai Dataset");
				return;
			}

			string fullDumpPathName = dumpDatasetTrajectoryPath + '-' + type + ".csv";
			fstream dsTrFd (fullDumpPathName, ios_base::out|ios_base::trunc);
			dsTrFd << fixed << setprecision(6);

			for (auto &ps: nuTrack) {
				dsTrFd << ps.timeSecond() << " "
						<< dumpVector(ps.position()) << " "
						<< dumpVector(ps.orientation())
						<< endl;
			}

			dsTrFd.close();
			debug("Dataset trajectory dumped to "+fullDumpPathName);
			return;
		}
	}

	void dataset_open_cmd(const string &dsPath, const string &modelDir)
	{
		boost::filesystem::path datasetPath(dsPath);

		if (boost::filesystem::is_directory(datasetPath)) {
			loadedDataset = OxfordDataset::load(datasetPath.string(), modelDir);
			slDatasourceType = OXFORD_DATASET_TYPE;
			debug ("Oxford-type Dataset Loaded");
		}

		else if (datasetPath.extension()==".bag") {
			loadedDataset = MeidaiBagDataset::load(datasetPath.string());
			slDatasourceType = MEIDAI_DATASET_TYPE;
			debug ("Nagoya University Dataset Loaded");
		}

		else {
			debug("Unsupported dataset type");
			return;
		}
	}


	void dataset_set_param(const stringTokens &command)
	{
		if (slDatasourceType==MEIDAI_DATASET_TYPE) {
			if (command[0]=="velodyne")
				meidaiNdtParameters.velodyneCalibrationPath = command[1];
			else if (command[0]=="pcdmap")
				meidaiNdtParameters.pcdMapPath = command[1];
		}
	}


	/*
	 * You can feed an offset number from dataset, or a path to an image
	 */
	void map_detect_cmd(const string &detectionStr)
	{
		if (localizer==NULL) {
			debug("Map not loaded");
			return;
		}

		cv::Mat img;

		if (boost::filesystem::exists(boost::filesystem::path(detectionStr))) {
			img = cv::imread(detectionStr, cv::IMREAD_COLOR);
			if (img.empty()) {
				debug ("Unable to open requested file");
				return;
			}
		}

		else {
			double d = std::stod(detectionStr);
			auto di = loadedDataset->atDurationSecond(d);
			img = di->getImage();
		}

		cv::cvtColor(img, img, CV_RGB2GRAY);
		kfid kmap;
		Pose computedPose;
		bool gotplace = localizer->detect(img, kmap, computedPose);

		if (gotplace) {
			debug(computedPose.str());
			debug("Result: "+to_string(kmap));
		}
		else {
			debug ("Unable to detect place");
		}
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

	void dataset_save_id(const string &sid)
	{
		dataItemId requestId = static_cast<dataItemId>(std::stoi(sid));

		auto md = loadedDataset->get(requestId);
		cv::Mat img = md->getImage();
		cv::imwrite(dumpImagePath, img);
		debug("Image #" + sid + " dumped to " + dumpImagePath);
	}

	void debug(const string &s, double is_error=false)
	{
		if (!is_error)
			cerr << s << endl << flush;
		else
			cout << s << endl << flush;
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


	void map_create_cmd (const stringTokens &cmd)
	{
		GenericDataset::Ptr targetDataset;
		double duration;
		uint32_t numOfFrames;

		MapBuilder2 mapBld;

		if (slDatasourceType==OXFORD_DATASET_TYPE) {
			OxfordDataset::Ptr oxfSubset;
			OxfordDataset::Ptr oxfAll = static_pointer_cast<OxfordDataset>(loadedDataset);
			bool isSubset;

			double start, stop;
			if (cmd.size() >= 2) {
				start = stod(cmd[0]);
				stop = stod(cmd[1]);
				duration = stop - start;
				oxfSubset = oxfAll->timeSubset(start, duration);
				isSubset = true;
			}
			else {
				oxfSubset = oxfAll;
				start = 0;
				duration = double(oxfSubset->getTimeLength().total_microseconds())/1e6;
				isSubset = false;
			}

			targetDataset = oxfSubset;
			numOfFrames = oxfSubset->size();

			// Information
			mapBld.getMap()->setInfo("sourceType", "Oxford");
			mapBld.getMap()->setInfo("originalPath", oxfSubset->getPath());
		}

		else if (slDatasourceType==MEIDAI_DATASET_TYPE) {

			// Meidai Dataset does not have integrated camera parameters (yet)
			MeidaiBagDataset::Ptr meidaiDs = static_pointer_cast<MeidaiBagDataset>(loadedDataset);
			meidaiDs->addCameraParameter(meidaiCamera1Params);

			if (cmd.size() >= 2) {
				double start, stop;
				start = stod(cmd[0]);
				stop = stod(cmd[1]);
				duration = stop - start;
				MeidaiBagDataset::Ptr meidaiSub = meidaiDs->subset(start, stop);
				numOfFrames = meidaiSub->size();
				targetDataset = meidaiSub;
			}

			else {
				targetDataset = meidaiDs;
				numOfFrames = meidaiDs->size();
				duration = meidaiDs->length();
			}

			// Information
			mapBld.getMap()->setInfo("sourceType", "Meidai");
			mapBld.getMap()->setInfo("originalPath", meidaiDs->getPath());
		}

		debug ("About to run mapping with duration "+to_string(duration) +" seconds, " +to_string(numOfFrames) + " frames");

		if (mask.empty()==false) {
			float zr = loadedDataset->getZoomRatio();
			cv::Mat mapMask;
			cv::resize(mask, mapMask, cv::Size(), zr, zr, cv::INTER_CUBIC);
			mapBld.setMask(mapMask);
		}

		buildMap(targetDataset, mapBld);

		const string mapFilePath = createMapFilename();
		mapBld.getMap()->save(mapFilePath);

		debug ("Mapping done");
		debug ("Dataset time elapsed: " + to_string(duration) + " seconds");
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


	void map_dump_images()
	{
		if (localizer==NULL) {
			debug ("Map not loaded");
			return;
		}

		MeidaiBagDataset::Ptr meidaiDs;
		auto originalPath = mapSrc->getInfo("originalPath");
		try {
			meidaiDs = MeidaiBagDataset::load(originalPath);
		} catch (exception &e) {
			debug ("Unable to open dataset "+originalPath);
			return;
		}

		boost::filesystem::path dumpDir = mapPath.parent_path() /= "/mapdump";
		boost::filesystem::create_directory(dumpDir);

		for (auto &kfI: mapSrc->allKeyFrames()) {
			auto srcItemId = mapSrc->keyframe(kfI)->getSourceItemId();
			auto srcDataItem = meidaiDs->get(srcItemId);
			cv::Mat cImage = srcDataItem->getImage();
			string imgNew = dumpDir.string() + '/' + (to_string(srcItemId) + ".jpg");
			cv::imwrite(imgNew, cImage);
		}

		debug ("Map keyframes dumped to " + dumpDir.string());
	}

};





int main (int argc, char *argv[])
{
	LocalizerApp mainApp(argc, argv);
	mainApp.loop();

	return 0;
}
