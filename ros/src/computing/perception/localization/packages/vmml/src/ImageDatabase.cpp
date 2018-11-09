/*
 * ImageDatabase.cpp
 *
 *  Created on: Jul 11, 2018
 *      Author: sujiwo
 */

#include <map>
#include "ImageDatabase.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "utilities.h"


using namespace std;
using namespace Eigen;


ImageDatabase::ImageDatabase(VMap *_m) :
	cMap(_m)
{
	// TODO Auto-generated constructor stub

}


ImageDatabase::~ImageDatabase()
{
	// TODO Auto-generated destructor stub
}


std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}


void
ImageDatabase::rebuildAll()
{
	// 1: Build Map Points' Descriptors

	const vector<mpid> mapPtList = cMap->getMapPointList();
	for (int i=0; i<mapPtList.size(); i++) {
		mpid mid = mapPtList[i];
		MapPoint *mp = cMap->mappoint(mid);
		vector<KeyMapPoint> kfkp;

		for (auto &kid: cMap->getRelatedKeyFrames(mid)) {
			KeyMapPoint kmp = {
				cMap->keyframe(kid),
				cMap->getKeyPointId(kid, mid)};
			kfkp.push_back(kmp);
		}

		mp->createDescriptor(kfkp);
	}

	// 2: Rebuild Vocabulary
	vector<vector<DBoW2::FORB::TDescriptor> > keymapFeatures;
	keymapFeatures.reserve(cMap->numOfKeyFrames());

//#pragma omp parallel for default(none) private(kid)
	for (auto &kid: cMap->allKeyFrames()) {
		vector<cv::Mat> kfDescriptor;

		map<mpid,kpid> mappts = cMap->allMapPointsAtKeyFrame(kid);
		for (auto &mpptr: mappts) {
			cv::Mat mpDescriptor = cMap->mappoint(mpptr.first)->getDescriptor();
			kfDescriptor.push_back(mpDescriptor);
		}

		keymapFeatures.push_back(kfDescriptor);
	}

	myVoc.create(keymapFeatures);

	// 3: Compute BoW & Feature Vectors
//#pragma omp parallel
	for (auto &kid: cMap->allKeyFrames()) {
		KeyFrame *kf = cMap->keyframe(kid);
		vector<cv::Mat> kfDescs = toDescriptorVector(kf->allDescriptors());

		// Build BoW descriptor of this keyframe
		BoWList[kid] = DBoW2::BowVector();
		FeatVecList[kid] = DBoW2::FeatureVector();
		myVoc.transform(kfDescs, BoWList[kid], FeatVecList[kid], 4);

		// Build Inverse Index
		for (auto &bowvec: BoWList[kid]) {
			const DBoW2::WordId wrd = bowvec.first;
			invertedKeywordDb[wrd].insert(kid);
		}
	}
}


// this function should be called when a new keyframe is appended to the map
void
ImageDatabase::newKeyFrameCallback (const kfid &k)
{
	seqSlamProvider.learn(cMap->keyframe(k)->getImage(), k);
}


/*
 * Reserved for loop closing
 */
kfid
ImageDatabase::find (const KeyFrame *kf) const
{
	return 0;
}


kfid
ImageDatabase::find (Frame &f, bool simple) const
{

}


vector<kfid>
ImageDatabase::findCandidates (const Frame &frame) const
{
	map<kfid, uint> kfCandidates;
	kfCandidates.clear();

	int maxCommonWords = 0;
	for (auto &bWrdPtr : frame.getWords()) {
		auto wordId = bWrdPtr.first;
		const set<kfid> &relatedKf = invertedKeywordDb.at(wordId);

		/*
		 * XXX: Mysteriously, the following loop changes kfCandidates when reading
		 */
		for (const kfid &k: relatedKf) {
			try {
				const uint count = kfCandidates.at(k);
				kfCandidates.at(k) = count+1;

			} catch (out_of_range&) {
				kfCandidates[k] = 1;
			}

			if (maxCommonWords < kfCandidates.at(k))
				maxCommonWords = kfCandidates.at(k);
		}
	}

	int minCommonWords = maxCommonWords * 0.8f;

	// Convert to scoring
	map<kfid,double> tKfCandidateScores(kfCandidates.begin(), kfCandidates.end());
	for (auto &ptr: kfCandidates) {
		const kfid &k = ptr.first;
		if (ptr.second < minCommonWords)
			tKfCandidateScores[k] = 0;
		else
			tKfCandidateScores[k] = myVoc.score(frame.getWords(), BoWList.at(k));
	}

	// Accumulate score by covisibility
	double bestAccScore = 0;
	map<kfid,double> tKfAccumScores;
	for (auto kfp: tKfCandidateScores) {

		double bestScore = kfp.second;
		double accScore = bestScore;
		kfid bestKf = kfp.first;

		vector<kfid> kfNeighs = cMap->getOrderedRelatedKeyFramesFrom(kfp.first, 10);
		for (auto &k2: kfNeighs) {
			try {
				double k2score = tKfCandidateScores.at(k2);
				accScore += k2score;
				if (bestScore < k2score) {
					bestKf = k2;
					bestScore = k2score;
				}
			} catch (exception &e) {
				continue;
			}
		}

		tKfAccumScores[kfp.first] = accScore;
		if (accScore > bestAccScore)
			bestAccScore = accScore;
	}

	// return all keyframes with accumulated scores higher than 0.75*bestAccumScore
	double minScoreToRetain = 0.75 * bestAccScore;
	vector<kfid> relocCandidates;
	for (auto &p: tKfAccumScores) {
		if (p.second > minScoreToRetain)
			relocCandidates.push_back(p.first);
	}
	// Sort
	sort(relocCandidates.begin(), relocCandidates.end(),
		[&](const kfid &k1, const kfid &k2)
		{
			double v1 = tKfAccumScores[k1],
				v2 = tKfAccumScores[k2];
			return v1>v2;
		}
	);

	return relocCandidates;
}
