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
		vector<cv::Mat> kfDescs = toDescriptorVector(kf->getDescriptors());

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


kfid
ImageDatabase::find (const KeyFrame *kf) const
{
	return 0;
}


kfid
ImageDatabase::find (Frame &f, bool simple) const
{
	f.computeBoW(*this);

	map<kfid, uint> kfCandidates;
	kfCandidates.clear();

	for (auto &bWrdPtr : f.getWords()) {
		auto wid = bWrdPtr.first;
		const set<kfid> &relatedKf = invertedKeywordDb.at(wid);

		for (const kfid &k: relatedKf) {

			try {
				const uint count = kfCandidates.at(k);
				kfCandidates.at(k) = count+1;

			} catch (out_of_range&) {
				kfCandidates[k] = 1;
			}
		}
	}

	if (simple) {
		auto ptr = maximumMapElement(kfCandidates);
		return ptr.first;
	}

	// Convert to scoring
	map<kfid,double> kfCandidateScores(kfCandidates.begin(), kfCandidates.end());
	for (auto &ptr: kfCandidates) {
		const kfid &k = ptr.first;
		kfCandidateScores[k] = myVoc.score(f.getWords(), BoWList.at(k));
	}

	auto ptr = maximumMapElement(kfCandidateScores);
	return ptr.first;
}
