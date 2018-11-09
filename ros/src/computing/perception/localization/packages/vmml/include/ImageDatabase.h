/*
 * ImageDatabase.h
 *
 *  Created on: Jul 11, 2018
 *      Author: sujiwo
 */

#ifndef IMAGEDATABASE_H_
#define IMAGEDATABASE_H_


#include <vector>
#include <set>
#include <map>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_member.hpp>
#include "DBoW2/BowVector.h"
#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"
#include "VMap.h"
#include "cvobj_serialization.h"
#include "SequenceSLAM.h"


// ===============

class ORBVocabulary : public DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
{
protected:
	friend class boost::serialization::access;

	template<class Archive>
	void save(Archive &ar, const unsigned int v) const
	{
		ar << m_k;
		ar << m_L;
		ar << m_scoring;
		ar << m_weighting;

		uint numNodes = m_nodes.size();
		uint numWords = m_words.size();
		ar << numNodes;
		ar << numWords;

		for (int i=0; i<numNodes; i++) {
			const Node &n = m_nodes[i];
			ar << n.id;
			ar << n.id;
			ar << n.weight;
			ar << n.children;
			ar << n.parent;
			ar << n.descriptor;
			ar << n.word_id;
		}
	}

	template<class Archive>
	void load(Archive &ar, const unsigned int v)
	{
		ar >> m_k;
		ar >> m_L;
		ar >> m_scoring;
		ar >> m_weighting;

		createScoringObject();

		uint numNodes, numWords;
		ar >> numNodes;
		ar >> numWords;

		// nodes
	    m_nodes.resize(numNodes);
	    m_words.resize(numWords);

	    for (int i=0; i<numNodes; i++) {
	    	Node &n = m_nodes[i];
	    	ar >> n.id;
			ar >> n.id;
			ar >> n.weight;
			ar >> n.children;
			ar >> n.parent;
			ar >> n.descriptor;
			ar >> n.word_id;

			if (n.isLeaf()) {
				m_words[n.word_id] = &n;
			}
	    }
	}

	BOOST_SERIALIZATION_SPLIT_MEMBER();
};


class Frame;


class ImageDatabase
{

public:
	ImageDatabase (VMap *_m);
	virtual ~ImageDatabase();

	void addKeyFrame (const kfid &kfId);

	void rebuildAll ();

	// Used for loop detection
	kfid find (const KeyFrame *kf) const;

	kfid find (Frame &f, bool simple=false) const;

	/*
	 * Find matched keyframe candidates from database using BoW method.
	 * The candidates are sorted according to number of similar words
	 */
	std::vector<kfid>
	findCandidates (const Frame &f) const;

	DBoW2::BowVector getWords (const kfid &k)
	{ return BoWList[k]; }

	const ORBVocabulary &vocabulary() const
	{ return myVoc; }

	void newKeyFrameCallback (const kfid &k);

	SequenceSLAM *getSequence ()
	{ return &seqSlamProvider; }

	DBoW2::FeatureVector& getFeatureVectorFromKeyFrame (const kfid kf)
	{ return FeatVecList.at(kf); }

protected:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive &ar, const unsigned int file_version)
	{
		ar & myVoc;
		ar & invertedKeywordDb;
		ar & BoWList;
		ar & FeatVecList;

		// XXX: Resulting string may get too big
//		ar & seqSlamProvider;
	}

private:
	ORBVocabulary myVoc;
	std::map<DBoW2::WordId, std::set<kfid> > invertedKeywordDb;

	VMap *cMap;

	std::map<kfid, DBoW2::BowVector> BoWList;
	std::map<kfid, DBoW2::FeatureVector> FeatVecList;

	SequenceSLAM seqSlamProvider;
};


std::vector<cv::Mat>
toDescriptorVector(const cv::Mat &Descriptors);


#endif /* IMAGEDATABASE_H_ */
