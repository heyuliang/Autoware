/**
 * File: FeatureVector.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: feature vector
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_FEATURE_VECTOR__
#define __D_T_FEATURE_VECTOR__

#include <map>
#include <vector>
#include <iostream>
#include "BowVector.h"
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>


namespace DBoW2 {

/// Vector of nodes with indexes of local features
class FeatureVector: 
  public std::map<NodeId, std::vector<unsigned int> >
{
public:

  /**
   * Constructor
   */
  FeatureVector(void);
  
  /**
   * Destructor
   */
  ~FeatureVector(void);
  
  /**
   * Adds a feature to an existing node, or adds a new node with an initial
   * feature
   * @param id node id to add or to modify
   * @param i_feature index of feature to add to the given node
   */
  void addFeature(NodeId id, unsigned int i_feature);

  /**
   * Sends a string versions of the feature vector through the stream
   * @param out stream
   * @param v feature vector
   */
  friend std::ostream& operator<<(std::ostream &out, const FeatureVector &v);
    

	friend class boost::serialization::access;

	template<class Archive>
	void serialize(Archive &ar, const unsigned int version)
	{
		ar & boost::serialization::base_object
			<std::map<NodeId, std::vector<unsigned int> > >
			(*this);
	}

//	template<class Archive>
//	void save(Archive &ar, const unsigned int version)
//	{
// 		ar << const_cast<std::map<NodeId, std::vector<unsigned int> > > (*this);
//	}
//
//	template<class Archive>
//	void load(Archive &ar, const unsigned int version)
//	{
//		std::map<NodeId, std::vector<unsigned int> > rCopy;
//		ar >> rCopy;
//	}
//
//	BOOST_SERIALIZATION_SPLIT_MEMBER()
};

} // namespace DBoW2

#endif

