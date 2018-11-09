/*
 * MTContainers.h
 *
 *  Created on: Nov 9, 2018
 *      Author: sujiwo
 */

#ifndef _MTCONTAINERS_H_
#define _MTCONTAINERS_H_


/*
 * XXX : Stub for thread-safe containers
 */



#include <vector>
#include <map>
#include <set>
#include <thread>
#include <mutex>


namespace Mt {

template<typename T>
class vector : public std::vector<T>
{
public:

	void push_back (const T& val)
	{
		cLock.lock();
		std::vector<T>::push_back(val);
		cLock.unlock();
	}

protected:
	std::mutex cLock;
};


template<typename K, typename V>
class map : public std::map<K,V>
{
public:

	// insert

	// key access

	// at

protected:

	std::mutex cLock;
};


}	// namespace Mt


#endif /* _MTCONTAINERS_H_ */
