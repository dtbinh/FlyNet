/*
 * TRACK.hpp
 *
 *  Created on: Oct 6, 2015
 *      Author: taylordean
 */

#ifndef TRACK_HPP_
#define TRACK_HPP_

#include "OBJECT.hpp"

using namespace std;

class TRACK
{
public:
	// Parameters
	OBJECT* history[1000];
	bool active;
	int length;

	// Functions
	TRACK(void);
	int size(void);
	void sort(void);
	bool isEmpty(void);
	OBJECT*& getItem(const int idx);
	OBJECT*& newest(void);
	OBJECT*& oldest(void);
	void append(OBJECT* obj);
	void append(TRACK* track);
	OBJECT*& operator[] (const int idx);
};

#endif /* TRACK_HPP_ */
