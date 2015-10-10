/*
 * TRACKLIST.hpp
 *
 *  Created on: Oct 6, 2015
 *      Author: taylordean
 */

#ifndef TRACKLIST_HPP_
#define TRACKLIST_HPP_

#include "TRACK.hpp"
#include "TRACKLIST.hpp"
#include "OBJECT_CONTAINER.hpp"

class TRACKLIST
{
public:
	// Parameters
	TRACK tracks[1000];
	int length;

	// Functions
	TRACKLIST(void);
	int size(void);
	TRACK& getItem(const int idx);
	void append(TRACK track);
	bool isEmpty(void);
	TRACK& operator[] (const int idx);
};

#endif /* TRACKLIST_HPP_ */
