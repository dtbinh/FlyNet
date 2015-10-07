/*
 * TRACKLIST.cpp
 *
 *  Created on: Oct 6, 2015
 *      Author: taylordean
 */

#include "TRACKLIST.hpp"

TRACKLIST::TRACKLIST(void)
{
	length = 0;
}

int TRACKLIST::size(void)
{
	return length;
}

TRACK& TRACKLIST::getItem(const int idx)
{
	return tracks[idx];
}

void TRACKLIST::append(TRACK track)
{
	tracks[length] = track;
	length++;
}

bool TRACKLIST::isEmpty(void) {
	return length==0;
}

TRACK& TRACKLIST::operator[] (const int idx)
{
	return tracks[idx];
}
