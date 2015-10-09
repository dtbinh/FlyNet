/*
 * TRACK.cpp
 *
 *  Created on: Oct 6, 2015
 *      Author: taylordean
 */
#include <iostream>
#include "TRACK.hpp"

using namespace std;

TRACK::TRACK(void)
{
	active = true;
	length = 0;
}

int TRACK::size(void)
{
	return length;
}

void TRACK::sort(void)
{
	bool changed = true;

	while (changed) {
		changed = false;
		for (int idx=1; idx<length - 1; idx++) {
			if (history[idx]->gmt > history[idx + 1]->gmt) {
				OBJECT* tempObj = history[idx];
				history[idx] = history[idx + 1];
				history[idx + 1] = tempObj;
				changed = true;
			}
		}
	}
}

OBJECT*& TRACK::oldest(void)
{
	return history[0];
}

OBJECT*& TRACK::newest(void)
{
	return history[length - 1];
}

void TRACK::append(OBJECT* obj)
{
	history[length] = obj;
	length++;
	sort();
}

void TRACK::append(TRACK* track)
{
	for (int trkIdx=0; trkIdx<track->size(); trkIdx++) {
		append(track->getItem(trkIdx));
	}
}

bool TRACK::isEmpty(void)
{
	return length==0;
}

OBJECT*& TRACK::getItem(const int idx)
{
	return history[idx];
}

OBJECT*& TRACK::operator[] (const int idx)
{
	return history[idx];
}
