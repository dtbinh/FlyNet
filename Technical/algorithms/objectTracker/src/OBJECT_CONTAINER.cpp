/*
 * OBJECT_CONTAINER.cpp
 *
 *  Created on: Oct 6, 2015
 *      Author: taylordean
 */

#include <iostream>
#include "OBJECT_CONTAINER.hpp"

using namespace std;

OBJECT_CONTAINER::OBJECT_CONTAINER(void)
{
	active = true;
	length = 0;
}

int OBJECT_CONTAINER::size(void)
{
	return length;
}

void OBJECT_CONTAINER::sort(void)
{
	bool changed = true;

	while (changed) {
		changed = false;
		for (int idx=1; idx<length - 1; idx++) {
			if (history[idx].gmt > history[idx + 1].gmt) {
				OBJECT tempObj = history[idx];
				history[idx] = history[idx + 1];
				history[idx + 1] = tempObj;
				changed = true;
			}
		}
	}
}

OBJECT& OBJECT_CONTAINER::oldest(void)
{
	return history[0];
}

OBJECT& OBJECT_CONTAINER::newest(void)
{
	return history[length - 1];
}

void OBJECT_CONTAINER::append(OBJECT obj)
{
	history[length] = obj;
	length++;
	sort();
}

void OBJECT_CONTAINER::append(OBJECT_CONTAINER objectContainer)
{
	for (int trkIdx=0; trkIdx<objectContainer.size(); trkIdx++) {
		append(objectContainer[trkIdx]);
	}
}

bool OBJECT_CONTAINER::isEmpty(void)
{
	return length==0;
}

OBJECT& OBJECT_CONTAINER::getItem(const int idx)
{
	return history[idx];
}

OBJECT& OBJECT_CONTAINER::operator[] (const int idx)
{
	return history[idx];
}
