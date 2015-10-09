/*
 * amm.cpp
 *
 *  Created on: Oct 6, 2015
 *      Author: taylordean
 */

#include <iostream>
#include "OBJECT.hpp"
#include "TRACK.hpp"
#include "TRACKLIST.hpp"

using namespace std;

void assembleTracks(OBJECT_CONTAINER* GLOBAL_RETURNS, TRACKLIST* GLOBAL_TRACKS) {
	if (GLOBAL_TRACKS->isEmpty()) {
		TRACK temp;
		for (int objIdx=0; objIdx<GLOBAL_RETURNS->size(); objIdx++) {
			temp.append(&GLOBAL_RETURNS->getItem(objIdx));
		}
		GLOBAL_TRACKS->append(temp);
	} else {
		GLOBAL_TRACKS->getItem(0).append(&GLOBAL_RETURNS->newest());
	}
}
