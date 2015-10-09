/*
 * OBJECT_CONTAINER.hpp
 *
 *  Created on: Oct 6, 2015
 *      Author: taylordean
 */

#ifndef OBJECT_CONTAINER_HPP_
#define OBJECT_CONTAINER_HPP_

#include "OBJECT.hpp"

using namespace std;

class OBJECT_CONTAINER
{
public:
	// Parameters
	OBJECT history[1000];
	bool active;
	int length;

	// Functions
	OBJECT_CONTAINER(void);
	int size(void);
	void sort(void);
	bool isEmpty(void);
	OBJECT& getItem(const int idx);
	OBJECT& newest(void);
	OBJECT& oldest(void);
	void append(OBJECT obj);
	void append(OBJECT_CONTAINER objectContainer);
	OBJECT& operator[] (const int idx);
};

#endif /* OBJECT_CONTAINER_HPP_ */
