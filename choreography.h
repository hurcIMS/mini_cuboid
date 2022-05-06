#pragma once
#include <map>
#include <vector>
#include "minicube_parametermap.h"

enum {
	LAY_DOWN = 1,
	BALANCE = 2,
	WIGGLE_SLOW = 3,
	WIGGLE_FAST = 4,
	WALK_LEFT = 5,
	WALK_RIGHT = 6,
};

static std::map<int, std::vector<int>> CHOREOGRAPHY = {
	{BLUE_1, {1,2,3,4,5,6,2,3,4,2,}},
	{BLUE_2, {1,1,2,4,6,5,2,1,2,2,}},
	{BLUE_3, {1,2,3,4,5,6,2,3,4,2,}},
	{BLUE_4, {1,1,2,4,6,5,2,1,2,2,}},
	{BLUE_5, {1,2,3,4,5,6,2,3,4,2,}},
	{BLUE_6, {1,1,2,4,6,5,2,1,2,2,}},
};
