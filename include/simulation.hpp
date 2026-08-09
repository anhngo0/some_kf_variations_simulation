#pragma once
#include "filters/basic_filter.hpp"
#include "filters/pcrlb.hpp"
#include "filters/pf.hpp"
#include "filters/upf.hpp"
#include "movement_sim/trajectory.hpp"
#include "../include/utils/evaluate.hpp"

#include <nlohmann/json.hpp>
#include <fstream>
#include <random>
#include <cmath>
#include <iostream>

const double FRAME_STEP = 1; /*1s*/
const int MAX_FRAME = 210;
const int MONTE_CARLO_SAMPLES = 150;
// const int MONTE_CARLO_SAMPLES = 1;
const double ROTATE_ANGLE_VEL = M_PI / 64;

using json = nlohmann::json;

void simulate();