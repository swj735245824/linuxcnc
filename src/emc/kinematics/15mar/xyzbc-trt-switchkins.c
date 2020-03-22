#define TRT_KINS_NAME        "xyzbc-trt-switchkins"
#define TRT_KINS_REQD_COORDS "xyzbc"
#define TRT_KINS_FORWARD      xyzbcKinematicsForward
#define TRT_KINS_INVERSE      xyzbcKinematicsInverse

// backwards compatible hal pin names:
#define TRT_KINS_PREFIX      "xyzbc-trt-kins"
#include "trt-switchkins.h"
