#ifndef UTILS_TEST_H
#define UTILS_TEST_H

#define CATCH_CONFIG_MAIN
#include <Utils/catch.hpp>

#define TestCase TEST_CASE
#define section SECTION

#define check CHECK
#define checkFalse CHECK_FALSE
#define require REQUIRE
#define requireFalse REQUIRE_FALSE

#ifndef NDEBUG
#define EIGEN_INITIALIZE_MATRICES_BY_NAN 
#endif

#endif
