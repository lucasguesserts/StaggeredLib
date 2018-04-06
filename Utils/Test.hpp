#ifndef UTILS_TEST_H
#define UTILS_TEST_H

#include <Utils/catch.hpp>
#include <vector>

#define TestCase TEST_CASE
#define section SECTION

#define check CHECK
#define checkFalse CHECK_FALSE
#define require REQUIRE
#define requireFalse REQUIRE_FALSE

#ifndef NDEBUG
#define EIGEN_INITIALIZE_MATRICES_BY_NAN
#endif

template <typename T>
bool operator==(const std::vector<T>& lhs, const std::vector<T>& rhs)
{
	bool vality = true;
	if(lhs.size()==rhs.size())
		for(typename std::vector<T>::size_type i=0 ; i<lhs.size() ; ++i)
			vality = vality && lhs[i]==rhs[i];
	else
		vality = false;
	return vality;
}

#endif
