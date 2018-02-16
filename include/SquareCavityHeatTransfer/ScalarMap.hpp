#ifndef SCALAR_MAP_HPP
#define SCALAR_MAP_HPP

#include <map>

using ScalarMap = std::map<unsigned,double>;

ScalarMap operator+(const ScalarMap& lhs, const ScalarMap& rhs);
ScalarMap operator*(const double scalar, const ScalarMap& scalarMap);

#endif
