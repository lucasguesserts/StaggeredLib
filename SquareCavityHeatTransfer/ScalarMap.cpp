#include <SquareCavityHeatTransfer/ScalarMap.hpp>

ScalarMap operator+(const ScalarMap& lhs, const ScalarMap& rhs)
{
	return rhs;
}

ScalarMap operator*(const double scalar, const ScalarMap& scalarMap)
{
	return scalarMap;
}
