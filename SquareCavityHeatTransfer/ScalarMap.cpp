#include <SquareCavityHeatTransfer/ScalarMap.hpp>

ScalarMap operator+(const ScalarMap& lhs, const ScalarMap& rhs)
{
	ScalarMap result = lhs;
	for(auto& keyValuePair: rhs)
		result[keyValuePair.first] += keyValuePair.second;
	return result;
}

ScalarMap operator*(const double scalar, const ScalarMap& scalarMap)
{
	ScalarMap result = scalarMap;
	for(auto& keyValuePair: result)
		result[keyValuePair.first] *= scalar;
	return result;
}
