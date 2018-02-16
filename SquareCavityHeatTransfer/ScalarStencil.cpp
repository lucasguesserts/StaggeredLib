#include <SquareCavityHeatTransfer/ScalarStencil.hpp>

ScalarStencil operator+(const ScalarStencil& lhs, const ScalarStencil& rhs)
{
	ScalarStencil result = lhs;
	for(auto& keyValuePair: rhs)
		result[keyValuePair.first] += keyValuePair.second;
	return result;
}

ScalarStencil operator*(const double scalar, const ScalarStencil& scalarMap)
{
	ScalarStencil result = scalarMap;
	for(auto& keyValuePair: result)
		result[keyValuePair.first] *= scalar;
	return result;
}
