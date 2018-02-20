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

double operator*(const ScalarStencil& scalarStencil, const Eigen::VectorXd& scalarField)
{
	double interpolatedValue = 0.0;
	for(auto& keyValuePair: scalarStencil)
	{
		const unsigned index = keyValuePair.first;
		const double weightValue = keyValuePair.second;
		interpolatedValue += weightValue * scalarField[index];
	}
	return interpolatedValue;
}
