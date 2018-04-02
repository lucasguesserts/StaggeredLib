#include <Stencil/ScalarStencil.hpp>

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

bool operator==(const ScalarStencil& lhs, const ScalarStencil& rhs)
{
	bool vality = true;
	if(lhs.size()==rhs.size())
	{
		for(auto& keyValuePair: lhs)
		{
			auto rhsValueIterator = rhs.find(keyValuePair.first);
			if(rhsValueIterator!=rhs.cend())
				vality = vality && keyValuePair.second==Approx((*rhsValueIterator).second);
			else
			{
				vality = false;
				break;
			}
		}
	}
	else
		vality = false;
	return vality;
}