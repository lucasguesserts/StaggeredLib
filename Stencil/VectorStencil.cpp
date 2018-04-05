#include <Stencil/VectorStencil.hpp>
#include <vector>
#include <Utils/catch.hpp>
#include <Utils/EigenTest.hpp>

VectorStencil operator*(const ScalarStencil& scalarStencil, const Eigen::Vector3d& vector)
{
	VectorStencil vectorStencil;
	for(auto& indexScalarPair: scalarStencil)
		vectorStencil[indexScalarPair.first] = indexScalarPair.second * vector;
	return vectorStencil;
}

VectorStencil operator+(const VectorStencil& lhs, const VectorStencil& rhs)
{
	VectorStencil sum = rhs;
	for(auto& indexVectorPair: lhs)
	{
		if( sum.find(indexVectorPair.first) != sum.end() )
			sum[indexVectorPair.first] += indexVectorPair.second;
		else
			sum[indexVectorPair.first] = indexVectorPair.second;
	}
	return sum;
}

ScalarStencil operator*(const Eigen::Vector3d& vector, const VectorStencil& vectorStencil)
{
	ScalarStencil scalarStencil;
	for(auto& indexVectorPair: vectorStencil)
		scalarStencil[indexVectorPair.first] = vector.dot(indexVectorPair.second);
	return scalarStencil;
}

VectorStencil operator*(const double scalar, const VectorStencil& vectorStencil)
{
	VectorStencil resultVectorStencil = vectorStencil;
	for(auto& indexVectorPair: resultVectorStencil)
		indexVectorPair.second *= scalar;
	return resultVectorStencil;
}

Eigen::Vector3d operator*(const VectorStencil& vectorStencil, const Eigen::VectorXd& scalarField)
{
	Eigen::Vector3d reconstructedValue = Eigen::Vector3d::Zero();
	for(auto& keyValuePair: vectorStencil)
	{
		const unsigned index = keyValuePair.first;
		const Eigen::Vector3d weightVector = keyValuePair.second;
		reconstructedValue += scalarField[index] * weightVector;
	}
	return reconstructedValue;
}


bool operator==(const VectorStencil& lhs, const VectorStencil& rhs)
{
	bool vality = true;
	if(lhs.size()==rhs.size())
	{
		for(auto& keyValuePair: lhs)
		{
			auto rhsValueIterator = rhs.find(keyValuePair.first);
			if(rhsValueIterator!=rhs.cend())
				vality = vality && keyValuePair.second==(*rhsValueIterator).second;
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

std::string vectorStencilPairToString(const std::string& initialChars, const std::pair<unsigned,Eigen::Vector3d>& pair, const std::string& finalChars)
{
	// numberString: "+1.1234567890e+123, " -> 18+2 char
	// row format: " -2.1234567890e+385, +8.9437813657e-028,\n" -> initialChars = " ", finalChars = ",\n"
	return initialChars +
	       "{" +
	       std::to_string(pair.first) +
	       "," +
	       eigenVector3dToString(pair.second) +
	       "}" +
	       finalChars;
}

std::string vectorStencilToString(const VectorStencil& vectorStencil)
{
	// Format:
	// {{1, [-1.27e-3,+4.19e+53,-2.83e-01]},
	// {{4, [-1.27e-3,+4.19e+53,-2.83e-01]}}
	std::string str;
	str += "{";
	VectorStencil::const_iterator it = vectorStencil.cbegin();
		str += vectorStencilPairToString("", *it, ",\n");
	for(it=(++vectorStencil.cbegin()) ; it!=(--vectorStencil.cend()) ; ++it)
		str += vectorStencilPairToString(" ", *it, ",\n");
	it = --vectorStencil.cend();
		str += vectorStencilPairToString(" ", *it, "");
	str += "}";
	return str;
}