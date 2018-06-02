#include <Stencil/Test.hpp>
#include <stdexcept>

bool operator==(const ScalarStencil& lhs, const ScalarStencil& rhs)
{
	bool vality = true;
	if(lhs.size()==rhs.size())
	{
		for(auto& keyValuePair: lhs)
		{
			auto rhsValueIterator = rhs.find(keyValuePair.first);
			if(rhsValueIterator!=rhs.cend())
			{
				if(keyValuePair.second==0.0 || (*rhsValueIterator).second==0.0)
					vality = vality && (keyValuePair.second - (*rhsValueIterator).second) < 1E-12;
				else
					vality = vality && keyValuePair.second==Approx((*rhsValueIterator).second);
			}
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

std::string scalarStencilToString(const ScalarStencil& scalarStencil)
{
	// Format: {{1, +5.9e-2},{7, -8.3e+5}}
	std::string str;
	str += "{";
	for(auto& keyValuePair: scalarStencil)
    {
		str += "{";
		str += std::to_string(keyValuePair.first);
		str += ",";
		str += doubleToString(keyValuePair.second);
		str += "},";
	}
	str += "}";
	return str;
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
	if(vectorStencil.size()==1)
	{
		VectorStencil::const_iterator it = vectorStencil.cbegin();
		str += vectorStencilPairToString("", *it, "");
	}
	else
	{
		VectorStencil::const_iterator it = vectorStencil.cbegin();
			str += vectorStencilPairToString("", *it, ",\n");
		for(it=(++vectorStencil.cbegin()) ; it!=(--vectorStencil.cend()) ; ++it)
			str += vectorStencilPairToString(" ", *it, ",\n");
		it = --vectorStencil.cend();
			str += vectorStencilPairToString(" ", *it, "");
	}
	str += "}";
	return str;
}