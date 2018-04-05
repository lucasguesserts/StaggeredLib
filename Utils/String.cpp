#include <Utils/String.hpp>

std::string doubleToString(const double value)
{
	// numberString: "+1.1234567890e+123, " -> 18+2 char
	return (boost::format("%+10.10le") % value).str();
}