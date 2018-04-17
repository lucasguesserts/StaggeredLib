#include <CGNSFile/BoundaryDefinition.hpp>
#include <stdexcept>

BoundaryDefinition& getBoundaryDefinitionInVector(const std::string& boundaryName, std::vector<BoundaryDefinition> boundaryVector)
{
	for(auto& boundary: boundaryVector)
		if(std::string(boundary.name)==boundaryName) return boundary;
	throw std::runtime_error(std::string(__FUNCTION__) + std::string(": ") + boundaryName + std::string(" not found."));
}