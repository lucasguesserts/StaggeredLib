#ifndef BOUNDARY_DEFINITION_HPP
#define BOUNDARY_DEFINITION_HPP

#include <vector>
#include <string>
namespace cgns
{
	#include <cgnslib.h>
}

struct BoundaryDefinition
{
    int index;
    char name[200];
    cgns::BCType_t type;
	cgns::cgsize_t numberOfElements;
    std::vector<unsigned> elementsIndexList;

	int normalIndex;
	cgns::PointSetType_t pointSetType;
	cgns::cgsize_t normalListSize;
	cgns::DataType_t dataType;
	int numberOfDataSets;
};

BoundaryDefinition& getBoundaryDefinitionInVector(const std::string& boundaryName, std::vector<BoundaryDefinition> boundaryVector);

#endif