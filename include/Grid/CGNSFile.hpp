#ifndef CGNS_FILE_HPP
#define CGNS_FILE_HPP

#include <string>
#include <vector>
#include <tuple>
namespace cgns
{
	#include <cgnslib.h>
}
#include <Grid/ElementDefinition.hpp>

class CGNSFile
{
	public:
		CGNSFile(const std::string cgnsFileName);
		std::vector<double> readCoordinate(const std::string& coordinateName);
		std::vector< ElementDefinition<4> > readQuadrangleElementsDefinition(void);
		std::vector< ElementDefinition<3> > readTriangleElementsDefinition(void);

		int fileIndex, zoneIndex, baseIndex;
		int cellDimension, physicalDimension; // read in base
		unsigned numberOfVertices, numberOfElements; // read in zone
		int numberOfSections;


	private:
		// TODO: add exceptions.
		void openFile(const std::string cgnsFileName);
		void openBase(void);
		void openZone(void);
		void verifyNumberOfGrids(void);
		void verifyNumberOfCoordinates(void);
		void readNumberOfSections(void);
		unsigned getNumberOfElementsOfType(const cgns::ElementType_t elementType);
		std::tuple<cgns::cgsize_t,cgns::cgsize_t,cgns::ElementType_t> readSection(const int sectionIndex);
		cgns::cgsize_t readSizeOfElementConnectivityDataArray(const int sectionIndex);
		std::vector<cgns::cgsize_t> readElementConnectivity(const int sectionIndex);
};

#endif
