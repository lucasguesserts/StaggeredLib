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
		template <unsigned NumberOfVerticesPerElement, cgns::ElementType_t ElementType> std::vector< ElementDefinition<NumberOfVerticesPerElement> > readElementsDefinition(void)
		{
			this->readNumberOfSections();
			unsigned numberOfElements = this->getNumberOfElementsOfType(ElementType);
			std::vector< ElementDefinition<NumberOfVerticesPerElement> > element(numberOfElements);
			for(int sectionIndex=1 ; sectionIndex<=this->numberOfSections ; ++sectionIndex)
			{
				std::tuple<cgns::cgsize_t,cgns::cgsize_t,cgns::ElementType_t> sectionData = this->readSection(sectionIndex);
				if(std::get<2>(sectionData)==ElementType)
				{
					const cgns::cgsize_t firstElementIndex = std::get<0>(sectionData);
					const cgns::cgsize_t lastElementIndex = std::get<1>(sectionData);
					std::vector<cgns::cgsize_t> elementConnectivity = this->readElementConnectivity(sectionIndex);
					for(unsigned elementCount=0 ; elementCount<numberOfElements ; ++elementCount)
					{
						element[elementCount].index = firstElementIndex + elementCount;
						for(unsigned vertexIndex=0 ; vertexIndex<NumberOfVerticesPerElement ; ++vertexIndex)
							element[elementCount].connectivity(vertexIndex) = elementConnectivity[NumberOfVerticesPerElement*elementCount+vertexIndex] - 1;
					}
				}
			}
			return element;
		}

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
