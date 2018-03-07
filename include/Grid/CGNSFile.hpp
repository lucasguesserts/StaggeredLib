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
		~CGNSFile() { cgns::cg_close(this->fileIndex); }
		std::vector<double> readCoordinate(const std::string& coordinateName);
		template <unsigned NumberOfVerticesPerElement, cgns::ElementType_t ElementType>
			std::vector< ElementDefinition<NumberOfVerticesPerElement> > readElementsDefinition(void);
		void writeSteadyScalarField(const std::string& solutionName, const std::string& scalarFieldName, const Eigen::VectorXd& scalarField);
		Eigen::VectorXd readSteadyScalarField(const std::string& solutionName, const std::string& scalarFieldName);
		void writeTransientScalarField(const std::string& scalarFieldName, const unsigned timeStep, const Eigen::VectorXd scalarField);
		void writeTransientInformation(const std::string& scalarFieldName, const Eigen::VectorXd& timeInstants);
		Eigen::VectorXd readTransientScalarField(void)
		{
			constexpr unsigned numberOfElements = 6;
			Eigen::VectorXd transientSolution;
			transientSolution.resize(numberOfElements);
			transientSolution << 0.0, 1.0, 3.0, 2.0, 5.0, 4.0;
			return transientSolution;
		}

		int cellDimension, physicalDimension; // read in base
		unsigned numberOfVertices, numberOfElements; // read in zone
	private:
		int fileIndex, zoneIndex, baseIndex;
		int numberOfSections;

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
		void checkNumberOfSections(void);
		template <unsigned NumberOfVerticesPerElement, cgns::ElementType_t ElementType>
			void setElementsDefinitionUsingSection(std::vector< ElementDefinition<NumberOfVerticesPerElement> >& element, const int sectionIndex);
		template <unsigned NumberOfVerticesPerElement>
			ElementDefinition<NumberOfVerticesPerElement> getElementDefinitionFromElementConnectivity(const std::vector<cgns::cgsize_t>& elementConnectivity,cgns::cgsize_t firstElementIndex, unsigned elementCount);
		int getSolutionIndex(const std::string solutionName);
		void verifyGridLocationOfSolution(const int solutionIndex);
		cgns::cgsize_t readSolutionSize(const int solutionIndex);
		void writeTimeIterativeBase(const std::string& timeIterativeBaseName, const int numberOfTimeSteps);
		void writeTimeInstantsInIterativeBase(const std::string& timeIterativeBaseName, const Eigen::VectorXd& timeInstants);
		void writeTimeIterativeZone(const std::string& scalarFieldName, const std::string& timeIterativeZoneName, const Eigen::VectorXd& timeInstants);
		std::string getSolutionName(const std::string& scalarFieldName, const unsigned timeStep);
		std::string getSolutionNamesForTimeIterativeZone(const std::string& scalarFieldName, const unsigned numberOfTimeSteps);
		void writeSimulationType(void);
};

#include <Grid/CGNSFile.tpp>

#endif
