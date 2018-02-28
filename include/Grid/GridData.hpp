#ifndef GRID_DATA_HPP
#define GRID_DATA_HPP

#include <Eigen/Core>
#include <string>
#include <vector>
#include <Grid/ElementDefinition.hpp>

struct GridData
{
	GridData(){}
	GridData(const std::string cgnsFileName);
	static const std::string projectGridDirectory;

	unsigned dimension;
	Eigen::Matrix<double,Eigen::Dynamic,3> coordinates;
	std::vector<ElementDefinition<3>> triangle;
	std::vector<ElementDefinition<4>> quadrangle;

	private:
		int fileIndex, zoneIndex, baseIndex, numberOfSections;
		unsigned numberOfVertices, numberOfElements;

		void openFile(const std::string cgnsFileName);
		void openBase(void);
		void openZone(void);
		void verifyNumberOfGrids(void);
		void verifyNumberOfCoordinates(void);
		void readCoordinates(void);
		void readNumberOfSections(void);
		void readElementConnectivity(void);
};

#endif
