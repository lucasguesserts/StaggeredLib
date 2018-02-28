#ifndef GRID_DATA_HPP
#define GRID_DATA_HPP

#include <Eigen/Core>
#include <string>

struct GridData
{
	GridData(){}
	GridData(const std::string cgnsFileName);
	static const std::string projectGridDirectory;

	unsigned dimension;
	Eigen::Matrix<double,Eigen::Dynamic,3> coordinates;
	Eigen::Matrix<unsigned,Eigen::Dynamic,3> triangleConnectivity;
	Eigen::Matrix<unsigned,Eigen::Dynamic,4> quadrangleConnectivity;

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
