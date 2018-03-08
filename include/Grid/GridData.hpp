#ifndef GRID_DATA_HPP
#define GRID_DATA_HPP

#include <Eigen/Core>
#include <string>
#include <vector>
#include <CGNSFile/CGNSFile.hpp>

class GridData
{
	public:
		GridData() = default;
		GridData(const std::string){};
		GridData(CGNSFile& cgnsFile);

		unsigned dimension;
		Eigen::Matrix<double,Eigen::Dynamic,3> coordinates;
		std::vector<ElementDefinition<3>> triangle;
		std::vector<ElementDefinition<4>> quadrangle;

	private:
		void readCoordinates(CGNSFile& cgnsFile);
		void readElementConnectivity(CGNSFile& cgnsFile);
};

#endif
