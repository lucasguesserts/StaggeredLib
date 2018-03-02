#ifndef GRID_DATA_HPP
#define GRID_DATA_HPP

#include <Eigen/Core>
#include <string>
#include <vector>
#include <Grid/CGNSFile.hpp>

class GridData
{
	public:
		GridData() = default;
		GridData(const std::string){};
		GridData(CGNSFile& cgnsFile);

		static const std::string projectGridDirectory;
		unsigned dimension;
		Eigen::Matrix<double,Eigen::Dynamic,3> coordinates;
		std::vector<ElementDefinition<3>> triangle;
		std::vector<ElementDefinition<4>> quadrangle;

	private:
		void readCoordinates(CGNSFile& cgnsFile);
		void readElementConnectivity(CGNSFile& cgnsFile);
};

#endif
