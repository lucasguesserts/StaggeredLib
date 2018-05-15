#ifndef GRID_DATA_2_HPP
#define GRID_DATA_2_HPP

#include <Eigen/Core>
#include <string>
#include <vector>
#include <CGNSFile/CGNSFile.hpp>
#include <CGNSFile/ElementDefinition.hpp>
#include <CGNSFile/BoundaryDefinition.hpp>

class GridData
{
	public:
		GridData() = default;
		GridData(const std::string){};
		GridData(CGNSFile& cgnsFile);

		unsigned dimension;
		Eigen::Matrix<double,Eigen::Dynamic,3> coordinates;
		std::vector<ElementDefinition<2>> line;
		std::vector<ElementDefinition<3>> triangle;
		std::vector<ElementDefinition<4>> quadrangle;
		std::vector<BoundaryDefinition> boundary;

		BoundaryDefinition& getBoundaryDefinition(const std::string& boundaryName);

	private:
		void readCoordinates(CGNSFile& cgnsFile);
		void readElementConnectivity(CGNSFile& cgnsFile);
		void readBoundaryDefinition(CGNSFile& cgnsFile);
};

#endif
