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
};

#endif
