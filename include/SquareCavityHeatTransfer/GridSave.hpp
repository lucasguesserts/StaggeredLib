#ifndef GRID_SAVE_HPP
#define GRID_SAVE_HPP

#include <string>
#include <Eigen/Core>

class GridSave
{
	public:
		static void savePermanentScalarFieldToCGNS(const std::string solutionName, const std::string scalarFieldName, const Eigen::VectorXd& scalarField, const std::string cgnsFileName);
		static Eigen::VectorXd readPermanentSolutionFromCGNSFile(const std::string cgnsFileName);
};

#endif
