#ifndef TERZAGHI_BOUNDARY_HPP
#define TERZAGHI_BOUNDARY_HPP

#include <array>
#include <vector>
#include <utility>
#include <Eigen/Core>

#include <GeometricEntity/StaggeredElement2D.hpp>


struct TerzaghiBoundary
{
	// Displacement
	std::vector<StaggeredElement2D*> staggeredTriangles;
	Eigen::Matrix<double,6,1> stress;
	std::array<bool,6> isStressPrescribed;
	std::array<std::pair<bool, double>,3> prescribedDisplacement;
	bool applyTerzaghiPressureInStaggeredTriangles;

	// Mass conservation
	bool isPressureDirichlet;
	Eigen::Vector3d pressureGradient;
	double pressurePrescribedValue;
};

#endif