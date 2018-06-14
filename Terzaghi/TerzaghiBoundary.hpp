#ifndef TERZAGHI_BOUNDARY_HPP
#define TERZAGHI_BOUNDARY_HPP

#include <Eigen/Core>
#include <tuple>
#include <vector>
#include <array>
#include <string>

#include <Terzaghi/Enums.hpp>

struct TerzaghiBoundary
{
	std::string name;
	std::vector<StaggeredElement2D*> staggeredTriangle;
	std::array<Component, static_cast<size_t>(Component::NumberOfComponents)> component;
	std::array<BoundaryConditionType, static_cast<size_t>(Component::NumberOfComponents)> boundaryConditionType;
	std::array<double, static_cast<size_t>(Component::NumberOfComponents)> prescribedValue;
	Eigen::Matrix<double,6,1> stress;
};

#endif