#ifndef STAGGERED_QUADRANGLE_HPP
#define STAGGERED_QUADRANGLE_HPP

#include <array>
#include <Eigen/Core>
#include <GeometricEntity/Entity.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <SquareCavityHeatTransfer/ScalarStencil.hpp>
#include <SquareCavityHeatTransfer/VectorStencil.hpp>

class StaggeredQuadrangle: public Entity
{
	std::array<Vertex*,2> vertices;
	std::array<Element*,2> elements;
	VectorStencil computeGreenGaussGradient(
			Eigen::Matrix<ScalarStencil,Eigen::Dynamic,1>& temperatureVertex_0,
			Eigen::Matrix<ScalarStencil,Eigen::Dynamic,1>& temperatureVertex_1,
			Eigen::Matrix<ScalarStencil,Eigen::Dynamic,1>& temperatureElement_0,
			Eigen::Matrix<ScalarStencil,Eigen::Dynamic,1>& temperatureElement_1)
	{
		return VectorStencil{ { 0, {0,0,0} } };
	}
};

#endif
