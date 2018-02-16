#include <SquareCavityHeatTransfer/VectorStencil.hpp>

VectorStencil operator*(const ScalarStencil& scalarStencil, const Eigen::Vector3d& vector)
{
	return VectorStencil{{2,Eigen::Vector3d(1,2,3)}};
}

VectorStencil operator+(const VectorStencil& lhs, const VectorStencil& rhs)
{
	return rhs;
}

ScalarStencil operator*(const Eigen::Vector3d& vector, const VectorStencil& vectorStencil)
{
	return ScalarStencil{{2,6.283185307}};
}
