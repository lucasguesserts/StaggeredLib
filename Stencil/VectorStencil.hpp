#ifndef VECTOR_STENCIL_HPP
#define VECTOR_STENCIL_HPP

#include <map>
#include <Eigen/Core>
#include <Stencil/ScalarStencil.hpp>
#include <string>
#include <Utils/String.hpp>

using VectorStencil = std::map<unsigned,Eigen::Vector3d>;

VectorStencil operator*(const ScalarStencil& scalarStencil, const Eigen::Vector3d& vector);
VectorStencil operator+(const VectorStencil& lhs, const VectorStencil& rhs);
ScalarStencil operator*(const Eigen::Vector3d& vector, const VectorStencil& vectorStencil);
VectorStencil operator*(const double scalar, const VectorStencil& vectorStencil);
Eigen::Vector3d operator*(const VectorStencil& vectorStencil, const Eigen::VectorXd& scalarField);

bool operator==(const VectorStencil& lhs, const VectorStencil& rhs);
std::string vectorStencilToString(const VectorStencil& vectorStencil);

namespace Catch
{
	template<>
	struct StringMaker<VectorStencil>
	{
		static std::string convert( VectorStencil const& vectorStencil )
		{
			return vectorStencilToString(vectorStencil);
		}
	};
}

#endif