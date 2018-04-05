#ifndef SCALAR_MAP_HPP
#define SCALAR_MAP_HPP

#include <Utils/catch.hpp>
#include <string>
#include <map>
#include <iostream>
#include <Eigen/Core>

using ScalarStencil = std::map<unsigned,double>;

ScalarStencil operator+(const ScalarStencil& lhs, const ScalarStencil& rhs);
ScalarStencil operator*(const double scalar, const ScalarStencil& scalarMap);
double operator*(const ScalarStencil& scalarStencil, const Eigen::VectorXd& scalarField);

bool operator==(const ScalarStencil& lhs, const ScalarStencil& rhs);

std::string scalarStencilToString(const ScalarStencil& scalarStencil);
namespace Catch
{
	template<>
	struct StringMaker<ScalarStencil>
	{
		static std::string convert( ScalarStencil const& scalarStencil )
		{
			return scalarStencilToString(scalarStencil);
		}
	};
}

#endif
