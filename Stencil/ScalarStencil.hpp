#ifndef SCALAR_MAP_HPP
#define SCALAR_MAP_HPP

#include <map>
#include <Eigen/Core>

using ScalarStencil = std::map<unsigned,double>;

ScalarStencil operator+(const ScalarStencil& lhs, const ScalarStencil& rhs);
ScalarStencil operator*(const double scalar, const ScalarStencil& scalarMap);
double operator*(const ScalarStencil& scalarStencil, const Eigen::VectorXd& scalarField);

#endif
