#include <map>
#include <Eigen/Core>
#include <SquareCavityHeatTransfer/ScalarStencil.hpp>

using VectorStencil = std::map<unsigned,Eigen::Vector3d>;

VectorStencil operator*(const ScalarStencil& scalarStencil, const Eigen::Vector3d& vector);
VectorStencil operator+(const VectorStencil& lhs, const VectorStencil& rhs);
ScalarStencil operator*(const Eigen::Vector3d& vector, const VectorStencil& vectorStencil);
VectorStencil operator*(const double scalar, const VectorStencil& vectorStencil);
Eigen::Vector3d operator*(const VectorStencil& vectorStencil, const Eigen::VectorXd& scalarField);
