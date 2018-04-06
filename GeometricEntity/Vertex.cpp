#include <GeometricEntity/Vertex.hpp>

Vertex::Vertex(const double x, const double y, const double z, const unsigned index)
	: Eigen::Vector3d(x,y,z), Entity(index) {}