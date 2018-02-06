#include <Grid/Vertex.hpp>

Vertex::Vertex(const double x, const double y, const double z, const unsigned handle)
	: Eigen::Vector3d(x,y,z), Entity(handle) {}

Vertex::Vertex(void): Eigen::Vector3d(0.0, 0.0, 0.0), Entity(0) {}
