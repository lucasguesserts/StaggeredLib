#include <Grid/Vertex.hpp>

Vertex::Vertex(const double x, const double y, const double z, const unsigned handle)
	: Eigen::Vector3d(x,y,z), Entity(handle) {}
