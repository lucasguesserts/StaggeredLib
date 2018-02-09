#ifndef VERTEX_HPP
#define VERTEX_HPP

#include <Grid/Entity.hpp>
#include <Eigen/Core>

class Vertex: public Eigen::Vector3d, public Entity
{
	public:
		Vertex(const double x, const double y, const double z, const unsigned handle);
};

#endif
