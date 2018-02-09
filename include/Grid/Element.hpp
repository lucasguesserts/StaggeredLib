#ifndef ELEMENT_HPP
#define ELEMENT_HPP

#include <vector>
#include <Eigen/Core>
#include <Grid/Entity.hpp>
#include <Grid/Vertex.hpp>

class Element: public Entity
{
	public:
		std::vector<const Vertex*> vertices;
		Eigen::Vector3d getCentroid(void);
		virtual double getVolume(void) = 0;
		virtual Eigen::VectorXd getShapeFunctionValues(const Eigen::Vector3d localCoordinates) const = 0;
		virtual Eigen::MatrixXd getShapeFunctionDerivatives(const Eigen::Vector3d localCoordinates) const = 0;
};

#endif
