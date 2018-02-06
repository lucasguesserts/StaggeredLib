#ifndef ELEMENT_HPP
#define ELEMENT_HPP

#include <Eigen/Core>
#include <Grid/Entity.hpp>
#include <Grid/VertexCollection.hpp>

class Element: public Entity, public VertexCollection
{
	public:
		Eigen::Vector3d getCentroid(void);
		virtual double getVolume(void) const = 0;
		virtual Eigen::VectorXd getShapeFunctionValues(const Eigen::Vector3d localCoordinates) const = 0;
		virtual Eigen::MatrixXd getShapeFunctionDerivatives(const Eigen::Vector3d localCoordinates) const = 0;
	private:
};

#endif
