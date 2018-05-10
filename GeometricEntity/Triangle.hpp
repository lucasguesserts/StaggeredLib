#ifndef TRIANGLE_HPP
#define TRIANGLE_HPP

#include <Eigen/Core>
#include <GeometricEntity/Element.hpp>

class Triangle: public Element
{
	public:
		virtual Eigen::Vector3d getCentroid(void) override;
		virtual Eigen::Vector3d getAreaVector(void) override;
		virtual double getVolume(void) override;
		virtual Eigen::VectorXd getShapeFunctionValues(const Eigen::Vector3d localCoordinates) final;
		virtual Eigen::MatrixXd getShapeFunctionDerivatives(const Eigen::Vector3d localCoordinates) final;

		static const std::array<Eigen::Vector3d,3> staggeredElementFaceCentroidLocalIndex;
};

#endif
