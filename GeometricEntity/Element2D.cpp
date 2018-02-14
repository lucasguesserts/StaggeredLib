#include <GeometricEntity/Element2D.hpp>

const unsigned Element2D::dimension = 2;

Eigen::Vector3d Element2D::computeTriangleAreaVector(
		const Vertex* const firstPoint,
		const Vertex* const secondPoint,
		const Vertex* const thirdPoint) const
{
	return (1.0/2.0) * (*(secondPoint) - *(firstPoint)).cross(*(thirdPoint) - *(firstPoint));
}
