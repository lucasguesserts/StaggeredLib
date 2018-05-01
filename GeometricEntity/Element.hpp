#ifndef ELEMENT_HPP
#define ELEMENT_HPP

#include <vector>
#include <Eigen/Core>
#include <GeometricEntity/Entity.hpp>
#include <GeometricEntity/Vertex.hpp>

class Element: public Entity
{
	public:
		std::vector<Vertex*> vertices;

		void addVertex(Vertex& vertex);
		virtual Eigen::Vector3d getCentroid(void) = 0;
		virtual Eigen::Vector3d getAreaVector(void) = 0;
		virtual double getVolume(void) = 0;
		static Eigen::Vector3d computeTriangleAreaVector(const Eigen::Vector3d& first, const Eigen::Vector3d& second, const Eigen::Vector3d& third);
};

#endif
