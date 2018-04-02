#ifndef ELEMENT_HPP
#define ELEMENT_HPP

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <GeometricEntity/Entity.hpp>
#include <GeometricEntity/Vertex.hpp>

class Element: public Entity
{
	public:
		std::vector<const Vertex*> vertices;

		void addVertex(const Vertex& vertex);
		virtual Eigen::Vector3d getCentroid(void) = 0;
		virtual Eigen::Vector3d getAreaVector(void) = 0;
		virtual double getVolume(void) = 0;
		static Eigen::Vector3d computeTriangleAreaVector(const Eigen::Vector3d& first, const Eigen::Vector3d& second, const Eigen::Vector3d& third);
};

bool operator==(const Element& lhs, const Element& rhs);
std::ostream& operator<<(std::ostream& os, const Element& element);

#endif
