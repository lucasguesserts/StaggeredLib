#include <GeometricEntity/Element.hpp>
#include <vector>

void Element::addVertex(const Vertex& vertex)
{
	#ifndef NDEBUG
	//if(this->vertices.size()>=4)
	//{
		//// Add exception
	//}
	#endif
	this->vertices.push_back(&vertex);
}

Eigen::Vector3d Element::getCentroid(void)
{
	Eigen::Vector3d centroid(0.0, 0.0, 0.0);
	for(const Vertex * vertex: this->vertices)
		centroid += *(vertex);
		//centroid += *(static_cast<Eigen::Vector3d>(vertex));
	centroid /= this->vertices.size();
	return centroid;
}

unsigned Element::getNumberOfVertices(void)
{
	return this->vertices.size();
}
