#include <Grid/Element.hpp>
#include <vector>

Eigen::Vector3d Element::getCentroid(void)
{
	Eigen::Vector3d centroid(0.0, 0.0, 0.0);
	for(const Vertex * vertex: this->getVertices())
		centroid += *(vertex);
		//centroid += *(static_cast<Eigen::Vector3d>(vertex));
	centroid /= this->getNumberOfVertices();
	return centroid;
}
