#include <GeometricEntity/Element.hpp>
#include <Eigen/Geometry>

void Element::addVertex(const Vertex& vertex)
{
	this->vertices.push_back(&vertex);
	return;
}

Eigen::Vector3d Element::computeTriangleAreaVector(const Eigen::Vector3d& first, const Eigen::Vector3d& second, const Eigen::Vector3d& third)
{
	return (1.0/2.0) * (second - first).cross(third - first);
}

bool operator==(const Element& lhs, const Element& rhs)
{
	bool vality = true;
	if(lhs.getIndex()==rhs.getIndex())
	{
		if(lhs.vertices.size()==rhs.vertices.size())
			for(unsigned vertexLocalIndex=0 ; vertexLocalIndex<lhs.vertices.size() ; ++vertexLocalIndex)
				vality = vality && (lhs.vertices[vertexLocalIndex]==rhs.vertices[vertexLocalIndex]);
		else
			vality = false;
	}
	else
		vality = false;
	return vality;
}

std::ostream& operator<<(std::ostream& os, const Element& element)
{
	os << "{";
	os << "I:" << element.getIndex();
	for(auto& vertex: element.vertices)
		os << "," << "v:" << vertex->getIndex();
	os << "}";
	return os;
}