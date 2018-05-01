#include <GeometricEntity/Test.hpp>

bool operator==(const Vertex& lhs, const Vertex& rhs)
{
	return lhs.getIndex()==rhs.getIndex() &&
	       lhs.coeff(0)==Approx(rhs.coeff(0)) &&
	       lhs.coeff(1)==Approx(rhs.coeff(1)) &&
	       lhs.coeff(2)==Approx(rhs.coeff(2));
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

bool operator==(const StaggeredElement& lhs, const StaggeredElement& rhs)
{
	bool vality = true;
	if(lhs.getIndex()==rhs.getIndex())
	{
		if((lhs.vertices.size()==rhs.vertices.size()) &&
		   (lhs.elements.size()==rhs.elements.size()))
		{
			for(unsigned vertexLocalIndex=0 ; vertexLocalIndex<lhs.vertices.size() ; ++vertexLocalIndex)
				vality = vality && (lhs.vertices[vertexLocalIndex]==rhs.vertices[vertexLocalIndex]);
			for(unsigned elementLocalIndex=0 ; elementLocalIndex<lhs.elements.size() ; ++elementLocalIndex)
				vality = vality && (lhs.elements[elementLocalIndex]==rhs.elements[elementLocalIndex]);
		}
		else
			vality = false;
	}
	else
		vality = false;
	return vality;
}

bool operator==(const Face& lhs, const Face& rhs)
{
	return
		lhs.getIndex()==rhs.getIndex() &&
		lhs.localIndex==rhs.localIndex &&
		lhs.parentElement==rhs.parentElement &&
		lhs.adjacentVertex==rhs.adjacentVertex &&
		lhs.backwardStaggeredElement==rhs.backwardStaggeredElement &&
		lhs.forwardStaggeredElement==rhs.forwardStaggeredElement;
}

std::ostream& operator<< (std::ostream& os, const Vertex& vertex)
{
	os << "Vertex{"
	   << vertex.getIndex()
	   << ","
	   << std::showpos << std::scientific << std::setprecision(10) << vertex.coeff(0)
	   << ","
	   << std::showpos << std::scientific << std::setprecision(10) << vertex.coeff(1)
	   << ","
	   << std::showpos << std::scientific << std::setprecision(10) << vertex.coeff(2)
	   << "}";
	return os;
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

std::ostream& operator<<(std::ostream& os, const Line& line)
{
	os << "Line" << static_cast<const Element&>(line);
	return os;
}

std::ostream& operator<<(std::ostream& os, const Triangle& triangle)
{
	os << "Triangle" << static_cast<const Element&>(triangle);
	return os;
}

std::ostream& operator<<(std::ostream& os, const Quadrangle& quadrangle)
{
	os << "Quadrangle" << static_cast<const Element&>(quadrangle);
	return os;
}

std::ostream& operator<<(std::ostream& os, const StaggeredElement& staggeredElement)
{
	os << "{";
	os << "I:" << staggeredElement.getIndex();
	for(unsigned localIndex=0 ; localIndex<staggeredElement.vertices.size() ; ++localIndex)
	{
		os<< "," << "v:" << staggeredElement.vertices[localIndex]->getIndex();
		if(localIndex<staggeredElement.elements.size())
			os<< "," << "e:" << staggeredElement.elements[localIndex]->getIndex();
	}
	os << "}";
	return os;
}

std::ostream& operator<<(std::ostream& os, const StaggeredTriangle& staggeredTriangle)
{
	os << "StaggeredTriangle" << static_cast<const StaggeredElement&>(staggeredTriangle);
	return os;
}

std::ostream& operator<< (std::ostream& os, const StaggeredQuadrangle& staggeredQuadrangle)
{
	os << "StaggeredQuadrangle" << static_cast<const StaggeredElement&>(staggeredQuadrangle);
	return os;
}

std::ostream& operator<< (std::ostream& os, const Face& face)
{
	os << "Face{"
	   << "I:" << face.getIndex() << ","
	   << "li:" << face.localIndex << ","
	   << "e:" << face.parentElement << ","
	   << "v:" << face.adjacentVertex << ","
	   << "b_se:" << face.backwardStaggeredElement << ","
	   << "f_se:" << face.forwardStaggeredElement
	   << "}";
	   return os;
}