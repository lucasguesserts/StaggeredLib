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

bool operator==(const StaggeredTriangle& lhs, const StaggeredTriangle& rhs)
{
	return lhs.getIndex()==rhs.getIndex() &&
	       lhs.vertices[0]==rhs.vertices[0] &&
	       lhs.vertices[1]==rhs.vertices[1] &&
	       lhs.elements==rhs.elements;
}

bool operator==(const StaggeredQuadrangle& lhs, const StaggeredQuadrangle& rhs)
{
	return lhs.getIndex()==rhs.getIndex() &&
	       *lhs.vertices[0]==*rhs.vertices[0] &&
	       *lhs.vertices[1]==*rhs.vertices[1] &&
	       lhs.elements[0]==rhs.elements[0] &&
	       lhs.elements[1]==rhs.elements[1];
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

std::ostream& operator<<(std::ostream& os, const StaggeredTriangle& staggeredTriangle)
{
	os << "StaggeredTriangle{"
	   << "I:" << staggeredTriangle.getIndex()
	   << ","
	   << "v:" << staggeredTriangle.vertices[0]->getIndex()
	   << ","
	   << "e:" << staggeredTriangle.elements[0]->getIndex()
	   << ","
	   << "v:" << staggeredTriangle.vertices[1]->getIndex()
	   << "}";
	return os;
}

std::ostream& operator<< (std::ostream& os, const StaggeredQuadrangle& staggeredQuadrangle)
{
	os << "StaggeredQuadrangle{"
	   << "I:" << staggeredQuadrangle.getIndex()
	   << ","
	   << "v:" << staggeredQuadrangle.vertices[0]->getIndex()
	   << ","
	   << "e:" << staggeredQuadrangle.elements[0]->getIndex()
	   << ","
	   << "v:" << staggeredQuadrangle.vertices[1]->getIndex()
	   << ","
	   << "e:" << staggeredQuadrangle.elements[1]->getIndex()
	   << "}";
	return os;
}