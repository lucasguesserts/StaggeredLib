#include <GeometricEntity/Vertex.hpp>
#include <Utils/catch.hpp>
#include <iomanip>

Vertex::Vertex(const double x, const double y, const double z, const unsigned index)
	: Eigen::Vector3d(x,y,z), Entity(index) {}

bool operator==(const Vertex& lhs, const Vertex& rhs)
{
	return lhs.getIndex()==rhs.getIndex() &&
	       lhs.coeff(0)==Approx(rhs.coeff(0)) &&
	       lhs.coeff(1)==Approx(rhs.coeff(1)) &&
	       lhs.coeff(2)==Approx(rhs.coeff(2));
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