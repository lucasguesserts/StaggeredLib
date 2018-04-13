#ifndef GEOMETRIC_ENTITY_TEST_HPP
#define GEOMETRIC_ENTITY_TEST_HPP

#include <iostream>
#include <iomanip>
#include <Utils/Test.hpp>

#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/Line.hpp>
#include <GeometricEntity/Quadrangle.hpp>
#include <GeometricEntity/Triangle.hpp>
#include <GeometricEntity/StaggeredTriangle.hpp>
#include <GeometricEntity/StaggeredQuadrangle.hpp>

bool operator==(const Vertex& lhs, const Vertex& rhs);
bool operator==(const Element& lhs, const Element& rhs);
bool operator==(const StaggeredTriangle& lhs, const StaggeredTriangle& rhs);
bool operator==(const StaggeredQuadrangle& lhs, const StaggeredQuadrangle& rhs);

std::ostream& operator<< (std::ostream& os, const Vertex& vertex);
std::ostream& operator<<(std::ostream& os, const Element& element);
std::ostream& operator<<(std::ostream& os, const Line& line);
std::ostream& operator<<(std::ostream& os, const Triangle& triangle);
std::ostream& operator<<(std::ostream& os, const Quadrangle& quadrangle);
std::ostream& operator<<(std::ostream& os, const StaggeredTriangle& staggeredTriangle);
std::ostream& operator<< (std::ostream& os, const StaggeredQuadrangle& StaggeredQuadrangle);

#endif