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
#include <GeometricEntity/StaggeredElement.hpp>
#include <GeometricEntity/StaggeredTriangle.hpp>
#include <GeometricEntity/StaggeredQuadrangle.hpp>
#include <GeometricEntity/Face.hpp>
#include <GeometricEntity/Face2D.hpp>
#include <GeometricEntity/StaggeredElement2D.hpp>

bool operator==(const Vertex& lhs, const Vertex& rhs);
bool operator==(const Element& lhs, const Element& rhs);
bool operator==(const StaggeredElement& lhs, const StaggeredElement& rhs);
bool operator==(const StaggeredElement2D& lhs, const StaggeredElement2D& rhs);
bool operator==(const Face& lhs, const Face& rhs);
bool operator==(const Face2D& lhs, const Face2D& rhs);

std::ostream& operator<< (std::ostream& os, const Vertex& vertex);
std::ostream& operator<<(std::ostream& os, const Element& element);
std::ostream& operator<<(std::ostream& os, const Line& line);
std::ostream& operator<<(std::ostream& os, const Triangle& triangle);
std::ostream& operator<<(std::ostream& os, const Quadrangle& quadrangle);
std::ostream& operator<<(std::ostream& os, const StaggeredElement& staggeredElement);
std::ostream& operator<<(std::ostream& os, const StaggeredElement2D& staggeredElement);
std::ostream& operator<<(std::ostream& os, const StaggeredTriangle& staggeredTriangle);
std::ostream& operator<< (std::ostream& os, const StaggeredQuadrangle& StaggeredQuadrangle);
std::ostream& operator<< (std::ostream& os, const Face& face);

#endif