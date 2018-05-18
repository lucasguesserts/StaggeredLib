#ifndef GRID_2D_WITH_STAGGERED_ELEMENTS_2
#define GRID_2D_WITH_STAGGERED_ELEMENTS_2

#include <vector>
#include <map>

#include <Grid/GridData_2.hpp>
#include <Grid/Grid2DVerticesWithNeighborElements.hpp>
#include <GeometricEntity/StaggeredElement2D.hpp>
#include <GeometricEntity/Face2D.hpp>
#include <Grid/Boundary.hpp>

class Grid2DWithStaggeredElements: public Grid2DVerticesWithNeighborElements
{
	public:
		Grid2DWithStaggeredElements(const GridData_2& gridData);
		Grid2DWithStaggeredElements(const std::string& fileName);
		std::vector<StaggeredElement2D> staggeredElements;
		std::vector<StaggeredElement2D*> staggeredQuadrangles;
		std::vector<StaggeredElement2D*> staggeredTriangles;
		std::vector<std::vector<StaggeredElement2D*>> verticesNeighborStaggeredElements;
		std::vector<Face2D> faces;
		std::map<std::string,Boundary> boundary;

		void createStaggeredElements(void);
		void createFaces(void);
		std::tuple<bool,unsigned> findStaggeredElement(const StaggeredElement2D& staggeredElement);
		bool staggeredElementsHaveTheSameVertices(const StaggeredElement2D& lhs, const StaggeredElement2D& rhs);
		std::array<unsigned,2> findStaggeredElements(Vertex* adjacentVertex, Element* element);
		void organizeStaggeredElementsForFace(Element* element, Vertex* adjacentVertex, StaggeredElement2D*& back, StaggeredElement2D*& front);
		void setStaggeredTrianglesAndQuadrangles(void);

		void createBoundaries(const GridData_2& gridData);
		std::vector<StaggeredElement2D*> findStaggeredTrianglesInBoundaryDefinition(const BoundaryDefinition& boundaryDefinition);
		Line& findLine(unsigned lineIndex);
		StaggeredElement2D* findStaggeredTriangle(const Line& line);

		void createBoundaries(void);
		std::vector<StaggeredElement2D*> findStaggeredTrianglesInBoundary(const BoundaryData& boundary);

		void setVerticesNeighborStaggeredElements(void);
};

#endif