#ifndef GRID_2D_WITH_STAGGERED_ELEMENTS_2
#define GRID_2D_WITH_STAGGERED_ELEMENTS_2

#include <vector>

#include <Grid/GridData.hpp>
#include <Grid/Grid2DVerticesWithNeighborElements.hpp>
#include <GeometricEntity/StaggeredElement2D.hpp>
#include <GeometricEntity/Face2D.hpp>

class Grid2DWithStaggeredElements_2: public Grid2DVerticesWithNeighborElements
{
	public:
		Grid2DWithStaggeredElements_2(const GridData& gridData);
		std::vector<StaggeredElement2D> staggeredElements;
		std::vector<StaggeredElement2D*> staggeredQuadrangles;
		std::vector<StaggeredElement2D*> staggeredTriangles;
		std::vector<Face2D> faces;

		void createStaggeredElements(void);
		void createFaces(void);
		std::tuple<bool,unsigned> findStaggeredElement(const StaggeredElement2D& staggeredElement);
		bool staggeredElementsHaveTheSameVertices(const StaggeredElement2D& lhs, const StaggeredElement2D& rhs);
		std::array<unsigned,2> findStaggeredElements(Vertex* adjacentVertex, Element* element);
		void organizeStaggeredElementsForFace(Element* element, Vertex* adjacentVertex, StaggeredElement2D*& back, StaggeredElement2D*& front);
		void setStaggeredTrianglesAndQuadrangles(void);
};

#endif