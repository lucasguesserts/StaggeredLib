#ifndef GRID_2D_WITH_STAGGERED_ELEMENTS
#define GRID_2D_WITH_STAGGERED_ELEMENTS

#include <vector>
#include <string>
#include <map>

#include <Grid/StaggeredElementDefinition.hpp>
#include <Grid/GridData.hpp>
#include <Grid/Grid2DVerticesWithNeighborElements.hpp>
#include <GeometricEntity/StaggeredQuadrangle.hpp>
#include <GeometricEntity/StaggeredTriangle.hpp>
#include <Grid/Boundary.hpp>

class Grid2DWithStaggeredElements: public Grid2DVerticesWithNeighborElements
{
	public:
		Grid2DWithStaggeredElements(const GridData& gridData);

		std::vector<StaggeredQuadrangle> staggeredQuadrangles;
		std::vector<StaggeredTriangle> staggeredTriangles;
		std::vector<StaggeredElementDefinition> staggeredElementDefinition;
		std::map<std::string,Boundary> boundary;

		void createStaggeredElementDefinitionVector(const GridData& gridData);
		void createStaggeredElements(void);
		template <unsigned NumberOfVertices>
			void addStaggeredElementDefinitionFromElementDefinition(const ElementDefinition<NumberOfVertices>& elementDefinition);
		void addStaggeredElementDefinition(const StaggeredElementDefinition& staggeredElementDefinition);
		std::tuple<bool,unsigned> findStaggeredElementDefinition(const StaggeredElementDefinition& staggeredElementDefinition);

		void createBoundaries(const GridData& gridData);
		std::vector<StaggeredTriangle*> findStaggeredTrianglesInBoundaryDefinition(const BoundaryDefinition& boundaryDefinition);
		Line& findLine(unsigned lineIndex);
		StaggeredTriangle* findStaggeredTriangle(const Line& line);

		static void organizeQuadrangle(StaggeredQuadrangle& staggeredQuadrangle);
		static void organizeTriangle(StaggeredTriangle& staggeredTriangle);
		void organizeStaggeredElements(void);

	private:
		void allocateStaggeredElementDefinition(const GridData& gridData);
		void shrinkStaggeredElementDefinition(void);
};

#include <Grid/Grid2DWithStaggeredElements.tpp>

#endif
