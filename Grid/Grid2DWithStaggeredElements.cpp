#include <Grid/Grid2DWithStaggeredElements.hpp>
#include <exception>
#include <algorithm>
#include <stdexcept>

Grid2DWithStaggeredElements::Grid2DWithStaggeredElements(const GridData& gridData)
	: Grid2DVerticesWithNeighborElements(gridData)
{
	this->allocateStaggeredElementDefinition(gridData);
	this->createStaggeredElementDefinitionVector(gridData);
	this->shrinkStaggeredElementDefinition();
	this->createStaggeredElements();
	this->organizeStaggeredElements();
	this->createBoundaries(gridData);
	return;
}

void Grid2DWithStaggeredElements::allocateStaggeredElementDefinition(const GridData& gridData)
{
	const unsigned numberOfTriangles = gridData.triangle.size();
	const unsigned numberOfQuadrangles = gridData.quadrangle.size();
	const unsigned maxNumberOfStaggeredElements = 3*numberOfQuadrangles + 2*numberOfTriangles + 1;
	this->staggeredElementDefinition.reserve(maxNumberOfStaggeredElements);
	return;
}

void Grid2DWithStaggeredElements::shrinkStaggeredElementDefinition(void)
{
	this->staggeredElementDefinition.shrink_to_fit();
	return;
}

void Grid2DWithStaggeredElements::createStaggeredElementDefinitionVector(const GridData& gridData)
{
	for(auto& elementDefinition: gridData.quadrangle)
		this->addStaggeredElementDefinitionFromElementDefinition(elementDefinition);
	for(auto& elementDefinition: gridData.triangle)
		this->addStaggeredElementDefinitionFromElementDefinition(elementDefinition);
	return;
}

void Grid2DWithStaggeredElements::createStaggeredElements(void)
{
	unsigned staggeredElementIndex = 0;
	for(auto& staggeredElementDefinition: this->staggeredElementDefinition)
	{
		Vertex& vertex_0 = this->vertices[staggeredElementDefinition.vertices[0]];
		Vertex& vertex_1 = this->vertices[staggeredElementDefinition.vertices[1]];
		Element* element_0 = this->elements[staggeredElementDefinition.elements[0]];
		if(staggeredElementDefinition.type==StaggeredElementDefinition::Type::Quadrangle)
		{
			Element* element_1 = this->elements[staggeredElementDefinition.elements[1]];
			this->staggeredQuadrangles.emplace_back(StaggeredQuadrangle(staggeredElementIndex,vertex_0,element_0,vertex_1,element_1));
		}
		else // if(staggeredElementDefinition.type==StaggeredElementDefinition::Type::Triangle)
			this->staggeredTriangles.emplace_back(StaggeredTriangle(staggeredElementIndex,vertex_0,element_0,vertex_1));
		staggeredElementIndex++;
	}
}

void Grid2DWithStaggeredElements::addStaggeredElementDefinition(const StaggeredElementDefinition& staggeredElementDefinition)
{
	std::tuple<bool,unsigned> elementDefinitionLocation = this->findStaggeredElementDefinition(staggeredElementDefinition);
	if( std::get<bool>(elementDefinitionLocation) )
		this->staggeredElementDefinition[std::get<unsigned>(elementDefinitionLocation)].addElement(staggeredElementDefinition.elements[0]);
	else
		this->staggeredElementDefinition.push_back(staggeredElementDefinition);
	return;
}

std::tuple<bool,unsigned> Grid2DWithStaggeredElements::findStaggeredElementDefinition(const StaggeredElementDefinition& staggeredElementDefinition)
{
	bool elementExists;
	auto iterator = std::find(this->staggeredElementDefinition.cbegin(), this->staggeredElementDefinition.cend(), staggeredElementDefinition);
	if(iterator==this->staggeredElementDefinition.cend())
		elementExists = false;
	else
		elementExists = true;
	unsigned position = std::distance(this->staggeredElementDefinition.cbegin(),iterator);
	return std::make_tuple(elementExists,position);
}

void Grid2DWithStaggeredElements::organizeQuadrangle(StaggeredQuadrangle& staggeredQuadrangle)
{
	if( staggeredQuadrangle.getAreaVector().dot(staggeredQuadrangle.elements[0]->getCentroid() - staggeredQuadrangle.elements[1]->getCentroid()) < 0 )
		std::swap(staggeredQuadrangle.elements[0],staggeredQuadrangle.elements[1]);
	return;
}
void Grid2DWithStaggeredElements::organizeTriangle(StaggeredTriangle& staggeredTriangle)
{
	if( staggeredTriangle.getAreaVector().dot(staggeredTriangle.element->getCentroid() - *(staggeredTriangle.vertices[0])) < 0 )
		std::swap(staggeredTriangle.vertices[0],staggeredTriangle.vertices[1]);
	return;
}

void Grid2DWithStaggeredElements::organizeStaggeredElements(void)
{
	for(auto& staggeredQuadrangle: this->staggeredQuadrangles)
		Grid2DWithStaggeredElements::organizeQuadrangle(staggeredQuadrangle);
	for(auto& staggeredTriangle: this->staggeredTriangles)
		Grid2DWithStaggeredElements::organizeTriangle(staggeredTriangle);
	return;
}

void Grid2DWithStaggeredElements::createBoundaries(const GridData& gridData)
{
	for(const BoundaryDefinition& boundaryDefinition: gridData.boundary)
		this->boundary[boundaryDefinition.name] = this->findStaggeredTrianglesInBoundaryDefinition(boundaryDefinition);
	return;
}

std::vector<StaggeredTriangle*> Grid2DWithStaggeredElements::findStaggeredTrianglesInBoundaryDefinition(const BoundaryDefinition& boundaryDefinition)
{
	const std::vector<unsigned>& lineIndices = boundaryDefinition.elementsIndexList;
	std::vector<StaggeredTriangle*> boundaryStaggeredTriangles;
	boundaryStaggeredTriangles.reserve(lineIndices.size());
	for(unsigned lineIndex: lineIndices)
	{
		Line& line = this->findLine(lineIndex);
		boundaryStaggeredTriangles.push_back( this->findStaggeredTriangle(line) );
	}
	return boundaryStaggeredTriangles;
}

Line& Grid2DWithStaggeredElements::findLine(unsigned lineIndex)
{
	for(Line& line: this->lines)
		if(line.getIndex()==lineIndex) return line;
	throw std::runtime_error(std::string(__FUNCTION__) + std::string(": not found line with index ") + std::to_string(lineIndex));
}

StaggeredTriangle* Grid2DWithStaggeredElements::findStaggeredTriangle(const Line& line)
{
	for(StaggeredTriangle& staggeredTriangle: this->staggeredTriangles)
		if((staggeredTriangle.vertices[0]==line.vertices[0] && staggeredTriangle.vertices[1]==line.vertices[1])
		   ||
		   (staggeredTriangle.vertices[1]==line.vertices[0] && staggeredTriangle.vertices[0]==line.vertices[1]))
		   { return &staggeredTriangle; }
	throw std::runtime_error(std::string(__FUNCTION__) + std::string(": not found StaggeredTriangle associated with line ") + std::to_string(line.getIndex()));
}