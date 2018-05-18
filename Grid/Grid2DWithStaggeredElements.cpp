#include <Grid/Grid2DWithStaggeredElements.hpp>
#include <array>
#include <stdexcept>

Grid2DWithStaggeredElements::Grid2DWithStaggeredElements(const GridData_2& gridData)
	: Grid2DVerticesWithNeighborElements(gridData)
{
	this->createStaggeredElements();
	this->createFaces();
	this->setVerticesNeighborStaggeredElements();
	this->setStaggeredTrianglesAndQuadrangles();
	this->createBoundaries(gridData);
}

Grid2DWithStaggeredElements::Grid2DWithStaggeredElements(const std::string& fileName)
	: Grid2DVerticesWithNeighborElements(fileName)
{
	this->createStaggeredElements();
	this->createFaces();
	this->setVerticesNeighborStaggeredElements();
	this->setStaggeredTrianglesAndQuadrangles();
	this->createBoundaries();
}

void Grid2DWithStaggeredElements::createStaggeredElements(void)
{
	// TODO: separate in several functions
	// TODO: add a fucntion to correct the vertices order or show to yourself that this algorithm will never fail!
	unsigned staggeredElementIndex = 0;
	for(Element* element: this->elements)
	{
		unsigned vertexLocalIndex;
		unsigned numberOfVertices = element->vertices.size();
		for(vertexLocalIndex=0 ; vertexLocalIndex<numberOfVertices-1 ; ++vertexLocalIndex)
		{
			Vertex* vertex_0 = element->vertices[vertexLocalIndex];
			Vertex* vertex_1 = element->vertices[vertexLocalIndex+1];
			StaggeredElement2D staggeredElement(staggeredElementIndex, *vertex_1, element, *vertex_0);
			std::tuple<bool,unsigned> location = this->findStaggeredElement(staggeredElement);
			if(std::get<bool>(location))
				this->staggeredElements[std::get<unsigned>(location)].elements[1] = element;
			else
			{
				this->staggeredElements.push_back(staggeredElement);
				++staggeredElementIndex;
			}
		}
		vertexLocalIndex = numberOfVertices - 1;
		{
			Vertex* vertex_0 = element->vertices[vertexLocalIndex];
			Vertex* vertex_1 = element->vertices[0];
			StaggeredElement2D staggeredElement(staggeredElementIndex, *vertex_1, element, *vertex_0);
			std::tuple<bool,unsigned> location = this->findStaggeredElement(staggeredElement);
			if(std::get<bool>(location))
				this->staggeredElements[std::get<unsigned>(location)].elements[1] = element;
			else
			{
				this->staggeredElements.push_back(staggeredElement);
				++staggeredElementIndex;
			}
		}
	}
}

void Grid2DWithStaggeredElements::createFaces(void)
{
	unsigned faceIndex = 0;
	for(Element* element: this->elements)
	{
		unsigned localIndex;
		for(localIndex=0 ; localIndex<element->vertices.size() ; ++localIndex)
		{
			Vertex *adjacentVertex = element->vertices[localIndex];
			std::array<unsigned,2> staggredElementsLocation = this->findStaggeredElements(adjacentVertex, element);
			StaggeredElement2D* back = &(this->staggeredElements[staggredElementsLocation[0]]);
			StaggeredElement2D* front = &(this->staggeredElements[staggredElementsLocation[1]]);
			this->organizeStaggeredElementsForFace(element, adjacentVertex, back, front);
			this->faces.emplace_back( Face2D(faceIndex, localIndex, *element, *adjacentVertex, *back, *front) );
			++faceIndex;
		}
	}
}

std::array<unsigned,2> Grid2DWithStaggeredElements::findStaggeredElements(Vertex* adjacentVertex, Element* element)
{
	unsigned staggeredElementPosition, numberOfStaggeredElementsFound = 0;
	std::array<unsigned,2> positions;
	const unsigned numberOfStaggeredElements = this->staggeredElements.size();
	for(staggeredElementPosition=0 ; staggeredElementPosition<numberOfStaggeredElements ; ++staggeredElementPosition)
	{
		// get elements
		std::array<Element*,2> elements;
		elements[0] = this->staggeredElements[staggeredElementPosition].elements[0];
		if(this->staggeredElements[staggeredElementPosition].elements.size()==2)
			elements[1] = this->staggeredElements[staggeredElementPosition].elements[1];
		else
			elements[1] = nullptr;
		// get vertices
		std::array<Vertex*,2> vertices;
		vertices[0] = this->staggeredElements[staggeredElementPosition].vertices[0];
		vertices[1] = this->staggeredElements[staggeredElementPosition].vertices[1];
		// verify
		if(((elements[0]==element) || (elements[1]==element))
		    &&
		    ((vertices[0]==adjacentVertex) || (vertices[1]==adjacentVertex)))
		{
			positions[numberOfStaggeredElementsFound] = staggeredElementPosition;
			++numberOfStaggeredElementsFound;
		}
	}
	if(numberOfStaggeredElementsFound<2)
		throw std::runtime_error("Found less than 2 staggered elements for a face.");
	if(numberOfStaggeredElementsFound>2)
		throw std::runtime_error("Found more than 2 staggered elements for a face.");

	return positions;
}

void Grid2DWithStaggeredElements::organizeStaggeredElementsForFace(Element* element, Vertex* adjacentVertex, StaggeredElement2D*& back, StaggeredElement2D*& front)
{
	Eigen::Vector3d areaVector = *adjacentVertex - element->getCentroid();
		std::swap(areaVector[0],areaVector[1]);
		areaVector[1] = - areaVector[1];
	Eigen::Vector3d staggeredElementVector = front->getCentroid() - element->getCentroid();
	if(staggeredElementVector.dot(areaVector) < 0.0)
		std::swap(front,back);
	return;
}

std::tuple<bool,unsigned> Grid2DWithStaggeredElements::findStaggeredElement(const StaggeredElement2D& staggeredElement)
{
	bool elementExists;
	unsigned staggeredElementPosition;
	const unsigned numberOfStaggeredElements = this->staggeredElements.size();
	for(staggeredElementPosition=0 ; staggeredElementPosition<numberOfStaggeredElements ; ++staggeredElementPosition)
		if( this->staggeredElementsHaveTheSameVertices(staggeredElement,this->staggeredElements[staggeredElementPosition]) ) break;
	if(staggeredElementPosition==numberOfStaggeredElements)
		elementExists = false;
	else
		elementExists = true;
	return std::make_tuple(elementExists,staggeredElementPosition);
}

bool Grid2DWithStaggeredElements::staggeredElementsHaveTheSameVertices(const StaggeredElement2D& lhs, const StaggeredElement2D& rhs)
{
	return
		(lhs.vertices[0]==rhs.vertices[0] && lhs.vertices[1]==rhs.vertices[1])
		||
		(lhs.vertices[1]==rhs.vertices[0] && lhs.vertices[0]==rhs.vertices[1]);
}

void Grid2DWithStaggeredElements::setStaggeredTrianglesAndQuadrangles(void)
{
	for(StaggeredElement2D& staggeredElement: this->staggeredElements)
	{
		if(staggeredElement.elements[1]==nullptr)
			this->staggeredTriangles.push_back(&staggeredElement);
		else
			this->staggeredQuadrangles.push_back(&staggeredElement);
	}
	return;
}

void Grid2DWithStaggeredElements::createBoundaries(void)
{
	for(auto& boundary: this->cgnsReader->gridData->boundaries)
	{
		if(boundary.facetsOnBoundary.empty())
			throw std::runtime_error(std::string(__FUNCTION__) +
			                        std::string(": facets on boundary '") +
									boundary.name +
									std::string("' empty. Boundaries cannot be built with vertices."));
		this->boundary[boundary.name].staggeredTriangle = this->findStaggeredTrianglesInBoundary(boundary);
	}
	return;
}

std::vector<StaggeredElement2D*> Grid2DWithStaggeredElements::findStaggeredTrianglesInBoundary(const BoundaryData& boundary)
{
	std::vector<StaggeredElement2D*> boundaryStaggeredTriangles;
	boundaryStaggeredTriangles.reserve(boundary.facetsOnBoundary.size());
	for(auto lineIndex: boundary.facetsOnBoundary)
	{
		Line& line = this->findLine(static_cast<unsigned>(lineIndex));
		boundaryStaggeredTriangles.push_back( this->findStaggeredTriangle(line) );
	}
	return boundaryStaggeredTriangles;
}

void Grid2DWithStaggeredElements::createBoundaries(const GridData_2& gridData)
{
	for(const BoundaryDefinition& boundaryDefinition: gridData.boundary)
		this->boundary[boundaryDefinition.name].staggeredTriangle = this->findStaggeredTrianglesInBoundaryDefinition(boundaryDefinition);
	return;
}

std::vector<StaggeredElement2D*> Grid2DWithStaggeredElements::findStaggeredTrianglesInBoundaryDefinition(const BoundaryDefinition& boundaryDefinition)
{
	const std::vector<unsigned>& lineIndices = boundaryDefinition.elementsIndexList;
	std::vector<StaggeredElement2D*> boundaryStaggeredTriangles;
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

StaggeredElement2D* Grid2DWithStaggeredElements::findStaggeredTriangle(const Line& line)
{
	for(StaggeredElement2D* staggeredTriangle: this->staggeredTriangles)
		if((staggeredTriangle->vertices[0]==line.vertices[0] && staggeredTriangle->vertices[1]==line.vertices[1])
		   ||
		   (staggeredTriangle->vertices[1]==line.vertices[0] && staggeredTriangle->vertices[0]==line.vertices[1]))
		   { return staggeredTriangle; }
	throw std::runtime_error(std::string(__FUNCTION__) + std::string(": not found StaggeredElement2D associated with line ") + std::to_string(line.getIndex()));
}

void Grid2DWithStaggeredElements::setVerticesNeighborStaggeredElements(void)
{
	const unsigned numberOfVertices = this->vertices.size();
	this->verticesNeighborStaggeredElements.resize(numberOfVertices);
	for(auto& staggeredElement: this->staggeredElements)
		for(auto vertex: staggeredElement.vertices)
			this->verticesNeighborStaggeredElements[vertex->getIndex()].push_back(&staggeredElement);
	return;
}