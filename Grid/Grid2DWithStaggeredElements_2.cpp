#include <Grid/Grid2DWithStaggeredElements_2.hpp>
#include <array>
#include <stdexcept>

Grid2DWithStaggeredElements_2::Grid2DWithStaggeredElements_2(const GridData& gridData)
	: Grid2DVerticesWithNeighborElements(gridData)
{
	this->createStaggeredElements();
	this->createFaces();
}

void Grid2DWithStaggeredElements_2::createStaggeredElements(void)
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
			StaggeredElement staggeredElement(staggeredElementIndex, *vertex_1, element, *vertex_0);
			std::tuple<bool,unsigned> location = this->findStaggeredElement(staggeredElement);
			if(std::get<bool>(location))
				this->staggeredElements[std::get<unsigned>(location)].elements.push_back(element);
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
			StaggeredElement staggeredElement(staggeredElementIndex, *vertex_1, element, *vertex_0);
			std::tuple<bool,unsigned> location = this->findStaggeredElement(staggeredElement);
			if(std::get<bool>(location))
				this->staggeredElements[std::get<unsigned>(location)].elements.push_back(element);
			else
			{
				this->staggeredElements.push_back(staggeredElement);
				++staggeredElementIndex;
			}
		}
	}
}

void Grid2DWithStaggeredElements_2::createFaces(void)
{
	unsigned faceIndex = 0;
	for(Element* element: this->elements)
	{
		unsigned localIndex;
		for(localIndex=0 ; localIndex<element->vertices.size() ; ++localIndex)
		{
			Vertex *adjacentVertex = element->vertices[localIndex];
			std::array<unsigned,2> staggredElementsLocation = this->findStaggeredElements(adjacentVertex, element);
			StaggeredElement* back = &(this->staggeredElements[staggredElementsLocation[0]]);
			StaggeredElement* front = &(this->staggeredElements[staggredElementsLocation[1]]);
			this->organizeStaggeredElements(element, adjacentVertex, back, front);
			this->faces.emplace_back( Face(faceIndex, localIndex, *element, *adjacentVertex, *back, *front) );
			++faceIndex;
		}
	}
}

std::array<unsigned,2> Grid2DWithStaggeredElements_2::findStaggeredElements(Vertex* adjacentVertex, Element* element)
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

void Grid2DWithStaggeredElements_2::organizeStaggeredElements(Element* element, Vertex* adjacentVertex, StaggeredElement*& back, StaggeredElement*& front)
{
	Eigen::Vector3d areaVector = *adjacentVertex - element->getCentroid();
		std::swap(areaVector[0],areaVector[1]);
		areaVector[1] = - areaVector[1];
	Eigen::Vector3d staggeredElementVector = front->getCentroid() - element->getCentroid();
	if(staggeredElementVector.dot(areaVector) < 0.0)
		std::swap(front,back);
	return;
}

std::tuple<bool,unsigned> Grid2DWithStaggeredElements_2::findStaggeredElement(const StaggeredElement& staggeredElement)
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

bool Grid2DWithStaggeredElements_2::staggeredElementsHaveTheSameVertices(const StaggeredElement& lhs, const StaggeredElement& rhs)
{
	return
		(lhs.vertices[0]==rhs.vertices[0] && lhs.vertices[1]==rhs.vertices[1])
		||
		(lhs.vertices[1]==rhs.vertices[0] && lhs.vertices[0]==rhs.vertices[1]);
}
