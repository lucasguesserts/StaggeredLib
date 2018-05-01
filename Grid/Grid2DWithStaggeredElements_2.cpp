#include <Grid/Grid2DWithStaggeredElements_2.hpp>

Grid2DWithStaggeredElements_2::Grid2DWithStaggeredElements_2(const GridData& gridData)
	: Grid2DVerticesWithNeighborElements(gridData)
{
	this->createStaggeredElements();
}

void Grid2DWithStaggeredElements_2::createStaggeredElements(void)
{
	unsigned faceIndex = 0;
	unsigned staggeredElementIndex = 0;
	for(Element* element: this->elements)
		this->addStaggeredEntitiesFromElement(element, staggeredElementIndex, faceIndex);
	return;
}

void Grid2DWithStaggeredElements_2::addStaggeredEntitiesFromElement(Element* element, unsigned& staggeredElementIndex, unsigned& faceIndex)
{
	const unsigned numberOfVertices = element->vertices.size();
	unsigned vertexLocalIndex;
	StaggeredElement *firstStaggeredElement, *backwardStaggeredElement, *forwardStaggeredElement;
	vertexLocalIndex = 0;
		Vertex* firstVertex = element->vertices[vertexLocalIndex];
		Vertex* secondVertex = element->vertices[vertexLocalIndex+1];
		firstStaggeredElement = this->addStaggeredElement(secondVertex, element, firstVertex, staggeredElementIndex);
		forwardStaggeredElement = firstStaggeredElement;
	for(vertexLocalIndex=1 ; vertexLocalIndex<(numberOfVertices-1) ; vertexLocalIndex++)
	{
		Vertex* firstVertex = element->vertices[vertexLocalIndex];
		Vertex* secondVertex = element->vertices[vertexLocalIndex+1];
		backwardStaggeredElement = this->addStaggeredElement(secondVertex, element, firstVertex, staggeredElementIndex);
		this->faces.emplace_back( Face(faceIndex, vertexLocalIndex, *element, *firstVertex, *backwardStaggeredElement, *forwardStaggeredElement) );
		++faceIndex;
		forwardStaggeredElement = backwardStaggeredElement;
	}
	vertexLocalIndex = numberOfVertices - 1;
		firstVertex = element->vertices[vertexLocalIndex];
		secondVertex = element->vertices[0];
		backwardStaggeredElement = this->addStaggeredElement(secondVertex, element, firstVertex, staggeredElementIndex);
		this->faces.emplace_back( Face(faceIndex, vertexLocalIndex, *element, *firstVertex, *backwardStaggeredElement, *forwardStaggeredElement) );
		++faceIndex;
		// first face
		forwardStaggeredElement = backwardStaggeredElement;
		backwardStaggeredElement = firstStaggeredElement;
		this->faces.emplace_back( Face(faceIndex, 0, *element, *secondVertex, *backwardStaggeredElement, *forwardStaggeredElement) );
		++faceIndex;
	return;
}

StaggeredElement* Grid2DWithStaggeredElements_2::addStaggeredElement(Vertex* firstVertex, Element* element, Vertex* secondVertex, unsigned& staggeredElementIndex)
{
	StaggeredElement staggeredElement(staggeredElementIndex, *firstVertex, element, *secondVertex);
	StaggeredElement *thisStaggeredElement;
	std::tuple<bool,unsigned> staggeredElementLocation = this->findStaggeredElement(staggeredElement);
	if( std::get<bool>(staggeredElementLocation)==true )
	{
		this->staggeredElements[std::get<unsigned>(staggeredElementLocation)].elements.push_back(element);
		thisStaggeredElement = &( this->staggeredElements[std::get<unsigned>(staggeredElementLocation)] );
	}
	else
	{
		this->staggeredElements.push_back(staggeredElement);
		++staggeredElementIndex;
		thisStaggeredElement = &( this->staggeredElements.back() );
	}
	return thisStaggeredElement;
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