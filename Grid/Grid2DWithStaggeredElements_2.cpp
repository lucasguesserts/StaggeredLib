#include <Grid/Grid2DWithStaggeredElements_2.hpp>

Grid2DWithStaggeredElements_2::Grid2DWithStaggeredElements_2(const GridData& gridData)
	: Grid2DVerticesWithNeighborElements(gridData)
{
	this->createStaggeredElements();
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