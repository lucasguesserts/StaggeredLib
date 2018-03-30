#include <Grid/Grid2DWithStaggeredElements.hpp>
#include <exception>

Grid2DWithStaggeredElements::Grid2DWithStaggeredElements(const GridData& gridData)
	: Grid2DVerticesWithNeighborElements(gridData)
{
	this->allocateStaggeredElementDefinition(gridData);
	this->createStaggeredElementDefinitionVector(gridData);
	this->shrinkStaggeredElementDefinition();
	this->createStaggeredElements();
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