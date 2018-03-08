template <unsigned NumberOfVerticesPerElement, cgns::ElementType_t ElementType>
std::vector< ElementDefinition<NumberOfVerticesPerElement> > CGNSFile::readElementsDefinition(void)
{
	unsigned numberOfElements = this->getNumberOfElementsOfType(ElementType);
	std::vector< ElementDefinition<NumberOfVerticesPerElement> > element;
	element.reserve(numberOfElements);
	this->readNumberOfSections();
	for(int sectionIndex=1 ; sectionIndex<=this->numberOfSections ; ++sectionIndex)
	{
		this->setElementsDefinitionUsingSection<NumberOfVerticesPerElement,ElementType>(element,sectionIndex);
	}
	return element;
}

template <unsigned NumberOfVerticesPerElement, cgns::ElementType_t ElementType>
void CGNSFile::setElementsDefinitionUsingSection(std::vector< ElementDefinition<NumberOfVerticesPerElement> >& element, const int sectionIndex)
{
	std::tuple<cgns::cgsize_t,cgns::cgsize_t,cgns::ElementType_t> sectionData = this->readSection(sectionIndex);
		const cgns::cgsize_t firstElementIndex = std::get<0>(sectionData);
		const cgns::cgsize_t lastElementIndex = std::get<1>(sectionData);
		const cgns::ElementType_t elementType = std::get<2>(sectionData);
	if(elementType==ElementType)
	{
		const unsigned numberOfElementsInSection = lastElementIndex - firstElementIndex + 1;
		std::vector<cgns::cgsize_t> elementConnectivity = this->readElementConnectivity(sectionIndex);
		for(unsigned elementCount=0 ; elementCount<numberOfElementsInSection ; ++elementCount)
		{
			ElementDefinition<NumberOfVerticesPerElement> elementDefinition = this->getElementDefinitionFromElementConnectivity<NumberOfVerticesPerElement>(elementConnectivity,firstElementIndex,elementCount);
			element.push_back(elementDefinition);
		}
	}
}

template <unsigned NumberOfVerticesPerElement>
ElementDefinition<NumberOfVerticesPerElement> CGNSFile::getElementDefinitionFromElementConnectivity(const std::vector<cgns::cgsize_t>& elementConnectivity,cgns::cgsize_t firstElementIndex, unsigned elementCount)
{
	ElementDefinition<NumberOfVerticesPerElement> elementDefinition;
	elementDefinition.index = firstElementIndex + elementCount;
	for(unsigned vertexIndex=0 ; vertexIndex<NumberOfVerticesPerElement ; ++vertexIndex)
	{
		const unsigned vertexIndexPosition = NumberOfVerticesPerElement * elementCount + vertexIndex;
		elementDefinition.connectivity(vertexIndex) = elementConnectivity[vertexIndexPosition] - 1;
	}
	return elementDefinition;
}
