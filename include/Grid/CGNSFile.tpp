template <unsigned NumberOfVerticesPerElement, cgns::ElementType_t ElementType> std::vector< ElementDefinition<NumberOfVerticesPerElement> > CGNSFile::readElementsDefinition(void)
{
	this->readNumberOfSections();
	unsigned numberOfElements = this->getNumberOfElementsOfType(ElementType);
	std::vector< ElementDefinition<NumberOfVerticesPerElement> > element(numberOfElements);
	for(int sectionIndex=1 ; sectionIndex<=this->numberOfSections ; ++sectionIndex)
	{
		std::tuple<cgns::cgsize_t,cgns::cgsize_t,cgns::ElementType_t> sectionData = this->readSection(sectionIndex);
		if(std::get<2>(sectionData)==ElementType)
		{
			const cgns::cgsize_t firstElementIndex = std::get<0>(sectionData);
			const cgns::cgsize_t lastElementIndex = std::get<1>(sectionData);
			std::vector<cgns::cgsize_t> elementConnectivity = this->readElementConnectivity(sectionIndex);
			for(unsigned elementCount=0 ; elementCount<numberOfElements ; ++elementCount)
			{
				element[elementCount].index = firstElementIndex + elementCount;
				for(unsigned vertexIndex=0 ; vertexIndex<NumberOfVerticesPerElement ; ++vertexIndex)
					element[elementCount].connectivity(vertexIndex) = elementConnectivity[NumberOfVerticesPerElement*elementCount+vertexIndex] - 1;
			}
		}
	}
	return element;
}
