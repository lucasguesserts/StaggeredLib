template <unsigned NumberOfVertices>
void Grid2DWithStaggeredElements::addStaggeredElementDefinitionFromElementDefinition(const ElementDefinition<NumberOfVertices>& elementDefinition)
{
    unsigned vertexLocalIndex;
    for(vertexLocalIndex=0 ; vertexLocalIndex<(NumberOfVertices-1) ; vertexLocalIndex++)
    {
        const unsigned firstVertexIndex = elementDefinition.connectivity[vertexLocalIndex];
        const unsigned secondVertexIndex = elementDefinition.connectivity[vertexLocalIndex+1];
        this->addStaggeredElementDefinition(StaggeredElementDefinition(firstVertexIndex,secondVertexIndex,elementDefinition.index));
    }
    vertexLocalIndex = NumberOfVertices - 1;
        const unsigned firstVertexIndex = elementDefinition.connectivity[vertexLocalIndex];
        const unsigned secondVertexIndex = elementDefinition.connectivity[0];
        this->addStaggeredElementDefinition(StaggeredElementDefinition(firstVertexIndex,secondVertexIndex,elementDefinition.index));
    return;
}