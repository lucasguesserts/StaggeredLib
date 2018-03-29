#ifndef ELEMENT_CONNECTIVITY_HPP
#define ELEMENT_CONNECTIVITY_HPP

#include <Eigen/Core>

template <unsigned NumberOfVertices>
struct ElementDefinition
{
	unsigned index;
	Eigen::Matrix<unsigned,NumberOfVertices,1> connectivity;
};

template <unsigned NumberOfVertices>
bool operator==(const ElementDefinition<NumberOfVertices>& lhs, const ElementDefinition<NumberOfVertices>& rhs)
{
	return (lhs.index==rhs.index) && (lhs.connectivity==rhs.connectivity);
}

#endif
