#ifndef ELEMENT_CONNECTIVITY_HPP
#define ELEMENT_CONNECTIVITY_HPP

#include <Eigen/Core>

template <unsigned NumberOfVertices>
struct ElementDefinition
{
	unsigned index;
	Eigen::Matrix<unsigned,NumberOfVertices,1> connectivity;
};

#endif
