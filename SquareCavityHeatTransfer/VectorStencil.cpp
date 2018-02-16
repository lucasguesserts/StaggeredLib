#include <SquareCavityHeatTransfer/VectorStencil.hpp>

VectorStencil operator*(const ScalarStencil& scalarStencil, const Eigen::Vector3d& vector)
{
	VectorStencil vectorStencil;
	for(auto& indexScalarPair: scalarStencil)
		vectorStencil[indexScalarPair.first] = indexScalarPair.second * vector;
	return vectorStencil;
}

VectorStencil operator+(const VectorStencil& lhs, const VectorStencil& rhs)
{
	VectorStencil sum = rhs;
	for(auto& indexVectorPair: lhs)
	{
		if( sum.find(indexVectorPair.first) != sum.end() )
			sum[indexVectorPair.first] += indexVectorPair.second;
		else
			sum[indexVectorPair.first] = indexVectorPair.second;
	}
	return sum;
}

ScalarStencil operator*(const Eigen::Vector3d& vector, const VectorStencil& vectorStencil)
{
	ScalarStencil scalarStencil;
	for(auto& indexVectorPair: vectorStencil)
		scalarStencil[indexVectorPair.first] = vector.dot(indexVectorPair.second);
	return scalarStencil;
}

VectorStencil operator*(const double scalar, const VectorStencil& vectorStencil)
{
	VectorStencil resultVectorStencil = vectorStencil;
	for(auto& indexVectorPair: resultVectorStencil)
		indexVectorPair.second *= scalar;
	return resultVectorStencil;
}
