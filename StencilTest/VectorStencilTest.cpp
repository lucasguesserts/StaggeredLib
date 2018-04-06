#include <Utils/Test.hpp>
#include <Stencil/Test.hpp>
#include <array>

#include <Stencil/VectorStencil.hpp>

TestCase("VectorStencil operator==", "[VectorStencil]")
{
	std::array<VectorStencil,7> VectorStencils = {{
		{ {0,{2.7, 5.9, 1.3}}, {7,{4.2,8.7,9.6}} },
		{ {0,{2.7, 5.9, 1.3}}, {7,{4.2,8.7,9.6}} },
		{ {0,{2.7, 5.9, 1.3}}, {7,{4.2,8.7,9.600001}} },
		{ {9,{2.7, 5.9, 1.3}}, {7,{4.2,8.7,9.6}} },
		{ {0,{0.1, 5.9, 1.3}}, {7,{4.2,8.7,9.6}} },
		{ {0,{2.7, 5.9, 1.3}}, {7,{4.2,8.7,9.6}}, {4,{2.2,8.4,2.9}} },
		{ {0,{2.7, 5.9, 1.3}} }
	}};
	section("true tests")
	{
		check(VectorStencils[0]==VectorStencils[1]);
		check(VectorStencils[0]==VectorStencils[2]);
	}
	section("false tests")
	{
		checkFalse(VectorStencils[0]==VectorStencils[3]);
		checkFalse(VectorStencils[0]==VectorStencils[4]);
		checkFalse(VectorStencils[0]==VectorStencils[5]);
		checkFalse(VectorStencils[0]==VectorStencils[6]);
	}
}

TestCase("Vector stencil", "[VectorStencil]")
{
	section("ScalarStencil * Eigen::Vector3d")
	{
		const double value[2] = {7.4, -8.3};
		const unsigned index[2] = {2, 9};
		ScalarStencil scalarStencil = { {index[0],value[0]}, {index[1],value[1]} };
		const Eigen::Vector3d vector(-6.3, 1.2, 4.5);
		VectorStencil result = scalarStencil*vector;
		check(result[index[0]]==static_cast<Eigen::Vector3d>(value[0]*vector));
		check(result[index[1]]==static_cast<Eigen::Vector3d>(value[1]*vector));
	}
	section("VectorStencil + VectorStencil")
	{
		const Eigen::Vector3d vector[4] = {{2.7,-1.8,4.3}, {-12.9,0.8,2.2}, {18.7,3.9,8.74}, {7.9,-8.3,-0.9}};
		const unsigned index[3] = {3, 27, 5};
		VectorStencil first = {{index[0],vector[0]}, {index[1],vector[1]}};
		VectorStencil second = {{index[1],vector[2]}, {index[2],vector[3]}};
		VectorStencil result = first + second;
		check(result[index[0]]==vector[0]);
		check(result[index[1]]==static_cast<Eigen::Vector3d>(vector[1]+vector[2]));
		check(result[index[2]]==vector[3]);
	}
	section("Eigen::Vector3d * VectorStencil")
	{
		const Eigen::Vector3d vector(-6.3, 1.2, 4.5);
		const Eigen::Vector3d vec[2] = {{2.7,-1.8,4.3}, {-12.9,0.8,2.2}};
		const unsigned index[2] = {3, 27};
		VectorStencil vectorStencil = {{index[0],vec[0]}, {index[1],vec[1]}};
		ScalarStencil result = vector * vectorStencil;
		check(result[index[0]]==vector.dot(vec[0]));
		check(result[index[1]]==vector.dot(vec[1]));
	}
	section("double * VectorStencil")
	{
		const double scalar = 2.23;
		const unsigned index = 554;
		const Eigen::Vector3d vector = {-1.6, 8.3, 2.0};
		const VectorStencil vectorStencil = {{index,vector}};
		VectorStencil result = scalar * vectorStencil;
		check(result[index]==static_cast<Eigen::Vector3d>(scalar*vector));
	}
}

TestCase("VectorStencil and scalar field used to reconstruct calculate a vectorial quantity", "[VectorStencil]")
{
	const unsigned numberOfValues = 5;
	Eigen::VectorXd scalarField;
	scalarField.resize(numberOfValues);
	scalarField << 8.1, 2.6, 6.7, 4.5, 1.8;
	section("reconstruction with complete ScalarStencil")
	{
		const unsigned vectorStencilSize = 5;
		const std::array<unsigned,vectorStencilSize> key = {0, 1, 2, 3, 4};
		std::array<Eigen::Vector3d,vectorStencilSize> value;
			value[0] << 5.2, 8.2, 6.0;
			value[1] << 8.9, 7.5, 9.9;
			value[2] << 3.4, 5.9, 1.6;
			value[3] << 5.9, 4.4, 3.5;
			value[4] << 2.8, 6.2, 1.3;
		VectorStencil vectorStencil;
		for(unsigned i=0 ; i<vectorStencilSize ; ++i)
			vectorStencil[key[i]] = value[i];
		Eigen::Vector3d reconstructedValue = Eigen::Vector3d::Zero();
		for(unsigned i=0 ; i<vectorStencilSize ; ++i)
			reconstructedValue += scalarField[key[i]] * vectorStencil[key[i]];
		check(reconstructedValue==(vectorStencil*scalarField));
	}
	section("reconstruction with partial ScalarStencil")
	{
		const unsigned vectorStencilSize = 3;
		const std::array<unsigned,vectorStencilSize> key = {0, 1, 4};
		std::array<Eigen::Vector3d,vectorStencilSize> value;
			value[0] << 5.2, 8.2, 6.0;
			value[1] << 8.9, 7.5, 9.9;
			value[2] << 3.4, 5.9, 1.6;
		VectorStencil vectorStencil;
		for(unsigned i=0 ; i<vectorStencilSize ; ++i)
			vectorStencil[key[i]] = value[i];
		Eigen::Vector3d reconstructedValue = Eigen::Vector3d::Zero();
		for(unsigned i=0 ; i<vectorStencilSize ; ++i)
			reconstructedValue += scalarField[key[i]] * vectorStencil[key[i]];
		check(reconstructedValue==(vectorStencil*scalarField));
	}
}