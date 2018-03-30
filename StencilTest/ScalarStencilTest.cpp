#include <Utils/Test.hpp>

#include <Stencil/ScalarStencil.hpp>

TestCase("Scalar stencil", "[ScalarStencil]")
{
	section("operator +")
	{
		section("Equal entries")
		{
			const unsigned index = 5;
			const double scalar[2] = {3.4, -2.3};
			ScalarStencil first = { {index,scalar[0]} };
			ScalarStencil second = { {index,scalar[1]} };
			ScalarStencil sum = first + second;
			check(sum[index]==(scalar[0]+scalar[1]));
		}
		section("Different entries")
		{
			const unsigned index[2] = {1, 8};
			const double scalar[2] = {9.9, -4.7};
			ScalarStencil first = { {index[0],scalar[0]} };
			ScalarStencil second = { {index[1],scalar[1]} };
			ScalarStencil sum = first + second;
			check(sum[index[0]]==scalar[0]);
			check(sum[index[1]]==scalar[1]);
		}
		section("Mixed")
		{
			const unsigned index[3] = {1, 8, 19};
			const double scalar[3] = {9.9, -4.7, 6.3};
			ScalarStencil first = { {index[0],scalar[0]} , {index[1],scalar[1]} };
			ScalarStencil second = { {index[1],scalar[1]}, {index[2],scalar[2]} };
			ScalarStencil sum = first + second;
			check(sum[index[0]]==scalar[0]);
			check(sum[index[1]]==(scalar[1]+scalar[1]));
			check(sum[index[2]]==scalar[2]);
		}
	}
	section("operator*(double scalar, ScalarStencil scalarMap)")
	{
		const unsigned index = 2;
		const double scalarMapEntry = 6.7;
		const double scalarMultiplier = 2.8;
		ScalarStencil scalarMap = { {index,scalarMapEntry} };
		ScalarStencil product = scalarMultiplier * scalarMap;
		check(product[index]==(scalarMultiplier*scalarMapEntry));
	}
	section("+ and * mixed")
	{
		const unsigned index[3] = {2, 7, 4};
		const double value[4] = {5.8, -9.5, -3.2, 6.1};
		ScalarStencil first = {{index[0], value[0]}, {index[1], value[1]}};
		ScalarStencil second = {{index[2], value[2]}, {index[1], value[3]}};
		const double scalar = 0.4;
		ScalarStencil result = scalar*first + second;
		check(result[2]==(scalar*value[0]));
		check(result[4]==(value[2]));
		check(result[7]==(scalar*value[1]+value[3]));
	}
}

TestCase("ScalarStencil and scalar field used to interpolate a value", "[ScalarStencil]")
{
	const unsigned numberOfValues = 8;
	Eigen::VectorXd scalarField;
	scalarField.resize(numberOfValues);
	scalarField << 8.1, 2.6, 6.7, 4.5, 1.8, 7.9, 1.3, 2.6;
	section("interpolation with complete ScalarStencil")
	{
		const unsigned scalarStencilSize = 8;
		const std::array<unsigned,scalarStencilSize> key = {0, 1, 2, 3, 4, 5, 6, 7};
		const std::array<double,scalarStencilSize> value = {0.2, 0.4, 0.3, 0.8, 0.1, 0.7, 0.4, 0.2};
		ScalarStencil scalarStencil;
		for(unsigned i=0 ; i<scalarStencilSize ; ++i)
			scalarStencil[key[i]] = value[i];
		double correctInterpolatedValue = 0.0;
		for(unsigned i=0 ; i<scalarStencilSize ; ++i)
			correctInterpolatedValue += scalarStencil[key[i]] * scalarField[key[i]];
		check(correctInterpolatedValue==(scalarStencil*scalarField));
	}
	section("interpolation with partial ScalarStencil")
	{
		const unsigned scalarStencilSize = 4;
		const std::array<unsigned,scalarStencilSize> key = {0, 2, 3, 5};
		const std::array<double,scalarStencilSize> value = {0.2, 0.3, 0.8, 0.7};
		ScalarStencil scalarStencil;
		for(unsigned i=0 ; i<scalarStencilSize ; ++i)
			scalarStencil[key[i]] = value[i];
		double correctInterpolatedValue = 0.0;
		for(unsigned i=0 ; i<scalarStencilSize ; ++i)
			correctInterpolatedValue += scalarStencil[key[i]] * scalarField[key[i]];
		check(correctInterpolatedValue==(scalarStencil*scalarField));
	}
}