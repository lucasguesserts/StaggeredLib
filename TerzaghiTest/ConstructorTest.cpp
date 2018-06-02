#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <Stencil/Test.hpp>

#include <Terzaghi/Terzaghi.hpp>

TestCase("Terzaghi indices transformation", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	section("simple tests")
	{
		check(terzaghi.transformIndex(Component::P, 5)==5);
		check(terzaghi.transformIndex(Component::U, 2)==4);
		check(terzaghi.transformIndex(Component::V, 3)==10);
		check(terzaghi.transformIndex(Component::W, 1)==13);
	}
	section("for loop")
	{
		constexpr unsigned index = 2;
		std::vector<unsigned> indices = {4, 9, 14};
		unsigned i = 0;
		for(Component component: Terzaghi::displacementComponents)
		{
			check(terzaghi.transformIndex(component,index)==indices[i]);
			++i;
		}
	}
	section("entities test")
	{
		Element* element = terzaghi.grid.elements[1];
		check(terzaghi.transformIndex(Component::P, element)==1);
		StaggeredElement2D* staggeredElement = terzaghi.grid.staggeredQuadrangles[0];
		check(terzaghi.transformIndex(Component::V, staggeredElement)==9);
	}
}

TestCase("Pressure gradient on staggered triangles", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	section("staggered triangle 0")
	{
		auto staggeredTriangle = terzaghi.grid.staggeredTriangles[0];
		auto terzaghiVectorStencil = terzaghi.pressureGradient[staggeredTriangle->getIndex()];
		auto vectorStencil = VectorStencil{ {0, {0.6, 1.2, 0.0}} };
		check(terzaghiVectorStencil==vectorStencil);
	}
	section("staggered triangle 1")
	{
		auto staggeredTriangle = terzaghi.grid.staggeredTriangles[1];
		auto terzaghiVectorStencil = terzaghi.pressureGradient[staggeredTriangle->getIndex()];
		auto vectorStencil = VectorStencil{ {0, {-1.2, -0.6, 0.0}} };
		check(terzaghiVectorStencil==vectorStencil);
	}
	section("staggered triangle 3")
	{
		auto staggeredTriangle = terzaghi.grid.staggeredTriangles[2];
		auto terzaghiVectorStencil = terzaghi.pressureGradient[staggeredTriangle->getIndex()];
		auto vectorStencil = VectorStencil{ {1, {-0.6, -1.2, 0.0}} };
		check(terzaghiVectorStencil==vectorStencil);
	}
	section("staggered triangle 4")
	{
		auto staggeredTriangle = terzaghi.grid.staggeredTriangles[3];
		auto terzaghiVectorStencil = terzaghi.pressureGradient[staggeredTriangle->getIndex()];
		auto vectorStencil = VectorStencil{ {1, {1.2, 0.6, 0.0}} };
		check(terzaghiVectorStencil==vectorStencil);
	}
}

TestCase("Pressure accumulation term")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.porosity = 0.1;
	terzaghi.alpha = 0.9;
	terzaghi.fluidCompressibility = 3;
	terzaghi.solidCompressibility = 4;
	constexpr double volume = 2;
	constexpr double diagonalValue = 7.0;
	section("matrix")
	{
		terzaghi.insertPressureAccumulationTermInMatrix();
		terzaghi.linearSystem.assemblyMatrix();
		for(auto element: terzaghi.grid.elements)
		{
			const unsigned index = terzaghi.transformIndex(Component::P,element);
			check(terzaghi.linearSystem.matrix.coeff(index,index)==diagonalValue);
		}
	}
	section("independent")
	{
		constexpr double oldPressureValue = 2.0;
		terzaghi.setOldPressure([oldPressureValue](Eigen::Vector3d) -> double {return oldPressureValue;});
		terzaghi.insertPressureAccumulationTermInIndependent();
		for(auto element: terzaghi.grid.elements)
		{
			const unsigned index = terzaghi.transformIndex(Component::P,element);
			check(terzaghi.linearSystem.independent[index]==(diagonalValue*oldPressureValue));
		}
	}
}

TestCase("Pressure diffusive term", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	const unsigned linearSystemSize = terzaghi.numberOfElements + 3 * terzaghi.numberOfStaggeredElements;
	terzaghi.permeability = 20.0;
	terzaghi.fluidViscosity = 4.0;
	terzaghi.timeImplicitCoefficient = 0.7;
	terzaghi.timeInterval = 1.1;
	section("matrix")
	{
		Eigen::SparseMatrix<double> matrix(linearSystemSize,linearSystemSize);
		std::vector< Eigen::Triplet<double,unsigned> > triplets = {
			{ 0, 0,  11.55 },
			{ 0, 1, -11.55 },
			{ 1, 0, -11.55 },
			{ 1, 1,  11.55 }
		};
		matrix.setFromTriplets(triplets.begin(), triplets.end());
		terzaghi.insertPressureDiffusiveTermInMatrix();
		terzaghi.linearSystem.assemblyMatrix();
		check(terzaghi.linearSystem.matrix==matrix);
	}
	section("independent")
	{
		std::vector<double> oldPressureValues = {13.0, 17.0};
		terzaghi.setOldPressure(oldPressureValues);
		terzaghi.insertPressureDiffusiveTermInIndependent();
		std::vector<double> independentValues = {19.8, -19.8};
		for(unsigned count=0 ; count<terzaghi.numberOfElements ; ++count)
		{
			auto element = terzaghi.grid.elements[count];
			auto independentIndex = terzaghi.transformIndex(Component::P,element);
			check(terzaghi.linearSystem.independent[independentIndex]==Approx(independentValues[count]));
		}
	}
}

TestCase("Volumetric dilatation in mass equation", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.alpha = 1.0;
	section("matrix")
	{
		std::vector<Eigen::Triplet<double,unsigned>> triplets = {
			{0,  2,  0.0},
			{0,  7, -2.0},
			{0, 12,  0.0},
			{0,  3,  2.0},
			{0,  8,  0.0},
			{0, 13,  0.0},
			{0,  4, -2.0},
			{0,  9,  2.0},
			{0, 14,  0.0},
			{1,  4,  2.0},
			{1,  9, -2.0},
			{1, 14,  0.0},
			{1,  5,  0.0},
			{1, 10,  2.0},
			{1, 15,  0.0},
			{1,  6, -2.0},
			{1,  1,  0.0},
			{1, 16,  0.0},
		};
		Eigen::SparseMatrix<double> matrix(terzaghi.linearSystemSize,terzaghi.linearSystemSize);
		matrix.setFromTriplets(triplets.cbegin(), triplets.cend());
		terzaghi.insertPressureVolumeDilatationTermInMatrix();
		terzaghi.linearSystem.assemblyMatrix();
		check(terzaghi.linearSystem.matrix==matrix);
	}
	section("independent")
	{
		std::vector<Eigen::Vector3d> displacements = {
			{ 1.0,  3.0,  5.0},
			{ 7.0, 11.0, 13.0},
			{17.0, 19.0, 23.0},
			{29.0, 31.0, 37.0},
			{41.0, 43.0, 47.0},
		};
		terzaghi.setOldDisplacement(displacements);
		terzaghi.insertPressureVolumeDilatationTermInIndependent();
		std::vector<double> independentValues = {12.0, -24.0};
		for(unsigned count=0 ; count<terzaghi.numberOfElements ; ++count)
		{
			auto element = terzaghi.grid.elements[count];
			auto independentIndex = terzaghi.transformIndex(Component::P,element);
			check(terzaghi.linearSystem.independent[independentIndex]==Approx(independentValues[count]));
		}
	}
}

TestCase("Displacement scalar stencil on elements", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	std::vector<ScalarStencil> displacementScalarStencilOnElements = {
		{{0, 1.0/3.0}, {1, 1.0/3.0}, {2, 1.0/3.0}},
		{{2, 1.0/3.0}, {3, 1.0/3.0}, {4, 1.0/3.0}}
	};
	check(terzaghi.displacementScalarStencilOnElements==displacementScalarStencilOnElements);
}

TestCase("Displacement scalar stencil on vertices", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	std::vector<double> aux = {0.36939806252, 0.26120387496, 1.0/2.0};
	std::vector<ScalarStencil> displacementScalarStencilOnVertices = {
		{{0, aux[0]}, {2, aux[1]}, {4, aux[0]}},
		{{0, aux[2]}, {1, aux[2]}},
		{{3, aux[2]}, {4, aux[2]}},
		{{1, aux[0]}, {2, aux[1]}, {3, aux[0]}},
	};
	check(terzaghi.displacementScalarStencilOnVertices==displacementScalarStencilOnVertices);
}

TestCase("Displacement gradient on faces", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	std::vector<double> aux = {0.36939806252, 0.26120387496, 1.0/2.0, 1.0/3.0};
	std::vector<double> epsilon = {20.0/9.0, 2.0, 8.0/9.0};
	std::vector<Eigen::Vector3d> faceVector = {
		{ 4.0/3.0,  2.0/3.0, 0.0},
		{ 2.0/3.0, -2.0/3.0, 0.0},
		{ 2.0/3.0,  4.0/3.0, 0.0},
		{ 2.0/3.0,  4.0/3.0, 0.0},
		{ 4.0/3.0,  2.0/3.0, 0.0},
		{-2.0/3.0,  2.0/3.0, 0.0}
	};
	VectorStencil displacementGradient_face_0 = {
		{0, Eigen::Vector3d(0.0,-1.0,0.0) + ((aux[3]-aux[0])/epsilon[0]) * faceVector[0]},
		{1, (aux[3]/epsilon[0]) * faceVector[0]},
		{2, Eigen::Vector3d(0.0,1.0,0.0) + ((aux[3]-aux[1])/epsilon[0]) * faceVector[0]},
		{4, (-aux[0]/epsilon[0]) * faceVector[0]}
	};
	VectorStencil displacementGradient_face_1 = {
		{0, (-1.0/epsilon[1]) * Eigen::Vector3d(1.0,1.0,0.0) + ((aux[2]-aux[3])/epsilon[2]) * faceVector[1]},
		{1, (1.0/epsilon[1]) * Eigen::Vector3d(1.0,1.0,0.0) + ((aux[2]-aux[3])/epsilon[2]) * faceVector[1]},
		{2, (-aux[3]/epsilon[2]) * faceVector[1]}
	};
	VectorStencil displacementGradient_face_2 = {
		{0, (-aux[3]/epsilon[0]) * faceVector[2]},
		{1, Eigen::Vector3d(1.0,0.0,0.0) + ((aux[0]-aux[3])/epsilon[0]) * faceVector[2]},
		{2, Eigen::Vector3d(-1.0,0.0,0.0) + ((aux[1]-aux[3])/epsilon[0]) * faceVector[2]},
		{3, (aux[0]/epsilon[0]) * faceVector[2]}
	};
	VectorStencil displacementGradient_face_3 = {
		{0, (-aux[0]/epsilon[0]) * faceVector[3]},
		{2, Eigen::Vector3d(1.0,0.0,0.0) + ((aux[3]-aux[1])/epsilon[0]) * faceVector[3]},
		{3, (aux[3]/epsilon[0]) * faceVector[3]},
		{4, Eigen::Vector3d(-1.0,0.0,0.0) + ((aux[3]-aux[0])/epsilon[0]) * faceVector[3]}
	};
	VectorStencil displacementGradient_face_4 = {
		{1, (aux[0]/epsilon[0]) * faceVector[4]},
		{2, Eigen::Vector3d(0.0,-1.0,0.0) + ((aux[1]-aux[3])/epsilon[0]) * faceVector[4]},
		{3, Eigen::Vector3d(0.0,1.0,0.0) + ((aux[0]-aux[3])/epsilon[0]) * faceVector[4]},
		{4, (-aux[3]/epsilon[0]) * faceVector[4]}
	};
	VectorStencil displacementGradient_face_5 = {
		{2, (-aux[3]/epsilon[2]) * faceVector[5]},
		{3, (1.0/epsilon[1]) * Eigen::Vector3d(1.0,1.0,0.0) + ((aux[2]-aux[3])/epsilon[2]) * faceVector[5]},
		{4, (-1.0/epsilon[1]) * Eigen::Vector3d(1.0,1.0,0.0) + ((aux[2]-aux[3])/epsilon[2]) * faceVector[5]}
	};
	std::vector<VectorStencil> displacementGradient = {
		displacementGradient_face_0,
		displacementGradient_face_1,
		displacementGradient_face_2,
		displacementGradient_face_3,
		displacementGradient_face_4,
		displacementGradient_face_5
	};
	check(terzaghi.displacementGradient==displacementGradient);
}

TestCase("Terzaghi mechanical properties matrix", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	constexpr unsigned matrixSize = 3;
	terzaghi.shearModulus = 1;
	terzaghi.poissonCoefficient = 0.25;
	auto testMechanicalPropertiesMatrix = [&terzaghi](Component c0, Component c1, double me00, double me11, double me22) -> void
	{
		Eigen::MatrixXd mechanicalPropertiesMatrix(matrixSize,matrixSize);
		mechanicalPropertiesMatrix << me00, 0.0,  0.0,
                                      0.0,  me11, 0.0,
                                      0.0,  0.0,  me22;
		check(terzaghi.getMechanicalPropertiesMatrix(c0,c1)==mechanicalPropertiesMatrix);
	};
	section("u")
	{
		testMechanicalPropertiesMatrix(Component::U, Component::U, 3.0, 1.0, 1.0);
		testMechanicalPropertiesMatrix(Component::U, Component::V, 1.0, 1.0, 0.0);
		testMechanicalPropertiesMatrix(Component::U, Component::W, 1.0, 0.0, 1.0);
	}
	section("v")
	{
		testMechanicalPropertiesMatrix(Component::V, Component::U, 1.0, 1.0, 0.0);
		testMechanicalPropertiesMatrix(Component::V, Component::V, 1.0, 3.0, 1.0);
		testMechanicalPropertiesMatrix(Component::V, Component::W, 0.0, 1.0, 1.0);
	}
	section("w")
	{
		testMechanicalPropertiesMatrix(Component::W, Component::U, 1.0, 0.0, 1.0);
		testMechanicalPropertiesMatrix(Component::W, Component::V, 0.0, 1.0, 1.0);
		testMechanicalPropertiesMatrix(Component::W, Component::W, 1.0, 1.0, 3.0);
	}
}

TestCase("Terzaghi permutation matrix", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	constexpr unsigned matrixSize = 3;
	auto testPermutationMatrix = [&terzaghi](Component c0, Component c1, unsigned column_0, unsigned column_1, unsigned column_2) -> void
	{
		Eigen::MatrixXd permutationMatrix = Eigen::MatrixXd::Zero(matrixSize,matrixSize);
		permutationMatrix(0,column_0) = 1.0;
		permutationMatrix(1,column_1) = 1.0;
		permutationMatrix(2,column_2) = 1.0;
		check(terzaghi.getPermutationMatrix(c0,c1)==permutationMatrix);
	};
	section("u")
	{
		testPermutationMatrix(Component::U, Component::U, 0, 1, 2);
		testPermutationMatrix(Component::U, Component::V, 1, 0, 2);
		testPermutationMatrix(Component::U, Component::W, 2, 1, 0);
	}
	section("v")
	{
		testPermutationMatrix(Component::V, Component::U, 1, 0, 2);
		testPermutationMatrix(Component::V, Component::V, 0, 1, 2);
		testPermutationMatrix(Component::V, Component::W, 0, 2, 1);
	}
	section("w")
	{
		testPermutationMatrix(Component::W, Component::U, 2, 1, 0);
		testPermutationMatrix(Component::W, Component::V, 0, 2, 1);
		testPermutationMatrix(Component::W, Component::W, 0, 1, 2);
	}
}

TestCase("Terzaghi insert displacement tension term in matrix", "[Terzaghi]")
// It is difficult to calculate by hand this example.
// No check done.
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.insertDisplacementTensionTermInMatrix();
	terzaghi.linearSystem.assemblyMatrix();
	// std::cout << std::endl << "Linear system matrix" << std:: endl << terzaghi.linearSystem.matrix << std::endl;
}

TestCase("Terzaghi insert displacement pressure gradient term in matrix ")
// No check done.
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.alpha = 1.0;
	terzaghi.insertDisplacementPressureTermInMatrix();
	terzaghi.linearSystem.assemblyMatrix();
	// std::cout << std::endl << "Linear system matrix" << std:: endl << terzaghi.linearSystem.matrix << std::endl;
}