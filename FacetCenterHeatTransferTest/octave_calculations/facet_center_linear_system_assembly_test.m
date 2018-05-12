clear
clc
format long

# auxiliar values
a = 0.36939806252/2.0;
b = 0.26120387496/2.0;
g = 0.5/2.0;

# Gradients on triangles
gradientInTriangle0 = [
      g-a g  -b 0 -a ;
      -g  a-g b a 0  ;
      0   0   0 0 0  ];
gradientInTriangle1 = [
      0  a b  a-g -g  ;
      -a 0 -b g   g-a ;
      0  0 0  0   0   ];

# faces area vector
face0_areaVector = [-2/3 4/3 0];
face1_areaVector = [-2/3 -2/3 0];
face2_areaVector = [4/3 -2/3 0];
face3_areaVector = [-4/3 2/3 0];
face4_areaVector = [2/3 -4/3 0];
face5_areaVector = [2/3 2/3 0];

# faces in each triangle
facesInTriangle0 = [face0_areaVector ; face1_areaVector ; face2_areaVector];
facesInTriangle1 = [face3_areaVector ; face4_areaVector ; face5_areaVector];

# scalar stencil on faces
scalarStencilInTriangle0 = facesInTriangle0 * gradientInTriangle0;
scalarStencilInTriangle1 = facesInTriangle1 * gradientInTriangle1;

scalarStencilOnFace0 = scalarStencilInTriangle0(1,:);
scalarStencilOnFace1 = scalarStencilInTriangle0(2,:);
scalarStencilOnFace2 = scalarStencilInTriangle0(3,:);
scalarStencilOnFace3 = scalarStencilInTriangle1(1,:);
scalarStencilOnFace4 = scalarStencilInTriangle1(2,:);
scalarStencilOnFace5 = scalarStencilInTriangle1(3,:);

# linear system matrix
linearSystemMatrix = [
	 +scalarStencilOnFace0 - scalarStencilOnFace1 ;
	 +scalarStencilOnFace1 - scalarStencilOnFace2 ;
	 -scalarStencilOnFace0 + scalarStencilOnFace2 + scalarStencilOnFace3 - scalarStencilOnFace4;
	 +scalarStencilOnFace4 - scalarStencilOnFace5 ;
	 -scalarStencilOnFace3 + scalarStencilOnFace5 ];

linearSystemMatrix