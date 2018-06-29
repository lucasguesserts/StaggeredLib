import matplotlib.pyplot as plt
import numpy as np

# read csv file
facetCenter = '/home/guesser/git_projects/StaggeredLib/grids/facetCenter_convergence.csv'
elementCenter = '/home/guesser/git_projects/StaggeredLib/grids/elementCenter_convergence.csv'
fileFacet = open(facetCenter)
fileElement = open(elementCenter)

def readLine(file):
	characteristicLengthLine = file.readline().split(',')
	characteristicLength = np.zeros(len(characteristicLengthLine))
	errorLine = file.readline().split(',')
	error = np.zeros(len(errorLine))
	for i in range(len(characteristicLengthLine)):
		characteristicLength[i] = float(characteristicLengthLine[i])
		error[i] = float(errorLine[i])
	return [characteristicLength, error]

facetPlot = []
elementPlot = []
for i in range(5):
	facetPlot.append(readLine(fileFacet))
	elementPlot.append(readLine(fileElement))
plotar = [facetPlot, elementPlot]


# triangles
def computeA( y, x, b ):
    a = y / ( x ** b )
    return a

def computeY( x, a, slope ):
    y = a * ( x ** slope )
    return y

def getUpperTri( x1, x2, yb, slope ):
    a = computeA( yb, x2, slope )
    X, Y = [], []
    X.append( [x1, x1] )
    X.append( [x1, x2] )
    X.append( [x2, x1] )
    y1 = computeY( x1, a, slope )
    y2 = yb
    Y.append( [y1, y2] )
    Y.append( [y2, y2] )
    Y.append( [y2, y1] )
    return X, Y

def getLowerTri( x1, x2, yb, slope ):
    a = computeA( yb, x1, slope )
    X, Y = [], []
    X.append( [x1, x2] )
    X.append( [x2, x2] )
    X.append( [x2, x1] )
    y1 = yb
    y2 = computeY( x2, a, slope )
    Y.append( [y1, y1] )
    Y.append( [y1, y2] )
    Y.append( [y2, y1] )
    return X, Y

# plot

fig, ax = plt.subplots(2,1, figsize=(6,12))

plt.rcParams['font.family'] = 'serif'
plt.rcParams['mathtext.fontset'] = 'dejavuserif'

titles = ['Facet Center', 'Element Center']
colors = ['k', 'r', 'g', 'b', 'c']
labels = ['cartesiana', 'cartesiana de triângulos', 'cartesiana mista', 'não estruturada triângulos', 'não estruturada quadriláteros']
for j in range(len(ax)):
	for i in range(5):
		ax[j].loglog(plotar[j][i][0], plotar[j][i][1], linestyle='-', marker='o', color=colors[i], linewidth='1.0', label=labels[i])
	ax[j].set_title(titles[j])
	ax[j].grid(True)
	ax[j].set_xlabel('Comprimento característico', size=14)
	ax[j].set_ylabel('Erro', size=14)
	plt.minorticks_on()
	ax[j].grid(which='major', linestyle=':')
	ax[j].grid(which='minor', linestyle=':')
	ax[j].set_xlim([1E-2, 1])
	ax[j].set_ylim([1E-4, 1E-1])
ax[0].legend(loc='upper left')
ax[1].legend(loc='lower right')


Xf, Yf = getUpperTri( 3E-2, 8E-2, 9E-3, 1.0 )
for x, y in zip( Xf, Yf ):
    ax[0].loglog(x, y, '-', color='grey')
Xf, Yf = getLowerTri( 8E-2, 3E-1, 2E-4, 2.0 )
for x, y in zip( Xf, Yf ):
    ax[0].loglog(x, y, '-', color='grey')

Xf, Yf = getUpperTri( 6E-2, 2E-1, 5E-2, 1.0 )
for x, y in zip( Xf, Yf ):
    ax[1].loglog(x, y, '-', color='grey')
Xf, Yf = getLowerTri( 1E-1, 3E-1, 1E-3, 2.0 )
for x, y in zip( Xf, Yf ):
    ax[1].loglog(x, y, '-', color='grey')
# plt.show()
fig.savefig('/home/guesser/git_projects/tcc_guesser/images/report_heat_transfer.svg')