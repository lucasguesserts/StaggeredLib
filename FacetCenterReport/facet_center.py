import matplotlib.pyplot as plt
import numpy as np

# read csv file
facetCenter = '/home/guesser/git_projects/StaggeredLib/grids/facetCenter_convergence.csv'
elementCenter = '/home/guesser/git_projects/StaggeredLib/grids/elementCenter_convergence.csv'
fileFacet = open(facetCenter)
fileElement = open(elementCenter)
# one error
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

fig, ax = plt.subplots(1,2, figsize=(8,16))
# plt.subplots_adjust(left=0.175, right=0.98, top=0.96, bottom=0.135, wspace=0.36)

plt.rcParams['font.family'] = 'serif'
plt.rcParams['mathtext.fontset'] = 'dejavuserif'

titles = ['Facet Center', 'Element Center']
colors = ['k', 'r', 'g', 'b', 'c']
labels = ['cartesiana', 'cartesiana de triângulos', 'cartesiana mista', 'não estruturada triângulos', 'não estruturada quadriláteros']
for j in range(len(ax)):
	for i in range(5):
		ax[j].loglog(plotar[j][i][0], plotar[j][i][1], linestyle='-', marker='o', color=colors[i], linewidth='1.0', label=labels[i])
	ax[j].set_title(titles[j])
	ax[j].legend()
	ax[j].grid(True)
	ax[j].set_xlabel('Comprimento característico', size=14)
	ax[j].set_ylabel('Erro', size=14)
	plt.minorticks_on()
	ax[j].grid(which='major', linestyle=':')
	ax[j].grid(which='minor', linestyle=':')
	ax[j].set_xlim([1E-2, 1])
	ax[j].set_ylim([1E-4, 1E-1])
plt.show()
