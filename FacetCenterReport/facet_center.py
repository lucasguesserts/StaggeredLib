import matplotlib.pyplot as plt
import numpy as np

# read csv file
fileName = '/home/guesser/git_projects/StaggeredLib/grids/facetCenter_convergence.csv'
file = open(fileName)
# one error
def readLine():
	characteristicLengthLine = file.readline().split(',')
	characteristicLength = np.zeros(len(characteristicLengthLine))
	errorLine = file.readline().split(',')
	error = np.zeros(len(errorLine))
	for i in range(len(characteristicLengthLine)):
		characteristicLength[i] = float(characteristicLengthLine[i])
		error[i] = float(errorLine[i])
	return [characteristicLength, error]

plotar = []
for i in range(5):
	plotar.append(readLine())

fig, ax = plt.subplots(1,1, figsize=(8,8))
plt.subplots_adjust(left=0.175, right=0.98, top=0.96, bottom=0.135, wspace=0.36)

plt.rcParams['font.family'] = 'serif'
plt.rcParams['mathtext.fontset'] = 'dejavuserif'

# markerList = [r'$\circ$', r'$\times$', r'$\diamondsuit$'] # procure por mathtext para mais markers

cartesian_length = [ 0.5, 0.25, 0.111111, 0.0526316]
cartesian_error = [0.0184619, 0.0117836, 0.00299693, 0.000706617]
ax.loglog(plotar[1][0], plotar[1][1], linestyle='-', marker='o', color='k', linewidth='1.0' )

ax.grid(True)
ax.set_xlabel('Comprimento característico', size=14)
ax.set_ylabel('Erro', size=14)
plt.minorticks_on()
ax.grid(which='major', linestyle=':')
ax.grid(which='minor', linestyle=':')
ax.set_xlim([1E-2, 1])
ax.set_ylim([1E-4, 1E-1])
plt.show()
