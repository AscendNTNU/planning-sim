import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from math import sin, cos, pi

np.set_printoptions(linewidth = 180)

planksize = 20/3
gamma = 0.9

def createGrid():
	grid = np.zeros((22,22,12))
	grid[:,0,:] = -1000
	grid[:,len(grid)-1,:] = -1000
	grid[len(grid)-1,:,:] = -1000
	grid[0,:,:] = 2000
	return grid

def oldValuegrid():
	valuegrid = createGrid()
	printGrid(valuegrid)
	print()

	for i in range(10000):
		for i in range(0,len(valuegrid)-2):
			for j in range(0,len(valuegrid[i])-2):
				valuegrid[i+1, j+1] = (valuegrid[i,j+1] + valuegrid[i+1, j] + valuegrid[i+1, j+2] + valuegrid[i+2, j+1])/4		
	
	printGrid(valuegrid)
	showGrid(valuegrid)
	return valuegrid

def printGrid(grid):
	for row in grid[1:21]:
		print(row[1:21])
		print()

def showGrid(grid):
	fig = plt.figure()
	ax = fig.gca(projection='3d')

	gsize = len(grid)
	X = np.arange(1, gsize -1)
	Y = np.arange(1,gsize -1)
	X, Y = np.meshgrid(X, Y)
	Z = grid[1:(gsize-1), 1:(gsize-1)]

	surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.bwr, linewidth=0, antialiased=False)
	ax.set_zlim(-1000, 2000)
	plt.show()

def posAfterPlank(y, x, theta):
	#asumes robot is at start of plank.
	#parameter: robot position and angle
	#returns robot position after 20sec (at end of plank)
	leny = sin(theta)*planksize
	lenx = cos(theta)*planksize

	newy = int(y - leny)
	newx = int(x + lenx)
	print('newx:', newx, 'newy:', newy)

	if newx < 1 or newx > 20 or newy > 20:
		print("RED! :(")
		return -1000 , -1000
	elif newy < 1:
		print("GREEN! :D")
		return 2000, 2000
	else:
		return newy , newx


def doNothing(y, x, theta):
	return posAfterPlank(y, x, theta)

def landOnTop(y, x, theta):
	return posAfterPlank(y, x, theta + pi/4)

def landInFront(y, x, theta):
	return posAfterPlank(y, x, theta + pi)


def imediateReward(newy, newx):
	if (1 <= newx <= 20) and (1 <= newy <= 20):
		return 0
	elif newx == -1000:
		return -1000
	elif newx == 2000:
		return 2000


def nextStateReward(newy, newx):
	if (1 <= newx <= 20) and (1 <= newy <= 20):
		return valuegrid[newy][newx]
	elif newx == -1000:
		return 0
	elif newx == 2000:
		return 0

def newVauegrid():
	#Do this many times, for 20*12 states, iterating over grid 10000 times, return grid
	for i in range(1000):
		print(i)
		for x in range(1, len(valuegrid) - 1):
			for y in range(1, len(valuegrid) - 1):
				for theta in range(0,2*pi, pi/6):
					y1, x1 = doNothing(y, x, theta)
					y2, x2 = landOnTop(y, x, theta)
					y3, x3 = landInFront(y, x, theta)

					# imediate reward with y,x instead of y1 and stuff??
					value1 = imediateReward(y1, x1) + gamma*nextStateReward(y1, x1)
					value2 = imediateReward(y2, x2) + gamma*nextStateReward(y2, x2)
					value3 = imediateReward(y3, x3) + gamma*nextStateReward(y3, x3)

					valuegrid[y][x][theta] = max(value1, value2, value3)
	print('done')


posgrid = createGrid() #ground robot position grid
valuegrid = createGrid() #valuegrid
newVauegrid()




#TESTS
#oldValuegrid() #checks if old valuegrid works

#checks if plank lenght is consistent with angles from 0 - 2pi
'''
for i in range(1, 41):
	x2, y2 = doNothing(10,10,(i*pi)/20)
	posgrid[y2, x2] = 1
printGrid()
'''

#test for landOnTop / LandInFront
'''
posgrid[10][10] = 1
x1, y1 = doNothing(10, 10, pi)
posgrid[y1][x1] = 2
x2, y2 = landOnTop(10, 10, pi) #look at LandInFront as well
posgrid[y2][x2] = 3
printGrid()
'''

#test for robot going out of field
'''
posgrid[10][10] = 1
posgrid[1][20] = 1
x, y = doNothing(1, 20, 0)
posgrid[x][y] = 2
'''