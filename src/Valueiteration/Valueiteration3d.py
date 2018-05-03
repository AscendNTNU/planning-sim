import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from math import sin, cos, pi


# Angles are 0 degrees when to the right

from datetime import datetime
print(datetime.now().strftime('%Y-%m-%d %H:%M:%S'))

np.set_printoptions(linewidth = 180)


v_robot = 1/3 #robot velocity
t_plank = 18.5 #time taken for a full plank


planksize = t_plank*v_robot
disc_thetas = [i*pi/6 for i in range(12)] #from 0 to 11pi/6 (aka 12 discrete angles for robots, i + Pi/6)
gamma = 0.9

def oldValuegrid(): #performs valuiteration, shows grid
	numiter = 10000
	valuegrid = createGrid2d()
	print()

	print("loading...")
	for k in range(numiter):
		print("Iteration " + str(k+1) + " out of " + str(numiter), end="\r")
		for i in range(0,len(valuegrid)-2): #x
			for j in range(0,len(valuegrid[i])-2): #y
				valuegrid[i+1, j+1] = (valuegrid[i,j+1] + valuegrid[i+1, j] + valuegrid[i+1, j+2] + valuegrid[i+2, j+1])/4		
	
	showGrid(valuegrid)
	return valuegrid

def valuegridWithObst():
	numiter = 1000
	valuegrid = createGrid2d()
	valuegrid = placeObstacleRing(valuegrid)
	showGrid(valuegrid)
	# printGrid(valuegrid)

	print()

	print("loading...")
	for k in range(numiter):
		print("Iteration " + str(k+1) + " out of " + str(numiter), end="\r")
		for i in range(0,len(valuegrid)-2): #x
			for j in range(0,len(valuegrid[i])-2): #y

				if k == 997:
					valuegrid = placeObstacleRing(valuegrid)

				valuegrid[i+1, j+1] = (valuegrid[i,j+1] + valuegrid[i+1, j] + valuegrid[i+1, j+2] + valuegrid[i+2, j+1])/4		
	
	#printGrid(valuegrid)
	showGrid(valuegrid)
	return valuegrid


def printGrid(grid): #prints 2d grid
	for row in grid[1:21]:
		print(row[1:21])
		print()

def printAngle(theta_i):
	angle_radians = (2*theta_i*pi) / 12
	angle_degrees = (angle_radians*180) / pi
	print("---- Angle ----")
	print("Radians: ", round(angle_radians,2))
	print("Degrees: ", round(angle_degrees,2))
	print("---------------")

def placeObstacleRing(valuegrid): #places values in a circle with sentrum in the middle of the grid
	r = 5 # radius of circle from the middle of grid
	iterations = 25 # should likely be set to 30
	for i in range(iterations):
		x = 5*cos(i*2*pi / iterations) + 11
		y = 5*sin(i*2*pi / iterations) + 11
		valuegrid[int(y)][int(x)] = -1500
	return valuegrid

def createGrid3d(): # initializing 3d grid (x,y, theta)
	grid = np.zeros((22,22,12))
	grid[:,0,:] = -1000
	grid[:,len(grid)-1,:] = -1000
	grid[len(grid)-1,:,:] = -1000
	grid[0,:,:] = 2000
	return grid

def createGrid2d(): # initializing 2d grid (x,y)
	grid = np.zeros((22,22))
	grid[:,0] = -1000
	grid[:,len(grid)-1] = -1000
	grid[len(grid)-1,:] = -1000
	grid[0,:] = 2000
	return grid


def showGrid(grid, zmin = -1000, zmax = 2000): # prints out 3d grid
	fig = plt.figure()
	ax = fig.gca(projection='3d')

	gsize = len(grid)
	X = np.arange(1, gsize -1)
	Y = np.arange(1,gsize -1)
	X, Y = np.meshgrid(X, Y)
	Z = grid[1:(gsize-1), 1:(gsize-1)]

	surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)
	ax.set_zlim(zmin, zmax)
	plt.gca().invert_yaxis()
	plt.show()


def posAfterPlank(y, x, theta):
	#asumes robot is at start of plank.
	#parameter: robot position and angle
	#returns robot position after 18.5sec (at end of plank)
	leny = sin(theta)*planksize
	lenx = cos(theta)*planksize

	next_y = int(y - leny)
	next_x = int(x + lenx)
	#print('next_x:', next_x, 'next_y:', next_y)

	if next_x < 1 or next_x > 20 or next_y > 20: # out red  
		return -1000 , -1000, theta
	elif next_y < 1: # out green
		return 2000, 2000, theta
	else:
		return next_y , next_x, theta


# Possible actions for a robot
def doNothing(y, x, theta):
	return posAfterPlank(y, x, theta)

def landOnTop(y, x, theta):
	return posAfterPlank(y, x, theta - pi/4)

def landInFront(y, x, theta):
	return posAfterPlank(y, x, theta + pi)


def imediateReward(next_y, next_x):
	if (1 <= next_x <= 20) and (1 <= next_y <= 20):
		return 0
	elif next_x == -1000:
		return -1000
	elif next_x == 2000:
		return 2000


def nextStateReward(next_y, next_x, t): # t is a theta index
	if (1 <= next_x <= 20) and (1 <= next_y <= 20):
		return valuegrid[next_y][next_x][t]
	elif next_x == -1000:
		return 0
	elif next_x == 2000:
		return 0

def indexClosestTheta(actual_theta): # creates discrete theta values
	thetamod = actual_theta % 2*pi
	closest = min(disc_thetas, key=lambda x:abs(x-thetamod))
	index = disc_thetas.index(closest)
	return index


def writeToFile(grid, filename): # will print actiongrid and valuegrid
	myfile = open(filename, "w")
	# array_str = np.array2string(grid)
	# myfile.write(array_str)
	myfile.write(" { ")
	for y in range(1, len(grid) - 1):
		myfile.write(" ( ")
		for x in range(1, len(grid) - 1):
			myfile.write(" [ ")
			for t in range(12):
				myfile.write(str(grid[x][y][t]))
				myfile.write(" , ")
			myfile.write(" ] ")
		myfile.write(" ) ")
	myfile.write(" } ")

	print("Done writing to file:", filename)
	myfile.close()


def valueiteration(): # valueiteration for robots and drone actions (valueiteration for (x,y,theta))
	numiter = 1000
	for i in range(numiter): # number of iterations
		print("Iteration " + str(i+1) + " out of " + str(numiter), end="\r")
		for y in range(1, len(valuegrid) - 1):
			for x in range(1, len(valuegrid) - 1):
				for t in range(0,12): # robot angles
					theta = t*pi/6
					y0, x0, theta1 = doNothing(y, x, theta) # DONOTHING == 1
					y1, x1, theta2 = landOnTop(y, x, theta) # LANDONTOP == 2
					y2, x2, theta3 = landInFront(y, x, theta)

					t0 = indexClosestTheta(theta1)
					t1 = indexClosestTheta(theta2)
					t2 = indexClosestTheta(theta3)


					value0 = imediateReward(y0, x0) + gamma*nextStateReward(y0, x0, t0) # expected value for action 0 (doNothing)
					value1 = imediateReward(y1, x1) + gamma*nextStateReward(y1, x1, t1) # expected value for action 1 (landOnTop)
					value2 = imediateReward(y2, x2) + gamma*nextStateReward(y2, x2, t2) # expected value for action 2 (landInFront)


					maxvalue = max(value0, value1, value2)
					valuegrid[y][x][t] = maxvalue # current x,y, theta will be set to the max future value

					if i == (numiter - 1):
						if maxvalue == value0:
							actiongrid[y][x][t] = 1 #donothing (shows gray)
						elif maxvalue == value1:
							actiongrid[y][x][t] = 2 #landontop 45degrees (shows blue in jupyter)
						elif maxvalue == value2:
							actiongrid[y][x][t] = 3 #landinfront 180degrees (shows red in jupyter)

	print()
	print()
	print('grid complete!')


# 	   -- MAIN -- 
actiongrid = createGrid3d()
valuegrid = createGrid3d()

valueiteration()

writeToFile(valuegrid, "Valuegrid.txt")
writeToFile(actiongrid, "Actiongrid.txt")


# 	   ----------

# 3d (4d) grid can be shown using showGrid(valuegrid[:,:,2])




#valueiteration()
#showGrid(valuegrid[:,:,0])



#TESTSS
#oldValuegrid() #checks if old valuegrid works

#checks if plank lenght is consistent with angles from 0 - 2pi
'''
for i in range(1, 41):
	x1, y1 = doNothing(10,10,(i*pi)/20)
	posgrid[y1, x1] = 1
printGrid()
'''

#test for landOnTop / LandInFront
'''
posgrid[10][10] = 1
x0, y0 = doNothing(10, 10, pi)
posgrid[y0][x0] = 2
x1, y1 = landOnTop(10, 10, pi) #look at LandInFront as well
posgrid[y1][x1] = 3
printGrid()
'''

#test for robot going out of field
'''
posgrid[10][10] = 1
posgrid[1][20] = 1
x, y = doNothing(1, 20, 0)
posgrid[x][y] = 2
'''
