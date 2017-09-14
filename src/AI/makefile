BUILDDIR = build/build

$(BUILDDIR)/run : main.o AI.o World.o State.o Robot.o Drone.o Plank.o
	g++ -Wall main.o World.o AI.o State.o Robot.o Drone.o Plank.o -o run

$(BUILDDIR)/main.o : main.cpp AI.cpp State.cpp Robot.cpp Drone.cpp Plank.cpp SimSim.cpp
	g++ -Wall -c -std=c++11 main.cpp

$(BUILDDIR)/World.o :  World.cpp
	g++ -Wall -c -std=c++11 World.cpp

$(BUILDDIR)/AI.o : AI.cpp State.cpp Robot.cpp Drone.cpp Plank.cpp
	g++ -Wall -c -std=c++11 AI.cpp

$(BUILDDIR)/State.o : State.cpp Robot.cpp Drone.cpp Plank.cpp
	g++ -Wall -c -std=c++11 State.cpp

$(BUILDDIR)/Robot.o : Robot.cpp Plank.cpp
	g++ -Wall -c -std=c++11 Robot.cpp

$(BUILDDIR)/Plank.o : Plank.cpp
	g++ -Wall -c -std=c++11 Plank.cpp

$(BUILDDIR)/Drone.o : Drone.cpp Robot.cpp	
	g++ -Wall -c -std=c++11 Drone.cpp

clean:
	\rm *.o *~ run