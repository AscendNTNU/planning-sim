#include "World.h"

//Constructors
/*
    Contructor for the grid world.

    @param orientation - float value describing the orientation of the field.
*/
World::World(float orientation){
	this->origin = point_Zero;
	this->orientation = orientation;
	this->bounds = (bounds_t){.x_Max = 20, .y_Max = 20};
    readGrids("/catkin_ws/src/planning-sim/src/Valueiteration/Valuegrid.txt", "/catkin_ws/src/planning-sim/src/Valueiteration/Actiongrid.txt"); // this will update valuegrid3d
}

//Get
point_t World::getOrigin(){
	return this->origin;
}
float World::getOrientation(){
	return this->orientation;
}
bounds_t World::getBounds(){
	return this->bounds;
}

void World::readGrids(std::string value_file, std::string action_file) {
    std::ifstream valuefile(value_file); // declears and opens file
    std::ifstream actionfile(action_file);
    std::string state_value;
    std::string action;

    int x = 0;
    int y = 0;
    int t = 0;

    if (valuefile.fail()) { // will be thrown if filename doesnt exsist
        std::cout << "Opening file " << value_file << " failed" << std::endl;
        throw "Failed to open file\n";
    }

    while(valuefile >> state_value && actionfile >> action) {
        //std::cout << state_value << std::endl;
        if (state_value == "}") {
            x = 0;
        }
        if (state_value == ")") {
            x++;
            y = 0;
        }
        else if (state_value == "]") {
            y++;
            t = 0;
        }
        else if (state_value == ",") {
            t++;
        }

        else if (state_value.size() > 1) {
            std::cout << "Placing value " << state_value;
            std::cout << " at (x,y,t) = (" << x << "," << y << "," << t << ")" << std::endl;
            std::cout << "Where the best action is " << action << std::endl;
            valuegrid3d[x][y][t] = std::stod(state_value);
            actiongrid3d[x][y][t] = std::stod(action);
        }
    }

    actionfile.close();
    valuefile.close();

        // printing out grid
/*    for (int x = 0; x < 20; x++) {
        for (int y = 0; y < 20; y++) {
            for (int t = 0; t < 12; t++) {
                std::cout << actiongrid3d[x][y][t] << std::endl;
            }
        }
    }*/
}

double World::getGridValue(int X, int Y, double T) { // T is the angle given in radians
    return valuegrid3d[X][Y][static_cast<int>(T)];
}

action_Type_t World::getAction(int X, int Y, double T) {
    int action = static_cast<int>(actiongrid3d[X][Y][static_cast<int>(T)]); // will output values where 1:landontop, 2:donothing, 3:landinfront
    switch(action) {
        case 1:
            return land_on_top_of;
        case 2:
            return land_in_front_of;
        case 3:
            return no_command;
    }
}


/*
    2D function of grid value. Determined using value iteration.

    @param x - The x position on the field
    @param y - The y position on the field
    @return the points value at the (x, y) position given
*/
float World::getGridValue(float X, float Y){
    float value = (-9.995004e+02)+(9.976812e+01)*X+(-1.004701e+02)*Y
        +(-5.785388e+01)*pow(X,2)+(1.161562e+01)*X*Y+(5.477725e+01)*pow(Y,2)
        +(1.260229e+01)*pow(X,3)+(1.299816e+01)*pow(X,2)*Y+(-1.438667e+01)*X*pow(Y,2)+(-1.158062e+01)*pow(Y,3)
        +(-1.404096e+00)*pow(X,4)+(-3.106303e+00)*pow(X,3)*Y+(4.263504e-01)*pow(X,2)*pow(Y,2)
        +(2.851553e+00)*X*pow(Y,3)+(1.301842e+00)*pow(Y,4)
        +(9.053408e-02)*pow(X,5)+(2.901147e-01)*pow(X,4)*Y+(1.327346e-01)*pow(X,3)*pow(Y,2)
        +(-1.761180e-01)*pow(X,2)*pow(Y,3)+(-2.603853e-01)*X*pow(Y,4)+(-8.415694e-02)*pow(Y,5)
        +(-3.615309e-03)*pow(X,6)+(-1.235169e-02)*pow(X,5)*Y+(-1.602868e-02)*pow(X,4)*pow(Y,2)
        +(3.840976e-03)*pow(X,3)*pow(Y,3)+(1.239923e-02)*pow(X,2)*pow(Y,4)
        +(1.283802e-02)*X*pow(Y,5)+(3.201336e-03)*pow(Y,6)
        +(8.890888e-05)*pow(X,7)+(1.960570e-04)*pow(X,6)*Y+(7.353331e-04)*pow(X,5)*pow(Y,2)
        +(-9.145182e-05)*pow(X,4)*pow(Y,3)+(8.794847e-10)*pow(X,3)*pow(Y,4)
        +(-6.113303e-04)*pow(X,2)*pow(Y,5)+(-2.451141e-04)*X*pow(Y,6)+(-7.627948e-05)*pow(Y,7)
        +(-1.058445e-06)*pow(X,8)+(4.059809e-11)*pow(X,7)*Y+(-1.167195e-05)*pow(X,6)*pow(Y,2)
        +(-4.630460e-12)*pow(X,5)*pow(Y,3)+(-1.355465e-11)*pow(X,4)*pow(Y,4)
        +(-5.731993e-12)*pow(X,3)*pow(Y,5)+(1.167198e-05)*pow(X,2)*pow(Y,6)
        +(3.539047e-11)*X*pow(Y,7)+(1.058675e-06)*pow(Y,8);

    return value;
}
