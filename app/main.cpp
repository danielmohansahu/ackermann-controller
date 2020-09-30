#include <iostream>
#include <Vehicle.hpp>
#include <AckermannVehicle.hpp>
#include <PIDController.hpp>

using namespace VehicleControl;
using namespace ControlAlgo;

// define global variable dt for timestamp, seconds
const double global_dt = 0.1;

int main()
{
    //
    AckermannVehicle Rover;
    Rover.set_wheelbase(1.);
    double tempbase = Rover.get_wheelbase();
    std::cout << tempbase << std::endl;
    return 0;
}
