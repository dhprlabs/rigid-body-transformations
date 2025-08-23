#include "RigidBodyTransforms.hpp"
#include <iostream>

int main() 
{
    using namespace RigidBodyTransforms;

    Transform2D t2d;

    size_t bodies;
    std::cout << "Enter number of bodies (including reference frame): ";
    std::cin >> bodies;

    t2d.setBodies(bodies);

    std::cout << "\n--- Enter reference frame (Frame 0) ---\n";
    float x0, y0, theta0;
    std::cout << "Translation X Y: ";
    std::cin >> x0 >> y0;
    std::cout << "Rotation (radians): ";
    std::cin >> theta0;
    t2d.addTranslation(x0, y0);
    t2d.addRotation(theta0);

    // Loop for other bodies
    for (size_t i = 1; i < bodies; ++i) 
    {
        std::cout << "\n--- Enter transform from Frame " << (i - 1) << " to Frame " << i << " ---\n";
        float x, y, theta;
        std::cout << "Translation X Y: ";
        std::cin >> x >> y;
        std::cout << "Rotation (radians): ";
        std::cin >> theta;
        t2d.addTranslation(x, y);
        t2d.addRotation(theta);
    }

    // Form and calculate
    t2d.formTransformMatrix();
    std::cout << "\nResultant transformation from Frame 0 to Frame " << (bodies - 1) << ":\n";
    t2d.printResultantMatrix();

    return 0;
}