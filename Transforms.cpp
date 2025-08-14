#include "RigidBodyTransforms.hpp"

int main() 
{
    RigidBodyTransforms::Transform2D t2d;

    t2d.setBodies(2);       
    t2d.formTranslationMatrix(); 
    t2d.formRotationMatrix();
    t2d.formTransformMatrix();
    t2d.calculateResultMatrix();

    t2d.printResultantMatrix();

    return 0;
}