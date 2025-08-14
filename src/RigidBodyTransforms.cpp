#include "RigidBodyTransforms.hpp"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

RigidBodyTransforms::Transform2D::Transform2D() : bodies(0)
{
    resultantMatrix = Eigen::Matrix3f::Identity();
}

void RigidBodyTransforms::Transform2D::setBodies(int count) 
{
    bodies = count;
}

void RigidBodyTransforms::Transform2D::formRotationMatrix()
{
    for (int i = 0; i < bodies - 1; i++)
    {
        Eigen::Matrix2f r;
        float theta;

        std::cout << "Enter theta(0) for " << i + 1 << ": ";
        std::cin >> theta;

        r(0, 0) = std::round(cos(theta));
        r(0, 1) = std::round(-sin(theta));
        r(1, 0) = std::round(sin(theta));
        r(1, 1) = std::round(cos(theta));
        
        rotationMatrix.push_back(r);
    }
}

void RigidBodyTransforms::Transform2D::formTranslationMatrix()
{
    for (int i = 0; i < bodies - 1; i++)
    {
        float x, y;
        std::cout << "Enter translation(x,y) for " << i + 1 << ": ";
        std::cin >> x >> y;

        translationMatrix.push_back(Eigen::Vector2f(x, y));
    }
}

void RigidBodyTransforms::Transform2D::formTransformMatrix()
{
    while (!translationMatrix.empty() && !rotationMatrix.empty())
    {
        Eigen::Matrix3f m;

        m.block<2, 2>(0, 0) = rotationMatrix.front();
        m.block<2, 1>(0, 2) = translationMatrix.front();
        m.row(2) << 0, 0, 1;

        transform2DMatrix.push_back(m);

        rotationMatrix.erase(rotationMatrix.begin());
        translationMatrix.erase(translationMatrix.begin());
    }
}

void RigidBodyTransforms::Transform2D::calculateResultMatrix()
{
    for (const auto& mat : transform2DMatrix) {
        resultantMatrix *= mat; 
    }
}

void RigidBodyTransforms::Transform2D::printResultantMatrix()
{
    std::cout << "result:" << std::endl;
    std::cout << resultantMatrix << std::endl;
}

void RigidBodyTransforms::Transform2D::printRotationMatrix()
{
    for (auto &&i : rotationMatrix)
    {
        std::cout << i << std::endl;
    }    
}

void RigidBodyTransforms::Transform2D::printTransform2DMatrix()
{
    for (auto &&i : transform2DMatrix)
    {
        std::cout << i << std::endl;
    }    
}

void RigidBodyTransforms::Transform2D::printTranslationMatrix()
{
    for (auto &&i : translationMatrix)
    {
        std::cout << i << std::endl;
    }
}