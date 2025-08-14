#pragma once

#include <vector>
#include <utility> 
#include <Eigen/Dense>

namespace RigidBodyTransforms 
{
    class Transform2D 
    {
    private:
        int bodies;
        Eigen::Matrix<float, 3, 3> resultantMatrix;

        std::vector<Eigen::Matrix<float, 2, 2>> rotationMatrix;
        std::vector<Eigen::Matrix<float, 2, 1>> translationMatrix;
        std::vector<Eigen::Matrix<float, 3, 3>> transform2DMatrix;

    public:
        // Constructor & destructor
        Transform2D();

        // Methods
        void setBodies(int count);
        void formTranslationMatrix();
        void formRotationMatrix();
        void formTransformMatrix();
        void calculateResultMatrix();
        void printTranslationMatrix();
        void printTransform2DMatrix();
        void printRotationMatrix();
        void printResultantMatrix();
    };
}