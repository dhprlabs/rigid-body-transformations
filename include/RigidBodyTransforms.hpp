#pragma once

#include <vector>
#include <Eigen/Dense>
#include <cstddef> // for size_t

namespace RigidBodyTransforms 
{
    class Transform2D 
    {
    private:
        size_t bodies = 0;
        Eigen::Matrix3f resultantMatrix = Eigen::Matrix3f::Identity();

        std::vector<Eigen::Matrix2f> rotationMatrix;
        std::vector<Eigen::Vector2f> translationMatrix;
        std::vector<Eigen::Matrix3f> transform2DMatrix;

    public:
        explicit Transform2D() = default;

        // Setup
        void setBodies(size_t count);

        // Building transforms
        void addRotation(float thetaRadians);
        void addTranslation(float x, float y);
        void formTransformMatrix();

        // Computation
        Eigen::Matrix3f calculateResultMatrix() const;

        // Debug printing
        void printTranslationMatrix() const;
        void printTransform2DMatrix() const;
        void printRotationMatrix() const;
        void printResultantMatrix() const;
    };

    class Transform3D 
    {
    private:
        size_t bodies = 0;
        Eigen::Matrix4f resultantMatrix = Eigen::Matrix4f::Identity();

        std::vector<Eigen::Matrix3f> rotationMatrix;
        std::vector<Eigen::Vector3f> translationMatrix;
        std::vector<Eigen::Matrix4f> transform3DMatrix;

    public:
        explicit Transform3D() = default;

        // Setup
        void setBodies(size_t count);

        // Building transforms
        void addRotation(float roll, float pitch, float yaw);
        void addTranslation(float x, float y, float z);
        void formTransformMatrix();

        // Computation
        Eigen::Matrix4f calculateResultMatrix() const;

        // Debug printing
        void printTranslationMatrix() const;
        void printTransform3DMatrix() const;
        void printRotationMatrix() const;
        void printResultantMatrix() const;
    };
}
