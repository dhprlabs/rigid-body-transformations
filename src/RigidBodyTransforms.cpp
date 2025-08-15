#include "RigidBodyTransforms.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <cmath>

namespace RigidBodyTransforms
{
    // ===== 2D Transform =====

    void Transform2D::setBodies(size_t count)
    {
        bodies = count;
    }

    void Transform2D::addRotation(float thetaRadians)
    {
        Eigen::Matrix2f r;
        r << std::cos(thetaRadians), -std::sin(thetaRadians),
             std::sin(thetaRadians),  std::cos(thetaRadians);
        rotationMatrix.push_back(r);
    }

    void Transform2D::addTranslation(float x, float y)
    {
        translationMatrix.push_back(Eigen::Vector2f(x, y));
    }

    void Transform2D::formTransformMatrix()
    {
        transform2DMatrix.clear();
        size_t count = std::min(rotationMatrix.size(), translationMatrix.size());

        for (size_t i = 0; i < count; ++i)
        {
            Eigen::Matrix3f m = Eigen::Matrix3f::Identity();
            m.block<2, 2>(0, 0) = rotationMatrix[i];
            m.block<2, 1>(0, 2) = translationMatrix[i];
            transform2DMatrix.push_back(m);
        }
    }

    Eigen::Matrix3f Transform2D::calculateResultMatrix() const
    {
        Eigen::Matrix3f result = Eigen::Matrix3f::Identity();
        for (const auto& mat : transform2DMatrix)
        {
            result *= mat;
        }
        return result;
    }

    void Transform2D::printTranslationMatrix() const
    {
        std::cout << "Translation matrices:\n";
        for (const auto& t : translationMatrix)
            std::cout << t << "\n\n";
    }

    void Transform2D::printTransform2DMatrix() const
    {
        std::cout << "2D Transform matrices:\n";
        for (const auto& m : transform2DMatrix)
            std::cout << m << "\n\n";
    }

    void Transform2D::printRotationMatrix() const
    {
        std::cout << "Rotation matrices:\n";
        for (const auto& r : rotationMatrix)
            std::cout << r << "\n\n";
    }

    void Transform2D::printResultantMatrix() const
    {
        std::cout << "Resultant Matrix:\n" << calculateResultMatrix() << "\n";
    }

    // ===== 3D Transform =====

    void Transform3D::setBodies(size_t count)
    {
        bodies = count;
    }

    void Transform3D::addRotation(float roll, float pitch, float yaw)
    {
        Eigen::Matrix3f Rx, Ry, Rz;

        Rx << 1, 0, 0,
              0, std::cos(roll), -std::sin(roll),
              0, std::sin(roll),  std::cos(roll);

        Ry << std::cos(pitch), 0, std::sin(pitch),
              0, 1, 0,
              -std::sin(pitch), 0, std::cos(pitch);

        Rz << std::cos(yaw), -std::sin(yaw), 0,
              std::sin(yaw),  std::cos(yaw), 0,
              0, 0, 1;

        Eigen::Matrix3f R = Rz * Ry * Rx;
        rotationMatrix.push_back(R);
    }

    void Transform3D::addTranslation(float x, float y, float z)
    {
        translationMatrix.push_back(Eigen::Vector3f(x, y, z));
    }

    void Transform3D::formTransformMatrix()
    {
        transform3DMatrix.clear();
        size_t count = std::min(rotationMatrix.size(), translationMatrix.size());

        for (size_t i = 0; i < count; ++i)
        {
            Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
            m.block<3, 3>(0, 0) = rotationMatrix[i];
            m.block<3, 1>(0, 3) = translationMatrix[i];
            transform3DMatrix.push_back(m);
        }
    }

    Eigen::Matrix4f Transform3D::calculateResultMatrix() const
    {
        Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
        for (const auto& mat : transform3DMatrix)
        {
            result *= mat;
        }
        return result;
    }

    void Transform3D::printTranslationMatrix() const
    {
        std::cout << "Translation matrices:\n";
        for (const auto& t : translationMatrix)
            std::cout << t << "\n\n";
    }

    void Transform3D::printTransform3DMatrix() const
    {
        std::cout << "3D Transform matrices:\n";
        for (const auto& m : transform3DMatrix)
            std::cout << m << "\n\n";
    }

    void Transform3D::printRotationMatrix() const
    {
        std::cout << "Rotation matrices:\n";
        for (const auto& r : rotationMatrix)
            std::cout << r << "\n\n";
    }

    void Transform3D::printResultantMatrix() const
    {
        std::cout << "Resultant Matrix:\n" << calculateResultMatrix() << "\n";
    }
}