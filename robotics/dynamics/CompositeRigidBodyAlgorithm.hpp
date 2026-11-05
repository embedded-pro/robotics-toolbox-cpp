#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/RevoluteJointLink.hpp"
#include "robotics/dynamics/SpatialInertia.hpp"
#include <array>
#include <cassert>
#include <cstddef>
#include <type_traits>

namespace dynamics
{
    template<typename T, std::size_t NumLinks>
    class CompositeRigidBodyAlgorithm
    {
        static_assert(std::is_floating_point_v<T>, "CompositeRigidBodyAlgorithm supports floating-point types");
        static_assert(NumLinks > 0, "Number of links must be positive");

    public:
        using Matrix3 = math::SquareMatrix<T, 3>;
        using JointVector = math::Vector<T, NumLinks>;
        using MassMatrix = math::SquareMatrix<T, NumLinks>;
        using LinkArray = std::array<RevoluteJointLink<T>, NumLinks>;

        OPTIMIZE_FOR_SPEED MassMatrix Compute(const LinkArray& links, const JointVector& q) const;

    private:
        using Rotations = std::array<Matrix3, NumLinks>;
        using CompositeInertias = std::array<SpatialInertia<T>, NumLinks>;

        static CompositeInertias ComputeCompositeInertias(const LinkArray& links, const Rotations& R);
        static void FillColumn(MassMatrix& M, std::size_t column, const LinkArray& links, const Rotations& R, const SpatialInertia<T>& composite);
    };

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED typename CompositeRigidBodyAlgorithm<T, NumLinks>::MassMatrix CompositeRigidBodyAlgorithm<T, NumLinks>::Compute(const LinkArray& links, const JointVector& q) const
    {
        Rotations R{};
        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            assert(HasUnitJointAxis(links[i]));
            R[i] = math::RotationAboutAxis(links[i].jointAxis, q[i]);
        }

        const auto composite{ ComputeCompositeInertias(links, R) };

        MassMatrix M{};
        for (std::size_t i = 0; i < NumLinks; ++i)
            FillColumn(M, i, links, R, composite[i]);

        return M;
    }

    template<typename T, std::size_t NumLinks>
    typename CompositeRigidBodyAlgorithm<T, NumLinks>::CompositeInertias CompositeRigidBodyAlgorithm<T, NumLinks>::ComputeCompositeInertias(const LinkArray& links, const Rotations& R)
    {
        CompositeInertias composite{};
        for (std::size_t i = 0; i < NumLinks; ++i)
            composite[i] = RigidBodyInertia(links[i]);

        for (std::size_t i = NumLinks - 1; i > 0; --i)
            composite[i - 1] += TransformInertiaToParent(composite[i], R[i], links[i].parentToJoint);

        return composite;
    }

    template<typename T, std::size_t NumLinks>
    void CompositeRigidBodyAlgorithm<T, NumLinks>::FillColumn(MassMatrix& M, std::size_t column, const LinkArray& links, const Rotations& R, const SpatialInertia<T>& composite)
    {
        const auto& axis{ links[column].jointAxis };
        SpatialWrench<T> wrench{ composite.cross.Transpose() * axis, composite.rot * axis };
        M.at(column, column) = math::DotProduct(axis, wrench.torque);

        for (std::size_t j = column; j > 0; --j)
        {
            wrench = TransformWrenchToParent(wrench, R[j], links[j].parentToJoint);
            const T entry{ math::DotProduct(links[j - 1].jointAxis, wrench.torque) };
            M.at(column, j - 1) = entry;
            M.at(j - 1, column) = entry;
        }
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class CompositeRigidBodyAlgorithm<float, 1>;
    extern template class CompositeRigidBodyAlgorithm<float, 2>;
    extern template class CompositeRigidBodyAlgorithm<float, 3>;
#endif
}
