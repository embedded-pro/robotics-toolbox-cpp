#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/JointLink.hpp"
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
        using LinkArray = std::array<JointLink<T>, NumLinks>;

        OPTIMIZE_FOR_SPEED MassMatrix Compute(const LinkArray& links, const JointVector& q) const;

    private:
        using Transforms = std::array<math::SE3Transform<T>, NumLinks>;
        using CompositeInertias = std::array<SpatialInertia<T>, NumLinks>;

        static CompositeInertias ComputeCompositeInertias(const LinkArray& links, const Transforms& transforms);
        static void FillColumn(MassMatrix& M, std::size_t column, const LinkArray& links, const Transforms& transforms, const SpatialInertia<T>& composite);
        static T Project(const JointLink<T>& link, const SpatialWrench<T>& wrench);
    };

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED typename CompositeRigidBodyAlgorithm<T, NumLinks>::MassMatrix CompositeRigidBodyAlgorithm<T, NumLinks>::Compute(const LinkArray& links, const JointVector& q) const
    {
        Transforms transforms{};
        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            assert(HasUnitJointAxis(links[i]));
            transforms[i] = links[i].JointTransform(q[i]);
        }

        const auto composite{ ComputeCompositeInertias(links, transforms) };

        MassMatrix M{};
        for (std::size_t i = 0; i < NumLinks; ++i)
            FillColumn(M, i, links, transforms, composite[i]);

        return M;
    }

    template<typename T, std::size_t NumLinks>
    typename CompositeRigidBodyAlgorithm<T, NumLinks>::CompositeInertias CompositeRigidBodyAlgorithm<T, NumLinks>::ComputeCompositeInertias(const LinkArray& links, const Transforms& transforms)
    {
        CompositeInertias composite{};
        for (std::size_t i = 0; i < NumLinks; ++i)
            composite[i] = RigidBodyInertia(links[i]);

        for (std::size_t i = NumLinks - 1; i > 0; --i)
            composite[i - 1] += TransformInertiaToParent(composite[i], transforms[i].rotation, transforms[i].translation);

        return composite;
    }

    template<typename T, std::size_t NumLinks>
    void CompositeRigidBodyAlgorithm<T, NumLinks>::FillColumn(MassMatrix& M, std::size_t column, const LinkArray& links, const Transforms& transforms, const SpatialInertia<T>& composite)
    {
        const auto angular{ links[column].AngularAxis() };
        const auto linear{ links[column].LinearAxis() };
        SpatialWrench<T> wrench{ composite.cross.Transpose() * angular + composite.lin * linear, composite.rot * angular + composite.cross * linear };
        M.at(column, column) = Project(links[column], wrench) + links[column].armature;

        for (std::size_t j = column; j > 0; --j)
        {
            wrench = TransformWrenchToParent(wrench, transforms[j].rotation, transforms[j].translation);
            const T entry{ Project(links[j - 1], wrench) };
            M.at(column, j - 1) = entry;
            M.at(j - 1, column) = entry;
        }
    }

    template<typename T, std::size_t NumLinks>
    T CompositeRigidBodyAlgorithm<T, NumLinks>::Project(const JointLink<T>& link, const SpatialWrench<T>& wrench)
    {
        return math::DotProduct(link.AngularAxis(), wrench.torque) + math::DotProduct(link.LinearAxis(), wrench.force);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class CompositeRigidBodyAlgorithm<float, 1>;
    extern template class CompositeRigidBodyAlgorithm<float, 2>;
    extern template class CompositeRigidBodyAlgorithm<float, 3>;
#endif
}
