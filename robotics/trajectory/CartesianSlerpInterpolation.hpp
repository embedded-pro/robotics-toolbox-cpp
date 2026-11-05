#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/Quaternion.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "numerical/motion_profiles/PolynomialProfile.hpp"
#include <algorithm>
#include <limits>
#include <type_traits>

namespace trajectory
{
    template<typename T>
    struct CartesianState
    {
        static_assert(std::is_floating_point_v<T>, "CartesianState supports floating-point types");

        math::SE3Transform<T> pose{};
        math::Vector6<T> twist{};
    };

    template<typename T, typename TimeLaw>
    class CartesianSlerpInterpolation
    {
        static_assert(std::is_floating_point_v<T>, "CartesianSlerpInterpolation supports floating-point types");

    public:
        using Transform = math::SE3Transform<T>;
        using Vector3 = math::Vector3<T>;
        using Quaternion = math::Quaternion<T>;

        CartesianSlerpInterpolation(const Transform& start, const Transform& goal, const TimeLaw& timeLaw);

        OPTIMIZE_FOR_SPEED CartesianState<T> Sample(T t) const;
        T Duration() const;

    private:
        static Quaternion ShortestArcGoal(const Quaternion& from, const Quaternion& to);
        static Vector3 RotationVector(const Quaternion& delta);

        Vector3 p0;
        Vector3 deltaP;
        Quaternion q0;
        Quaternion q1;
        Vector3 rotationVector;
        TimeLaw timeLaw;
    };

    template<typename T, typename TimeLaw>
    CartesianSlerpInterpolation<T, TimeLaw>::CartesianSlerpInterpolation(const Transform& start, const Transform& goal, const TimeLaw& timeLaw)
        : p0{ start.translation }
        , deltaP{ goal.translation - start.translation }
        , q0{ Quaternion::FromRotationMatrix(start.rotation) }
        , q1{ ShortestArcGoal(q0, Quaternion::FromRotationMatrix(goal.rotation)) }
        , rotationVector{ RotationVector(q1 * q0.Conjugate()) }
        , timeLaw{ timeLaw }
    {}

    template<typename T, typename TimeLaw>
    OPTIMIZE_FOR_SPEED CartesianState<T> CartesianSlerpInterpolation<T, TimeLaw>::Sample(T t) const
    {
        const auto law{ timeLaw.Sample(std::clamp(t, T{}, Duration())) };
        const T s{ std::clamp(law.position, T{}, T{ 1 }) };

        CartesianState<T> state{ Transform{ Quaternion::Slerp(q0, q1, s).ToRotationMatrix(), p0 + deltaP * s }, math::Vector6<T>{} };
        state.twist.SetBlock(Vector3{ deltaP * law.velocity }, 0, 0);
        state.twist.SetBlock(Vector3{ rotationVector * law.velocity }, 3, 0);
        return state;
    }

    template<typename T, typename TimeLaw>
    T CartesianSlerpInterpolation<T, TimeLaw>::Duration() const
    {
        return timeLaw.Duration();
    }

    template<typename T, typename TimeLaw>
    typename CartesianSlerpInterpolation<T, TimeLaw>::Quaternion CartesianSlerpInterpolation<T, TimeLaw>::ShortestArcGoal(const Quaternion& from, const Quaternion& to)
    {
        const T dot{ from.w * to.w + from.x * to.x + from.y * to.y + from.z * to.z };
        return dot < T{} ? -to : to;
    }

    template<typename T, typename TimeLaw>
    typename CartesianSlerpInterpolation<T, TimeLaw>::Vector3 CartesianSlerpInterpolation<T, TimeLaw>::RotationVector(const Quaternion& delta)
    {
        const Vector3 axis{ delta.x, delta.y, delta.z };
        const T sinHalfAngle{ math::Sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z) };
        const T scale{ sinHalfAngle > std::numeric_limits<T>::epsilon() ? T{ 2 } * math::Atan2(sinHalfAngle, delta.w) / sinHalfAngle : T{ 2 } };
        return axis * scale;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class CartesianSlerpInterpolation<float, motion_profiles::PolynomialProfile<float>>;
#endif
}
