#include "simulator/dynamics/RobotArm/application/RobotArmSimulator.hpp"
#include "infra/util/ReallyAssert.hpp"

namespace simulator::dynamics
{
    namespace
    {
        using Vector3 = math::Vector<float, 3>;
        using Link = ::dynamics::RevoluteJointLink<float>;

        constexpr float rodAxialInertia{ 0.001f };

        const Vector3 xAxis{ 1.0f, 0.0f, 0.0f };
        const Vector3 yAxis{ 0.0f, 1.0f, 0.0f };
        const Vector3 zAxis{ 0.0f, 0.0f, 1.0f };

        Link MakeRod(float mass, float length, const Vector3& jointAxis, const Vector3& parentToJoint, const Vector3& direction)
        {
            const float transverse{ mass * length * length / 12.0f };
            const auto alongRod{ math::OuterProduct(direction, direction) };
            const auto inertia{ (math::SquareMatrix<float, 3>::Identity() - alongRod) * transverse + alongRod * rodAxialInertia };

            return Link{ mass, inertia, jointAxis, parentToJoint, direction * (length / 2.0f) };
        }

        template<std::size_t N>
        math::Vector<float, N> ToVector(const std::vector<float>& values)
        {
            math::Vector<float, N> result{};

            for (std::size_t i = 0; i < N; ++i)
                result.at(i, 0) = values[i];

            return result;
        }
    }

    void RobotArmSimulator::Configure(const RobotArmConfig& cfg)
    {
        really_assert(cfg.dof == 2 || cfg.dof == 3);
        really_assert(cfg.linkLengths.size() >= static_cast<std::size_t>(cfg.dof));
        really_assert(cfg.linkMasses.size() >= static_cast<std::size_t>(cfg.dof));

        config = cfg;
        torques.assign(config.dof, 0.0f);
        Reset();
    }

    void RobotArmSimulator::SetTorques(const std::vector<float>& t)
    {
        for (int i = 0; i < config.dof && i < static_cast<int>(t.size()); ++i)
            torques[i] = t[i];
    }

    void RobotArmSimulator::SetInitialPositions(const std::vector<float>& positions)
    {
        for (int i = 0; i < config.dof && i < static_cast<int>(positions.size()); ++i)
            state.q[i] = positions[i];

        state.qDot.assign(config.dof, 0.0f);
        UpdateForwardKinematics();
    }

    void RobotArmSimulator::Step()
    {
        if (config.dof == 3)
            StepChain(BuildLinks3(), aba3, rnea3);
        else
            StepChain(BuildLinks2(), aba2, rnea2);

        state.time += config.dt;
        UpdateForwardKinematics();
        UpdateTrail();
    }

    void RobotArmSimulator::Reset()
    {
        state.q.assign(config.dof, 0.0f);
        state.qDot.assign(config.dof, 0.0f);
        state.qDDot.assign(config.dof, 0.0f);
        state.inverseDynamicsTorques.assign(config.dof, 0.0f);
        state.time = 0.0f;
        state.endEffectorTrail.clear();
        UpdateForwardKinematics();
    }

    const RobotArmState& RobotArmSimulator::GetState() const
    {
        return state;
    }

    const RobotArmConfig& RobotArmSimulator::GetConfig() const
    {
        return config;
    }

    template<std::size_t N>
    void RobotArmSimulator::StepChain(const LinkArray<N>& links, const ::dynamics::ArticulatedBodyAlgorithm<float, N>& aba,
        const ::dynamics::RecursiveNewtonEuler<float, N>& rnea)
    {
        const Vector3 g{ 0.0f, 0.0f, -gravity };
        const auto q{ ToVector<N>(state.q) };
        const auto qDot{ ToVector<N>(state.qDot) };

        const auto qDDot{ aba.ForwardDynamics(links, q, qDot, ToVector<N>(torques), g) };
        const auto inverseDynamicsTorques{ rnea.InverseDynamics(links, q, qDot, qDDot, g) };

        for (std::size_t i = 0; i < N; ++i)
        {
            state.qDDot[i] = qDDot.at(i, 0);
            state.inverseDynamicsTorques[i] = inverseDynamicsTorques.at(i, 0);
            state.qDot[i] += qDDot.at(i, 0) * config.dt;
            state.q[i] += state.qDot[i] * config.dt;
            state.qDot[i] *= 1.0f - config.damping * config.dt;
        }
    }

    RobotArmSimulator::LinkArray<2> RobotArmSimulator::BuildLinks2() const
    {
        const auto& lengths = config.linkLengths;
        const auto& masses = config.linkMasses;

        return LinkArray<2>{
            MakeRod(masses[0], lengths[0], yAxis, Vector3{}, xAxis),
            MakeRod(masses[1], lengths[1], yAxis, xAxis * lengths[0], xAxis)
        };
    }

    RobotArmSimulator::LinkArray<3> RobotArmSimulator::BuildLinks3() const
    {
        const auto& lengths = config.linkLengths;
        const auto& masses = config.linkMasses;

        return LinkArray<3>{
            MakeRod(masses[0], lengths[0], zAxis, Vector3{}, zAxis),
            MakeRod(masses[1], lengths[1], yAxis, zAxis * lengths[0], xAxis),
            MakeRod(masses[2], lengths[2], yAxis, xAxis * lengths[1], xAxis)
        };
    }

    template<std::size_t N>
    void RobotArmSimulator::UpdateJointPositions(const LinkArray<N>& links)
    {
        const ::kinematics::ForwardKinematics<float, N> fk{ xAxis * config.linkLengths[N - 1] };
        const auto positions{ fk.Compute(links, ToVector<N>(state.q)) };

        state.jointPositions.resize(N + 1);

        for (std::size_t i = 0; i <= N; ++i)
            state.jointPositions[i] = { positions[i].at(0, 0), positions[i].at(1, 0), positions[i].at(2, 0) };
    }

    void RobotArmSimulator::UpdateForwardKinematics()
    {
        if (config.dof == 3)
            UpdateJointPositions(BuildLinks3());
        else
            UpdateJointPositions(BuildLinks2());
    }

    void RobotArmSimulator::UpdateTrail()
    {
        if (!state.jointPositions.empty())
        {
            state.endEffectorTrail.push_back(state.jointPositions.back());
            if (state.endEffectorTrail.size() > maxTrailSize)
                state.endEffectorTrail.erase(state.endEffectorTrail.begin());
        }
    }
}
