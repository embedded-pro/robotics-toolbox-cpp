#include "simulator/dynamics/RobotArm/application/RobotArmForm.hpp"
#include <numbers>

namespace simulator::dynamics
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::Condition;
        using ui::model::FieldKind;
        using ui::model::FieldSpec;
        using ui::model::GroupSpec;
        using ui::model::OptionSpec;

        constexpr float torqueScale{ 10.0f };
        constexpr float degreesToRadians{ std::numbers::pi_v<float> / 180.0f };

        constexpr std::array<OptionSpec, 2> degreeOptions{
            OptionSpec{ "2 DOF (Planar)", 2 },
            OptionSpec{ "3 DOF (Spatial)", 3 }
        };

        constexpr Condition spatialOnly{ field::degreesOfFreedom, 0b10 };

        constexpr std::array<GroupSpec, 7> groups{
            GroupSpec{ field::robot, "Robot Configuration", {} },
            GroupSpec{ field::firstLink, "Link 1", {} },
            GroupSpec{ field::secondLink, "Link 2", {} },
            GroupSpec{ field::thirdLink, "Link 3", spatialOnly },
            GroupSpec{ field::torques, "Joint Torques (N·m)", {} },
            GroupSpec{ field::positions, "Initial Position (deg)", {} },
            GroupSpec{ field::simulation, "Simulation", {} }
        };

        constexpr std::array<FieldSpec, 14> fields{
            FieldSpec{ field::degreesOfFreedom, field::robot, FieldKind::Choice, "DOF:", "", {}, degreeOptions, {}, {} },
            FieldSpec{ field::firstLength, field::firstLink, FieldKind::Number, "Length (m):", "", { 0.05, 2.0, 0.05, 0.5, 2 }, {}, {}, {} },
            FieldSpec{ field::firstMass, field::firstLink, FieldKind::Number, "Mass (kg):", "", { 0.1, 20.0, 0.1, 1.0, 2 }, {}, {}, {} },
            FieldSpec{ field::secondLength, field::secondLink, FieldKind::Number, "Length (m):", "", { 0.05, 2.0, 0.05, 0.4, 2 }, {}, {}, {} },
            FieldSpec{ field::secondMass, field::secondLink, FieldKind::Number, "Mass (kg):", "", { 0.1, 20.0, 0.1, 0.8, 2 }, {}, {}, {} },
            FieldSpec{ field::thirdLength, field::thirdLink, FieldKind::Number, "Length (m):", "", { 0.05, 2.0, 0.05, 0.3, 2 }, {}, spatialOnly, {} },
            FieldSpec{ field::thirdMass, field::thirdLink, FieldKind::Number, "Mass (kg):", "", { 0.1, 20.0, 0.1, 0.6, 2 }, {}, spatialOnly, {} },
            FieldSpec{ field::firstTorque, field::torques, FieldKind::Slider, "τ1:", "", { -200.0, 200.0, 1.0, 0.0, 0, 50.0 }, {}, {}, {} },
            FieldSpec{ field::secondTorque, field::torques, FieldKind::Slider, "τ2:", "", { -200.0, 200.0, 1.0, 0.0, 0, 50.0 }, {}, {}, {} },
            FieldSpec{ field::thirdTorque, field::torques, FieldKind::Slider, "τ3:", "", { -200.0, 200.0, 1.0, 0.0, 0, 50.0 }, {}, spatialOnly, {} },
            FieldSpec{ field::firstPosition, field::positions, FieldKind::Slider, "q1:", "", { -180.0, 180.0, 1.0, 0.0, 0, 45.0 }, {}, {}, {} },
            FieldSpec{ field::secondPosition, field::positions, FieldKind::Slider, "q2:", "", { -180.0, 180.0, 1.0, 0.0, 0, 45.0 }, {}, {}, {} },
            FieldSpec{ field::thirdPosition, field::positions, FieldKind::Slider, "q3:", "", { -180.0, 180.0, 1.0, 0.0, 0, 45.0 }, {}, spatialOnly, {} },
            FieldSpec{ field::damping, field::simulation, FieldKind::Number, "Damping:", "", { 0.0, 5.0, 0.01, 0.05, 3 }, {}, {}, {} }
        };

        constexpr std::array<ActionSpec, 3> actions{
            ActionSpec{ field::start, "Start", ui::theme::ButtonRole::Start, 0 },
            ActionSpec{ field::stop, "Stop", ui::theme::ButtonRole::Stop, 0 },
            ActionSpec{ field::reset, "Reset", ui::theme::ButtonRole::Reset, 0 }
        };

        constexpr std::array<ui::model::FieldId, 3> lengthFields{ field::firstLength, field::secondLength, field::thirdLength };
        constexpr std::array<ui::model::FieldId, 3> massFields{ field::firstMass, field::secondMass, field::thirdMass };
        constexpr std::array<ui::model::FieldId, 3> torqueFields{ field::firstTorque, field::secondTorque, field::thirdTorque };
        constexpr std::array<ui::model::FieldId, 3> positionFields{ field::firstPosition, field::secondPosition, field::thirdPosition };
    }

    RobotArmForm::RobotArmForm()
        : spec{ groups, fields, actions, {} }
        , model{ spec, values, {} }
    {}

    ui::model::FormModel& RobotArmForm::Model()
    {
        return model;
    }

    const ui::model::FormModel& RobotArmForm::Model() const
    {
        return model;
    }

    std::size_t RobotArmForm::DegreesOfFreedom() const
    {
        return static_cast<std::size_t>(model.SelectedData(field::degreesOfFreedom));
    }

    RobotArmConfig RobotArmForm::BuildConfiguration() const
    {
        RobotArmConfig config;

        config.dof = static_cast<int>(DegreesOfFreedom());
        config.damping = model.Float(field::damping);

        config.linkLengths.resize(DegreesOfFreedom());
        config.linkMasses.resize(DegreesOfFreedom());

        for (std::size_t i = 0; i < DegreesOfFreedom(); ++i)
        {
            config.linkLengths[i] = model.Float(lengthFields[i]);
            config.linkMasses[i] = model.Float(massFields[i]);
        }

        return config;
    }

    std::vector<float> RobotArmForm::Torques() const
    {
        std::vector<float> torques(DegreesOfFreedom());

        for (std::size_t i = 0; i < torques.size(); ++i)
            torques[i] = model.Float(torqueFields[i]) / torqueScale;

        return torques;
    }

    std::vector<float> RobotArmForm::InitialPositions() const
    {
        std::vector<float> positions(DegreesOfFreedom());

        for (std::size_t i = 0; i < positions.size(); ++i)
            positions[i] = model.Float(positionFields[i]) * degreesToRadians;

        return positions;
    }
}
