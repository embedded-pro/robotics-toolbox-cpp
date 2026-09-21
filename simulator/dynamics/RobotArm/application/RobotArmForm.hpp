#pragma once

#include "simulator/dynamics/RobotArm/application/RobotArmTypes.hpp"
#include "ui/model/FormModel.hpp"
#include <array>
#include <vector>

namespace simulator::dynamics
{
    namespace field
    {
        inline constexpr ui::model::FieldId degreesOfFreedom{ 1 };
        inline constexpr ui::model::FieldId firstLength{ 2 };
        inline constexpr ui::model::FieldId firstMass{ 3 };
        inline constexpr ui::model::FieldId secondLength{ 4 };
        inline constexpr ui::model::FieldId secondMass{ 5 };
        inline constexpr ui::model::FieldId thirdLength{ 6 };
        inline constexpr ui::model::FieldId thirdMass{ 7 };
        inline constexpr ui::model::FieldId firstTorque{ 8 };
        inline constexpr ui::model::FieldId secondTorque{ 9 };
        inline constexpr ui::model::FieldId thirdTorque{ 10 };
        inline constexpr ui::model::FieldId firstPosition{ 11 };
        inline constexpr ui::model::FieldId secondPosition{ 12 };
        inline constexpr ui::model::FieldId thirdPosition{ 13 };
        inline constexpr ui::model::FieldId damping{ 14 };

        inline constexpr ui::model::GroupId robot{ 1 };
        inline constexpr ui::model::GroupId firstLink{ 2 };
        inline constexpr ui::model::GroupId secondLink{ 3 };
        inline constexpr ui::model::GroupId thirdLink{ 4 };
        inline constexpr ui::model::GroupId torques{ 5 };
        inline constexpr ui::model::GroupId positions{ 6 };
        inline constexpr ui::model::GroupId simulation{ 7 };

        inline constexpr ui::model::ActionId start{ 1 };
        inline constexpr ui::model::ActionId stop{ 2 };
        inline constexpr ui::model::ActionId reset{ 3 };
    }

    class RobotArmForm
    {
    public:
        RobotArmForm();

        [[nodiscard]] ui::model::FormModel& Model();
        [[nodiscard]] const ui::model::FormModel& Model() const;

        [[nodiscard]] RobotArmConfig BuildConfiguration() const;
        [[nodiscard]] std::vector<float> Torques() const;
        [[nodiscard]] std::vector<float> InitialPositions() const;

    private:
        [[nodiscard]] std::size_t DegreesOfFreedom() const;

        static constexpr std::size_t fieldCount{ 14 };

        std::array<ui::model::FieldValue, fieldCount> values{};

        ui::model::FormSpec spec;
        ui::model::FormModel model;
    };
}
