#pragma once

#include <array>
#include <ostream>

namespace dofbot{
    enum class RobotMode{
        kOther,
        kIdle,
        kMove,
        kGuiding,
        kReflex,
        kUserStopped,
        kAutomaticErrorRecovery
    };

    struct RobotState{
        std::array<double, 16> O_T_EE{};

        std::array<double, 16> O_T_EE_d{};

        /**
         * Measured joint position
        */
        std::array<double, 6> q{};

        /**
         * Desired joint position
        */
        std::array<double, 6> q_d{};

        /**
         * Measured joint velocity.
        */
       std::array<double, 6> dq{};

       /**
        * Desired joint velocity
       */
      std::array<double, 6> dq_d{};
    };
}