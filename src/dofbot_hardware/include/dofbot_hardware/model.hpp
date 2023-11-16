#pragma once

#include <array>

namespace dofbot_hardware{

class Model{
public:
    explicit Model(dofbot::Model* model) : model_(model) {};
    virtual ~Model() = default;

    [[nodiscard]] std::array<double, 16> pose(
        franka:Frame frame,
    )
};

}