// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/DigitalInput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/MathUtil.h>
#include <rev/SparkFlex.h>
#include <rev/config/SparkFlexConfig.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  enum class ElevatorState{
    kTop,
    kMiddleTop,
    kMiddleBottom,
    kBottom,
    kSwitching
  };

  ElevatorSubsystem();
  void Periodic() override;

  bool AtTarget();

  ElevatorState GetTarget();

  void SetTarget(ElevatorState state);

  bool GetTopLimitSwitch();
  bool GetBottomLimitSwitch();

  frc2::CommandPtr SetTargetCMD(ElevatorState state) {
    return this->RunOnce([this, state] { SetTarget(state); });
  }

 private:
  double StateToOutput(ElevatorState state) const;

  void CheckState();

  std::string ToStr(ElevatorState state) const;

  // motors
  rev::spark::SparkFlex m_elevator;

  rev::spark::SparkFlexConfig m_elevatorConfig;

  rev::spark::SparkRelativeEncoder m_elevatorEncoder;

  rev::spark::SparkClosedLoopController m_elevatorController;
  // motors
  
  frc::DigitalInput m_topLimitSwitch;
  frc::DigitalInput m_bottomLimitSwitch;

  ElevatorState m_target;
  ElevatorState m_actual;
};