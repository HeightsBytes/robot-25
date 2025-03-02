// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/DigitalInput.h>
#include <rev/SparkMax.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  enum class InnerState{
    kTop,
    kMiddleTop,
    kMiddleBottom,
    kBottom,
    kStopped
  };
  enum class OuterState{
    kTop,
    kMiddleTop,
    kMiddleBottom,
    kBottom,
    kStopped
  };

  ElevatorSubsystem();
  void Periodic() override;

  bool AtInnerTarget();
  bool AtOuterTarget();

  InnerState GetInnerTarget();
  OuterState GetOuterTarget();

  void SetInnerTarget(InnerState state);
  void SetOuterTarget(OuterState state);

  bool GetInnerLimitSwitch();
  bool GetOuterLimitSwitch();

  frc2::CommandPtr SetInnerTargetCMD(InnerState state) {
    return this->RunOnce([this, state] { SetInnerTarget(state); });
  }
  frc2::CommandPtr SetOuterTargetCMD(OuterState state) {
    return this->RunOnce([this, state] { SetOuterTarget(state); });
  }

 private:
  double StateToOutput(OuterState state) const;
  double StateToOutput(InnerState state) const;

  void CheckState();

  std::string ToStr(InnerState state) const;
  std::string ToStr(OuterState state) const;

  // motors
  rev::spark::SparkBase m_inner; // this will be changed from base soon to flex or max idk whats going on there
  rev::spark::SparkBase m_outer;

  rev::spark::SparkBaseConfig m_innerConfig;
  rev::spark::SparkBaseConfig m_outerConfig;

  rev::spark::SparkRelativeEncoder m_innerEncoder;
  rev::spark::SparkRelativeEncoder m_outerEncoder;

  rev::spark::SparkClosedLoopController m_innerController;
  rev::spark::SparkClosedLoopController m_outerController;
  // motors
  
  frc::DigitalInput m_innerLimitSwitch;
  frc::DigitalInput m_outerLimitSwitch;
};
