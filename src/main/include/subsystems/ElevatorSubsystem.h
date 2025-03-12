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
    kL4,
    kL3,
    kL2,
    kL1,
    kIntake,
    kSwitching
  };

  ElevatorSubsystem();
  void Periodic() override;

  bool AtTarget(){
    return m_actual == m_target;
  };

  ElevatorState GetTarget() const{
    return m_target;
  };
  ElevatorState GetActual() const{
    return m_actual;
  };

  void SetTarget(ElevatorState state){
    m_target = state;
  }

  static ElevatorState GetNextState(ElevatorState state);
  static ElevatorState GetPreviousState(ElevatorState state);

/*
  bool GetTopLimitSwitch() const {
    return m_topLimitSwitch.Get();
  };
  bool GetBottomLimitSwitch() const {
    return m_bottomLimitSwitch.Get();
  };
*/
  frc2::CommandPtr SetTargetCMD(ElevatorState state) {
    return this->RunOnce([this, state] { SetTarget(state); });
  }

 private:
  double StateToOutput(ElevatorState state) const;

  void CheckState();

  std::string ToStr(ElevatorState state) const;

  // motors
  rev::spark::SparkFlex m_elevator;
  rev::spark::SparkFlex m_elevator2;

  rev::spark::SparkFlexConfig m_elevatorConfig;
  rev::spark::SparkFlexConfig m_elevator2Config;

  rev::spark::SparkRelativeEncoder m_elevatorEncoder;
  rev::spark::SparkRelativeEncoder m_elevator2Encoder;

  rev::spark::SparkClosedLoopController m_elevatorController;
  rev::spark::SparkClosedLoopController m_elevator2Controller;
  // motors
  
  //frc::DigitalInput m_topLimitSwitch;
  //frc::DigitalInput m_bottomLimitSwitch;

  ElevatorState m_target;
  ElevatorState m_actual;
};