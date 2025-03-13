// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
  enum for intake state: 
    kIntaking, kStopped, kEjecting or something along those lines
  enum for pivot state:
    (this is an assumption and may need to be changed)
    kSwitching, kL1, kL2, kL3, kCoral

  public functions
    GetAngle()
    bool AtPivotTarget()
    
    GetPivotTarget()
    GetPivotActual()
    GetIntakeTarget()
    GetIntakeActual()

    SetPivotTarget()
    SetIntakeTarget()

    bool GetSensor()

    cmd Intake() (intakes until sensor is found that the coral is in the intake)
    Cmd SetPivotTargetCMD()
    Cmd SetIntakeTargetCMD()

  private
    Pivot Motor (includes encoder and stuff)
    Intake Motor (includes encoder and stuff())

    PivotState PivotActual
    PivotState PivotTarget

    IntakeState IntakeActual
    IntakeState IntakeTarget
    
*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <rev/SparkMax.h>
#include <rev/SparkFlex.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/config/SparkFlexConfig.h>
#include <rev/SparkMaxAlternateEncoder.h>
#include <units/angle.h>
#include <units/power.h>

#include "Constants.h"

class ClawSubsystem : public frc2::SubsystemBase {
 public:
  enum class IntakeState {
    kIntaking,
    kStopped,
    kEjecting
  };
  enum class PivotState {
    kSwitching,
    kIntake,
    kL2,
    kL3,
    kL4,
    kCoral
  };
  ClawSubsystem();

  void Periodic() override;

  double GetPivotAngle() const{
    return m_pivotEncoder.GetPosition();
  };
  bool AtPivotTarget();

  void SetPivotTarget(PivotState state){
    m_pivotTarget = state;
  }
  void SetIntakeTarget(IntakeState state){
    m_intakeTarget = state;
  };

  PivotState GetPivotTarget() const {
    return m_pivotTarget;
  }
  PivotState GetPivotActual() const {
    return m_pivotActual;
  }
  IntakeState GetIntakeTarget() const {
    return m_intakeTarget;
  }

  bool GetSensor() const {
    return m_sensor.Get();
  }
/*
  void Intake(){
    while(GetSensor()){
      SetIntakeTarget(IntakeState::kEjecting);
    }
    SetIntakeTarget(IntakeState::kStopped);
  }
*/

  frc2::CommandPtr SetIntakeTargetCMD(IntakeState state){
    return this->RunOnce([this, state]{SetIntakeTarget(state); });
  }

  frc2::CommandPtr SetPivotTargetCMD(PivotState state){
    return this->RunOnce([this, state]{SetPivotTarget(state); });
  }

 private:
  double StateToOutput(PivotState state) const;
  double StateToOutput(IntakeState state) const;

  void CheckState();

  std::string ToStr(PivotState state) const;
  std::string ToStr(IntakeState state) const;

  // motors
  rev::spark::SparkMax m_pivot;
  rev::spark::SparkFlex m_intake;

  rev::spark::SparkMaxConfig m_pivotConfig;
  rev::spark::SparkFlexConfig m_intakeConfig;

  rev::spark::SparkRelativeEncoder m_pivotEncoder;
  rev::spark::SparkClosedLoopController m_pivotController;
  // motors

  PivotState m_pivotActual;
  PivotState m_pivotTarget;

  IntakeState m_intakeTarget;

  frc::DigitalInput m_sensor;

};
