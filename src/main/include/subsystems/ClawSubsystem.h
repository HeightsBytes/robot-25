// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
  enum for intake state: 
    kIntaking, kStopped, kEjecting or something along those lines
  enum for pivot state:
    (this is an assumption and may need to be changed)
    kStowed, kSwitching, kIntake, kEject

  public functions
    GetAngle()
    bool AtPivotTarget()
    
    GetPivotTarget()
    GetPivotActual()
    GetIntakeTarget()
    GetIntakeActual()

    SetPivotTarget()
    SetIntakeTarget()

    Cmd SetPivotTargetCMD()
    Cmd SetIntakeTargetCMD()

  private
    Pivot Motor
    Intake Motor

    CheckState()

    PivotState PivotActual
    PivotState PivotTarget

    IntakeState IntakeActual
    IntakeState IntakeTarget
    
*/

#pragma once

#include <frc2/command/SubsystemBase.h>

class ClawSubsystem : public frc2::SubsystemBase {
 public:
  ClawSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
