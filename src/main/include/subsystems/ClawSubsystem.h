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
