// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utils/swerve/CANCoder.h"

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

class SwerveModule{
  public:
    SwerveModule(int driveMotorPort, int turningMotorPort, int turningEncoderPort, double offset, bool inverted);

    frc::SwerveModuleState GetState() const;
    frc::SwerveModulePosition GetPosition() const;

    void SetDesiredState(const frc::SwerveModuleState& state);
    void ResetEncoders();
    void ZeroTurnEncoder();
    void StopMotors();

  private:
    rev::spark::SparkMax m_driveMotor;
    rev::spark::SparkMax m_turningMotor;

    rev::spark::SparkMaxConfig m_driveConfig;
    rev::spark::SparkMaxConfig m_turningConfig;

    rev::spark::SparkRelativeEncoder m_sparkDriveEncoder;
    rev::spark::SparkRelativeEncoder m_sparkTurnEncoder;

    rev::spark::SparkClosedLoopController m_tController;
    rev::spark::SparkClosedLoopController m_dController;

    hb::S_CANCoder m_turningEncoder;

    int m_id;
};