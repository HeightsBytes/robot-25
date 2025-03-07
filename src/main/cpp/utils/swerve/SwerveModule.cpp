// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/swerve/SwerveModule.h"

#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorPort, int turningMotorPort, int turningEncoderPort, double offset, bool inverted)
    : m_driveMotor(driveMotorPort, rev::spark::SparkMax::MotorType::kBrushless),
    m_turningMotor(turningMotorPort, rev::spark::SparkMax::MotorType::kBrushless),
    m_turningEncoder(turningEncoderPort, offset),
    m_id(turningEncoderPort),
    m_sparkDriveEncoder(m_driveMotor.GetEncoder()),
    m_sparkTurnEncoder(m_turningMotor.GetEncoder()),
    m_dController(m_driveMotor.GetClosedLoopController()),
    m_tController(m_turningMotor.GetClosedLoopController()) {

        //motors break by default
        m_driveConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
        m_turningConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
        m_turningConfig.Inverted(inverted);

        // turn conversion factors
        m_turningConfig.encoder
            .PositionConversionFactor(ModuleConstants::kTurnEncoderRatio);

        // set drive conversion factor
        m_driveConfig.encoder
            .VelocityConversionFactor(ModuleConstants::kDriveEncoderDistancePerPulse / 60)
            .PositionConversionFactor(ModuleConstants::kDriveEncoderDistancePerPulse);

        // turning closed loop constants
        m_turningConfig.closedLoop
            .PositionWrappingEnabled(true)
            .PositionWrappingMaxInput(std::numbers::pi)
            .PositionWrappingMinInput(-std::numbers::pi)
            .Pid(
                ModuleConstants::kPTurn,
                ModuleConstants::kITurn,
                ModuleConstants::kDTurn
            )
            .VelocityFF(ModuleConstants::kFFTurn) // IF THERE IS A PROBLEM CHECK HERE
            .OutputRange(-1, 1);

        // driving closed loop constants
        m_driveConfig.closedLoop
            .Pid(
                ModuleConstants::kPDrive,
                ModuleConstants::kIDrive,
                ModuleConstants::kDDrive
            )
            .VelocityFF(ModuleConstants::kFFDrive * 1 /12)
            .OutputRange(-1, 1);

        // now that all param have been set, apply to motors.
        m_driveMotor.Configure(m_driveConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
        m_turningMotor.Configure(m_turningConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

        //sleep(10);
        m_sparkTurnEncoder.SetPosition(m_turningEncoder.Get().value());
    } // constructor

frc::SwerveModuleState SwerveModule::GetState() const {
    return {
        units::meters_per_second_t{m_sparkDriveEncoder.GetVelocity()},
        frc::Rotation2d{units::radian_t(m_sparkTurnEncoder.GetPosition())}
    };
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
    return {
        units::meter_t(m_sparkDriveEncoder.GetPosition()),
        units::radian_t(m_sparkTurnEncoder.GetPosition())
        };
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
   const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(m_sparkTurnEncoder.GetPosition()));

  if (std::fabs(state.speed.value()) < 0.01) {
    // Check to see if the input is very small, if it is, cancel all outputs
    StopMotors();
  } else {
    // If the outputs are sufficient, apply them with the PID Controllers
    m_dController.SetReference(state.speed.value(),
                               rev::spark::SparkMax::ControlType::kVelocity);
    m_tController.SetReference(state.angle.Radians().value(),
                               rev::spark::SparkMax::ControlType::kPosition);
  }
}

void SwerveModule::ResetEncoders() {
  m_sparkDriveEncoder.SetPosition(0);
  m_sparkTurnEncoder.SetPosition(m_turningEncoder.Get().value());
}

void SwerveModule::ZeroTurnEncoder() {
  // This is useful if you don't want to change the drive encoder reading
  m_sparkTurnEncoder.SetPosition(m_turningEncoder.Get().value());
}

void SwerveModule::StopMotors() {
  m_driveMotor.Set(0);
  m_turningMotor.Set(0);
}