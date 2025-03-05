// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClawSubsystem.h"

ClawSubsystem::ClawSubsystem() :
    m_intake(ClawConstants::kIntakeMotorPort, rev::spark::SparkMax::MotorType::kBrushless),
    m_pivot(ClawConstants::kPivotMotorPort, rev::spark::SparkMax::MotorType::kBrushless),
    m_pivotEncoder(m_pivot.GetAbsoluteEncoder()),
    m_pivotController(m_pivot.GetClosedLoopController()),
    m_pivotActual(PivotState::kSwitching),
    m_pivotTarget(PivotState::kCoral),
    m_intakeTarget(IntakeState::kStopped) {

        m_pivotConfig
            .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake)
            .Inverted(false)
            .SmartCurrentLimit(30);
        m_intakeConfig
            .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kCoast)
            .Inverted(false);
        
        m_pivotConfig.encoder
            .PositionConversionFactor(ClawConstants::kPivotEncoderRatio)

    }   

// This method will be called once per scheduler run
void ClawSubsystem::Periodic() {}
