// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/PowerDistribution.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include <optional>
#include <string>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/ClawSubsystem.h"
#include "commands/DefaultDrive.h"
#include "commands/AutoAlign.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();
  frc::SendableChooser<std::string> m_chooser;


 private:

  frc2::CommandXboxController m_driverController{
    OIConstants::kDriverControllerPort
  };
  frc2::CommandXboxController m_operatorController{
    OIConstants::kOperatorControllerPort
  };

  frc::PowerDistribution m_pdp{0, frc::PowerDistribution::ModuleType::kCTRE};

  // The robot's subsystems
  DriveSubsystem m_drive;
  ElevatorSubsystem m_elevator;
  ClawSubsystem m_claw;
  // VisionSubsystem& m_vision = VisionSubsystem::GetInstance();

  void ConfigureDriverButtons();

  void ConfigureOperatorButtons();
};
