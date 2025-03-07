// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  m_chooser.SetDefaultOption("None", "None");
  m_chooser.AddOption("New Auto", "New Auto");
  m_chooser.AddOption("super cool auto", "super cool auto");

  // Other Commands
  pathplanner::NamedCommands::registerCommand(
      "drive_switch", std::move(m_drive.SetGyro(180_deg)));

  frc::SmartDashboard::PutData("PDP", &m_pdp);
  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);
  frc::SmartDashboard::PutData("Command Scheduler",
                               &frc2::CommandScheduler::GetInstance());
  frc::SmartDashboard::PutBoolean("Operator Manual Mode", false);

  // Configure the button bindings
  ConfigureDriverButtons();
  ConfigureOperatorButtons();

  // Uses right trigger + left stick axis + right stick axis
  m_drive.SetDefaultCommand(DefaultDrive(
      &m_drive, [this] { return m_driverController.GetLeftX(); },
      [this] { return m_driverController.GetLeftY(); },
      [this] { return m_driverController.GetRightX(); },
      [this] { return m_driverController.GetRightTriggerAxis(); }));
}


void RobotContainer::ConfigureDriverButtons() {
  m_driverController.A().OnTrue(frc2::cmd::Print("Example!"));
}

void RobotContainer::ConfigureOperatorButtons() {
  if(!frc::SmartDashboard::GetBoolean("Operator Manual Mode", false)){
    m_operatorController.A().OnTrue(m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kMiddleBottom));
    m_operatorController.B().OnTrue(m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kBottom));
    m_operatorController.X().OnTrue(m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kMiddleTop));
    m_operatorController.Y().OnTrue(m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kTop));

    
  }
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  auto selected = m_chooser.GetSelected();
  if (selected == "None") {
    return frc2::cmd::None();
  } else {
    return pathplanner::PathPlannerAuto(selected).ToPtr();
  }
}
