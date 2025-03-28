// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/POVButton.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "commands/Intake.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  m_chooser.SetDefaultOption("None", "None");
  m_chooser.AddOption("leave", "leave");
  m_chooser.AddOption("side-auto", "side-auto");

  // Other Commands
  pathplanner::NamedCommands::registerCommand(
      "drive_switch", std::move(m_drive.SetGyro(180_deg)));
  pathplanner::NamedCommands::registerCommand("ElevatorL2", m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kL2));
  pathplanner::NamedCommands::registerCommand("StartIntaking", m_claw.SetIntakeTargetCMD(ClawSubsystem::IntakeState::kIntaking));
  pathplanner::NamedCommands::registerCommand("StopIntaking", m_claw.SetIntakeTargetCMD(ClawSubsystem::IntakeState::kStopped));
  pathplanner::NamedCommands::registerCommand("ElevatorIntake", m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kIntake));
  pathplanner::NamedCommands::registerCommand("ClawL2", m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kL2));
  pathplanner::NamedCommands::registerCommand("ClawIntake", m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kIntake));
  pathplanner::NamedCommands::registerCommand("ClawElevator", m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kElevator));
  pathplanner::NamedCommands::registerCommand("ElevatorAlgae1", m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kAlgae1));
  pathplanner::NamedCommands::registerCommand("ElevatorAlgae2", m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kAlgae2));
  pathplanner::NamedCommands::registerCommand("ClawAlgae", m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kCoral));
  pathplanner::NamedCommands::registerCommand("StartEjecting", m_claw.SetIntakeTargetCMD(ClawSubsystem::IntakeState::kEjecting));

  frc::SmartDashboard::PutData("PDP", &m_pdp);
  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);
  frc::SmartDashboard::PutData("Command Scheduler",
                               &frc2::CommandScheduler::GetInstance());
  //frc::SmartDashboard::PutBoolean("Operator Manual Mode", false);

  //frc::CameraServer::StartAutomaticCapture();

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
  /*
  m_driverController.LeftBumper().OnTrue(
    AutoAlign(&m_drive, false).ToPtr()
  );
  m_driverController.RightBumper().OnTrue(
    AutoAlign(&m_drive, true).ToPtr()
  );
  */
}

void RobotContainer::ConfigureOperatorButtons() {

    /* claw
   *///up
    frc2::POVButton(&m_operatorController.GetHID(), 90).OnTrue(m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kL2)); // right
    frc2::POVButton(&m_operatorController.GetHID(), 180).OnTrue(m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kIntake)); // down
    frc2::POVButton(&m_operatorController.GetHID(), 270).OnTrue(m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kElevator)); // left

    // intake
    m_operatorController.L1().OnTrue(Intake(&m_claw).ToPtr());
    m_operatorController.R1()
      .OnTrue(m_claw.SetIntakeTargetCMD(ClawSubsystem::IntakeState::kIntaking))
      .OnFalse(m_claw.SetIntakeTargetCMD(ClawSubsystem::IntakeState::kStopped));
    m_operatorController.R2()
      .OnTrue(m_claw.SetIntakeTargetCMD(ClawSubsystem::IntakeState::kEjecting))
      .OnFalse(m_claw.SetIntakeTargetCMD(ClawSubsystem::IntakeState::kStopped));

    // elevator
    m_operatorController.Cross().OnTrue(
      m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kElevator)
      .AndThen(m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kIntake))
    );
    m_operatorController.Circle().OnTrue(
      m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kElevator)
      .AndThen(m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kL1))
    );
    m_operatorController.Square().OnTrue(
      m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kElevator)
      .AndThen(m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kL2))
    );
    m_operatorController.Triangle().OnTrue(
      m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kElevator)
      .AndThen(m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kL3))
    );

    // algae
    m_operatorController.L3().OnTrue(
      m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kElevator).AndThen(
        m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kAlgae1).AndThen(
          m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kCoral)
        )
      )
    ); 
    m_operatorController.R3().OnTrue(
      m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kElevator).AndThen(
        m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kAlgae2).AndThen(
          m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kCoral)
        )
      )
    );
    //m_operatorController.LeftTrigger().OnTrue(m_claw.Intake());
    //m_operatorController.LeftBumper().OnTrue(m_claw.SetIntakeTargetCMD(ClawSubsystem::IntakeState::kStopped));
  
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  auto selected = m_chooser.GetSelected();
  if (selected == "None") {
    return frc2::cmd::None();
  } else {
    return pathplanner::PathPlannerAuto(selected).ToPtr();
  }
}

