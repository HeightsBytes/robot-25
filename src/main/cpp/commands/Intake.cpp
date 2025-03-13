// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake.h"


Intake::Intake(ClawSubsystem* claw) : m_claw(claw){
  AddRequirements(claw);
}

// Called when the command is initially scheduled.
void Intake::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Intake::Execute() {
  m_claw->SetIntakeTarget(ClawSubsystem::IntakeState::kIntaking);
}

// Called once the command ends or is interrupted.
void Intake::End(bool interrupted) {
  m_claw->SetIntakeTarget(ClawSubsystem::IntakeState::kStopped);
}

// Returns true when the command should end.
bool Intake::IsFinished() {
  if(m_claw->GetSensor()){
    return false;
  } else {
    return true;
  }
}
