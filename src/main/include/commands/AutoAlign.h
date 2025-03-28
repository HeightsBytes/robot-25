// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc2/command/CommandHelper.h>
#include <photon/PhotonCamera.h>
#include <units/angle.h>

#include "subsystems/DriveSubsystem.h"
#include "Constants.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoAlign
    : public frc2::CommandHelper<frc2::Command, AutoAlign> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  AutoAlign(DriveSubsystem* drive, bool kIsRight);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DriveSubsystem* m_drive;

  bool m_hasTarget;
  bool kIsRight;

  photon::PhotonCamera m_camera;
  photon::PhotonPipelineResult m_result;
  photon::PhotonTrackedTarget m_target;
  frc::Transform3d m_bestCameraToTarget;

  frc::HolonomicDriveController m_controller{
    m_xController, m_yController, m_rotController
  };

  frc::PIDController m_xController{AutoAlignConstants::kXP, AutoAlignConstants::kXI, AutoAlignConstants::kXD};
  frc::PIDController m_yController{AutoAlignConstants::kYP, AutoAlignConstants::kYI, AutoAlignConstants::kYD};
  frc::ProfiledPIDController<units::radian> m_rotController{
    AutoAlignConstants::kRotP, AutoAlignConstants::kRotI, AutoAlignConstants::kRotD,
    frc::TrapezoidProfile<units::radian>::Constraints{
      DriveConstants::kMaxAngularSpeed, DriveConstants::kMaxAngularAcceleration
    }
  };


  frc::Pose2d m_targetPose;
  frc::Pose2d m_robotPose;
};
