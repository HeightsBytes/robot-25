// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClawSubsystem.h"

ClawSubsystem::ClawSubsystem() :
    m_intake(ClawConstants::kIntakeMotorPort, rev::spark::SparkMax::MotorType::kBrushless),
    m_pivot(ClawConstants::kPivotMotorPort, rev::spark::SparkMax::MotorType::kBrushless),
    m_pivotEncoder(m_pivot.GetEncoder()),
    m_pivotController(m_pivot.GetClosedLoopController()),
    m_pivotActual(PivotState::kSwitching),
    m_pivotTarget(PivotState::kIntake),
    m_sensor(ClawConstants::kSensorPort),
    m_intakeTarget(IntakeState::kStopped) {

        m_pivotConfig
            .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake)
            .Inverted(false)
            .SmartCurrentLimit(30);
        m_intakeConfig
            .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kCoast)
            .Inverted(false);
        m_pivotConfig.encoder
            .PositionConversionFactor(ClawConstants::kPivotEncoderRatio);
        m_pivotConfig.closedLoop
            .Pid(
                ClawConstants::kP, 
                ClawConstants::kI, 
                ClawConstants::kD)
            .PositionWrappingEnabled(false)
            .OutputRange(-.5, .5);

        m_intake.Configure(m_intakeConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
        m_pivot.Configure(m_pivotConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    }   

// This method will be called once per scheduler run
void ClawSubsystem::Periodic() {
    CheckState();

    m_pivotController.SetReference(StateToOutput(m_pivotTarget), 
        rev::spark::SparkMax::ControlType::kPosition);
    
    if(StateToOutput(m_intakeTarget) == ClawConstants::Speeds::kStopped){
        m_intake.StopMotor();
    } else {
        m_intake.Set(StateToOutput(m_intakeTarget));
    }

    frc::SmartDashboard::PutString("Claw Target", ToStr(m_pivotTarget));
    frc::SmartDashboard::PutString("Claw Actual", ToStr(m_pivotActual));
    frc::SmartDashboard::PutBoolean("sensor", GetSensor());
    //frc::SmartDashboard::PutBoolean("sensor", GetSensor());
}

double ClawSubsystem::StateToOutput(IntakeState state) const{
    using enum IntakeState;
    namespace IS = ClawConstants::Speeds;

    switch(state){
        case kStopped:
            return IS::kStopped;
            break;
        case kIntaking:
            return IS::kIntake;
            break;
        case kEjecting:
            return IS::kEject;
            break;
        default:
            return 0;
            break;
    }
}

double ClawSubsystem::StateToOutput(PivotState state) const {
    using enum PivotState;
    namespace PP = ClawConstants::PivotPositions;

    switch(state){
        case kIntake:
            return PP::kIntake;
            break;
        case kL2:
            return PP::kL2;
            break;
        case kElevator:
            return PP::kElevator;
            break;
        case kL1:
            return PP::kL1;
            break;
        case kCoral:
            return PP::kCoral;
            break;
        default:
            return 0;
            break;
    }
}

void ClawSubsystem::CheckState(){
    using enum PivotState;
    namespace PP = ClawConstants::PivotPositions;

    double angle = GetPivotAngle();

    if(frc::IsNear(PP::kIntake, angle, PP::kTollerance)){
        m_pivotActual =  kIntake;
        return;
    }
    if(frc::IsNear(PP::kL2, angle, PP::kTollerance)){
        m_pivotActual =  kL2;
        return;
    }
    if(frc::IsNear(PP::kElevator, angle, PP::kTollerance)){
        m_pivotActual =  kElevator;
        return;
    }
    if(frc::IsNear(PP::kL1, angle, PP::kTollerance)){
        m_pivotActual =  kL1;
        return;
    }
        if(frc::IsNear(PP::kCoral, angle, PP::kTollerance)){
        m_pivotActual =  kCoral;
        return;
    }

    m_pivotActual = kSwitching;
}

std::string ClawSubsystem::ToStr(IntakeState state) const{
    using enum IntakeState;
    
    switch(state){
        case kIntaking:
            return "Intaking";
            break;
        case kEjecting:
            return "Ejecting";
            break;
        case kStopped:
            return "Stopped";
            break;
    }
}

std::string ClawSubsystem::ToStr(PivotState state) const{
    using enum PivotState;

    switch(state){
        case kIntake:
            return "Intake";
            break;
        case kL2:
            return "L2";
            break;
        case kElevator:
            return "Elevator";
            break;
        case kL1:
            return "L1";
            break;
        case kCoral:
            return "Coral";
            break;
        case kSwitching:
            return "Switching";
            break;
        default:
            return "";
            break;
    }
}