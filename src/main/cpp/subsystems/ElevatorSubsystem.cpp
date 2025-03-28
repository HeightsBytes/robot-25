// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem() :
    m_elevator(ElevatorConstants::kElevatorMotorPort, rev::spark::SparkFlex::MotorType::kBrushless),
    m_elevator2(ElevatorConstants::kElevatorMotor2Port, rev::spark::SparkFlex::MotorType::kBrushless),

    m_elevatorEncoder(m_elevator.GetEncoder()),
    m_elevator2Encoder(m_elevator2.GetEncoder()),

    m_elevatorController(m_elevator.GetClosedLoopController()),
    m_elevator2Controller(m_elevator2.GetClosedLoopController()),
/*
    m_topLimitSwitch(ElevatorConstants::kTopLimitChannel),
    m_bottomLimitSwitch(ElevatorConstants::kBottomLimitChannel),
*/
    m_target(ElevatorState::kIntake)
    {
    m_elevatorConfig
        .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kCoast)
        .Inverted(true);
    m_elevator2Config
        .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kCoast)
        .Inverted(false);

    //m_elevatorConfig.encoder
        //.PositionConversionFactor(ElevatorConstants::kElevatorEncoderRatio);
    //m_elevator2Config.encoder
        //.PositionConversionFactor(ElevatorConstants::kElevatorEncoderRatio);
    
    m_elevatorConfig.closedLoop
        .PositionWrappingEnabled(false)
        .Pid(ElevatorConstants::kP, ElevatorConstants::kI, ElevatorConstants::kD);
    m_elevator2Config.closedLoop
        .PositionWrappingEnabled(false)
        .Pid(ElevatorConstants::kP, ElevatorConstants::kI, ElevatorConstants::kD);

    m_elevator.Configure(m_elevatorConfig, rev::spark::SparkFlex::ResetMode::kResetSafeParameters, rev::spark::SparkFlex::PersistMode::kPersistParameters);
    m_elevator2.Configure(m_elevator2Config, rev::spark::SparkFlex::ResetMode::kResetSafeParameters, rev::spark::SparkFlex::PersistMode::kPersistParameters);
        
}

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {
    CheckState();

    m_elevatorController.SetReference(StateToOutput(m_target), rev::spark::SparkFlex::ControlType::kPosition);
    m_elevator2Controller.SetReference(StateToOutput(m_target), rev::spark::SparkFlex::ControlType::kPosition);

    frc::SmartDashboard::PutString("Elevator Position", ToStr(m_actual));
    frc::SmartDashboard::PutString("Elevator Target", ToStr(m_target));
}

double ElevatorSubsystem::StateToOutput(ElevatorState state) const{
    using enum ElevatorState;
    namespace P = ElevatorConstants::Positions;

    switch(state){
        case kAlgae1:
            return P::kAlgae1;
            break;
        case kAlgae2:
            return P::kAlgae2;
            break;
        case kL3:
            return P::kL3;
            break;
        case kL2:
            return P::kL2;
            break;
        case kL1:
            return P::kL1;
            break;
        case kIntake:
            return P::kIntake;
            break;
        default:
            return 0;
            break;
    }
}

void ElevatorSubsystem::CheckState(){
    using enum ElevatorState;
    namespace P = ElevatorConstants::Positions;

    double position = m_elevatorEncoder.GetPosition();

    if(frc::IsNear(P::kAlgae1, position, P::kTolerance)){
        m_actual = kAlgae1;
        return;
    }
    if(frc::IsNear(P::kAlgae2, position, P::kTolerance)){
        m_actual = kAlgae2;
        return;
    }
    if(frc::IsNear(P::kL3, position, P::kTolerance)){
        m_actual = kL3;
        return;
    }
    if(frc::IsNear(P::kL2, position, P::kTolerance)){
        m_actual = kL2;
        return;
    }
    if(frc::IsNear(P::kL1, position, P::kTolerance)){
        m_actual = kL1;
        return;
    }
    if(frc::IsNear(P::kIntake, position, P::kTolerance)){
        m_actual = kIntake;
        return;
    }
    
}

std::string ElevatorSubsystem::ToStr(ElevatorState state) const {
    using enum ElevatorState;
    switch(state) {
        case kAlgae1:
            return "Algae1";
            break;
        case kAlgae2:
            return "Algae2";
            break;
        case kL3:
            return "L3";
            break;
        case kL2:
            return "L2";
            break;
        case kL1:
            return "L1";
            break;
        case kIntake:
            return "Intake";
            break;
        case kSwitching:
            return "Switching";
            break;
        default:
            return "Unknown";
            break;
    }
}
/*
ElevatorSubsystem::ElevatorState ElevatorSubsystem::GetNextState(ElevatorState state) {
    using enum ElevatorState;
    switch(state) {
        case kAlgae1:
            return kAlgae1;
            break;
        case kL3:
            return kAlgae;
            break;
        case kL2:
            return kL3;
            break;
        case kL1:
            return kL2;
            break;
        case kIntake:
            return kL1;
            break;
        default:
            return kIntake;
            break;
    }
}

ElevatorSubsystem::ElevatorState ElevatorSubsystem::GetPreviousState(ElevatorState state) {
    using enum ElevatorState;
    switch(state) {
        case kAlgae:
            return kL3;
            break;
        case kL3:
            return kL2;
            break;
        case kL2:
            return kL1;
            break;
        case kL1:
            return kIntake;
            break;
        case kIntake:
            return kIntake;
            break;
        default:
            return kIntake;
            break;
    }
}
*/