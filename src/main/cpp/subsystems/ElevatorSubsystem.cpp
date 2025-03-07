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

    m_topLimitSwitch(ElevatorConstants::kTopLimitChannel),
    m_bottomLimitSwitch(ElevatorConstants::kBottomLimitChannel)
    {
    m_elevatorConfig
        .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kBrake)
        .Inverted(true);
    m_elevator2Config
        .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kBrake)
        .Inverted(false);

    m_elevatorConfig.encoder
        .PositionConversionFactor(ElevatorConstants::kElevatorEncoderRatio);
    m_elevator2Config.encoder
        .PositionConversionFactor(ElevatorConstants::kElevatorEncoderRatio);
    
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
        case kTop:
            return P::kTop;
            break;
        case kMiddleTop:
            return P::kMiddleTop;
            break;
        case kMiddleBottom:
            return P::kMiddleBottom;
            break;
        case kBottom:
            return P::kBottom;
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

    if(frc::IsNear(P::kTop, position, P::kTolerance)){
        m_actual = kTop;
        return;
    }
    if(frc::IsNear(P::kMiddleTop, position, P::kTolerance)){
        m_actual = kMiddleTop;
        return;
    }
    if(frc::IsNear(P::kMiddleBottom, position, P::kTolerance)){
        m_actual = kMiddleBottom;
        return;
    }
    if(frc::IsNear(P::kBottom, position, P::kTolerance)){
        m_actual = kBottom;
        return;
    }
    
}

std::string ElevatorSubsystem::ToStr(ElevatorState state) const {
    using enum ElevatorState;
    switch(state) {
        case kTop:
            return "Top";
            break;
        case kMiddleTop:
            return "Middle Top";
            break;
        case kMiddleBottom:
            return "Middle Bottom";
            break;
        case kBottom:
            return "Bottom";
            break;
        case kSwitching:
            return "Switching";
            break;
        default:
            return "Unknown";
            break;
    }
}
