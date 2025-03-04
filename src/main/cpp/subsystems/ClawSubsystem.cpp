// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClawSubsystem.h"
#include <rev/SparkMax.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

ClawSubsystem::ClawSubsystem() = default;

// This method will be called once per scheduler run
void ClawSubsystem::Periodic() {}

class Robot : public frc::TimedRobot {
public:
    ~Robot() noexcept override {
    }

private:

    rev::CANSparkMax m_pivotMotor{1, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxPIDController m_pidController = m_pivotMotor.GetPIDController();
    rev::SparkMaxRelativeEncoder m_encoder = m_pivotMotor.GetEncoder();
    frc::XboxController m_xboxController{0};
// Those should be the correct names I am unsure of why they are not registering properly
    double kP = 0.1;
    double kI = 0.0;
    double kD = 0.0;

    double m_targetPosition = 0.0;

public:

    Robot() {
        // Set PID constants
        m_pidController.SetP(kP);
        m_pidController.SetI(kI);
        m_pidController.SetD(kD);
    }
    
    void TeleopPeriodic() override {
        double pivotSpeed = m_xboxController.GetLeftY();

        if (std::abs(pivotSpeed) < 0.1) { // Deadzone of 10%
            pivotSpeed = 0.0;  // If joystick is near zero, stop the motor
        }

        // If we want to set a target position instead of just controlling speed:
        if (m_xboxController.GetAButtonPressed()) {
            // If the A button is pressed, we set a target position (e.g., 90 degrees)
            m_targetPosition = 90.0;  // Set to 90 degrees, for example
        }

        // Set the motor's target position using the PID controller (if targeting a position)
        m_pidController.SetReference(m_targetPosition, rev::CANSparkMax::ControlType::kPosition);
    }

};

int main(){
    return frc::StartRobot<Robot>();
}