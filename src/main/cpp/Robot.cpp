/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include "frc/Joystick.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "SwerveModule.h"
#include "Drivetrain.h"

class Robot : public frc::TimedRobot {
 public:
  void AutonomousPeriodic() override {
    // DriveWithJoystick(false);
    m_swerve.UpdateOdometry();
  }

  void TeleopPeriodic() override {
    bool fieldRelative = !joy0.GetRawButton(4);
    DriveWithJoystick(fieldRelative);
  

    frc::SmartDashboard::PutNumber("FL Drive Encoder Velocity", m_swerve.m_frontLeft.getDriveEncoderVelocity());
    frc::SmartDashboard::PutNumber("FL Encoder Angle", m_swerve.m_frontLeft.getTurnEncoderAngle());

    frc::SmartDashboard::PutNumber("FR Drive Encoder Velocity", m_swerve.m_frontRight.getDriveEncoderVelocity());
    frc::SmartDashboard::PutNumber("FR Encoder Angle", m_swerve.m_frontRight.getTurnEncoderAngle());

    frc::SmartDashboard::PutNumber("BL Drive Encoder Velocity", m_swerve.m_backLeft.getDriveEncoderVelocity());
    frc::SmartDashboard::PutNumber("BL Encoder Angle", m_swerve.m_backLeft.getTurnEncoderAngle());

    frc::SmartDashboard::PutNumber("BR Encoder Angle", m_swerve.m_backRight.getTurnEncoderAngle());   
    frc::SmartDashboard::PutNumber("BR Drive Encoder Velocity", m_swerve.m_backRight.getDriveEncoderVelocity());
  }

 private:
  frc::Joystick joy0{0};
  frc::Joystick m_controller{2};
  // frc::XboxController m_controller{2};
  Drivetrain m_swerve;

  void DriveWithJoystick(bool fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed =
        -m_controller.GetRawAxis(5) * Drivetrain::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed =
        -m_controller.GetRawAxis(4) * Drivetrain::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -m_controller.GetRawAxis(0) *
                     Drivetrain::kMaxAngularSpeed;

    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
