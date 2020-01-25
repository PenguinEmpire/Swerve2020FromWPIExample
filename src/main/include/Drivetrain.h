/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/math>

#include "SwerveModule.h"

#include "AHRS.h"
#include "frc/SPI.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() {
    m_gyro->Reset();
  }

  /**
   * Get the robot angle as a Rotation2d.
   */
  frc::Rotation2d GetAngle() const {
    // Negating the angle because WPILib Gyros are CW positive.
    return frc::Rotation2d(units::degree_t(-m_gyro->GetAngle()));
  }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative); 
  void UpdateOdometry();

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::math::pi};  // 1/2 rotation per second

 private:
  frc::Translation2d m_frontLeftLocation{+0.2698_m, +0.3048_m};  // may reorient sidedness TODO
  frc::Translation2d m_frontRightLocation{+0.2698_m, -0.3048_m}; 
  frc::Translation2d m_backLeftLocation{-0.2698_m, +0.3048_m}; 
  frc::Translation2d m_backRightLocation{-0.2698_m, -0.3048_m};

  SwerveModule m_frontLeft{7, 8, 3};
  SwerveModule m_frontRight{5, 6, 2};
  SwerveModule m_backLeft{3, 4, 1};
  SwerveModule m_backRight{1, 2, 0};

  // frc::AnalogGyro m_gyro{0};

  AHRS* m_gyro = new AHRS(frc::SPI::Port::kMXP);

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, GetAngle()};
};
