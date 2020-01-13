/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <wpi/math>

#include "rev/CANSparkMax.h"
#include "rev/CANPIDController.h"
#include "rev/SparkMax.h"

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel);
  frc::SwerveModuleState GetState() const;
  void SetDesiredState(const frc::SwerveModuleState& state);

 private:
  static constexpr double kWheelRadius = 0.0508; // constant_todo
  static constexpr int kEncoderResolution = 4096; //constant_todo

  static constexpr auto kModuleMaxAngularVelocity =
      wpi::math::pi * 1_rad_per_s;  // radians per second // constant_todo
  static constexpr auto kModuleMaxAngularAcceleration =
      wpi::math::pi * 2_rad_per_s / 1_s;  // radians per second^2 // constant_todo

  static constexpr units::revolutions_per_minute_t maxAngVel{kModuleMaxAngularVelocity};
  static constexpr units::revolutions_per_minute_per_second_t maxAngAccel kModuleMaxAngularAcceleration};

//   frc::PWMVictorSPX m_driveMotor;
//   frc::PWMVictorSPX m_turningMotor;
  rev::SparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  frc::Encoder m_driveEncoder{0, 1};
  frc::Encoder m_turningEncoder{2, 3};

  frc2::PIDController m_drivePIDController{1.0, 0, 0};

  rev::CANPIDController m_revTurningController;


  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      1.0,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
};
