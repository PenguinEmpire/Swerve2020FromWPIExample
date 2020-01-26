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
#include "rev/CANEncoder.h"
#include "rev/SparkMax.h"
#include "frc/AnalogEncoder.h"
#include "frc/AnalogInput.h"

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel, int analogEncoderChannel);
  frc::SwerveModuleState GetState();
  void SetDesiredState(const frc::SwerveModuleState& state);

  double getDriveEncoderVelocity() {
    return SwerveModule::m_driveEncoder.GetVelocity();


  }
  double getTurnEncoderAngle() {

    return units::radian_t(SwerveModule::m_turningEncoder.Get()).to<double>();

  }


 private:
  static constexpr double kWheelRadius = 0.1016; // 4in in meters. used to be 0508.
  static constexpr int kEncoderResolution = 4096; //constant_todo

  static constexpr auto kModuleMaxAngularVelocity =
      wpi::math::pi * 1_rad_per_s;  // radians per second // constant_todo
  static constexpr auto kModuleMaxAngularAcceleration =
      wpi::math::pi * 2_rad_per_s / 1_s;  // radians per second^2 // constant_todo

  static constexpr units::revolutions_per_minute_t maxAngVel{kModuleMaxAngularVelocity};

  static constexpr double radPerSecToRevPerMinRatio = 9.5492965; // ration to convert from rad/sec to rev/min; eq to 30/pi

  static constexpr double moduleMaxAngularAccelerationRevPerMinPerSec = kModuleMaxAngularAcceleration.to<double>() * radPerSecToRevPerMinRatio;



  // frc::PWMVictorSPX m_driveMotor;
  // frc::PWMVictorSPX m_turningMotor;
  // frc::Encoder m_driveEncoder{0, 1};
  // frc::Encoder m_turningEncoder{2, 3};
  // frc2::PIDController m_drivePIDController{1.0, 0, 0};
//   frc::ProfiledPIDController<units::radians> m_turningPIDController{
//       1.0,
//       0.0,
//       0.0,
//       {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};


  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  frc2::PIDController m_turningPIDController{1.5, 0, 0.5}; 


  frc::AnalogEncoder m_turningEncoder;



  rev::CANEncoder m_driveEncoder = m_driveMotor.GetEncoder();

//  rev::CANPIDController m_revTurningController{m_driveMotor};


};
