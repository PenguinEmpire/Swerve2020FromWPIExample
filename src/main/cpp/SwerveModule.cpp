/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>

#include "frc/AnalogInput.h"

#include <cmath>

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel, int analogEncoderChannel, double turningEncoderOffset)
  : m_driveMotor(driveMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
    m_turningMotor(turningMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
    m_turningEncoder{analogEncoderChannel, turningEncoderOffset, false},
    m_ID{analogEncoderChannel}
    {   
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  // m_driveEncoder.SetDistancePerPulse(2 * wpi::math::pi * kWheelRadius / kEncoderResolution);
  m_driveEncoder.SetVelocityConversionFactor(kWheelRadius * wpi::math::pi * 2 * (1/60));

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * wpi::math::pi)
  // divided by the encoder resolution.
  // m_turningEncoder.SetDistancePerPulse(2 * wpi::math::pi / kEncoderResolution);

  // Limit the PID Controller's input range between -pi and pi and set the input to be continuous.
  // m_turningPIDController.EnableContinuousInput(-units::radian_t(wpi::math::pi), units::radian_t(wpi::math::pi));
  m_turningPIDController.EnableContinuousInput(-wpi::math::pi, wpi::math::pi);
}

frc::SwerveModuleState SwerveModule::GetState() {//const {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& state) {
  const auto driveOutput = state.speed.to<double>();

  // Calculate the turning motor output from the turning PID controller.
  printf("ID %i turn measurement: %f\n", m_ID, units::radian_t(m_turningEncoder.Get()).to<double>());
  printf("ID %i turn measurement mod-ed: %f\n", m_ID, fmod(
      units::radian_t(m_turningEncoder.Get()).to<double>(),
      (2 * wpi::math::pi)
      ));
  printf("ID %i turn set point: %f\n", m_ID, state.angle.Radians().to<double>());
  const auto turnOutput = m_turningPIDController.Calculate(
      fmod(
        units::radian_t(m_turningEncoder.Get()).to<double>(),
        (2 * wpi::math::pi)
      ),
      state.angle.Radians().to<double>()
  );
 
  // Set the motor outputs.
  // m_driveMotor.Set(driveOutput);
  m_turningMotor.Set(turnOutput);
  printf("ID %i turn output: %f \n", m_ID, turnOutput);
  // printf(" ID %i drive output %f \n", m_id,  driveOutput);
}
