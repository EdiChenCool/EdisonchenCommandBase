// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.DriverAction;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstant;

public class SwerveModule extends SubsystemBase {

  /** Creates a new SwerveModule. */
private final CANSparkMax driveMotor, turningMotor;
private final CANEncoder driveEncoder, turningEncoder;

private final PIDController turningPidController;

private final boolean absoluteEncoderReversed;
private final double absoluteEncoderOffsetRad;
private final CANcoder absoluteEncoder;

  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANcoder(absoluteEncoderId);
    
    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(ModuleConstant.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstant.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setPositionConversionFactor(ModuleConstant.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstant.kTurningEncoderRPM2RadPerSec);

    turningPidController = new PIDController(ModuleConstant.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    
    resetEncoders();
  }

  public double getDrivePosition()
  {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition()
  {
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity()
  {
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity()
  {
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad()
  {
    double angle = absoluteEncoder.getAbsolutePosition().getValue() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders()
  {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState()
  {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state)
  {
    if (Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
  }

  public void stop()
  {
    driveMotor.set(0);
    turningMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
