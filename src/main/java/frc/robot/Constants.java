// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  // public static class OperatorConstants {
  //   public static final int kDriverControllerPort = 0;
  // }

  public static final class ModuleConstant
  {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1/5.8462;  //these numbers will differ cuz diff gear ratios
    public static final double kTurningMotorGearRatio = 1/18.0;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
    
    
  }

  public static final class DriveConstants
  {
    public static final double ktrackWidth = Units.inchesToMeters(21); //change accordingly; length b/w r/l wheels

    public static final double kWheelBase = Units.inchesToMeters(25.5); //length b/w f/b wheels

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
         new Translation2d(kWheelBase / 2, -ktrackWidth / 2), 
         new Translation2d(kWheelBase / 2, ktrackWidth /2),
         new Translation2d(-kWheelBase / 2, ktrackWidth/2),
         new Translation2d(-kWheelBase / 2, -ktrackWidth/2));


    public static final int kFrontLeftDriveMotorPort = 9; //change ports according to robot
    public static final int kBackLeftDriveMotorPort = 17; 
    public static final int kFrontRightDriveMotorPort = 10; 
    public static final int kBackRightDriveMotorPort = 18; 

    public static final int kFrontLeftTurningMotorPort = 14;
    public static final int kBackLeftTurningMotorPort = 6;
    public static final int kBackRightTurningMotorPort = 16;
    public static final int kFrontRightTurningMotorPort = 15;

    public static final boolean kFrontleftDriveEncoderReversed =  true;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kBackleftDriveEncoderReversed =   true;
    public static final boolean kBackRightDriveEncoderReversed =  true;

    public static final boolean kFrontleftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;//inversing direction
    public static final boolean kBackleftTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 50;//change ports accordingly
    public static final int kBackLeftDriveAbsoluteEncoderPort = 51;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 53;
    public static final int kBackRightDriveAbsoluteEncoderPort = 52;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;//change offset numbers accordingly
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;

    public static final boolean kBackleftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontleftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    
  }

  public static final class OIConstants
  {
    public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        
        public static final double kDeadband = 0.05;
  }
}

