package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = 0.10033;
        public static final double kDriveMotorGearRatio = 1 / 6.86;
        public static final double kTurningMotorGearRatio = 1 /12.8;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.1;



    }

    public static final class DriveConstants{

        public static final double kTrackWidth = 0.5969; 
        public static final double kWheelBase = 0.5969;

        public static final double kMaxAngularSpeed = Math.PI;
        public static final double kMaxAngularVelocity = kMaxAngularSpeed * 2;
        public static final double kFastModeSpeed = 10.0;
        public static final double kSlowModeSpeed = 3.0;

        // frontLeft Module
        public static final int kFrontLeftDriveMotorChannel = 1;
        public static final int kFrontLeftTurningMotorChannel = 2;
        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kFrontLeftTurningMotorReversed = true;
        public static final int kFrontLeftTurnAbsoluteEncoderChannel = 22;
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = -1.612 * 180 / Math.PI;

        // frontRight Module
        public static final int kFrontRightDriveMotorChannel = 5;
        public static final int kFrontRightTurningMotorChannel = 6;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kFrontRightTurningMotorReversed = true;
        public static final int kFrontRightTurnAbsoluteEncoderChannel = 26;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = -2.393 * 180 / Math.PI;

        // backLeft Module
        public static final int kBackLeftDriveMotorChannel = 3;
        public static final int kBackLeftTurningMotorChannel = 4;
        public static final boolean kBackLeftDriveMotorReversed = true;
        public static final boolean kBackLeftTurningMotorReversed = true;
        public static final int kBackLeftTurnAbsoluteEncoderChannel = 24;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = 0.615 * 180 / Math.PI;

        // backRight Module
        public static final int kBackRightDriveMotorChannel = 7;
        public static final int kBackRightTurningMotorChannel = 8;
        public static final boolean kBackRightDriveMotorReversed = true;
        public static final boolean kBackRightTurningMotorReversed = true;
        public static final int kBackRightTurnAbsoluteEncoderChannel = 28;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;
        public static final double kBackRightDriveAbsoluteEncoderOffsetDeg =  0.097 * 180 / Math.PI;

        
        
        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * (ModuleConstants.kDriveMotorGearRatio) * ModuleConstants.kWheelDiameterMeters * Math.PI * 2;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0 * 3);

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final double kDeadband = 0.05;

        public static final int driverController = 0;
        public static final int auxController = 1;

        public static final int leftStickXAxis = 0;
        public static final int leftStickYAxis = 1;

        public static final int leftTrigger = 2;
        public static final int RightTrigger = 3;

        public static final int rightStickXAxis = 4;
        public static final int rightStickYAxis = 5;

        public static final int leftBumper = 5;
        public static final int rightBumper = 6;

        public static final int A_BUTTON = 1;
        public static final int B_BUTTON = 2;
        public static final int X_BUTTON = 3;
        public static final int Y_BUTTON = 4;
    }
}