// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  //public static final double kMaxSpeed = 10.0; // 3 meters per second
  //public static final double kMaxAngularSpeed = 2*Math.PI; // 1/2 rotation per second

  final SwerveModule m_frontLeft = new SwerveModule(
    Constants.DriveConstants.kFrontLeftDriveMotorChannel,
    Constants.DriveConstants.kFrontLeftTurningMotorChannel,
    Constants.DriveConstants.kFrontLeftTurnAbsoluteEncoderChannel,
    -Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg);

  final SwerveModule m_frontRight = new SwerveModule(
    Constants.DriveConstants.kFrontRightDriveMotorChannel,
    Constants.DriveConstants.kFrontRightTurningMotorChannel,
    Constants.DriveConstants.kFrontRightTurnAbsoluteEncoderChannel,
    -Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg);

  final SwerveModule m_backLeft = new SwerveModule(
    Constants.DriveConstants.kBackLeftDriveMotorChannel,
    Constants.DriveConstants.kBackLeftTurningMotorChannel,
    Constants.DriveConstants.kBackLeftTurnAbsoluteEncoderChannel,
    -Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg);

  final SwerveModule m_backRight = new SwerveModule(
    Constants.DriveConstants.kBackRightDriveMotorChannel,
    Constants.DriveConstants.kBackRightTurningMotorChannel,
    Constants.DriveConstants.kBackRightTurnAbsoluteEncoderChannel,
    -Constants.DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg);

  private final AHRS m_gyro = new AHRS();

  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.DriveConstants.kWheelBase / 2, Constants.DriveConstants.kTrackWidth / 2), //front left
            new Translation2d(Constants.DriveConstants.kWheelBase / 2, -Constants.DriveConstants.kTrackWidth / 2), //front right
            new Translation2d(-Constants.DriveConstants.kWheelBase / 2, Constants.DriveConstants.kTrackWidth / 2), //back left
            new Translation2d(-Constants.DriveConstants.kWheelBase / 2, -Constants.DriveConstants.kTrackWidth / 2)); //backright

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double maxSpeed) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    SmartDashboard.putNumber("Gyro: ", getAngle());
    SmartDashboard.putString("Rot 2D:", m_gyro.getRotation2d().toString());
    SmartDashboard.putString("FL Desired State: ", swerveModuleStates[0].toString());
    SmartDashboard.putString("FR Desired State: ", swerveModuleStates[1].toString());
    SmartDashboard.putString("BL Desired State: ", swerveModuleStates[2].toString());
    SmartDashboard.putString("BR Desired State: ", swerveModuleStates[3].toString());
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public double getAngle()
  {
    return -m_gyro.getYaw();
  }

  public void resetGyro()
  {
    m_gyro.zeroYaw();
  }

    /**
   * Resets the field-relative position to a specific location.
   *
   * @param pose The position to reset to.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()},
        pose);
  }

  /**
   * Returns the pose of the robot.
   *
   * @return The pose of the robot.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public SwerveDriveOdometry getOdometry()
  {
    return m_odometry;
  }
}
