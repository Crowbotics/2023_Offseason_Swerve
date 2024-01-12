// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final Joystick m_driverGamepad = new Joystick(Constants.OIConstants.driverController);
  private final Joystick m_auxGamepad = new Joystick(Constants.OIConstants.auxController);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  //private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);
  private double maxSpeed = 10.0;
  private double maxAngularSpeed = 4* Math.PI;

  private Trajectory m_trajectory;
  private final RamseteController m_ramseteController = new RamseteController();
  private Field2d m_field;
  private Timer m_timer;

  private CANSparkMax intake = new CANSparkMax(10, MotorType.kBrushless);
  private CANSparkMax arm = new CANSparkMax(12, MotorType.kBrushless);

  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("Troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("Troubleshooting").getEntry("Y");

  String trajectoryJSON = "output/Unnamed.wpilib.json";

  public void robotInit()
  {
   
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }catch (IOException ex) {
      DriverStation.reportError("Unable to open Trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

                // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Push the trajectory to Field2d.
    m_field.getObject("traj").setTrajectory(m_trajectory);
  }

  public void robotPeriodic()
  {
    //SmartDashboard.putString("RL Position: ", m_swerve.m_backRight.getPosition().toString());

    if(m_driverGamepad.getRawButton(Constants.OIConstants.rightBumper))
      maxSpeed = 3.0;
    else
      maxSpeed = 10.0;

    SmartDashboard.putString("FL Current State: ", m_swerve.m_frontLeft.getState().toString());
    SmartDashboard.putString("FR Current State: ", m_swerve.m_frontRight.getState().toString());
    SmartDashboard.putString("BL Current State: ", m_swerve.m_backLeft.getState().toString());
    SmartDashboard.putString("BR Current State: ", m_swerve.m_backRight.getState().toString());

    m_swerve.updateOdometry();

    var translation = m_swerve.getOdometry().getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());

    if(m_driverGamepad.getRawButton(1))
    intake.set(0.5);
    else if (m_driverGamepad.getRawButton((4)))
    intake.set(-.5);
    else
    intake.set(0);

    if(m_driverGamepad.getRawButton(2))
    arm.set(0.3);
    else if (m_driverGamepad.getRawButton((3)))
    arm.set(-.3);
    else
    arm.set(0);

    
  }

  @Override
  public void autonomousInit() {
    // Initialize the timer.
    m_timer = new Timer();
    m_timer.start();

    // Reset the drivetrain's odometry to the starting pose of the trajectory.
    m_swerve.resetOdometry(m_trajectory.getInitialPose());
  }

  @Override
  public void autonomousPeriodic() {
    // Update odometry.
    m_swerve.updateOdometry();

    // Update robot position on Field2d.
    m_field.setRobotPose(m_swerve.getPose());

    if (m_timer.get() < m_trajectory.getTotalTimeSeconds()) {
      // Get the desired pose from the trajectory.
      var desiredPose = m_trajectory.sample(m_timer.get());

      // Get the reference chassis speeds from the Ramsete controller.
      var refChassisSpeeds = m_ramseteController.calculate(m_swerve.getPose(), desiredPose);

      // Set the linear and angular speeds.
      m_swerve.drive(refChassisSpeeds.vxMetersPerSecond,
        refChassisSpeeds.vyMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond, false, 5);
    } else {
      m_swerve.drive(0, 0,0,true,0);
    }
    
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);

    if(m_driverGamepad.getRawButton(Constants.OIConstants.A_BUTTON))
      m_swerve.resetGyro();
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driverGamepad.getRawAxis(Constants.OIConstants.leftStickYAxis), 0.1))
            * maxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driverGamepad.getRawAxis(Constants.OIConstants.leftStickXAxis), 0.1))
            * maxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -MathUtil.applyDeadband(m_driverGamepad.getRawAxis(Constants.OIConstants.rightStickXAxis), 0.1)
            * maxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, maxSpeed);
  }

  public double getMaxSpeed()
  {
    return maxSpeed;
  }
}
