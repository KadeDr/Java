// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import java.util.List;
import java.util.ArrayList;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.apriltag.AprilTagPoseEstimate.*;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.RobotContainer;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.geometry.Pose3d;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;


  private RobotContainer m_robotContainer;
  private PhotonCamera camera;
  
  private DriveSubsystem driveSubsystem;

  private static final double CAMERA_HEIGHT_METERS = 0.6;
  private static final double TARGET_HEIGHT_METERS = 1.5;
  private static final double CAMERA_PITCH_RADIANS = Math.toRadians(20);
  private static final double FIELD_TAG_POSITION_X = 3.0;
  private static final double FIELD_TAG_POSITION_Y = 2.0;

  AprilTagFieldLayout aprilTagFieldLayout;
  ArrayList <AprilTag> ApriList;
  double fieldlength = 651.25;
  double fieldwidth = 315.5;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    driveSubsystem = new DriveSubsystem();
  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    var result = camera.getLatestResult();


    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();


      int targetId = target.getFiducialId();
      double yaw = target.getYaw();
      double pitch = target.getPitch();
      double area = target.getArea();
      double skew = target.getSkew();
      Transform3d pose = target.getBestCameraToTarget();
      List<TargetCorner> corners = target.getDetectedCorners();




      double range = PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS,
        TARGET_HEIGHT_METERS,
        CAMERA_PITCH_RADIANS,
        Math.toRadians(pitch)
      );


      double robotX = FIELD_TAG_POSITION_X - range * Math.cos(Math.toRadians(yaw));
      double robotY = FIELD_TAG_POSITION_Y - range * Math.sin(Math.toRadians(yaw));


      SmartDashboard.putNumber("Target ID", targetId);
      SmartDashboard.putNumber("Yaw", yaw);
      SmartDashboard.putNumber("Pitch",pitch);
      SmartDashboard.putNumber("Range", range);
      SmartDashboard.putNumber("Robot X", robotX);
      SmartDashboard.putNumber("Robot Y", robotY);

      // Current Test for Estimating Field Relative Pose with April Tags
      if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent())
      {
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), pose);
      }
    }
    else {
      SmartDashboard.putString("Status", "No targets detected");
    }



    



    }
 


  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    driveSubsytem.setX();
  }


  @Override
  public void disabledPeriodic() {}


  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();


    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */


    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }


  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}


  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }


  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}



