// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.auto.AutoManager;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.RobotState;
import frc.robot.lib.util.Pose;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.LoopController;
import frc.robot.loops.RobotStateLoop;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Vision;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  final SubsystemManager subManager = SubsystemManager.getInstance();
  final AutoManager autoManager = AutoManager.getInstance();

  final ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain");
  final NetworkTableEntry leftDistEntry = driveTab.add("Left Distance", -5000).getEntry();
  final NetworkTableEntry rightDistEntry = driveTab.add("Right Distance", -5000).getEntry();

  @Override
  public void robotInit() {
    RobotState.getInstance().reset(new Pose());
    Vision.getInstance().init();
    subManager.init();
    autoManager.InitChoices();
    LoopController.getInstance().register(Drive.getInstance().getVelocityPIDLoop());
    LoopController.getInstance().register(DriveLoop.getInstance());
    LoopController.getInstance().register(RobotStateLoop.getInstance());
    LoopController.getInstance().start();
  }
  @Override
  public void robotPeriodic() {
    subManager.updateShuffleboard();
    leftDistEntry.setDouble(DriveState.getInstance().getLeftDistanceInches());
    rightDistEntry.setDouble(DriveState.getInstance().getRightDistanceInches());
  }
  @Override
  public void autonomousInit() {
    autoManager.init();
  }
  @Override
  public void autonomousPeriodic() {
    subManager.run();
  }
  @Override
  public void teleopInit() {}
  @Override
  public void teleopPeriodic() {
    DriverInteraction.getInstance().run();
    subManager.run();
  }
  @Override
  public void disabledInit() {autoManager.stop();}
  @Override
  public void disabledPeriodic() {
    subManager.disable();
  }
  @Override
  public void testInit() {}
  @Override
  public void testPeriodic() {}
  @Override
  public void simulationInit() {}
  @Override
  public void simulationPeriodic() {}
}
