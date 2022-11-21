// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.LoopController;
import frc.robot.subsystems.SubsystemManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  final SubsystemManager subManager = SubsystemManager.getInstance();

  @Override
  public void robotInit() {
    subManager.init();
    LoopController.getInstance().register(DriveLoop.getInstance());
    LoopController.getInstance().start();
  }
  @Override
  public void robotPeriodic() {
    subManager.updateShuffleboard();
  }
  @Override
  public void autonomousInit() {
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
  public void disabledInit() {}
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
