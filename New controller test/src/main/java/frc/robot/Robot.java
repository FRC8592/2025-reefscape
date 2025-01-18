// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    //Button 1 is Y/Δ
  //Button 2 is B/O
  //Button 3 is A/X
  //Button 4 is X/□
  //Button 5 is LB/L1
  //Button 6 is RB/R1
  //Button 7 is LT/L2
  //Button 8 is RT/R2
  //Button 9 is SELECT
  //Button 10 is START
  //Button 13 is PS/HOME
  //There isn't a button 11 or button 12
  Logger.recordOutput("HID_B1", robotContainer.coralController.getRawButton(1));
  Logger.recordOutput("HID_B2", robotContainer.coralController.getRawButton(2));
  Logger.recordOutput("HID_B3", robotContainer.coralController.getRawButton(3));
  Logger.recordOutput("HID_B4", robotContainer.coralController.getRawButton(4));
  Logger.recordOutput("HID_B5", robotContainer.coralController.getRawButton(5));
  Logger.recordOutput("HID_B6", robotContainer.coralController.getRawButton(6));
  Logger.recordOutput("HID_B7", robotContainer.coralController.getRawButton(7));
  Logger.recordOutput("HID_B8", robotContainer.coralController.getRawButton(8));
  Logger.recordOutput("HID_B9", robotContainer.coralController.getRawButton(9));
  Logger.recordOutput("HID_B10", robotContainer.coralController.getRawButton(10));
  Logger.recordOutput("HID_B13", robotContainer.coralController.getRawButton(13));  
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
