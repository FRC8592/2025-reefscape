// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveTrain driveTrain = new DriveTrain();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveController =
      new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
// // Control motor speed with the right controller stick as the default behavior 
//     driveTrain.setDefaultCommand(
//       driveTrain.run(() -> driveTrain.setPercentMotorOutput(driveController.getRightY()))
//       );

      // The B button on a Gamepad 0 wil set power to 50% 
    //Trigger halfPower = new JoystickButton(driveController.getHID(),XboxController.Button.kB.value);
    // halfPower.onTrue(run(() -> driveTrain.setPercentMotorOutput(0.5)));
    driveController.b().whileTrue(
      driveTrain.startEnd(
        () -> driveTrain.setPercentMotorOutput(0.5),
        () -> driveTrain.setPercentMotorOutput(0)
      )
    );

      // The Y button on a gamepad 0 will set the power to 10%
    driveController.y().onTrue(
      Commands.sequence(
        driveTrain.run(() -> driveTrain.setPercentMotorOutput(0.1)).withTimeout(3),
        driveTrain.run(() -> driveTrain.setPercentMotorOutput(0.5)).withTimeout(3),
        driveTrain.runOnce(() -> driveTrain.setPercentMotorOutput(0.0))
      )
    );
    //   driveTrain.runOnce(
    //     () -> driveTrain.setPercentMotorOutput(0.1)
    //   ).withTimeout(3).andThen(
    //     driveTrain.runOnce(
    //       () -> driveTrain.setPercentMotorOutput(0.5)
    //     )
    //   ).withTimeout(3).andThen(
    //     driveTrain.runOnce(
    //       () -> driveTrain.setPercentMotorOutput(0)
    //     )
    //   )
    // );

    //
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return Commands.none();
  }

}
