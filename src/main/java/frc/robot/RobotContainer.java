package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.DriveBackwardsPID;
import frc.robot.commands.DriveForwardsPID;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
public class RobotContainer {

  // SUBSYSTEM
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

  // CONTROLLER
  private final XboxController xboxController = new XboxController(1);

  // DEFAULT COMMANDS
  public RobotContainer() {

    configureBindings();
  }

  // CONFIGURING BUTTONS
  private void configureBindings() {

    new JoystickButton(xboxController, 1).onTrue(new DriveBackwardsPID(drivetrainSubsystem, -50));
    new JoystickButton(xboxController, 4).onTrue(new DriveForwardsPID(drivetrainSubsystem, 50));
    
  }

  // AUTONOMOUS CODE
  public Command getAutonomousCommand() {
    return Autos.drivetrainTestAuto(drivetrainSubsystem);
  }
}
