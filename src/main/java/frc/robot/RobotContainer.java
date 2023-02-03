package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.DriveBackwardsEncoders;
import frc.robot.commands.DriveForwardsEncoders;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
public class RobotContainer {

  // SUBSYSTEM
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

  // CONTROLLER
  private final XboxController xboxController = new XboxController(1);

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {

    new JoystickButton(xboxController, 1).onTrue(new DriveBackwardsEncoders(drivetrainSubsystem, 50));
    new JoystickButton(xboxController, 4).onTrue(new DriveForwardsEncoders(drivetrainSubsystem, 50));
    
  }

  public Command getAutonomousCommand() {
    return Autos.testAuto(drivetrainSubsystem);
  }
}
