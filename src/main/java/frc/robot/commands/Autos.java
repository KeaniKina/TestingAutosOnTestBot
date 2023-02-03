package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {

  public static CommandBase testAuto(DrivetrainSubsystem drivetrainSubsystem) {
    return Commands.sequence(drivetrainSubsystem.testAutoCommand(),new DriveBackwardsEncoders(drivetrainSubsystem, 50), new DriveForwardsEncoders(drivetrainSubsystem, 50) /*subsystem.exampleMethodCommand(), new ExampleCommand(subsystem)*/);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
