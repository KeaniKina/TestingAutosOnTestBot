package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {

  public static CommandBase testAuto(DrivetrainSubsystem drivetrainSubsystem) {
    SmartDashboard.putString("Robot State", "Auto");
    drivetrainSubsystem.resetEncoder();
    return Commands.sequence(new DriveBackwardsEncoders(drivetrainSubsystem), new DriveForwardsEncoders(drivetrainSubsystem) /*subsystem.exampleMethodCommand(), new ExampleCommand(subsystem)*/);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
