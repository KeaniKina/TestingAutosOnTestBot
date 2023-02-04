package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {

  public static CommandBase testAuto(DrivetrainSubsystem drivetrainSubsystem) {

    // PRINTS STATE TO AUTO; RESET ENCODERS; 
    SmartDashboard.putString("Robot State", "Auto");
    drivetrainSubsystem.resetEncoders();

    // DRIVE BACKWARDS TO -20; DRIVE FORWARDS TO 20
    return Commands.sequence(new DriveBackwardsEncoders(drivetrainSubsystem, -50), new DriveForwardsEncoders(drivetrainSubsystem, 50));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
