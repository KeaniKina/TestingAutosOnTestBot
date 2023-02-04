package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {

  public static CommandBase drivetrainTestAuto(DrivetrainSubsystem drivetrainSubsystem) {

    // PRINTS STATE TO AUTO; RESET ENCODERS; 
    SmartDashboard.putString("Robot State", "Auto");
    drivetrainSubsystem.resetEncoders();

    // Balancing on Charge Station
    // return Commands.sequence();
    return (new BalanceOnChargeStationCommand(drivetrainSubsystem));
  }

  /*public static CommandBase armTestAuto(ArmSubsystem armSubsystem) {

    // PRINTS STATE TO AUTO; RESET ENCODERS
    SmartDashboard.putString("Robot State", "Auto");
    armSubsystem.resetArmEncoder();

    // GO FORWARDS FOR 5 SECONDS
    return Commands.deadline(null, null)
  }*/

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
