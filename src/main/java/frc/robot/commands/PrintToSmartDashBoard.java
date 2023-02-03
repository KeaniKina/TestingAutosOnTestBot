package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class PrintToSmartDashBoard extends CommandBase{
    private final DrivetrainSubsystem drivetrainSubsystem;

    public PrintToSmartDashBoard(DrivetrainSubsystem drivetrainSubsystem){
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Encoder Count Test", drivetrainSubsystem.getEncoder());
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
