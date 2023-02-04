package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceOnChargeStationCommand extends CommandBase{
    private final DrivetrainSubsystem drive;
    private final PIDController pid; 

    public BalanceOnChargeStationCommand(DrivetrainSubsystem newDrive) {
        drive = newDrive; 
        pid = new PIDController(0, 0, 0); 

        addRequirements(drive); 
    }

    public double calculateP(){
        double error = pid.calculate(drive.getPitch(), 0);

        if (error > 1){
            return 1;
        }
        else if (error < -1){
            return -1;
        }
        else{
            return error;
        }
    }

    public void controlI(){
        double currentPosition = pid.getPositionError();
        double errorPosition = 0;

        if (currentPosition > 0 && errorPosition < 0){
            pid.reset();
        }
        else if (currentPosition < 0 && errorPosition > 0){
            pid.reset();
        }
        errorPosition = pid.getPositionError();

        SmartDashboard.putNumber("ErrorPosition", errorPosition);
        SmartDashboard.putNumber("CurrentPosition", currentPosition);
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("P", calculateP());
    }


    

}
