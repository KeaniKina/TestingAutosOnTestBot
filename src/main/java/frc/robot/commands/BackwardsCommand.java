package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BackwardsCommand extends CommandBase{

    // SUBSYSTEM
    private final DrivetrainSubsystem drivetrainSubsystem;

    // ENCODERS
    private double desiredEnc;
    private double enc;

    // SWITCHCASE
    private int switchCase;
    
    // CLASS CONSTRUCTOR
    public BackwardsCommand(DrivetrainSubsystem drivetrainSubsystem, double desiredEnc){
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.desiredEnc = desiredEnc;
        enc = drivetrainSubsystem.getLeftEncoder();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){

        drivetrainSubsystem.stop();
        switchCase = 0;
        SmartDashboard.putString("Robot State","Backwards");
    }

    @Override
    public void execute(){

        // PRINT TO SMARTDASHBOARD; UPDATE ENCODER VALUE
        SmartDashboard.putNumber("Encoder Count", enc);
        SmartDashboard.putNumber("SwitchCase", switchCase);
        enc = drivetrainSubsystem.getLeftEncoder();

        // IF ENCODER REACHES DESIRED VALUE -> STOP; ELSE DRIVE BACKWARDS
        switch (switchCase) {

            case 0:
                if (enc <= desiredEnc) {
                    switchCase++;
                    break;
                } else {
                    drivetrainSubsystem.backwards();
                }

            case 1:

            break;

            default:
        }
    }

    @Override
    public void end(boolean interrupted){

        drivetrainSubsystem.stop();
        switchCase = 0;
    }

    @Override
    public boolean isFinished(){
        return switchCase == 1;
    }
    
}
