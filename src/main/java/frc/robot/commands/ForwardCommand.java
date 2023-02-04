package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ForwardCommand extends CommandBase{

    // SUBSYSTEM
    private final DrivetrainSubsystem drivetrainSubsystem;

    // ENCODERS
    private double desiredEnc;
    private double enc;

    // SWITCHCASE
    private int switchCase = 0;


    //CLASS CONSTRUCTOR
    public ForwardCommand(DrivetrainSubsystem drivetrainSubsystem, double desiredEnc){
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.desiredEnc = desiredEnc;
        enc = drivetrainSubsystem.getLeftEncoder();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){

        drivetrainSubsystem.stop();
        switchCase = 0;
        SmartDashboard.putString("Robot State","Forwards");
    }

    @Override
    public void execute(){

        // PRINT TO SMARTDASHBOARD; UPDATE ENCODER VALUE
        SmartDashboard.putNumber("Encoder Count", enc);
        SmartDashboard.putNumber("SwitchCase", switchCase);
        enc = drivetrainSubsystem.getLeftEncoder();

        switch (switchCase) {

            // IF ENCODER REACHES DESIRED VALUE -> STOP; ELSE DRIVE FORWARDS
            case 0:
                if (enc >= desiredEnc) {
                    switchCase++;
                    break;
                } else {
                    drivetrainSubsystem.forwards();
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
