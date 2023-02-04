package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmForwardsEncoders extends CommandBase{

    // SUBSYSTEM
    private final ArmSubsystem armSubsystem;

    // SWITCHCASE 
    private int switchCase = 0;

    // ENCODERS
    private double enc;
    private double desiredEnc;

    // CLASS CONSTRUCTOR
    public ArmForwardsEncoders(ArmSubsystem armSubsystem, double desiredEnc){
        this.armSubsystem = armSubsystem;
        this.desiredEnc = desiredEnc;
        enc = armSubsystem.getArmEncoder();
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){

        // STOP ARM AND RESET ENCODER
        armSubsystem.stopArm();
        switchCase = 0;

    }

    @Override
    public void execute(){

        // PRINT TO SMARTDASHBOARD; UPDATE ENCODER VALUE
        SmartDashboard.putNumber("Arm Encoder Count", enc);
        SmartDashboard.putNumber("Arm SwitchCase", switchCase);
        enc = armSubsystem.getArmEncoder();

        switch (switchCase) {

            // IF ENCODER REACHES DESIRED VALUE -> STOP; ELSE ARM FORWARDS
            case 0:
                if (enc >= desiredEnc) {
                    switchCase++;
                    break;
                } else {
                    armSubsystem.armForward();
                }

            case 1:

            break;

            default:
        }

    }

    @Override
    public void end(boolean interrupted){
        
        armSubsystem.stopArm();
        switchCase = 0;
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
