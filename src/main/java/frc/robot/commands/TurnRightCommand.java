package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnRightCommand extends CommandBase{
    
    // SUBSYSTEM
    private final DrivetrainSubsystem drivetrainSubsystem;

    // ENCODERS
    private double desiredAngle;
    private double currentAngle;

    // SWITCHCASE
    private int switchCase;


    // CLASS CONSTRUCTOR
    public TurnRightCommand(DrivetrainSubsystem drivetrainSubsystem, double desiredTurn){
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.desiredAngle = (desiredTurn + drivetrainSubsystem.getAngle()) % 360;
        currentAngle = drivetrainSubsystem.getAngle();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){

        drivetrainSubsystem.stop();
        switchCase = 0;
        SmartDashboard.putString("Robot State","Turning Right");
    }

    @Override
    public void execute(){

        // PRINT TO SMARTDASHBOARD; UPDATE ANGLE VALUE
        SmartDashboard.putNumber("SwitchCase", switchCase);
        SmartDashboard.putNumber("Current Angle", currentAngle);

        currentAngle = drivetrainSubsystem.getAngle();

        switch (switchCase) {

            // IF ENCODER REACHES DESIRED VALUE -> STOP; ELSE TURN RIGHT
            case 0:
                if (currentAngle != desiredAngle) {
                    switchCase++;
                    break;
                } else {
                    drivetrainSubsystem.turnRight();
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
