package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveForwardsEncoders extends CommandBase {
    // SUBSYSTEM
    private final DrivetrainSubsystem drivetrainSubsystem;
    private int switchCase = 0;
    private double enc;
    private double desiredEnc;

    // CLASS CONSTRUCTOR
    public DriveForwardsEncoders(DrivetrainSubsystem drivetrainSubsystem, double desiredEnc) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        enc = drivetrainSubsystem.getEncoder();
        this.desiredEnc = desiredEnc;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        
        // PRINT TO SMARDASHBOARD; RESET DRIVETRAIN AND SWITCHCASE
        SmartDashboard.putString("State", "Driving Forwards");
        drivetrainSubsystem.stop();
        switchCase = 0;
    }

    @Override
    public void execute() {

        // PRINT TO SMARTDASHBOARD; UPDATE ENCODER VALUE
        SmartDashboard.putNumber("Encoder Count", drivetrainSubsystem.getEncoder());
        SmartDashboard.putNumber("SwitchCase", switchCase);
        enc = drivetrainSubsystem.getEncoder();

        switch (switchCase) {

            case 0:
                if (enc >= desiredEnc) {
                    switchCase++;
                    break;
                } else {
                    drivetrainSubsystem.drivePID(desiredEnc);
                }

            case 1:

            break;

            default:
        }

    }

    @Override
    public void end(boolean interrupted) {

        // RESET DRIVETRAIN AND SWITCHCASE
        drivetrainSubsystem.stop();
        switchCase = 0;
        SmartDashboard.putString("State", "Stopped");
    }

    @Override
    public boolean isFinished() {
        return switchCase == 1;
    }

}
