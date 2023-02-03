package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveForwardsEncoders extends CommandBase {
    // SUBSYSTEM
    private final DrivetrainSubsystem drivetrainSubsystem;
    private int switchCase = 0;
    private double enc;
    private double wantedEncoder;

    // CLASS CONSTRUCTOR
    public DriveForwardsEncoders(DrivetrainSubsystem drivetrainSubsystem, double wantedEncoder) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.wantedEncoder = wantedEncoder;
        enc = drivetrainSubsystem.getEncoder();
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("State", "Driving Forwards");
        drivetrainSubsystem.stop();
        switchCase = 0;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Encoder Count", drivetrainSubsystem.getEncoder());

        switch (switchCase) {

            case 0:
                if (enc >= wantedEncoder) {
                    switchCase++;
                    break;
                } else {
                    drivetrainSubsystem.forwards();
                }
        }

    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
        switchCase = 0;
        SmartDashboard.putString("State", "Stopped");
    }

    @Override
    public boolean isFinished() {
        return switchCase == 1;
    }

}
