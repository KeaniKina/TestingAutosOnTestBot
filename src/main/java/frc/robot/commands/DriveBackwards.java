package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveBackwards extends CommandBase {
    // SUBSYSTEM
    private final DrivetrainSubsystem drivetrainSubsystem;
    private int switchCase = 0;
    private final double enc;
    private final double wantedEncoder;

    // CLASS CONSTRUCTOR
    public DriveBackwards(DrivetrainSubsystem drivetrainSubsystem, double wantedEncoder) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.wantedEncoder = -(Math.abs(wantedEncoder));
        enc = drivetrainSubsystem.getEncoder();
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("State", "Driving Backwards");
        drivetrainSubsystem.stop();
    }

    @Override
    public void execute() {

        switch (switchCase) {

            case 0:
                if (enc <= wantedEncoder) {
                    switchCase++;
                    break;
                } else {
                    drivetrainSubsystem.backwards();
                }
        }

    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
        SmartDashboard.putString("State", "Stopped");
    }

    @Override
    public boolean isFinished() {
        return switchCase == 1;
    }

}
