package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase{
    // MOTORS
    private final CANSparkMax leftFront = new CANSparkMax(DrivetrainConstants.leftFrontPort, MotorType.kBrushless);
    private final CANSparkMax leftback = new CANSparkMax(DrivetrainConstants.leftBackPort, MotorType.kBrushless);
    private final CANSparkMax rightFront = new CANSparkMax(DrivetrainConstants.rightFrontPort, MotorType.kBrushless);
    private final CANSparkMax rightBack = new CANSparkMax(DrivetrainConstants.rightBackPort, MotorType.kBrushless);

    // MOTOR CONTROLLER GROUPS
    private final MotorControllerGroup leftSide = new MotorControllerGroup(leftFront, leftback);
    private final MotorControllerGroup rightSide = new MotorControllerGroup(rightFront, rightBack);

    // ENCODDER
    private RelativeEncoder relEnc;

    // CONSTRUCTOR
    public DrivetrainSubsystem(){
        relEnc = leftFront.getEncoder();
    }

    // PERIODIC 
    public void periodic(){}

    // AUTO
    public CommandBase testAutoCommand(){
        return runOnce(
            () -> {
                resetEncoder();
            });
    }

    ///////////////////////
    //      METHODS     //
    ///////////////////////


    // RETURNS ENCODER
    public double getEncoder(){
        return relEnc.getPosition();
    }

    // RESETS ENCODER TO 0
    public void resetEncoder(){
        relEnc.setPosition(0);
    }

    // DRIVE FORWARDS
    public void forwards(){
        leftSide.set(DrivetrainConstants.motorSpeed);
        rightSide.set(-DrivetrainConstants.motorSpeed);
    }

    // DRIVE BACKWARDS
    public void backwards(){
        leftSide.set(-DrivetrainConstants.motorSpeed);
        rightSide.set(DrivetrainConstants.motorSpeed);
    }

    // TURN LEFT
    public void turnLeft(){
        leftSide.set(-DrivetrainConstants.motorSpeed);
        rightSide.set(-DrivetrainConstants.motorSpeed);
    }

    // TURN RIGHT
    public void turnRight(){
        leftSide.set(DrivetrainConstants.motorSpeed);
        rightSide.set(DrivetrainConstants.motorSpeed);
    }

    // STOP MOTORS
    public void stop(){
        leftSide.set(0);
        rightSide.set(0);
    }

}
