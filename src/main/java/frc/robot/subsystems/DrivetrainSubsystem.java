package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // DIFFERENTIAL DRIVE
    private final DifferentialDrive diffDrive = new DifferentialDrive(leftSide, rightSide);

    // ENCODERS
    private RelativeEncoder leftRelEnc;
    private RelativeEncoder rightRelEnc;

    // PID VARIABLES 
    private final PIDController drivePID = new PIDController(0.007, 0.0008, 0.0);
    private double errorPosition = 0;

    // NAVX
    private final AHRS navX;


    // CONSTRUCTOR
    public DrivetrainSubsystem(){
        leftRelEnc = leftFront.getEncoder();
        rightRelEnc = rightFront.getEncoder();
        drivePID.setTolerance(8);
        navX = new AHRS(SPI.Port.kMXP);
    }

    // PERIODIC 
    public void periodic(){
        SmartDashboard.putNumber("Speed", leftFront.get());
        SmartDashboard.putNumber("Left Side Encoder", getLeftEncoder());
        SmartDashboard.putNumber("Right Side Encoder", getRightEncoder());

        SmartDashboard.putNumber("Yaw", getYaw());
        SmartDashboard.putNumber("Current Angle", navX.getAngle());
        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putNumber("Roll", getRoll());
    }

    // AUTO
    public CommandBase testAutoCommand(){
        return runOnce(
            () -> {
                resetEncoders();
            });
    }

    ///////////////////////
    //      METHODS      //
    ///////////////////////

    // RETURNS YAW
    public double getYaw(){
        return navX.getYaw();
    }

    // ZERO YAW
    public void zeroYaw(){
        navX.zeroYaw();
    }

    // RETURNS ANGLE 
    public double getAngle(){
        return navX.getAngle();
    }

    // RETURNS PITCH
    public double getPitch(){
        return navX.getPitch();
    }

    // RETURNS ROLL
    public double getRoll(){
        return navX.getRoll();
    }

    // RETURNS SPEED
    public double getSpeed(){
        return leftFront.get();
    }

    // RETURNS LEFT SIDE ENCODER
    public double getLeftEncoder(){
        return leftRelEnc.getPosition();
    }

    // RETURNS RIGHT SIDE ENCODER
    public double getRightEncoder(){
        return -rightRelEnc.getPosition();
    }

    // RESETS ENCODER TO 0
    public void resetEncoders(){
        leftRelEnc.setPosition(0);
        rightRelEnc.setPosition(0);
    }

    // CALCULATING P
    public double calculateP(Double setPoint){
        double error = drivePID.calculate(getRightEncoder(), setPoint);

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

    // CONTROLING INTEGRAL
    public void controlI(){
        double currentPosition = drivePID.getPositionError();

        if (currentPosition > 0 && errorPosition < 0){
            drivePID.reset();
        }
        else if (currentPosition < 0 && errorPosition > 0){
            drivePID.reset();
        }
        errorPosition = drivePID.getPositionError();

        SmartDashboard.putNumber("ErrorPosition", errorPosition);
        SmartDashboard.putNumber("CurrentPosition", currentPosition);
    }


    // TANK DRIVE
    /*public void tankDrive(){
        diffDrive.tankDrive( , );
    }*/

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


    // PID DRIVE
    public void drivePID(double setPoint){
        leftSide.set(calculateP(setPoint));
        rightSide.set(-calculateP(setPoint));

        SmartDashboard.putNumber("Set Point", setPoint);
    }
}
