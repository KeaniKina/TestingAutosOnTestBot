package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase{

    // MOTORS
    private final CANSparkMax armFront = new CANSparkMax(ArmConstants.armFrontPort, MotorType.kBrushless);
    private final CANSparkMax armBack = new CANSparkMax(ArmConstants.armBackPort, MotorType.kBrushless);

    // MOTOR CONTROLLER
    private final MotorControllerGroup arm = new MotorControllerGroup(armFront, armBack);

    // ENCODER
    private RelativeEncoder armRelEnc;

    
    // CONSTRUCTOR
    public ArmSubsystem(){
        armRelEnc = armFront.getEncoder();
    }

    // PERIODIC
    public void periodic(){
        SmartDashboard.putNumber("Arm Encoder", getArmEncoder());
    }

    ///////////////////////
    //      METHODS      //
    ///////////////////////

    // RETURNS ARM ENCODER
    public double getArmEncoder(){
        return armRelEnc.getPosition();
    }

    // RESETS ENCODER
    public void resetArmEncoder(){
        armRelEnc.setPosition(0);
    }

    // ARM FORWARD
    public void armForward(){
        arm.set(ArmConstants.motorSpeed);
    }

    // ARM REVERSE
    public void armReverse(){
        arm.set(-ArmConstants.motorSpeed);
    }

    // STOP ARM
    public void stopArm(){
        arm.set(0);
    }

}
