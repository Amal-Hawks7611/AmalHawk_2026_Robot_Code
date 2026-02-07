package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OI;

public class IntakeArm extends SubsystemBase {
    public SparkMax leader_motor;
    public SparkMax slave_motor;
    public Timer timer;
    public boolean current_movement;

    public IntakeArm() {
        leader_motor = new SparkMax(Constants.IntakeArm.INTAKE_ARM_LEADER_PORT, MotorType.kBrushless);
        slave_motor = new SparkMax(Constants.IntakeArm.INTAKE_ARM_SLAVE_PORT, MotorType.kBrushless);
        leader_motor.getEncoder().setPosition(0);
        slave_motor.getEncoder().setPosition(0);
        current_movement = true;
    }

    public double getLeaderEncoder() {
        return -leader_motor.getEncoder().getPosition();
    }

    public double getSlaveEncoder() {
        return slave_motor.getEncoder().getPosition();
    }

    public void ArmUP() {
        leader_motor.set(Constants.IntakeArm.INTAKE_ARM_MANUAL_SPEED);
        slave_motor.set(Constants.IntakeArm.INTAKE_ARM_MANUAL_SPEED);
    }

    public void ArmDOWN() {
        leader_motor.set(-Constants.IntakeArm.INTAKE_ARM_MANUAL_SPEED);
        slave_motor.set(-Constants.IntakeArm.INTAKE_ARM_MANUAL_SPEED);
    }

    public void StopMotors() {
        leader_motor.set(0);
        slave_motor.set(0);
    }

    public void OcalPID() {
        if (Math.abs(
                Constants.IntakeArm.INTAKE_SETPOINT - getLeaderEncoder()) > Constants.IntakeArm.OCALPID_TOLERANCE_VALUE
                &&
                getLeaderEncoder() < Constants.IntakeArm.INTAKE_SETPOINT && OI.IS_PROCESSING) {
            leader_motor.set(-Constants.IntakeArm.INTAKE_OCALPID_SPEED);
            slave_motor.set(-Constants.IntakeArm.INTAKE_OCALPID_SPEED);
        } else {
            StopMotors();
            OI.IS_PROCESSING = false;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("IsIntakeArmProcessing", OI.IS_PROCESSING);
        SmartDashboard.putNumber("LeaderEncoder", getLeaderEncoder());
        SmartDashboard.putNumber("SlaveEncoder", getSlaveEncoder());
        SmartDashboard.putBoolean("CanMoveUp", frc.robot.Constants.IntakeArm.CAN_MOVE_UP);
        SmartDashboard.putBoolean("CanMoveDown", frc.robot.Constants.IntakeArm.CAN_MOVE_DOWN);
        double pos = getLeaderEncoder();

        if (pos < frc.robot.Constants.IntakeArm.TOP_BREAK) {
            frc.robot.Constants.IntakeArm.CAN_MOVE_UP = false;
            OI.IS_PROCESSING = false;
        } else {
            frc.robot.Constants.IntakeArm.CAN_MOVE_UP = true;
        }
        if (pos > frc.robot.Constants.IntakeArm.DOWN_BREAK) {
            frc.robot.Constants.IntakeArm.CAN_MOVE_DOWN = false;
            OI.IS_PROCESSING = false;
        } else {
            frc.robot.Constants.IntakeArm.CAN_MOVE_DOWN = true;
        }
        if(!OI.IS_PROCESSING){
            if(pos < frc.robot.Constants.IntakeArm.TOP_BREAK+2){
                leader_motor.set(-0.05);
                slave_motor.set(-0.05);
            }else{
                leader_motor.set(0);
                slave_motor.set(0);
            }
            if(pos > frc.robot.Constants.IntakeArm.DOWN_BREAK - 2){
                leader_motor.set(0.05);
                slave_motor.set(0.05);
            }
            else{
                leader_motor.set(0);
                slave_motor.set(0);
            }
        }
        // if (!OI.IS_PROCESSING) {
           // double time = Timer.getFPGATimestamp();
          //  double speed = -Constants.IntakeArm.INTAKE_PRD_SPEED * Math.sin(time * 4.0) * 0.7;
          //  if (speed > 0 && !frc.robot.Constants.IntakeArm.CAN_MOVE_UP) {
         //       speed = 0;
         //   }
        //    if (speed < 0 && !frc.robot.Constants.IntakeArm.CAN_MOVE_DOWN) {
        //        speed = 0;
        //    }
        //    leader_motor.set(speed);
        //    slave_motor.set(speed);
        // } 

    }
}
