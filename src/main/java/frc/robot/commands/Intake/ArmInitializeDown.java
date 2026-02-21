package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EnabledParts;
import frc.robot.Constants.OI;
import frc.robot.subsystems.IntakeArm;

public class ArmInitializeDown extends Command {
    public final IntakeArm intakeArm;
    private boolean hasReachedPosition = false;

    public ArmInitializeDown(IntakeArm intakeArm) {
        this.intakeArm = intakeArm;
        addRequirements(intakeArm);
    }

    @Override
    public void initialize() {
        System.out.println("Intake Arm Initializing to Down Position");
        OI.IS_PROCESSING = true;
        hasReachedPosition = false;
    }

    @Override
    public void execute() {
        if (EnabledParts.IS_INTAKE_ARM_ENABLED) {
            double leaderPos = intakeArm.getLeaderEncoder();
            double slavePos = intakeArm.getSlaveEncoder();
            
            double leaderError = Math.abs(leaderPos - frc.robot.Constants.IntakeArm.Leader.DOWN_LIMIT);
            double slaveError = Math.abs(slavePos - frc.robot.Constants.IntakeArm.Slave.DOWN_LIMIT);
            
            if (leaderError < 0.1 && slaveError < 0.1) {
                intakeArm.StopMotors();
                hasReachedPosition = true;
                System.out.println("Arm reached down position");
            } else {
                if (leaderPos < frc.robot.Constants.IntakeArm.Leader.DOWN_LIMIT) {
                    intakeArm.leader_motor.set(-0.08); 
                } else {
                    intakeArm.leader_motor.set(0.08); 
                }
                
                if (slavePos < frc.robot.Constants.IntakeArm.Slave.DOWN_LIMIT) {
                    intakeArm.slave_motor.set(-0.08); 
                } else {
                    intakeArm.slave_motor.set(0.08); 
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Arm initialization completed");
        OI.IS_PROCESSING = false;
        intakeArm.setIsPidActive(false);
        intakeArm.StopMotors();
    }

    @Override
    public boolean isFinished() {
        return hasReachedPosition;
    }
}
