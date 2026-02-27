package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EnabledParts;
import frc.robot.Constants.IntakeArm.Leader;
import frc.robot.Constants.IntakeArm.Slave;
import frc.robot.Constants.OI;
import frc.robot.subsystems.IntakeArm;

public class ArmPIDUP extends Command {
    public final IntakeArm intakeArm;

    public ArmPIDUP(IntakeArm intakeArm) {
        this.intakeArm = intakeArm;
        addRequirements(intakeArm);
    }

    @Override
    public void initialize() {
        System.out.println("Intake PID Upwards Moving Ininialized");
        OI.IS_PROCESSING = true;
        intakeArm.setIsPidUpActive(true);
    }

    @Override
    public void execute() {
        if (EnabledParts.IS_INTAKE_ARM_ENABLED && intakeArm.getCanMoveUp()) {
            intakeArm.OcalPIDUP();
        } else {
            this.end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        OI.IS_PROCESSING = false;
        Leader.TOP_LIMIT = 0.05;
        Slave.TOP_LIMIT = 0.05;
        intakeArm.setIsPidUpActive(false);
        intakeArm.StopMotors();
    }

    @Override
    public boolean isFinished() {
        return !OI.IS_PROCESSING;
    }
}