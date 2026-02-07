package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EnabledParts;
import frc.robot.Constants.OI;
import frc.robot.subsystems.IntakeArm;

public class ArmPID extends Command {
    public final IntakeArm intakeArm;

    public ArmPID(IntakeArm intakeArm) {
        this.intakeArm = intakeArm;
        addRequirements(intakeArm);
    }

    @Override
    public void initialize() {
        System.out.println("Intake PID Moving Ininialized");
        OI.IS_PROCESSING = true;

    }

    @Override
    public void execute() {
        if (EnabledParts.IS_INTAKE_ARM_ENABLED && frc.robot.Constants.IntakeArm.CAN_MOVE_DOWN) {
            intakeArm.OcalPID();
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("PID ENDED.");
        intakeArm.StopMotors();
        OI.IS_PROCESSING = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
