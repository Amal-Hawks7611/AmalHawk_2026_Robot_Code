
package frc.robot;

import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StatusLED;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Controlls;
import frc.robot.commands.Feeder.Feeder;
import frc.robot.commands.Intake.ArmDOWN;
import frc.robot.commands.Intake.ArmUP;
import frc.robot.commands.Intake.ArmPID;
import frc.robot.commands.Intake.ArmInitializeDown;
import frc.robot.commands.Intake.Intake;
import frc.robot.commands.Led.LEDMorseScroller;
import frc.robot.commands.Led.LEDStateCycler;
import frc.robot.commands.Shooter.stage1;
import frc.robot.commands.Shooter.stage2;
import frc.robot.commands.Shooter.stage3;
import frc.robot.commands.Trajectory.AutonPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import java.io.File;

import swervelib.SwerveInputStream;

public class RobotContainer {

        private final SendableChooser<Command> autoChooser;
        public final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));
        public final LimelightSubsystem limelight = new LimelightSubsystem();
        public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(this);
        public final StatusLED ledSubsystem = new StatusLED();
        public final IntakeArm intakeArm = new IntakeArm();
        public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(this);
        public final FeederSubsystem feederSubsystem = new FeederSubsystem(this);
        public final AutonPath otonom_path = new AutonPath();
        public CommandPS5Controller driverPs5 = Controlls.DRIVER_CONTROLLER;

        public final Command zerogyro;
        public final Command admin;

        public final Intake f_intake;
        public final ArmUP arm_up;
        public final ArmDOWN arm_down;
        public final ArmPID arm_pid;
        public final ArmInitializeDown arm_initialize_down;

        public final Feeder feed;
        public final stage1 shooterStage1;
        public final stage2 shooterStage2;
        public final stage3 shooterStage3;

        public final ParallelCommandGroup feedandshoots1;
        public final ParallelCommandGroup feedandshoots2;
        public final ParallelCommandGroup feedandshoots3;

        public final LEDStateCycler led_cycle;
        public final LEDMorseScroller led_morse;

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverPs5.getLeftY(),
                        () -> driverPs5.getLeftX())
                        .withControllerRotationAxis(() -> -driverPs5.getRawAxis(
                                        2))
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        public RobotContainer() {
                DriverStation.silenceJoystickConnectionWarning(true);
                NamedCommands.registerCommand("fIntake", new Intake(intakeSubsystem));
                zerogyro = new InstantCommand(() -> drivebase.zeroGyro());
                admin = new InstantCommand(() -> CommandScheduler.getInstance().cancelAll());

                led_cycle = new LEDStateCycler(ledSubsystem);
                led_morse = new LEDMorseScroller(ledSubsystem, 180, "AMAL IN DA HOUSE");

                f_intake = new Intake(intakeSubsystem);
                arm_down = new ArmDOWN(intakeArm);
                arm_up = new ArmUP(intakeArm);
                arm_pid = new ArmPID(intakeArm);
                arm_initialize_down = new ArmInitializeDown(intakeArm);

                feed = new Feeder(feederSubsystem);
                shooterStage1 = new stage1(shooterSubsystem);
                shooterStage2 = new stage2(shooterSubsystem);
                shooterStage3 = new stage3(shooterSubsystem);

                feedandshoots1 = new ParallelCommandGroup(new Feeder(feederSubsystem), new stage1(shooterSubsystem));
                feedandshoots2 = new ParallelCommandGroup(
                                new Feeder(feederSubsystem), new stage2(shooterSubsystem));
                feedandshoots3 = new ParallelCommandGroup(
                                new Feeder(feederSubsystem), new stage3(shooterSubsystem));

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
                configureBindings();
        }

        private void configureBindings() {
                Command driveFieldOrientedCommand = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
                drivebase.setDefaultCommand(driveFieldOrientedCommand);
        }

        public void configureButtonBindings() {
                Controlls.INTAKE.toggleOnTrue(f_intake);
                Controlls.INTAKE_ARM_UP.whileTrue(arm_up);
                Controlls.INTAKE_ARM_DOWN.whileTrue(arm_down);
                Controlls.Intake_ARM_PID.onChange(arm_pid);
                Controlls.STAGE_1.onChange(feedandshoots1);
                Controlls.STAGE_2.onChange(feedandshoots2);
                Controlls.STAGE_3.onChange(feedandshoots3);
                Controlls.ZERO_GYRO.onChange(zerogyro);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}
