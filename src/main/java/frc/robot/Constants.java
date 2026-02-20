package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.math.Matter;

public final class Constants {
    public static final double ROBOT_MASS = 104.72 * 0.453592; // TODO GİRİLECEK
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; 
    public static final double MAX_SPEED = 3;

    public static Color setBrightness(Color currentColor, double brightness) {
        return new Color(currentColor.red * brightness, currentColor.green * brightness,
                currentColor.blue * brightness);
    }

    public static final class DrivebaseConstants {
        public static final double WHEEL_LOCK_TIME = 10;
        public static final PIDConstants transation = new PIDConstants(12, 0.0, 0.05);
        public static final PIDConstants angle = new PIDConstants(3, 0.0, 0.05);
    }

    public static class OperatorConstants {
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static final class LedSubsystem {
        public static int LED_PWM_PORT = 0;
        public static int LED_LENGTH = 180;
        public static int BREATHE_MAGNITUDE = 5;
        public static LEDPattern ELEVATOR_PROCESS_COLOR = LEDPattern.solid(setBrightness(Color.kPurple, 0.69 * 0.8));
        public static LEDPattern TARGET_FOCUS_COLOR = LEDPattern.solid(setBrightness(Color.kGreen, 0.69 * 0.8));
        public static LEDPattern INTAKE_COLOR = LEDPattern.solid(setBrightness(Color.kWhite, 0.69 * 0.8));
        public static LEDPattern ALGEA_INTAKE_COLOR = LEDPattern.solid(setBrightness(Color.kAqua, 0.69 * 0.8));
        public static LEDPattern BREATHE_COLOR = LEDPattern.gradient(GradientType.kDiscontinuous,
                setBrightness(Color.kOrangeRed, 0.69 * 0.8), setBrightness(Color.kRed, 0.69 * 0.8),
                setBrightness(Color.kDarkRed, 0.69 * 0.8));
    }


    public static class Intake {
        public static int INTAKE_LEADER_MOTOR_PORT = 16;
        public static double INTAKE_SPEED = 0.7;
        public static double INTAKE_TIME = 0.5;
    }
    public static class Feeder {
        public static int FEEDER_LEADER_MOTOR_PORT = 21;
        public static double FEEDER_SPEED = 0.8;
        public static double FEEDER_TIME = 9;
    }
    public static class Shooter {
        public static int SHOOTER_LEADER_PORT = 22;
        public static int SHOOTER_SLAVE_PORT = 23;
        public static double STAGE1_SPEED = 0.5;
        public static double STAGE2_SPEED = 0.7;
        public static double STAGE3_SPEED = 1;
        public static double SHOOTER_TIME = 10;
    }
    public static class IntakeArm {
        public static int INTAKE_ARM_LEADER_PORT = 19;
        public static int INTAKE_ARM_SLAVE_PORT = 20;
        public static double INTAKE_SETPOINT = 9;
        public static double INTAKE_ARM_MANUAL_SPEED = 0.15;
        public static double INTAKE_OCALPID_SPEED = -0.05; 
        public static double OCALPID_TOLERANCE_VALUE = 0.5;
        public static class Leader {
            public static double TOP_LIMIT = 0.05;
            public static double DOWN_LIMIT = 9.36;
        }

        public static class Slave {
            public static double TOP_LIMIT = 0.05;
            public static double DOWN_LIMIT = 8.64;
        }
    }
    public static class Controlls {
        public static CommandPS5Controller DRIVER_CONTROLLER = new CommandPS5Controller(OI.DRIVER_CONTROLLER_PORT);
        public static CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(
                OI.OPERATOR_CONTROLLER_PORT);

        public static Trigger INTAKE = Controlls.DRIVER_CONTROLLER.L1();
        public static Trigger INTAKE_ARM_UP = Controlls.DRIVER_CONTROLLER.R2();
        public static Trigger INTAKE_ARM_DOWN = Controlls.DRIVER_CONTROLLER.L2();
        public static Trigger Intake_ARM_PID = Controlls.DRIVER_CONTROLLER.povDown();
        public static Trigger STAGE_1 = Controlls.DRIVER_CONTROLLER.cross();
        public static Trigger STAGE_2 = Controlls.DRIVER_CONTROLLER.circle();
        public static Trigger STAGE_3 = Controlls.DRIVER_CONTROLLER.triangle();
    }

    public static class EnabledParts {
        public static boolean IS_LED_ENABLED = false;
        public static boolean IS_SWERVE_ENABLED = true;
        public static boolean IS_INTAKE_ENABLED = true;
        public static boolean IS_INTAKE_ARM_ENABLED = true;
        public static boolean IS_FEEDER_ENABLED = true;
        public static boolean IS_SHOOTER_ENABLED = true;
    }

    public static class OI {
        public static int DRIVER_CONTROLLER_PORT = 0;
        public static int OPERATOR_CONTROLLER_PORT = 1;
        public static String SWERVE_CANBUS_STRING = "rio";
        public static String RIO_CANBUS_STRING = "rio";
        public static boolean IS_TEST = false;
        public static boolean IS_PROCESSING = false;
        public static boolean IS_SWERVE_FOCUSED = false;
        public static boolean IS_INTAKING = false;
        public static boolean IS_LED_CYCLING = false;
        public static boolean IS_LED_MORSE_SHOWING = false;
        public static boolean IS_FOCUSED = false;
        public static boolean IS_GONE = false;
        public static double LIMELIGHT_SPEED = 0.4;
    }
}
