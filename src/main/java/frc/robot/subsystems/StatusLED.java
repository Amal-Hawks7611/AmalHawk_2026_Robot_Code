package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.EnabledParts;
import frc.robot.Constants.LedSubsystem;

public class StatusLED extends SubsystemBase {
    public AddressableLED led;
    public AddressableLEDBuffer buffer;
    public RobotContainer container;
    public double scrollOffset = 0;
    public boolean ledMore = false;

    public StatusLED(RobotContainer container) {
        try {
            if (EnabledParts.IS_LED_ENABLED) {
                led = new AddressableLED(LedSubsystem.LED_PWM_PORT);
                buffer = new AddressableLEDBuffer(LedSubsystem.LED_LENGTH);
                led.setLength(buffer.getLength());
                led.setData(buffer);
            }
        } catch (Exception e) {
            System.err.println("Failed to initialize StatusLED: " + e.getMessage());
        }
        this.container = container;
    }

    public void setDefault() {
        if (led == null || buffer == null) {
            return;
        }

        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();

        boolean isRed = true;

        if (alliance.isPresent()) {
            isRed = alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
        }

        for (int i = 0; i < buffer.getLength(); i++) {
            double position = (double) i / buffer.getLength();
            double wave = 0.5 + 0.5 * Math.sin(2 * Math.PI * (position + scrollOffset));

            int value = (int) (50 + wave * 205);

            if (isRed) {
                buffer.setRGB(i, value, 0, 0);
            } else {
                buffer.setRGB(i, 0, 0, value);
            }
        }

        led.setData(buffer);
        led.start();
        scrollOffset += 0.02;
        if (scrollOffset >= 1) {
            scrollOffset = 0;
        }
    }

    public Color setBrightness(Color currentColor, double brightness) {
        return new Color(currentColor.red * brightness, currentColor.green * brightness,
                currentColor.blue * brightness);
    }

    public void setShoot() {
        if (led == null || buffer == null) {
            return;
        }
        LedSubsystem.SHOOT_COLOR.applyTo(buffer);
        led.setData(buffer);
        led.start();
    }
    public void setMorse(boolean action){
        ledMore = action;
    }
    public boolean isMorseing(){
        return ledMore;
    }
    public void setFocus() {
        if (led == null || buffer == null) {
            return;
        }
        LedSubsystem.FOCUS_COLOR.applyTo(buffer);
        led.setData(buffer);
        led.start();
    }

    public void setFocusleft() {
        if (led == null || buffer == null) {
            return;
        }
        LedSubsystem.FOCUS_COLOR_LEFT.applyTo(buffer);
        led.setData(buffer);
        led.start();
    }

    public void setFocusCenter() {
        if (led == null || buffer == null) {
            return;
        }
        LedSubsystem.FOCUS_COLOR_CENTER.applyTo(buffer);
        led.setData(buffer);
        led.start();
    }

    public void setFocusRight() {
        if (led == null || buffer == null) {
            return;
        }
        LedSubsystem.FOCUS_COLOR_RIGHT.applyTo(buffer);
        led.setData(buffer);
        led.start();
    }

public void checkForProcess() {
    if (!isMorseing()) {

        var target = container.limelightSubsystem.getCurrentTarget();
        boolean aligned = container.limelightSubsystem.isAligned();

        if (target != LimelightSubsystem.AlignmentTarget.NONE) {

            if (aligned) {
                setFocus();
                return;
            }

            switch (target) {
                case LEFT:
                    setFocusleft();
                    return;
                case CENTER:
                    setFocusCenter();
                    return;
                case RIGHT:
                    setFocusRight();
                    return;
                default:
                    break;
            }
        }

        if (container.shooterSubsystem.isShooting()) {
            setShoot();
        } else {
            setDefault();
        }
    }
}

    @Override
    public void periodic() {
        checkForProcess();
    }
}