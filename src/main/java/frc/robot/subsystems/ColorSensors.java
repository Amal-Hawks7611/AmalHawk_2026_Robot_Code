package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensors extends SubsystemBase {

    private final ColorSensorV3 colorSensor1;
    private final ColorSensorV3 colorSensor2;
    private final ColorMatch colorMatcher;

    private final Color kYellowTarget = new Color(0.330078125, 0.545166015625, 0.125);

    private boolean ballDetected1 = false;
    private boolean ballDetected2 = false;

    public ColorSensors() {
        colorSensor1 = new ColorSensorV3(I2C.Port.kOnboard);
        colorSensor2 = new ColorSensorV3(I2C.Port.kMXP);
        colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(kYellowTarget);
        colorMatcher.setConfidenceThreshold(0.92);
    }

    @Override
    public void periodic() {
        Color detectedColor = colorSensor1.getColor();
        Color detectedColor2 = colorSensor2.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        ColorMatchResult match2 = colorMatcher.matchClosestColor(detectedColor2);
        if (match != null 
                && match.confidence >= 0.92
                && match.color.equals(kYellowTarget)) {

            ballDetected1 = true;

        } else {
            ballDetected1 = false;
        }
        if (match2 != null 
                && match2.confidence >= 0.92
                && match2.color.equals(kYellowTarget)) {

            ballDetected2 = true;

        } else {
            ballDetected2 = false;
        }
        SmartDashboard.putNumber("Red 1", detectedColor.red);
        SmartDashboard.putNumber("Green 1", detectedColor.green);
        SmartDashboard.putNumber("Blue 1", detectedColor.blue);
        SmartDashboard.putNumber("Confidence 1", match != null ? match.confidence : 0.0);
        SmartDashboard.putBoolean("Ball Detected 1", ballDetected1);

        SmartDashboard.putNumber("Red 2", detectedColor2.red);
        SmartDashboard.putNumber("Green 2", detectedColor2.green);
        SmartDashboard.putNumber("Blue 2", detectedColor2.blue);
        SmartDashboard.putNumber("Confidence 2", match2 != null ? match2.confidence : 0.0);
        SmartDashboard.putBoolean("Ball Detected 2", ballDetected2);

        SmartDashboard.putBoolean("Ball Detected", ballDetected1 || ballDetected2);
    }

    public boolean isBallDetected() {
        return ballDetected1 || ballDetected2;
    }
}