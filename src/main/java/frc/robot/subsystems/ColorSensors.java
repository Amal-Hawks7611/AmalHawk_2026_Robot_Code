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
    public int loopCounter;

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

    public boolean isBallDetected() {
        Color detectedColor = colorSensor1.getColor();
        Color detectedColor2 = colorSensor2.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        ColorMatchResult match2 = colorMatcher.matchClosestColor(detectedColor2);

        ballDetected1 = match != null
                && match.confidence >= 0.92
                && match.color.equals(kYellowTarget);

        ballDetected2 = match2 != null
                && match2.confidence >= 0.92
                && match2.color.equals(kYellowTarget);
                SmartDashboard.putBoolean("Ball", ballDetected1 || ballDetected2);
        return ballDetected1 || ballDetected2;
    }
}