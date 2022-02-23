package frc.robot.NetworkTables;


import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;

public class Ultrasonic extends TimedRobot{
    private final AnalogInput ultrasonic = new AnalogInput(0);

    public double getDistance() {
        double rawValue = ultrasonic.getValue();
        double voltage_scale_factor = 5/RobotController.getVoltage5V();
        double currentDistanceCentimeters = rawValue * voltage_scale_factor * 0.125;
        return currentDistanceCentimeters;
    }

}