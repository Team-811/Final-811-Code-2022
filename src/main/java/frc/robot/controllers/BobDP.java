package frc.robot.controllers;

import frc.robot.controllers.BobAB.ThresholdType;
import frc.robot.controllers.BobXboxController.XboxAxis;
import frc.robot.controllers.BobXboxController.XboxDpad;
import edu.wpi.first.wpilibj.Joystick;

//DP stands for Dpad
public class BobDP {
	public final Joystick joy;
	public BobAB Up;
	public BobAB Down;
	public BobAB Left;
	public BobAB Right;
	public BobAB UpLeft;
	public BobAB UpRight;
	public BobAB DownLeft;
	public BobAB DownRight;

	public BobDP(Joystick joystick) {
		this.joy = joystick;
		this.Up = new BobAB(joy, XboxAxis.DPAD, XboxDpad.UP.value, ThresholdType.POV);
		this.Down = new BobAB(joy, XboxAxis.DPAD, XboxDpad.DOWN.value, ThresholdType.POV);
		this.Left = new BobAB(joy, XboxAxis.DPAD, XboxDpad.LEFT.value, ThresholdType.POV);
		this.Right = new BobAB(joy, XboxAxis.DPAD, XboxDpad.RIGHT.value, ThresholdType.POV);
		this.UpLeft = new BobAB(joy, XboxAxis.DPAD, XboxDpad.UP_LEFT.value, ThresholdType.POV);
		this.UpRight = new BobAB(joy, XboxAxis.DPAD, XboxDpad.UP_RIGHT.value, ThresholdType.POV);
		this.DownLeft = new BobAB(joy, XboxAxis.DPAD, XboxDpad.DOWN_LEFT.value, ThresholdType.POV);
		this.DownRight = new BobAB(joy, XboxAxis.DPAD, XboxDpad.DOWN_RIGHT.value, ThresholdType.POV);
	}

	public double getValue() {
		return joy.getRawAxis(XboxAxis.DPAD.value);
	}

}
