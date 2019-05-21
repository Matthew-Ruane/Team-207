package org.usfirst.frc.team294.utilities;

import edu.wpi.first.wpilibj.Relay;

public class LEDSet {
	private final Relay ledRelay;
	
	/**
	 * Creates a new set of LEDs
	 * @param channel RoboRio relay channel attached to LED Spike controller
	 */
	public LEDSet(int channel) {
		ledRelay = new Relay(channel);
		setOff();
	}
	
	/**
	 * Turn off LEDs
	 */
	public void setOff() {
//		ledRelay.set(Relay.Value.kOff);
		ledRelay.set(Relay.Value.kOn);
	}

	/**
	 * Light LEDs RED
	 */
	public void setRed() {
		ledRelay.set(Relay.Value.kForward);
	}

	/**
	 * Light LEDs BLUE
	 */
	public void setBlue() {
		ledRelay.set(Relay.Value.kForward);
//		ledRelay.set(Relay.Value.kReverse);
	}

	/**
	 * Light LEDs PURPLE
	 */
	public void setPurple() {
		ledRelay.set(Relay.Value.kForward);
//		ledRelay.set(Relay.Value.kOff);
	}
}
