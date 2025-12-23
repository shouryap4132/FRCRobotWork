package frc.robot.subsystems.telearm.elevator;


public interface ElevatorIO {
    public static class ElevatorData {
        public double positionMeters = 0;
        // public double armLengthMeters = 10;
        public double velocityMetersPerSecond = 0;
        public double accelerationMetersPerSecondSquared = 0;
        public double currentAmps = 0;
        public double appliedVolts = 0;
        public double tempCels = 0;
    }

    /**
	 * Updates the set of loggable inputs.
	 * 
	 * @param data
	 */
	public default void updateData(ElevatorData data) {
	};

	/**
	 * Run the motor at the specified voltage.
	 * 
	 * @param volts
	 */
	public default void setVoltage(double volts) {
	};

	/**
	 * Enable or disable brake mode on the motor.
	 * 
	 * @param enable
	 */
	public default void setBrakeMode(boolean enable) {
	};
}