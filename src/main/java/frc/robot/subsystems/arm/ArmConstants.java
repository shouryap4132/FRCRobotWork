package frc.robot.subsystems.arm;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.LoggedTunableNumber;

public class ArmConstants {
    public static final int motorID = 0;
    public static final int encoderAID = 0;
    public static final int numMotors = 1;
    public static final int gearing = 100;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kMaxVelocity = 0;
    public static final double kMaxAcceleration = 0;
    


    public static final boolean simulateGravity = true;

    public static final double armLengthMeters = Units.inchesToMeters(24.0);
    public static final double armLenghtInches =  24.0;

    public static final int armMinAngleDegrees = 10;
	public static final int armMaxAngleDegrees = 180;
	public static final int armStartingAngleDegrees = 90;


    public enum ArmStates {
		CORAL_PICKUP(-0.68),
		CORAL_STATION(Math.PI /2),

		L1(Math.PI/4),
		HAND_OFF(2.27),
		STOW(2.2),
		STOPPED(0),
		CLIMB(1.9);

		public final double setPointRad;

		private ArmStates(double setPoint) {
			this.setPointRad = setPoint;
    }
    
    }
}

