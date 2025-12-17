package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class ElevatorConstants {


  public static final int MOTOR_ID = 0;

  public static final int ENCODER_A_ID = 0;
  public static final int ENCODER_B_ID = 1;


  public static final double DRUM_RADIUS_METERS =
      Units.inchesToMeters(1.5);


  public static final double CARRIAGE_MASS_KG = 6.0;

  public static final double GEAR_RATIO = 10.0;

  public static final double MIN_HEIGHT_METERS = 0.0;
  public static final double MAX_HEIGHT_METERS =
      Units.inchesToMeters(60.0);


  public static final double MAX_VELOCITY_METERS_PER_SEC = 1.5;
  public static final double MAX_ACCELERATION_METERS_PER_SEC_SQ = 2.0;

  public static final TrapezoidProfile.Constraints MOTION_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          MAX_VELOCITY_METERS_PER_SEC,
          MAX_ACCELERATION_METERS_PER_SEC_SQ);


  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;



  public static final double kS = 0.2;  // stat friction 
  public static final double kG = 1.0;  // grav 
  public static final double kV = 2.2;  // velo  
  public static final double kA = 0.1;  // acc


  public static final int ENCODER_CPR = 2048;

  public static final double DISTANCE_PER_PULSE_METERS =
      (2 * Math.PI * DRUM_RADIUS_METERS) / ENCODER_CPR;


  public static final double STOW_HEIGHT_METERS = 0.0;
  public static final double LOW_HEIGHT_METERS = 0.35;
  public static final double MID_HEIGHT_METERS = 0.85;
  public static final double HIGH_HEIGHT_METERS = 1.35;


  public static final boolean SIMULATE_GRAVITY = true;

  private ElevatorConstants() {}
}
