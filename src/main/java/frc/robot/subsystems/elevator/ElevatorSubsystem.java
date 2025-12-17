package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {


  private final PWMSparkMax motor =
      new PWMSparkMax(ElevatorConstants.MOTOR_ID);

  private final Encoder encoder =
      new Encoder(
          ElevatorConstants.ENCODER_A_ID,
          ElevatorConstants.ENCODER_B_ID);


  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          ElevatorConstants.kP,
          ElevatorConstants.kI,
          ElevatorConstants.kD,
          ElevatorConstants.MOTION_CONSTRAINTS);

  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kS,
          ElevatorConstants.kG,
          ElevatorConstants.kV,
          ElevatorConstants.kA);


  private ElevatorSim elevatorSim;
  private EncoderSim encoderSim;

  public ElevatorSubsystem() {
    encoder.setDistancePerPulse(
        ElevatorConstants.DISTANCE_PER_PULSE_METERS);

    controller.setTolerance(0.02);

    if (RobotBase.isSimulation()) {
      elevatorSim =
          new ElevatorSim(
              DCMotor.getNEO(1),
              ElevatorConstants.GEAR_RATIO,
              ElevatorConstants.CARRIAGE_MASS_KG,
              ElevatorConstants.DRUM_RADIUS_METERS,
              ElevatorConstants.MIN_HEIGHT_METERS,
              ElevatorConstants.MAX_HEIGHT_METERS,
              ElevatorConstants.SIMULATE_GRAVITY, 0);

      encoderSim = new EncoderSim(encoder);
    }
  }


  public void setGoal(double heightMeters) {
    heightMeters =
        Math.max(
            ElevatorConstants.MIN_HEIGHT_METERS,
            Math.min(ElevatorConstants.MAX_HEIGHT_METERS, heightMeters));

    controller.setGoal(heightMeters);

    System.out.println("Goal set to: " + heightMeters);
   //  SmartDashboard.putNumber("Goal Set To", heightMeters);
  }

  public boolean atGoal() {
    return controller.atGoal();
  }

  public double getHeightMeters() {
    return encoder.getDistance();
  }

  public void stop() {
    motor.stopMotor();
  }


  @Override
  public void periodic() {
    double pidOutput = controller.calculate(getHeightMeters());
    SmartDashboard.putNumber("Elevator Height", getHeightMeters());

    TrapezoidProfile.State setpoint = controller.getSetpoint();

    double ffOutput =
        feedforward.calculate(
            setpoint.velocity,
            setpoint.position);

    motor.setVoltage(pidOutput + ffOutput);
    SmartDashboard.putString("Goal", controller.getGoal().toString());
    SmartDashboard.putNumber("pidOutput", pidOutput);

    //System.out.println("It is getting ran");
  }

//   @Override
//   public void simulationPeriodic() {
//     elevatorSim.setInput(motor.get() * RoboRioSim.getVInVoltage());
//     elevatorSim.update(0.02);

//     encoderSim.setDistance(elevatorSim.getPositionMeters());
    
//     encoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());
//   }
}
