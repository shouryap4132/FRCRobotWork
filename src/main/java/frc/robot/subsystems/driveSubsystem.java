package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkRelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveSubsystem extends SubsystemBase {

  private final SparkMax motor = new SparkMax(10, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  // PID constants
  private double kP = 0.1;
  private double kI = 0.0;
  private double kD = 0.0;

  private double targetSpeed = 0;   // RPM
  private double error = 0;
  private double lastError = 0;
  private double integral = 0;

  public driveSubsystem() {}

  // Set desired motor speed (RPM) 
  public void setTargetSpeed(double speed) {
    targetSpeed = speed;
  }

  // Read motor speed (RPM) 
  public double getSpeed() {
    return encoder.getVelocity();
    
  }

  
  @Override
  public void periodic() {
    // PID runs every 20ms
    double currentSpeed = getSpeed();

    error = targetSpeed - currentSpeed;
    integral += error * 0.02;         
    double derivative = (error - lastError) / 0.02;

    double output = kP * error + kI * integral + kD * derivative;

    
    motor.setVoltage(output);

    lastError = error;
  }

  
  public void stop() {
    motor.setVoltage(0);
  }
}
