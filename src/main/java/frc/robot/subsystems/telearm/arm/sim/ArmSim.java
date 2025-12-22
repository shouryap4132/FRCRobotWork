package frc.robot.subsystems.telearm.arm.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.telearm.arm.ArmSubsystemIO;
import frc.robot.subsystems.telearm.arm.ArmConstants;
import frc.robot.subsystems.telearm.elevator.ElevatorIO;
import frc.robot.subsystems.telearm.elevator.ElevatorIO.ElevatorData;

import frc.robot.utils.MiscConstants.SimConstants;






public class ArmSim implements ArmSubsystemIO{
    


    private SingleJointedArmSim armSim;

    private double inputVolts = 0.0;
    private double previousVelocity = 0.0;
    private double velocity = 0.0;

    public ArmSim() {
        armSim = new SingleJointedArmSim(
            DCMotor.getNEO(ArmConstants.numMotors),
            ArmConstants.gearing,
            ArmConstants.armLengthMeters,
            ArmConstants.armLengthMeters,
            Math.toRadians(ArmConstants.armMinAngleDegrees),
            Math.toRadians(ArmConstants.armMaxAngleDegrees),
            ArmConstants.simulateGravity,
            ArmConstants.armStartingAngleDegrees
        );
    }

    @Override

    public void updateData(ArmData data) {
        armSim.update(SimConstants.loopPeriodSec);
        previousVelocity = velocity;
        velocity = armSim.getVelocityRadPerSec();
		data.positionRad = armSim.getAngleRads();
		data.velocityRadsPerSecond = velocity;
		data.accelerationRadsPerSecondSquared = ( velocity - previousVelocity ) / SimConstants.loopPeriodSec;
		data.motorCurrentAmps = armSim.getCurrentDrawAmps();
		data.motorAppliedVolts = inputVolts;
		data.motorTempCelcius = 0;

    }

    @Override

    public void setVoltage(double volts) {
        inputVolts = MathUtil.clamp(volts, -12, 12);
        armSim.setInputVoltage(inputVolts);
    }

}
