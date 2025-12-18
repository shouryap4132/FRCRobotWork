package frc.robot.subsystems.arm.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.ArmSubsystemIO;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.utils.MiscConstants.SimConstants;



public class ArmSim implements ArmSubsystemIO{
    
    private SingleJointedArmSim armSim;

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
        armSim.update(0.02);
        double velocity = armSim.getVelocityRadPerSec();
        data.appliedVolts = inputVolts;
		data.positionRad = armSim.getAngleRads();
		data.velocityRadsPerSecond = velocity;
		data.accelerationRadsPerSecondSquared = velocity / SimConstants.;
		data.motorCurrentAmps = 0;
		data.motorAppliedVolts = 0;
		data.motorTempCelcius = 0;
		data.inputVolts = 0;

    }

}
