package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class HopperIOSim implements HopperIO {
    
    private final FlywheelSim indexerSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.005, 1.0),
        DCMotor.getNEO(1)
    );

    private final FlywheelSim kickerSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), 0.005, 1.0),
        DCMotor.getNeoVortex(1)
    );

    private double indexerAppliedVolts = 0.0;
    private double kickerAppliedVolts = 0.0;

    public HopperIOSim() {}

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        indexerSim.setInput(indexerAppliedVolts);
        kickerSim.setInput(kickerAppliedVolts);

        indexerSim.update(0.020);
        kickerSim.update(0.020);

        inputs.indexerVoltage = indexerAppliedVolts;
        inputs.indexerVelocity = indexerSim.getAngularVelocityRPM();
        inputs.indexerCurrent = indexerSim.getCurrentDrawAmps();

        inputs.kickVoltage = kickerAppliedVolts;
        inputs.kickVelocity = kickerSim.getAngularVelocityRPM();
        inputs.kickCurrent = kickerSim.getCurrentDrawAmps();
    }

    @Override
    public void indexerOn(boolean test) {
        indexerAppliedVolts = test ? HopperConstants.testIndexerVoltage : HopperConstants.indexerVoltage;
    }

    @Override
    public void indexerOff() {
        indexerAppliedVolts = 0.0;
    }

    @Override
    public void kickerOn(boolean test) {
        kickerAppliedVolts = test ? HopperConstants.testKickerVoltage : HopperConstants.kickerVoltage;
    }

    @Override
    public void kickerOff() {
        kickerAppliedVolts = 0.0;
    }

    @Override
    public void kickerSpit() {
        kickerAppliedVolts = HopperConstants.kickerSpitVoltage;
    }


}