package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class HopperIOTalonFX implements HopperIO {
    protected final TalonFX motor = new TalonFX(Constants.hopperFalconCanID, "canivore");

    private final StatusSignal<Voltage> motorAppliedVoltage = motor.getMotorVoltage();
    private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity();
    private final StatusSignal<Current> motorCurrentAmps = motor.getSupplyCurrent();
    
    public HopperIOTalonFX() {
        
    }

    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperVelocity = this.motorVelocity.getValueAsDouble();
        inputs.hopperVoltage = this.motorAppliedVoltage.getValueAsDouble();
        inputs.hopperCurrent = this.motorCurrentAmps.getValueAsDouble();
    }

    public void turnOn() {
        this.motor.setVoltage(HopperConstants.SpinningVoltage);
    }

    public void turnOff() {
        this.motor.setVoltage(0.0);
    }
}
