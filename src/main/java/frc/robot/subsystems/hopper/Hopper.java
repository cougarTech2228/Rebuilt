package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
	private final HopperIO io;
    private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();
	
	private boolean on = false; 
	

	public Hopper(HopperIO io) {
		this.io = io;

		SmartDashboard.putBoolean("IndexerTest", false);
	}

    @Override
    public void periodic() {
        super.periodic();
    	io.updateInputs(hopperInputs);
        Logger.processInputs("Hopper", hopperInputs);
    }

	public boolean isOn() {
		return this.on;
	}

	public void indexerOn(boolean test) {
		this.on = true;
		this.io.indexerOn(test);
	}
	
	public void indexerOff() {
		this.on = false;
		this.io.indexerOff();
	}

	public boolean safeToRetract() {
		return this.io.safeToRetract();
	}
}


