package frc.robot.commands.pathplanner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeAngle;
import frc.robot.subsystems.turret.Turret;

public class RetractIntakeCommand extends Command {
	private final Hopper hopper;
	private final Intake intake;

	public RetractIntakeCommand(Hopper hopper, Intake intake) {
		this.hopper = hopper;
		this.intake = intake;
	}

	private boolean initialized = false;
	private boolean didStartRetraction = false;
	
	@Override
	public void initialize() {
		didStartRetraction = false;
	}

	@Override
	public void execute() {
		if(hopper.safeToRetract()) {
			intake.setIntakeAngle(IntakeAngle.HOME);
			didStartRetraction = true;
		}
	}

	@Override
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {
		return didStartRetraction;
	}
}