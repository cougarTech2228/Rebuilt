package frc.robot.commands.pathplanner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeAngle;
import frc.robot.subsystems.turret.Turret;

public class DeployIntakeCommand extends Command {
	private final Hopper hopper;
	private final Intake intake;

	public DeployIntakeCommand(Hopper hopper, Intake intake) {
		this.hopper = hopper;
		this.intake = intake;
	}

	private boolean initialized = false;
	
	@Override
	public void initialize() {
		intake.setIntakeAngle(IntakeAngle.DEPLOYED);
	}

	@Override
	public void execute() {

	}

	@Override
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {
		return true;
	}
}