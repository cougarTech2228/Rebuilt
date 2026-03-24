package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.Intake.OscillateType;
import frc.robot.subsystems.turret.Turret;

public class StopIntakeCommand extends Command {
	private final Intake intake;

	public StopIntakeCommand(Intake intake) {
		this.intake = intake;
	}


	@Override
	public void initialize() {
		intake.setIntakeVoltage(IntakeConstants.INTAKE_MOTOR_IDLE_VOLTAGE);
	}


	@Override
	public boolean isFinished() {
		return true;
	}
}