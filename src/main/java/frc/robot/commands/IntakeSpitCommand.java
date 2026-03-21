package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.Intake.IntakeMode;

public class IntakeSpitCommand extends Command {
    private final Intake intake;
    private final Climber climber;
    private final Hopper hopper;

    public IntakeSpitCommand(Intake intake, Climber climber, Hopper hopper) {
        this.intake = intake;
        this.climber = climber;
        this.hopper = hopper;
        // only require intake, climber is only used for reading status
        addRequirements(intake, hopper);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.setIntakeVoltage(IntakeConstants.INTAKE_MOTOR_SPIT_VOLTAGE);
        hopper.kickerSpit();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeVoltage(IntakeConstants.INTAKE_MOTOR_INTAKE_VOLTAGE);
        hopper.kickerOff();
        // Optional: set intake back to default voltage
    }

}
