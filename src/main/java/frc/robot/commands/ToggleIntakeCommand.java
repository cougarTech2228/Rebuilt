package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class ToggleIntakeCommand extends Command {
    private final Intake intake;
    private final Climber climber;
    private final Hopper hopper;
    private boolean toggeled;

    public ToggleIntakeCommand(Intake intake, Climber climber, Hopper hopper) {
        this.intake = intake;
        this.climber = climber;
        this.hopper = hopper;
        // only require intake, climber is only used for reading status
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        toggeled = false;
    }

    @Override
    public void execute() {
        // if the climber is extender, just ignore this request, and finish the command
        if (climber.isExtensionHome()) {
            if (!intake.isRetracted()) {
                // Intake is already out
                if (intake.isSpitting()) {
                    // If its spitting, you want to intake again
                    // Case 1 -> 2: If it's spitting, switch back to intaking and turn kicker off
                    intake.setIntakeVoltage(IntakeConstants.INTAKE_MOTOR_INTAKE_VOLTAGE);
                } else {
                    // Off or currently intaking: toggle it and ensure kicker is off
                    intake.toggleIntake();
                }
                
            } else {
                // Intake is idol, toggle it like normal
                // Case 3: Intake is idle/retracted, toggle it out and turn kicker off
                intake.toggleIntake();
            }
            hopper.kickerOff();
        } 
        toggeled = true;
    }

    @Override
    public boolean isFinished() {
        return toggeled;
    }
}
