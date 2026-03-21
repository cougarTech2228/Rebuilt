package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeAngle;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.CTSequentialCommandGroup;

public class OscillateIntakeCommand extends CTSequentialCommandGroup {
    // Extend to L3 w/ limit switch, climb to L1

    public OscillateIntakeCommand(Climber climber, Intake intake) {
        this.addCommands(
            new OscillateIntakeOnce(intake, climber),
            new WaitCommand(0.25)
        );
    }
}
