package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.CTSequentialCommandGroup;

public class AutoClimbL1Command extends CTSequentialCommandGroup {
    // Extend to L3 w/ limit switch, climb to L1

    public AutoClimbL1Command(Drive drive, Climber climber, Turret turret, Intake intake) {
        this.addCommands(
            new AlignClimbCommand(drive, climber, turret),
            
            new ToggleIntakeCommand(intake, climber)
                .onlyIf(() -> !intake.isRetracted()), 

            new ExtendClimberCommand(climber, intake, ClimberLevel.L3, turret),
            new ClimbCommand(climber, ClimberLevel.L1)
        );
    }
}
