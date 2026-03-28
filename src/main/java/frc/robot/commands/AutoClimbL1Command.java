package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;

public class AutoClimbL1Command extends SequentialCommandGroup {

    public AutoClimbL1Command(Drive drive, Climber climber, Turret turret, Intake intake) {
        this.addCommands(
            // Safely retract intake if needed
            new ToggleIntakeCommand(intake, climber)
                .onlyIf(() -> intake.isDeployed()),
            
            // Fail safe turn off intake voltage anyways
            new StopIntakeCommand(intake),

            // PathPlanner to localized position near the ladder
            new AlignL1ClimbCommand(drive, turret)
                .alongWith(new ExtendClimberCommand(climber, intake, ClimberLevel.L1, turret)),

            // Slow drive until flush with ladder and latched
            drive.run(() -> drive.runVelocity(new ChassisSpeeds(-0.2, 0, 0)))
                 .withTimeout(1),
                 
            drive.run(() -> drive.runVelocity(new ChassisSpeeds(-0.05, 0.2, Math.toRadians(-10))))
                 .until(climber::isReadyToClimb),

            // Little oomph just incase the switch is early activated
            drive.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0.05, 0)))
                 .withTimeout(0.1),
                 
            // Stop the drivebase once we are latched
            Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))),

            // Climb
            new ClimbCommand(climber, ClimberLevel.L1)
        );
    }
}