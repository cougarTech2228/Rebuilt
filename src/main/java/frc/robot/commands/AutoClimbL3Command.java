package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.CTSequentialCommandGroup;

public class AutoClimbL3Command extends CTSequentialCommandGroup {

    public AutoClimbL3Command(Drive drive, Climber climber, Turret turret, Intake intake) {
        this.addCommands(
            // Safely retract intake if needed
            new ToggleIntakeCommand(intake, climber)
                .onlyIf(() -> intake.isDeployed()),
            
            // Fail safe turn off intake voltage anyways
            new StopIntakeCommand(intake),

            // PathPlanner to localized position near the ladder
            new AlignL1ClimbCommand(drive, turret)
                .alongWith(new ExtendClimberCommand(climber, intake, ClimberLevel.L3, turret)),

            // Slow drive until flush with ladder and latched
            // .asProxy() prevents the requirement conflict with the Align command!
            drive.run(() -> drive.runVelocity(new ChassisSpeeds(-0.1, 0, 0)))
                 .withTimeout(1.5)
                 .asProxy(),
        
            drive.run(() -> drive.runVelocity(new ChassisSpeeds(-0.05, 0.2, Math.toRadians(-5))))
                 .until(climber::isReadyToClimb)
                 .asProxy(),

            // Little oomph just incase the switch is early activated
            drive.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0.05, 0)))
                 .withTimeout(0.1)
                 .asProxy(),   
                 
            // Stop the drivebase once we are latched
            Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0)), drive)
                 .asProxy(),

            // Climb
            new ClimbCommand(climber, ClimberLevel.L3)
        );
    }
}