package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeAngle;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.CTSequentialCommandGroup;

public class AutoClimbL1Command extends CTSequentialCommandGroup {

    public AutoClimbL1Command(Drive drive, Climber climber, Turret turret, Intake intake, Hopper hopper) {
        this.addCommands(
            // Safely retract intake if needed
            new ToggleIntakeCommand(intake, climber, hopper)
                .onlyIf(() -> !intake.isRetracted()),

            // PathPlanner to localized position near the ladder
            new AlignL1ClimbCommand(drive, climber, turret)
                .alongWith(new ExtendClimberCommand(climber, intake, ClimberLevel.L1, turret)),
            
            // 3. Extend climber out to L1
            // new ExtendClimberCommand(climber, intake, ClimberLevel.L1, turret),
            
            // // 4. Slow drive until flush with ladder and latched
            // // (Uses the speed parameters you had in your original Align command)
            // drive.run(() -> drive.runVelocity(new ChassisSpeeds(-0.5, 0, 0)))
            //      .withTimeout(1.0),
                 
            // drive.run(() -> drive.runVelocity(new ChassisSpeeds(-0.1, 0.5, 0)))
            //      .until(climber::isReadyToClimb),
                 
            // // CRITICAL: Stop the drivebase once we are latched!
            // Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0)), drive),

            // // 5. Climb!
            // new ClimbCommand(climber, ClimberLevel.

            // 4. Slow drive until flush with ladder and latched
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
                 
            // CRITICAL: Stop the drivebase once we are latched!
            Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0)), drive)
                 .asProxy(),

            // // 5. Climb!
            new ClimbCommand(climber, ClimberLevel.L1)
        );
    }
}