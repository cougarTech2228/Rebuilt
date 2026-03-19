package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.CTSequentialCommandGroup;

public class AutoClimbL1Command extends CTSequentialCommandGroup {

    public AutoClimbL1Command(Drive drive, Climber climber, Turret turret, Intake intake) {
        this.addCommands(
            // 1. PathPlanner to localized position near the ladder
            new AlignL1ClimbCommand(drive, climber, turret),
            
            // 2. Safely retract intake if needed
            new ToggleIntakeCommand(intake, climber)
                .onlyIf(() -> !intake.isRetracted()), 

            // 3. Extend climber out to L1
            new ExtendClimberCommand(climber, intake, ClimberLevel.L1, turret),
            
            // // 4. Slow drive until flush with ladder and latched
            // // (Uses the speed parameters you had in your original Align command)
            // drive.run(() -> drive.runVelocity(new ChassisSpeeds(-0.5, 0, 0)))
            //      .withTimeout(1.0),
                 
            // drive.run(() -> drive.runVelocity(new ChassisSpeeds(-0.1, 0.5, 0)))
            //      .until(climber::isReadyToClimb),
                 
            // // CRITICAL: Stop the drivebase once we are latched!
            // Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0)), drive),

            // // 5. Climb!
            // new ClimbCommand(climber, ClimberLevel.L1)

            // 4. Slow drive until flush with ladder and latched
            // .asProxy() prevents the requirement conflict with the Align command!
            drive.run(() -> drive.runVelocity(new ChassisSpeeds(-0.25, 0, 0)))
                 .withTimeout(1.0)
                 .asProxy(),
                 
            drive.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.25, 0)))
                 .until(climber::isReadyToClimb)
                 .asProxy(),
                 
            // CRITICAL: Stop the drivebase once we are latched!
            Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0)), drive)
                 .asProxy()

            // // 5. Climb!
            // new ClimbCommand(climber, ClimberLevel.L1)
        );
    }
}