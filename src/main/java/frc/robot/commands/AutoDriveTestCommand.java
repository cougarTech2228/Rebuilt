package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.CTSequentialCommandGroup;

public class AutoDriveTestCommand extends CTSequentialCommandGroup {

    public AutoDriveTestCommand(Drive drive, Climber climber, Turret turret, Intake intake) {
        this.addCommands(
            drive.run(() -> drive.runVelocity(new ChassisSpeeds(-0.25, 0, 0)))
                 .withTimeout(1.0)
                 .asProxy(),
                 
            drive.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.25, 0)))
                 .until(climber::isReadyToClimb)
                 .asProxy(),
                 
            // CRITICAL: Stop the drivebase once we are latched!
            Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0)), drive)
                 .asProxy()
        );
    }
}