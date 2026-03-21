package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {

    public enum ClimberLevel {
        L1,
        L3
    }
    private final ClimberIO climberIO;
    private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();
    public boolean isDescending = false;

    Alert climberNotHomedAlert = new Alert("Climber has not been homed", AlertType.kError);
    Alert extensionNotHomedAlert = new Alert("Climber extension has not been homed", AlertType.kError);
    Alert extensionHomeError = new Alert("climber extenstion tried homing before climber!", AlertType.kError);
    Alert climberNotEngaged = new Alert("Trying to climb without claw engaged!", AlertType.kError);

    public Climber(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    public void extend(ClimberLevel level) {
        climberIO.extend(level);
    }

    public void retract() {
        climberIO.retract();
    }

    public void climb(ClimberLevel level) {
        if (!climberInputs.isClimberReady) {
            climberNotEngaged.set(true);
        } else {
            climberNotEngaged.set(false);
            climberIO.climb(level);
        }
    }

    public void descend() {
        climberIO.homeClimber();
        isDescending = true;
    }

    public boolean isClimberHome() {
        return climberInputs.isClimberHome;
    }

    public boolean isExtensionHome() {
        return climberInputs.isExtensionHome;
    }

    public void homeExtension() {
        // only home the extension if the climber is already homed
        if (isClimberHome()) {
          climberIO.homeExtension();
        } else {
            extensionHomeError.set(true);
        }
    }

    public void homeClimber() {
        climberIO.homeClimber();
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(climberInputs);

        climberNotHomedAlert.set(!climberInputs.hasClimberHomed);
        extensionNotHomedAlert.set(!climberInputs.hasExtensionHomed);
        
        Logger.processInputs("Climber", climberInputs);
        
        if (!climberInputs.hasClimberHomed || !climberInputs.hasExtensionHomed) {
            // always home the climber before homing the extension
            if (!climberInputs.hasClimberHomed) {
                climberIO.homeClimber();
            }
            if (climberInputs.hasClimberHomed && !climberInputs.hasExtensionHomed) {
                climberIO.homeExtension();
            }
        }

        RobotContainer.climberPose = new Pose3d(
            RobotContainer.climberXBase - (climberInputs.extendMotorPosition * ClimberConstants.EXTENSION_METERS_PER_ROTATION),
            RobotContainer.climberPose.getY(),
            RobotContainer.climberPose.getZ(),
            new Rotation3d(-Units.degreesToRadians(ClimberConstants.CLIMBER_ROATATION_DEG_PER_MOTOR_ROTATION * climberInputs.climberMotorPosition) , 0, 0));

        if (climberInputs.isClimberReady) {
            climberIO.stopExtension();
        }

        // if (isDescending) {
        //     if (climberInputs.isClimberHome) {
        //         isDescending = false;
        //     } else {
        //         if (climberInputs.climberMotorPosition <= 0) {
        //             climberIO.homeClimber();
        //         }
        //     }
        // }
    }

    public boolean isClimbComplete() {
        return climberInputs.isClimbComplete;
    }

    // called from aborted commands, just stop everything!
    public void stop() {
        climberIO.stopClimber();
        climberIO.stopExtension();
    }

    public boolean isSafeToClimb(ClimberLevel level) {
        return (climberIO.isExtended(level) && climberInputs.isClimberReady);
    }

    public boolean isExtended(ClimberLevel level) {
        return climberIO.isExtended(level);
    }

    public boolean isReadyToClimb() {
        return climberInputs.isClimberReady;
    }
}
