package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    public enum ClimberLevel {
        L1,
        L3
    }
    private final ClimberIO climberIO;
    private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    Alert climberNotHomedAlert = new Alert("Climber has not been homed", AlertType.kError);
    Alert extensionNotHomedAlert = new Alert("Climber extension has not been homed", AlertType.kError);
    Alert extensionHomeError = new Alert("climber extenstion tried homing before climber!", AlertType.kError);
    Alert climberNotEngaged = new Alert("Trying to climb without claw engaged!", AlertType.kError);

    public Climber(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    public boolean isExtended() {
        return climberIO.isExtended();
    }

    public void extend(ClimberLevel level) {
        climberIO.extend(level);
    }

    public void retract() {
        climberIO.retract();
    }

    public void climb(ClimberLevel level) {
        if (!climberInputs.idClimberReady) {
            climberNotEngaged.set(true);
        } else {
            climberNotEngaged.set(false);
            climberIO.climb(level);
        }
    }

    public void descend() {
        climberIO.descend();
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
        

        climberInputs.hasClimberHomed = climberInputs.hasClimberHomed;

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
    }
}
