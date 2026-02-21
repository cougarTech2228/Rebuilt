package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final ClimberIO climberIO;
    private boolean hasClimberHomed = false;
    private boolean hasExtensionHomed = false;
    private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    Alert climberNotHomedAlert = new Alert("Climber has not been homed", AlertType.kError);
    Alert extensionNotHomedAlert = new Alert("Climber extension has not been homed", AlertType.kError);

    public Climber(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    public boolean isExtended() {
        return climberIO.isExtended();
    }

    public void extend() {
        climberIO.extend();
    }

    public void retract() {
        climberIO.retract();
    }

    public void climb() {
        climberIO.climb();
    }

    public void descend() {
        climberIO.descend();
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(climberInputs);

        if (!hasExtensionHomed && climberInputs.isExtensionHome) {
            hasExtensionHomed = true;
        }

        if (!hasClimberHomed && climberInputs.isClimberHome) {
            hasClimberHomed = true;
        }

        climberNotHomedAlert.set(!hasClimberHomed);
        extensionNotHomedAlert.set(!hasExtensionHomed);
        
        climberInputs.hasExtensionHomed = hasExtensionHomed;
        climberInputs.hasClimberHomed = hasClimberHomed;

        Logger.processInputs("Climber", climberInputs);

        // always home the climber before homing the extension
        if (!hasClimberHomed) {
            climberIO.homeClimber();
        }
        if (hasClimberHomed && !hasExtensionHomed) {
            climberIO.homeExtension();
        }
    }
}
