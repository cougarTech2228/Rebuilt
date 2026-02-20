package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final ClimberIO climberIO;

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
}
