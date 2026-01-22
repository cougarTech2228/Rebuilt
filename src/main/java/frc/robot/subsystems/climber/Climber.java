package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
    private final ClimberIO climberIO;

    public Climber(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }
}
