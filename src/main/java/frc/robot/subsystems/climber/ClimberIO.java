package frc.robot.subsystems.climber;

public interface ClimberIO {
    
    boolean isExtended();

    void extend();

    void retract();

    void ascend();

    void descend();
}
