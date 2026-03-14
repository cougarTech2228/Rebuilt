package frc.robot.subsystems.intake;

public class IntakeConstants {
    
    public static final double INTAKE_MOTOR_IDLE_VOLTAGE = 0.0;
    public static final double INTAKE_MOTOR_INTAKE_VOLTAGE = -10;
    public static final double INTAKE_MOTOR_SPIT_VOLTAGE = 10;

    public static final double ANGLE_MOTOR_HOME_POSITION = 0.0;
    public static final double ANGLE_MOTOR_DEPLOYED_POSITION = 35.3;
    public static final double ANGLE_MOTOR_BUMPED_POSITION = 21.8;
    public static final double ANGLE_PID_THRESHOLD = 2;

    public static final double ANGLE_MOTOR_STALL_CURRENT_THRESHOLD = 15;
    public static final double ANGLE_MOTOR_STALL_VELOCITY_THRESHOLD = 100;
    public static final double INTAKE_ENCODER_OFFSET = 0.094482;
}
