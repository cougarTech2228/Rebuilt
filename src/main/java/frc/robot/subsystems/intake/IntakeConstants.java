package frc.robot.subsystems.intake;

public class IntakeConstants {
    
    public static final double INTAKE_MOTOR_IDLE_VOLTAGE = 0.0;
    public static final double INTAKE_MOTOR_INTAKE_VOLTAGE = -12;
    public static final double INTAKE_MOTOR_SPIT_VOLTAGE = 10;

    public static final double ANGLE_MOTOR_HOME_POSITION = 0.0
    ;
    public static final double ANGLE_MOTOR_DEPLOYED_POSITION = 19.3;
    public static final double ANGLE_MOTOR_BUMPED_POSITION = ANGLE_MOTOR_DEPLOYED_POSITION * 0.4;
    public static final double ANGLE_PID_THRESHOLD = 1;

    public static final double ANGLE_MOTOR_BUMP_WAVE_1 = ANGLE_MOTOR_DEPLOYED_POSITION * 0.8;
    public static final double ANGLE_MOTOR_BUMP_WAVE_2 = ANGLE_MOTOR_DEPLOYED_POSITION * 0.6;
    public static final double ANGLE_MOTOR_BUMP_WAVE_3 = ANGLE_MOTOR_DEPLOYED_POSITION * 0.4;

    public static final double ANGLE_MOTOR_STALL_CURRENT_THRESHOLD = 15;
    public static final double ANGLE_MOTOR_STALL_VELOCITY_THRESHOLD = 100;
    public static final double INTAKE_ENCODER_OFFSET = 0.447266;
    public static final double INTAKE_START_WHEELS_POSITION = 28;
}
