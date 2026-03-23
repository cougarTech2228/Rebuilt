package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.OscillateIntakeCommand;

public class Intake extends SubsystemBase{
    private final IntakeIO io;
    
    private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

    public enum IntakeAngle {
        HOME,
        DEPLOYED,
        BUMPED
    }

    public enum IntakeMode {
        INTAKE,
        SPIT,
        IDLE
    }

     public enum OscillateType {
        STOP,
        ONCE,
        WAVE
    }

    private OscillateType currentOscillateType = OscillateType.STOP;
    private IntakeMode currentIntakeMode = IntakeMode.IDLE;

    public Intake(IntakeIO io) {
        // Constructor
        this.io = io;

        SmartDashboard.putNumber("IntakePosition", 0.0);
        SmartDashboard.putNumber("IntakeVelocity", 0.0);
    }

    public void setIntakeAngle(double angle) {
        io.manualSetIntakeAngle(angle);
    }

    public void setIntakeVoltage(double voltage) {
        io.manualSetIntakeVoltage(voltage);
    }
    
    /**
     * Sets intake mode
     * @param mode Intake, Spit, or Idle
     */
    public void setIntakeMode(IntakeMode mode) {
        currentIntakeMode = mode;
        oscillationState = OscillationState.IDLE;
        switch (mode) {
            case IDLE:
                io.manualSetIntakeAngle(IntakeConstants.ANGLE_MOTOR_HOME_POSITION);
                break;
            case INTAKE:
                io.manualSetIntakeAngle(IntakeConstants.ANGLE_MOTOR_DEPLOYED_POSITION);
                break;
            case SPIT:
                io.manualSetIntakeAngle(IntakeConstants.ANGLE_MOTOR_DEPLOYED_POSITION);
                break;
        }
        io.setIntakeMode(mode);
    }

    public void toggleIntake() {
        System.out.println("toggleIntake()");
        if (currentIntakeMode == IntakeMode.IDLE) {
            System.out.println("setIntakeMode(IntakeMode.INTAKE)");
            setIntakeMode(IntakeMode.INTAKE);
        } else {
            System.out.println("setIntakeMode(IntakeMode.IDLE)");
            setIntakeMode(IntakeMode.IDLE);
        }
    }

    /**
     * Sets intake angle
     * @param angle Home or Deployed
     */
    public void setIntakeAngle(IntakeAngle angle) {
        switch (angle) {
            case HOME:
                io.manualSetIntakeAngle(IntakeConstants.ANGLE_MOTOR_HOME_POSITION);
                break;
            case DEPLOYED:
                io.manualSetIntakeAngle(IntakeConstants.ANGLE_MOTOR_DEPLOYED_POSITION);
                break;
            case BUMPED:
                io.manualSetIntakeAngle(IntakeConstants.ANGLE_MOTOR_BUMPED_POSITION);
        }
    }

    private enum OscillationState {
        IDLE,
        ONCE_DEPLOY_1,
        ONCE_BUMP,
        ONCE_DEPLOY_2,

        WAVE_DEPLOY_1,
        WAVE_BUMP_1,
        WAVE_DEPLOY_2,
        WAVE_BUMP_2,
        WAVE_DEPLOY_3,
        WAVE_BUMP_3,
        WAVE_DEPLOY_4,
        WAVE_BUMP_4
    };
    OscillationState oscillationState = OscillationState.IDLE;

    public void periodic() {
        io.updateInputs(intakeInputs);
        Logger.processInputs("Intake", intakeInputs);

        RobotContainer.intakePose = new Pose3d(
            RobotContainer.intakePose.getX(),
            RobotContainer.intakePose.getY(),
            RobotContainer.intakePose.getZ(),
            new Rotation3d(0, Units.degreesToRadians(intakeInputs.intakeEncoder * 360), 0));

        switch (currentOscillateType) {
            case ONCE: {
                switch (oscillationState){
                    case ONCE_DEPLOY_1:
                        if (isDeployed()) {
                            oscillationState = OscillationState.ONCE_BUMP;
                            setIntakeAngle(IntakeAngle.BUMPED);
                        }
                        break;
                    case ONCE_BUMP:
                        if (isAtPosition(IntakeConstants.ANGLE_MOTOR_BUMPED_POSITION)) {
                            oscillationState = OscillationState.ONCE_DEPLOY_2;
                            setIntakeAngle(IntakeAngle.DEPLOYED);
                        }
                        break;
                    case ONCE_DEPLOY_2:
                        if (isDeployed()) {
                            oscillationState = OscillationState.IDLE;
                            setIntakeMode(IntakeMode.INTAKE);
                        }
                        break;
                }
                break;
            }
                
            case STOP:
                break;
            case WAVE:
                switch (oscillationState) {
                    case WAVE_DEPLOY_1:
                        if (isDeployed()) {
                            oscillationState = OscillationState.WAVE_BUMP_1;
                            setIntakeAngle(IntakeConstants.ANGLE_MOTOR_BUMP_WAVE_1);
                        }
                        break;
                    case WAVE_BUMP_1:
                        if (isAtPosition(IntakeConstants.ANGLE_MOTOR_BUMP_WAVE_1)) {
                            oscillationState = OscillationState.WAVE_DEPLOY_2;
                            setIntakeAngle(IntakeAngle.DEPLOYED);
                        }
                        break;
                    case WAVE_DEPLOY_2:
                        if (isDeployed()) {
                            oscillationState = OscillationState.WAVE_BUMP_2;
                            setIntakeAngle(IntakeConstants.ANGLE_MOTOR_BUMP_WAVE_2);
                        }
                        break;
                    case WAVE_BUMP_2:
                        if (isAtPosition(IntakeConstants.ANGLE_MOTOR_BUMP_WAVE_2)) {
                            oscillationState = OscillationState.WAVE_DEPLOY_3;
                            setIntakeAngle(IntakeAngle.DEPLOYED);
                        }
                        break;
                    case WAVE_DEPLOY_3:
                        if (isDeployed()) {
                            oscillationState = OscillationState.WAVE_BUMP_3;
                            setIntakeAngle(IntakeConstants.ANGLE_MOTOR_BUMP_WAVE_3);
                        }
                        break;
                    case WAVE_BUMP_3:
                        if (isAtPosition(IntakeConstants.ANGLE_MOTOR_BUMP_WAVE_3)) {
                            oscillationState = OscillationState.WAVE_DEPLOY_4;
                            setIntakeAngle(IntakeAngle.DEPLOYED);
                        }
                        break;
                    case WAVE_DEPLOY_4:
                        if (isDeployed()) {
                            oscillationState = OscillationState.WAVE_BUMP_4;
                            setIntakeAngle(IntakeConstants.ANGLE_MOTOR_BUMP_WAVE_4);
                        }
                        break;
                    case WAVE_BUMP_4:
                        if (isAtPosition(IntakeConstants.ANGLE_MOTOR_BUMP_WAVE_4)) {
                            oscillationState = OscillationState.IDLE;
                            setIntakeMode(IntakeMode.INTAKE);
                        }
                        break;
                }
                break;
            default:
                break;

        } 
    }

    private boolean isAtPosition(double position){
        return (intakeInputs.angleMotorPIDSetpoint == position &&
            (Math.abs (intakeInputs.angleMotorPosition - intakeInputs.angleMotorPIDSetpoint) < IntakeConstants.ANGLE_PID_THRESHOLD));
    }

    public boolean isRetracted() {
        return isAtPosition (IntakeConstants.ANGLE_MOTOR_HOME_POSITION);
    }

    public boolean isDeployed() {
        return isAtPosition (IntakeConstants.ANGLE_MOTOR_DEPLOYED_POSITION);
    }

    public boolean isSpitting() {
        return intakeInputs.intakeMotorVoltage == IntakeConstants.INTAKE_MOTOR_SPIT_VOLTAGE;
    }

    public void stop() {
        io.stop();
    };

    public void oscillate(OscillateType type) {

        currentOscillateType = type;
        switch (type){
            case ONCE:
                oscillationState = OscillationState.ONCE_DEPLOY_1;
                setIntakeAngle(IntakeAngle.DEPLOYED);
                break;
            case STOP:
                oscillationState = OscillationState.IDLE;
                break;
            case WAVE:
                oscillationState = OscillationState.WAVE_DEPLOY_1;
                setIntakeAngle(IntakeAngle.DEPLOYED);
                break;
            default:
                break;
            
        }
        
    }
}
