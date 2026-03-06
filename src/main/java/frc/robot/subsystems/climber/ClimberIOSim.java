package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber.ClimberLevel;

public class ClimberIOSim implements ClimberIO {

    private final DigitalInput climberReadyDIO = new DigitalInput(Constants.DIO_CLIMBER_READY);

    private final DCMotorSim extensionSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.0002, 1.0),
        DCMotor.getNeo550(1)
    );

    private final DCMotorSim climberSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1.0),
        DCMotor.getKrakenX60(1)
    );

    private final ProfiledPIDController extensionPID = new ProfiledPIDController(
        0.5, 0, 0.0,
        new TrapezoidProfile.Constraints(9000.0 / 60.0, 20000.0 / 60.0) 
    );
    
    private final SimpleMotorFeedforward extensionFF = new SimpleMotorFeedforward(
        0.0, 
        12.0 / (11000.0 / 60.0),
        0.014
    );

    private final ProfiledPIDController climberPID = new ProfiledPIDController(
        0.5, 0, 0.0,
        new TrapezoidProfile.Constraints(90.0, 200.0)
    );
    
    private final SimpleMotorFeedforward climberFF = new SimpleMotorFeedforward(
        0.0, 
        12.0 / 100.0,
        0.011
    );

    private double extensionSetpoint = 0.0;
    private double climberSetpoint = 0.0;
    
    private double lastExtensionVel = 0.0;
    private double lastClimberVel = 0.0;

    private boolean isExtensionHoming = false;
    private boolean isClimberHoming = false;
    private boolean hasExtensionHomed = false;
    private boolean hasClimberHomed = false;
    private boolean extensionClosedLoop = false;
    private boolean climberClosedLoop = false;

    private double extensionVolts = 0.0;
    private double climberVolts = 0.0;

    public ClimberIOSim() {
        extensionPID.reset(0);
        climberPID.reset(0);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        if (extensionClosedLoop) {
            double pidVolts = extensionPID.calculate(extensionSim.getAngularPositionRotations(), extensionSetpoint);
            double currentSetpointVel = extensionPID.getSetpoint().velocity;
            double accel = (currentSetpointVel == 0.0) ? 0.0 : (currentSetpointVel - lastExtensionVel) / 0.020;
            
            lastExtensionVel = currentSetpointVel;
            double ffVolts = extensionFF.calculateWithVelocities(currentSetpointVel, accel);
            extensionVolts = MathUtil.clamp(pidVolts + ffVolts, -12.0, 12.0);
        } else {
            extensionPID.reset(extensionSim.getAngularPositionRotations());
            lastExtensionVel = 0.0;
        }
        
        if (climberClosedLoop) {
            double pidVolts = climberPID.calculate(climberSim.getAngularPositionRotations(), climberSetpoint);
            double currentSetpointVel = climberPID.getSetpoint().velocity;
            double accel = (currentSetpointVel == 0.0) ? 0.0 : (currentSetpointVel - lastClimberVel) / 0.020;
            
            lastClimberVel = currentSetpointVel;
            double ffVolts = climberFF.calculateWithVelocities(currentSetpointVel, accel);
            climberVolts = MathUtil.clamp(pidVolts + ffVolts, -12.0, 12.0);
        } else {
            climberPID.reset(climberSim.getAngularPositionRotations());
            lastClimberVel = 0.0;
        }

        extensionSim.setInput(extensionVolts);
        climberSim.setInput(climberVolts);

        extensionSim.update(0.020);
        climberSim.update(0.020);

        // Hardware limit switch / hard-stop emulation
        if (extensionSim.getAngularPositionRotations() <= 0.0 && extensionVolts < 0.0) {
            extensionSim.setState(VecBuilder.fill(0.0, 0.0));
        }
        if (climberSim.getAngularPositionRotations() <= 0.0 && climberVolts < 0.0) {
            climberSim.setState(VecBuilder.fill(0.0, 0.0));
        }

        inputs.isExtensionHome = extensionSim.getAngularPositionRotations() <= 0.05;
        inputs.isClimberHome = climberSim.getAngularPositionRotations() <= 0.05;

        inputs.climberMotorPosition = climberSim.getAngularPositionRotations();
        inputs.climberMotorCurrent = climberSim.getCurrentDrawAmps();
        inputs.climberMotorTemp = 30.0;
        inputs.climberMotorPIDTarget = climberSetpoint;

        inputs.extendMotorPosition = extensionSim.getAngularPositionRotations();
        inputs.extendMotorCurrent = extensionSim.getCurrentDrawAmps();
        inputs.extendMotorTemp = 30.0;
        inputs.extendMotorPIDTarget = extensionSetpoint;

        inputs.isClimberExtended = isExtended(extensionSetpoint);
        inputs.isClimberReady = !climberReadyDIO.get();

        if (!hasExtensionHomed && inputs.isExtensionHome) {
            hasExtensionHomed = true;
        }
        if (!hasClimberHomed && inputs.isClimberHome) {
            hasClimberHomed = true;
        }
        inputs.hasExtensionHomed = hasExtensionHomed;
        inputs.hasClimberHomed = hasClimberHomed;

        if (isExtensionHoming && inputs.isExtensionHome) {
            isExtensionHoming = false;
            stopExtension();
        }
        if (isClimberHoming && inputs.isClimberHome) {
            isClimberHoming = false;
            stopClimber();
        }

        inputs.isClimbComplete = (Math.abs(inputs.climberMotorPIDTarget - inputs.climberMotorPosition) < ClimberConstants.CLIMBER_PID_THRESHOLD);
    }

    @Override
    public void extend(ClimberLevel level) {
        switch (level) {
            case L1: extensionSetpoint = ClimberConstants.EXTENSION_EXTENDED_L1_POSITION; break;
            case L3: extensionSetpoint = ClimberConstants.EXTENSION_EXTENDED_L3_POSITION; break;
            default: extensionSetpoint = 0; break;
        }
        extensionClosedLoop = true;
    }

    @Override
    public void retract() {
        extensionSetpoint = ClimberConstants.EXTENSION_HOME_POSITION;
        extensionClosedLoop = true;
    }

    @Override
    public void climb(ClimberLevel level) {
        switch (level) {
            case L1: climberSetpoint = ClimberConstants.CLIMBER_L1_POSITION; break;
            case L3: climberSetpoint = ClimberConstants.CLIMBER_L3_POSITION; break;
            default: climberSetpoint = 0; break;
        }
        climberClosedLoop = true;
    }

    @Override
    public void descend() {
        climberSetpoint = 0;
        climberClosedLoop = true;
    }

    @Override
    public boolean isExtended(ClimberLevel level) {
        double target = 0;
        switch (level) {
            case L1: target = ClimberConstants.EXTENSION_EXTENDED_L1_POSITION; break;
            case L3: target = ClimberConstants.EXTENSION_EXTENDED_L3_POSITION; break;
        }
        return extensionSetpoint > 0 &&
            (Math.abs(extensionSim.getAngularPositionRotations() - target) < ClimberConstants.EXTENSION_PID_THRESHOLD);
    }

    private boolean isExtended(double setpoint) {
        return extensionSetpoint > 0 &&
            (Math.abs(extensionSim.getAngularPositionRotations() - extensionSetpoint) < ClimberConstants.EXTENSION_PID_THRESHOLD);
    }

    @Override
    public void homeExtension() {
        isExtensionHoming = true;
        if (extensionSim.getAngularPositionRotations() <= 0.05) {
            extensionClosedLoop = false;
            extensionVolts = 0;
        } else {
            if (hasExtensionHomed) {
                extensionSetpoint = 0;
                extensionClosedLoop = true;
            } else {
                extensionClosedLoop = false;
                extensionVolts = ClimberConstants.EXTENSION_HOME_SPEED * 12.0;
            }
        }
    }

    @Override
    public void homeClimber() {
        isClimberHoming = true;
        if (climberSim.getAngularPositionRotations() <= 0.05) {
            climberClosedLoop = false;
            climberVolts = 0;
        } else {
            if (hasClimberHomed) {
                climberSetpoint = 0;
                climberClosedLoop = true;
            } else {
                climberClosedLoop = false;
                climberVolts = ClimberConstants.CLIMBER_HOME_SPEED * 12.0;
            }
        }
    }

    @Override
    public void stopClimber() {
        climberClosedLoop = false;
        climberVolts = 0;
    }

    @Override
    public void stopExtension() {
        extensionClosedLoop = false;
        extensionVolts = 0;
    }
}