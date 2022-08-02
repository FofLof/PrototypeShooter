package com.team2073.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2073.robot.FalconSim.SimulationSubsystemFalcons;
import edu.wpi.first.wpilibj.DigitalInput;

public class ApplicationContext {
    public static ApplicationContext instance;
    private CANSparkMax motor1;
    private CANSparkMax motor2;
    private Subsystem subsystem;
    private SimulationSubsystemFalcons simulationSubsystem;
    private DigitalInput dI;
    private DigitalInput dI2;
    private final MotorType kMotorType = MotorType.kBrushless;



    public static ApplicationContext getInstance() {
        if (instance == null) {
            instance = new ApplicationContext();
        }
        return instance;
    }

    public SimulationSubsystemFalcons getSimulationSubsystem() {
        if (simulationSubsystem == null) {
            simulationSubsystem = new SimulationSubsystemFalcons();
        }
        return simulationSubsystem;
    }

    public DigitalInput getdI() {
        if (dI == null) {
            dI = new DigitalInput(0);
        }
        return dI;
    }

    public DigitalInput getdI2() {
        if (dI2 == null) {
            dI2 = new DigitalInput(8);
        }
        return dI2;
    }

    public CANSparkMax getMotor1() {
        if (motor1 == null) {
            motor1 = new CANSparkMax(2, kMotorType);
        }
        return motor1;
    }

    public CANSparkMax getMotor2() {
        if (motor2 == null) {
            motor2 = new CANSparkMax(3, kMotorType);
        }
        return motor2;
    }

    public Subsystem getSubsystem() {
        if (subsystem == null) {
            subsystem = new Subsystem();
        }
        return subsystem;
    }
}
