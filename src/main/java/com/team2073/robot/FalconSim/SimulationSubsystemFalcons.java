package com.team2073.robot.FalconSim;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import com.team2073.common.controlloop.PidfControlLoop;
import com.team2073.common.periodic.AsyncPeriodicRunnable;
import com.team2073.common.util.MathUtil;
import com.team2073.robot.ApplicationContext;
import com.team2073.robot.FalconSim.PhysicsSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.CTREPCMSim;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;

public class SimulationSubsystemFalcons implements AsyncPeriodicRunnable {

    WPI_TalonFX testFalcon = new WPI_TalonFX(1);

    CANSparkMax testMotor = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);

    ShuffleboardTab testingPIDTuningSimulation = Shuffleboard.getTab("Testing PID tuning");
    private NetworkTableEntry p =testingPIDTuningSimulation.add("P",0).getEntry();
    private NetworkTableEntry i =testingPIDTuningSimulation.add("I",0).getEntry();
    private NetworkTableEntry d =testingPIDTuningSimulation.add("D",0).getEntry();
    private NetworkTableEntry goal =testingPIDTuningSimulation.add("Goal",0).getEntry();
//    PidfControlLoop simulationPID = new PidfControlLoop(p.getDouble(0),i.getDouble(0),d.getDouble(0),0d,1d);
PidfControlLoop simulationPID = new PidfControlLoop(p.getDouble(0),i.getDouble(0),d.getDouble(0),0d,1d);
    TalonFXSensorCollection testEncoder = testFalcon.getSensorCollection();
    Joystick controller = new Joystick(0);

    int count = 0;
    boolean allowMovement = false;
    public double getPosition(){
        return testMotor.getEncoder().getPosition();
    }

    public SimulationSubsystemFalcons() {
        REVPhysicsSim.getInstance().addSparkMax(testMotor, DCMotor.getNEO(1));
        autoRegisterWithPeriodicRunner();
        simulationPID.setPositionSupplier(() -> testMotor.getEncoder().getPosition());
        testMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        PhysicsSim.getInstance().addTalonFX(testFalcon, 0.5, 5100);
        testFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,0);
        testFalcon.selectProfileSlot(0,0);
        testFalcon.config_kF(0,0,0);
        testFalcon.config_kP(0,p.getDouble(0), 0);
        testFalcon.config_kI(0, i.getDouble(0),0);
        testFalcon.config_kD(0, d.getDouble(0), 0);
        testFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 100, 0);
        testFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 100, 0);
        testFalcon.configMotionCruiseVelocity(15000, 0);
        testFalcon.configMotionAcceleration(6000, 0);
    }



    @Override
    public void onPeriodicAsync() {
//        simulationPID = new PidfControlLoop(p.getDouble(0),i.getDouble(0),d.getDouble(0),0d,1d);
//        simulationPID.setPositionSupplier(() -> testMotor.getEncoder().getPosition());
        simulationPID.setPID(p.getDouble(0), i.getDouble(0), d.getDouble(0));
        testFalcon.config_kP(0,p.getDouble(0), 0);
        testFalcon.config_kI(0, i.getDouble(0),0);
        testFalcon.config_kD(0, d.getDouble(0), 0);
        PhysicsSim.getInstance().run();
        REVPhysicsSim.getInstance().run();
//        testMotor.getEncoder().setPosition(1000);
//        System.out.println(testMotor.getAppliedOutput());
        count++;
        if (count > 100) {
            count = 0;
//            System.out.println(goal.getDouble(0));
        }

//
        if (allowMovement) {
            testFalcon.set(ControlMode.MotionMagic, goal.getDouble(0));
            simulationPID.updateSetPoint(goal.getDouble(0));
            testMotor.setVoltage(simulationPID.getOutput() * 12);
            simulationPID.updatePID();
        } else {
            testFalcon.set(ControlMode.PercentOutput, Math.abs(controller.getRawAxis(0)) < 0.2 ? 0 : -controller.getRawAxis(0) * 12);
            testMotor.setVoltage(Math.abs(-controller.getRawAxis(1)) < 0.2 ? 0 : -controller.getRawAxis(1) * 12);

//            System.out.println(-controller.getRawAxis(1));
        }
    }

    public void setAllowMovement(boolean allowMovement) {
        this.allowMovement = allowMovement;
    }
}
