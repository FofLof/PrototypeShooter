package com.team2073.robot;

import com.team2073.common.robot.AbstractRobotDelegate;
import com.team2073.robot.FalconSim.SimulationSubsystemFalcons;

public class RobotDelegate extends AbstractRobotDelegate {
    Subsystem subsystem;
    SimulationSubsystemFalcons simulationSubsystem;
    ApplicationContext appCTX = ApplicationContext.getInstance();
    OperatorInterface oi = new OperatorInterface();

    public RobotDelegate(double period) {
        super(period);
    }
    @Override
    public void robotInit() {
//        simulationSubsystem = appCTX.getSimulationSubsystem();
        oi.init();
//        appCTX.getMotor1().getEncoder().setPosition(1000);
    }

    @Override
    public void robotPeriodic() {

    }


    public enum AutoRun {
        FIVEBALL,
        FOUR_BALL_A,
        FOURBALL_B,
        CLOSE_SHOT,
        TWOBALL;
    }
}
