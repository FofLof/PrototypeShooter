package com.team2073.robot;

import com.revrobotics.CANSparkMax;
import com.team2073.common.controlloop.PidfControlLoop;
import com.team2073.common.periodic.AsyncPeriodicRunnable;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import com.team2073.common.util.*;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Subsystem implements AsyncPeriodicRunnable {
    ApplicationContext appCTX = ApplicationContext.getInstance();
//    CANSparkMax motor1 = appCTX.getMotor1();
//    CANSparkMax motor2 =  appCTX.getMotor2();
//    Servo servo = new Servo(1);
    private final double radiusBigWheel =  2.0 / 12;
    private final double radiusSmallWheel = (2.25/2)/12;
    private final double MAX_RPM = 11000;
    private final double displacementY = (double) (104-52) / 12;
    private Timer timer = new Timer();
    int angle = 0;
    boolean timerRestart = true;
    DigitalInput first = appCTX.getdI();
//    AnalogPotentiometer p = new AnalogPotentiometer(2);
    DigitalInput second = appCTX.getdI2();
    WaitCommand waitCommand = new WaitCommand(200);
    WaitCommand secondWait = new WaitCommand(1000);
    public Subsystem() {

        timer.start();
//        servo.setBounds(2d, 1.5, 1.5, 1.5, 1);


        autoRegisterWithPeriodicRunner();


//        motor2.setInverted(true);
//        motor1.setInverted(true);
    }
//
//    public void setSpeed(double RPM) {
////        motor1.set(RPM / MAX_RPM);
//        motor2.set(RPM / MAX_RPM);
//    }

    boolean velocityFirstStopped = false;
    boolean velocityStopped;
    int velocityCycleNumber = 0;
    int j = 0;

    @Override
    public void onPeriodicAsync() {
//        System.out.println(first.get());
        //setSpeed(500);
        getVelocity();
//        servo.setAngle(0);
//        while (servo.getAngle() < 180) {
//            if (timerRestart) {
//                timer.start();
//                timerRestart = false;
//            }
//            if (timer.hasWaited(8)) {
//                angle++;
//                servo.setAngle(angle);
//                timer.stop();
//                timerRestart = true;
//            }
//        }
//
//        while (servo.getAngle() > 0) {
//            if (timerRestart) {
//                timer.start();
//                timerRestart = false;
//            }
//            if (timer.hasWaited(8)) {
//                angle--;
//                servo.setAngle(angle);
//                timer.stop();
//                timerRestart = true;
//            }
//        }
//        servo.setAngle(180);
//        if (timer.hasWaited(1000)) {
//            servo.setAngle(0);
//            timer.stop();
//            timer.start();
//        }






        //angle = 0;
//        for (int i = 0; i<= 180; i++) {
//            waitCommand.start();
//            setAngle(i);
//        }
        //setSpeed(calcRPM(10));

    }
//
//    public void setAngle(double angle) {
//        angle = MathUtil.clamp(angle, 0, 180);
//        servo.setAngle(angle);
//    }



//    public  double getRPMFromVelocity(double angle, double displacementX) {
//        return ((30 * getVelocity(angle, displacementX)) / (Math.PI * radiusBigWheel));  //+ 3 * ((30 * getVelocity(angle, displacementX)) / (Math.PI * radiusSmallWheel));
//    }
//
//    public  double getRPMFromVelocity2(double angle, double displacementX) {
//        return ((30 * getVelocity(angle, displacementX)) / (Math.PI)) * ((double)12/5);
//    }

//    public  double getVelocity(double angle, double displacementX) {
//        return (Math.sqrt(((-32.1522) *  Math.pow(displacementX, 2))
//                / (2 * (displacementY - (Math.tan(Math.toRadians(angle)) * displacementX)))))
//                * ( 1/ Math.cos(Math.toRadians(angle)));
//    }

    public double calcRPM(double velocity) {
        return (Math.log(velocity) - 1.75) / 2.34E-4;
    }
    public void getVelocity() {
        if(first.get() && second.get()){
            j++;
        }else if (first.get()) {
            if (!velocityFirstStopped) {
                System.out.println("Started");
                timer.start();
                velocityFirstStopped = true;
                velocityStopped = false;
                velocityCycleNumber++;
            }
        }else if (second.get() && velocityFirstStopped) {
            //System.out.println(timer.getElapsedTime() / 1000);
            double velocity = 1.0/(timer.getElapsedTime()/1000d);
            if (!velocityStopped) {
                timer.stop();
                System.out.println("Stopped");
                System.out.println(velocity);
                velocityStopped = true;
                velocityCycleNumber++;
            }
        }
        if (velocityCycleNumber % 2 == 0 && velocityCycleNumber != 0) {
            velocityFirstStopped = false;
        }
    }

}
