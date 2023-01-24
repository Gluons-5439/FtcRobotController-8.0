package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {


    private Servo claw;
    private Servo claw2;

    private final double CLAW_OPEN = 0.1; //numbers subject to change
    private final double CLAW_CLOSE = 0.24; //numbers subject to change
    private final double CLAW2_OPEN=0.33;
    private final double CLAW2_CLOSE=0.18;

    public Servos(Servo c, Servo b)
    {
        //servo hardware moments
//        r.setDirection(Servo.Direction.FORWARD);
        c.setDirection(Servo.Direction.FORWARD);
        claw = c;
        b.setDirection(Servo.Direction.FORWARD);
        claw2=b;

    }

    public void open() {
        claw.setPosition(CLAW_OPEN);
        claw2.setPosition(CLAW2_OPEN);
    }
//
    public void close() {
        claw.setPosition(CLAW_CLOSE);
        claw2.setPosition(CLAW2_CLOSE);
    }

    public double getClawPosition()
    {
        return claw.getPosition();
    }
    public double getClaw2Position()
    {
        return claw2.getPosition();
    }


//    public void setBoxPosition(Lift lift)
//    {
//        boxOpen=lift.liftMotor.getCurrentPosition()/lift.TICKS_PER_REVOLUTION*boxDrop.MAX_POSITION-0.667;
//        boxClose=lift.liftMotor.getCurrentPosition()/lift.TICKS_PER_REVOLUTION*boxDrop.MAX_POSITION;
//    }
//
//    public double getBoxPosition()
//    {
//        return boxDrop.getPosition();
//    }
}
