package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Servos {


    private Servo drop;
    private Servo boxDrop;

    private final double DROP_OPEN = 0.76; //numbers subject to change
    private final double DROP_CLOSE = 0.6; //numbers subject to change
    private final double DROP_OPEN_LIFTED=1.0;

    private final double BOX_ZERO = 0.73;
    private final double BOX_L_CLOSED = 0.6;

    public double boxOpen=0.667;
    public double boxClose= 0;

    public Servos(Servo d)
    {
        //servo hardware moments
//        r.setDirection(Servo.Direction.FORWARD);
        d.setDirection(Servo.Direction.FORWARD);
        drop = d;
//        b.setDirection(Servo.Direction.FORWARD);
//        boxDrop=b;

    }

    public void open() {drop.setPosition(DROP_OPEN); }

    public void openLifted() {drop.setPosition(DROP_OPEN_LIFTED); }
//
    public void close() {drop.setPosition(DROP_CLOSE);}

    public double getClawPosition()
    {
        return drop.getPosition();
    }

    public void bOpen() {boxDrop.setPosition(boxOpen); }
    //
    public void bClose() {boxDrop.setPosition(boxClose);}

    public void setBoxPosition(Lift lift)
    {
        boxOpen=lift.liftMotor.getCurrentPosition()/lift.TICKS_PER_REVOLUTION*boxDrop.MAX_POSITION-0.667;
        boxClose=lift.liftMotor.getCurrentPosition()/lift.TICKS_PER_REVOLUTION*boxDrop.MAX_POSITION;
    }

    public double getBoxPosition()
    {
        return boxDrop.getPosition();
    }
}
