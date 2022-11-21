/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

/**
 * Drive Train Class
 */

public class RobotMotors {
    public DcMotorEx frontRight;    // Hub 3 Slot 0    GAMER MOMENTS 2020
    public DcMotorEx frontLeft;     // Hub 2 Slot 0     GAMER MOMENTS 2020
    public DcMotorEx backRight;     // Hub 3 Slot 1    GAMER MOMENTS 2020
    public DcMotorEx backLeft;      // Hub 2 Slot 1    GAMER MOMENTS 2020
    protected ArrayList<DcMotorEx> wheels = new ArrayList<>();

    private static final double TICKS_PER_REV = 751.8; //encoder resolution of 223 rpm motor
    private static final double WHEEL_RADIUS = 2;   // Radius in inches.
    private static final double IN_PER_REV = 2 * Math.PI * WHEEL_RADIUS;
    private static double TICKS_PER_IN = TICKS_PER_REV / IN_PER_REV;


    protected MoveStyle direction;

    public enum MoveStyle {
        FORWARD, BACKWARD, LEFT, RIGHT, TURN_LEFT, TURN_RIGHT
    }

    public RobotMotors(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        frontLeft = (DcMotorEx) fl;
        frontRight = (DcMotorEx) fr;
        backLeft = (DcMotorEx) bl;
        backRight = (DcMotorEx) br;

        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wheels.add(frontLeft);
        wheels.add(frontRight);
        wheels.add(backLeft);
        wheels.add(backRight);

        setMotorPower();
    }

    public ArrayList<DcMotorEx> getWheels() {
        return wheels;
    }

    public void setMotorPower(double flpower, double frpower, double blpower, double brpower){
        frontLeft.setPower(flpower);
        frontRight.setPower(frpower);
        backLeft.setPower(blpower);
        backRight.setPower(brpower);
    }

    public void setMotorPower() {
        for (int i = 0; i < wheels.size(); i++) {
            wheels.get(i).setPower(0);
//            wheels.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void setMotorPower(double power) {
        for (int i = 0; i < wheels.size(); i++) {
            wheels.get(i).setPower(power);
        }
    }

    protected boolean motorsAreBusy() {
        return frontRight.isBusy() && frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy();
    }

    public void turnOffEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void turnOnEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setMotorPower();
    }



    /**
     * Activates the robot motors for a period of time.
     *
     * @param moveTime The time in milliseconds to move.
     * @param speed The speed of the motors.
     *
     */

    public void moveForward(int moveTime, double speed) throws InterruptedException {
        /*
        HOW TO USE:
        MAXSPEED   67.5 in/sec
        DISTANCE TRAVELED = MAXSPEED * (moveTime / 1000) * speed
         */
        frontRight.setPower(speed);
        frontLeft.setPower(speed);
        backRight.setPower(speed);
        backLeft.setPower(speed);
        Thread.sleep(moveTime);
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }


    /**
     * Activates the motors in a way that the robot will turn.
     *
     * @param degree The degree to turn the robot.
     * @param dir The direction the robot should turn in.
     *
     *
     */

    public void turn(int degree, char dir) throws InterruptedException {
        /*
        HOW TO USE:
        Enter degree and direction
         */
        if (dir == 'r') {
            setMotorPower(1, -1, 1, -1);
        }
        else if (dir == 'l') {
            setMotorPower(-1, 1, -1, 1);
        }
        Thread.sleep((int)(440 * degree / 90));
        setMotorPower(0);
    }

    /**
     * Activates the motors in a way that the robot will strafe.
     *
     * @param time The time to activate the robot's motors.
     * @param dir The direction to strafe in.
     *
     */

    public void strafe(int time, char dir) throws InterruptedException {

        if (dir == 'r') {

            frontRight.setPower(-.8);
            frontLeft.setPower(.8);
            backRight.setPower(.8);
            backLeft.setPower(-.8);
        }
        else if (dir == 'l'){

            frontRight.setPower(.8);
            frontLeft.setPower(-.8);
            backRight.setPower(-.8);
            backLeft.setPower(.8);
        }
        Thread.sleep(time);
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    // ENCODER METHODS
    public void moveForwardEn(double distance) /* in, sec */ throws InterruptedException
    {
//        double s= distance/time * TICKS_PER_REV / IN_PER_REV; //in ticks/second
//        resetEncoders();

//        for (int i = 0; i < wheels.size(); i++) {
//            wheels.get(i).setPower(s);
//        }
//
//    Thread.sleep((long)(time*1000));
//        for (int i = 0; i < wheels.size(); i++) {
//            wheels.get(i).setPower(0);
//        }
        resetEncoders();
        turnOnEncoders();
        for(int i=0; i<4; i++)
        {
            wheels.get(i).setTargetPosition((int)(TICKS_PER_IN*distance));
            wheels.get(i).setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheels.get(i).setPower(0.8);
        }
        while(!stopWhenReached())
        {

        }
        setMotorPower(0);
        turnOffEncoders();

    }

    public boolean stopWhenReached()
    {
        int target=frontRight.getTargetPosition();
        if(frontRight.getCurrentPosition()>=target-3 && frontRight.getCurrentPosition()<=target+3)
        {
//            frontRight.setPower(0);
            return true;

        }
        return false;
    }


}