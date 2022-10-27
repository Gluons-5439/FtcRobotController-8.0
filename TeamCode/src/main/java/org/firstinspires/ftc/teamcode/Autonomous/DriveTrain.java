/*
 * Copyright (c) 2018 Craig MacFarlane
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Craig MacFarlane nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.RobotMotorsAuto;


public class DriveTrain extends RobotMotorsAuto {


    public DriveTrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        super(frontLeft, frontRight, backLeft, backRight);
    }

    @Override
    public void move(int inches, double power, int[] dir) {
        for (int i = 0; i < 3; i++) {
            wheels.get(i).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheels.get(i).setTargetPosition(getTicks(inches) * dir[i]);
            wheels.get(i).setPower(power * dir[i]);
            wheels.get(i).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while (motorsAreBusy());

        setMotorPower();
    }


//    public void moveForward(int inches, double power) {
//        direction = MoveStyle.FORWARD;
//        move(inches, power, getDirs(direction));
//    }

//    public void moveBackward(int inches, double power) {
//        direction = MoveStyle.BACKWARD;
//        move(inches, power, getDirs(direction));
//    }
//
//    public void turnLeft(int inches, double power) {
//        direction = MoveStyle.TURN_LEFT;
//        move(inches, power, getDirs(direction));
//    }
//
//    public void turnRight(int inches, double power) {
//        direction = MoveStyle.TURN_RIGHT;
//        move(inches, power, getDirs(direction));
//    }
//
//    public void strafeLeft(int inches, double power) {
//        direction = MoveStyle.LEFT;
//        move(inches, power, getDirs(direction));
//    }
//
//    public void strafeRight(int inches, double power) {
//        direction = MoveStyle.BACKWARD;
//        move(inches, power, getDirs(direction));
//    }
}
