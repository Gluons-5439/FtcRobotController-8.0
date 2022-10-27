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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class Hardware_ {

    ArrayList<DcMotor> wheels = new ArrayList<>();

    // DEVICES
    DcMotor frontRight;     // Hub 3 Slot 0
    DcMotor frontLeft;      // Hub 2 Slot 0
    DcMotor backRight;      // Hub 3 Slot 1
    DcMotor backLeft;       // Hub 2 Slot 1

    // MECHANISMS
    Servo flap;
    Servo kick;
//    Servo release;
    Servo latch;
    CRServo buffer;

    DcMotor WheelStick;
    DcMotor Flywheel;
//    DcMotor wobbleGoal;

    // SENSORS
//    ColorSensor color;      //Hub 3 I2C Bus 1 Name: 'colorSensor'

    public BNO055IMU imu;

    HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();


    public Hardware_(){
        // Empty because we have nothing to initialise when creating a new object. GAMER MOMENTS 2020
    }


    public void init(HardwareMap ahwMap, boolean initAuto) throws InterruptedException {
        hwMap = ahwMap;
        initDevices();
        initMotorSettings(initAuto);
        initDefaultPosition();
    }


    private void initDevices() {
        // Initialise all parts connected to the expansion hubs GAMER MOMENTS 2020


        frontRight = hwMap.dcMotor.get("frontRight");
        frontLeft = hwMap.dcMotor.get("frontLeft");
        backRight = hwMap.dcMotor.get("backRight");
        backLeft = hwMap.dcMotor.get("backLeft");
        WheelStick = hwMap.dcMotor.get("WheelStick");
        Flywheel = hwMap.dcMotor.get("Flywheel");

        flap = hwMap.servo.get("flap");
        kick = hwMap.servo.get("kick");
//        release = hwMap.servo.get("release");
        latch = hwMap.servo.get("latch");
        buffer = hwMap.crservo.get("buffer");


//        color = hwMap.colorSensor.get("colorSensor");
    }

    private void initMotorSettings(boolean initAuto) {
        // Set motor Mode and Direction GAMER MOMENTS 2020


        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelStick.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        Flywheel.setDirection(DcMotor.Direction.FORWARD);
        WheelStick.setDirection(DcMotor.Direction.FORWARD);

        wheels.add(frontLeft);
        wheels.add(frontRight);
        wheels.add(backLeft);
        wheels.add(backRight);

        flap.setDirection(Servo.Direction.FORWARD);
        kick.setDirection(Servo.Direction.FORWARD);
//        release.setDirection(Servo.Direction.FORWARD);
        latch.setDirection(Servo.Direction.FORWARD);

    }

    private void initDefaultPosition() throws InterruptedException {
        // Set default positions for motors. GAMER MOMENTS 2020
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        flap.setPosition(0);
        kick.setPosition(0);
//        release.setPosition(0);
        latch.setPosition(0);
    }

    public void waitForTick(long periodMs) throws InterruptedException {
        long remaining = periodMs - (long) period.milliseconds();

        if (remaining > 0)
            Thread.sleep(remaining);

        period.reset();
    }
}
