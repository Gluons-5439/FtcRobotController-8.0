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

//import android.support.annotation.NonNull;
//import android.support.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Autonomous.DriveTrainVel;
import org.firstinspires.ftc.teamcode.Hardware.CarouselTurn;
import org.firstinspires.ftc.teamcode.Hardware.Flywheel;
import org.firstinspires.ftc.teamcode.Hardware.IMU;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Hardware.RobotMotors;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Hardware.Servos;
import org.firstinspires.ftc.teamcode.Hardware.WobbleGoal;




public class Robot {
    public HardwareMap hardwareMap;
    public DriveTrainVel driveTrain;
    public Servos s;
    public Sensors c;
    public IMU imu;
    public RobotMotors robotMotors;
//    public WheelStick wheelStick;
    public Flywheel flywheel;
    public WobbleGoal wobbleGoal;
    public CarouselTurn carouselTurn;
    public Lift lift;
    public Intake intake;

    public VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = "AfmBbcz/////AAAAGbLGg++zzk4MiOrcPTc3t9xQj3QHfISJprebOgt5JJ4+83xtFO+ApGlI3GVY/aMgCpoGEIzaJse9sXiYDiLYpJQlGDX765tWJUrqM+pzqLxVXjWA1J6c968/YqYq74Vq5emNxGHj5SF3HP3m43Iq/YYgkSdMv4BR+RThPPnIIzrbAjEAHHtMgH7vVh036+bcw9UqBfSdD/IBqrKpJLERn5+Qi/4Q4EoReCC0CTDfZ+LcY0rUur0QZRkMpxx/9s4eCgIU+qfOcSlBvjoX7QAQ2MImUME1y5yJiyaWueamnhRBOwERGBuDKyGp4eBWp4i3esJcplrWYovjzPg9fL7Thy8v9KnrHy22PUFAYY+1vjKp";
    public TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    BNO055IMU gyro;
    public Orientation angles;


    double previousSpeed;
    double previousXInches;
    double previousYInches;
    double previousTime;

    boolean gsFirstRun = true;
    ElapsedTime speedTimer = new ElapsedTime();

    public Robot() {

    }

    public void init(HardwareMap ahwMap) throws InterruptedException {
        hardwareMap = ahwMap;
        initRobot();
    }


    public void initRobot() {

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        DcMotor turnMotor = hardwareMap.dcMotor.get("turnMotor");
//        DcMotor inMotor = hardwareMap.dcMotor.get("inMotor");
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");

        Servo drop = hardwareMap.servo.get("drop");
//        Servo boxDrop=hardwareMap.servo.get("boxDrop");

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

// MUST FIX        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
// MUST FIX        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

//      Initialize Vuforia engine

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //Initialize TensorFlow object detection engine
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);


        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(imuParameters);

        imu = new IMU(gyro);


//        s = new Servos(flap, kick, latch, buffer);
        s = new Servos(drop);
        driveTrain = new DriveTrainVel(frontLeft, frontRight, backLeft, backRight);
        robotMotors = new RobotMotors(frontLeft, frontRight, backLeft, backRight);
        carouselTurn = new CarouselTurn(turnMotor);
//        intake = new Intake(inMotor);
        lift = new Lift(liftMotor);
    }

    public void waitForTick(long periodMs) throws InterruptedException {
        long remaining = periodMs - (long) speedTimer.milliseconds();

        if (remaining > 0)
            Thread.sleep(remaining);

        speedTimer.reset();
    }



//    public void driveToPoint(double xInches, double yInches, double heading, double speedModifier) {
//
//
//                telemetry.addLine("5439-starting driveToPoint"
//                        + " X:" + xInches
//                        + " Y:" + yInches
//                        + " Heading:" + heading
//                        + " SPM:" + speedModifier);
//        );
//
//        double wheel_encoder_ticks = 2400;
//        double wheel_diameter = 2.3622;  // size of wheels
//        double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);
//
//        double cubackRightentXInches;
//        double cubackRightentYInches;
//        double startXPos = odometers.getXPos();
//        double startYPos = odometers.getYPos();
//
//        double frontLeftPower;
//        double frontRightPower;
//        double backLeftPower;
//        double backRightPower;
//
//        double cubackRightentSpeed;
//
//        double maxWheelPower;
//        double wheelPower = .15; //Minimum speed we start at
//
//        gsFirstRun = true;
//
//        ElapsedTime timeoutTimer = new ElapsedTime();
//
//        cubackRightentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
//        cubackRightentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
//        double distanceToX = xInches - cubackRightentXInches;
//        double distanceToY = yInches - cubackRightentYInches;
//        cubackRightentSpeed = getSpeed(cubackRightentXInches, cubackRightentYInches);
//
//        double distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the triangle is the remaining distance
//
//        telemetry.addLine(
//                "5439 driveToPoint"
//                        + " XPos:" + Double.toString(cubackRightentXInches)
//                        + " YPos:" + Double.toString(cubackRightentYInches)
//                        + " Wheel Power:" + Double.toString(wheelPower)
//                        + " Distance remaining:" + Double.toString(distanceRemaining)
//        );
//
//        while ((distanceRemaining > 1 || cubackRightentSpeed > 3) && opModeIsActive() && timeoutTimer.seconds() < .75) {
//
//            maxWheelPower = (Math.pow(distanceRemaining / speedModifier, 3) + 25) / 100;
//
//            double speedIncrease = .15;
//
//            wheelPower += speedIncrease;
//            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
//                wheelPower = maxWheelPower;
//            }
//
//            double angleRadians;
//            angleRadians = Math.atan2(distanceToY, distanceToX) - Math.PI / 2;
//
//            double adjustment = headingAdjustment2(heading);
//
//            frontLeftPower = wheelPower * Math.cos(angleRadians) - adjustment;
//            frontRightPower = wheelPower * Math.sin(angleRadians) + adjustment;
//            backLeftPower = wheelPower * Math.sin(angleRadians) - adjustment;
//            backRightPower = wheelPower * Math.cos(angleRadians) + adjustment;
//
//            driveTrain.setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
//
//            telemetry.addData("XPos: ", cubackRightentXInches);
//            telemetry.addData("YPos: ", cubackRightentYInches);
//            telemetry.addData("CubackRightent Speed:", cubackRightentSpeed);
//            telemetry.addData("Wheel Power: ", wheelPower);
//            telemetry.addData("distanceToX: ", distanceToX);
//            telemetry.addData("distanceToY: ", distanceToY);
//            telemetry.addData("Distance remaining: ", distanceRemaining);
//            telemetry.addData("andleradians: ", angleRadians);
//            telemetry.addData("angleradianDegrees: ", Math.toDegrees(angleRadians));
//            telemetry.update();
//
//            cubackRightentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
//            cubackRightentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
//            distanceToX = xInches - cubackRightentXInches;
//            distanceToY = yInches - cubackRightentYInches;
//
//            cubackRightentSpeed = getSpeed(cubackRightentXInches, cubackRightentYInches);
//
//            if (Math.abs(cubackRightentSpeed) > .5) {
//                timeoutTimer.reset();
//            }
//            distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the x and y is the remaining distance
//
//            telemetry.addLine(
//                    "5439 driveToPoint"
//                            + " XPos:" + cubackRightentXInches
//                            + " YPos:" + cubackRightentYInches
//                            + " distanceToX:" + distanceToX
//                            + " distanceToY:" + distanceToY
//                            + " Wheel Power:" + wheelPower
//                            + " Distance remaining:" + distanceRemaining
//                            + " angleradianDegrees:" + Math.toDegrees(angleRadians + Math.PI / 4)
//                            + " cubackRightentSpeed:" + cubackRightentSpeed
//                            + " ajustment:" + adjustment
//                            + " cubackRightent heading:" + imu.getCubackRightentHeading()
//            );
//        }
//        driveTrain.setMotorPower(0, 0, 0, 0);
//    }

}
