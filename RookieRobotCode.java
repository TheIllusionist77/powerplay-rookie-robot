package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Rookie Robot TeleOp")
public class RookieRobotCode extends LinearOpMode {
    double UNCLAMPED = 0.25, CLAMPED = 0;
    double UNLIFTED = 1, LIFTED = 0.8;
    boolean prev, curr;

    DcMotor fl, bl, fr, br, left_slide, right_slide;
    BNO055IMU imu;

    double shift = 0, imu_offset;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo clamp, rotate;

        fl = hardwareMap.get(DcMotor.class, "front_left_motor");
        bl = hardwareMap.get(DcMotor.class, "back_left_motor");
        fr = hardwareMap.get(DcMotor.class, "front_right_motor");
        br = hardwareMap.get(DcMotor.class, "back_right_motor");
        left_slide = hardwareMap.get(DcMotor.class, "left_intake");
        right_slide = hardwareMap.get(DcMotor.class, "right_intake");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        left_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotate = hardwareMap.get(Servo.class, "rotate_servo");
        rotate.setPosition(UNLIFTED);
        clamp = hardwareMap.get(Servo.class, "clamp_servo");
        clamp.setPosition(CLAMPED);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) {
            telemetry.addData("Status", "Calibrating IMU");
            telemetry.update();
        }

        shift = -(getAngle() + 90);
        imu_offset = getAngle();

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {
                fieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            } else {
                fieldRelative(-gamepad1.left_stick_y / 2, gamepad1.left_stick_x / 2, gamepad1.right_stick_x / 2);
            }

            curr = gamepad1.right_bumper;

            if (gamepad1.a) {
                clamp.setPosition(CLAMPED);
            }
            if (gamepad1.b) {
                clamp.setPosition(UNCLAMPED);
            }

            if (!prev && curr) {
                left_slide.setTargetPosition(2000);
                right_slide.setTargetPosition(2000);
                left_slide.setPower(1);
                right_slide.setPower(1);
                left_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotate.setPosition(LIFTED);
            } else if (gamepad1.left_bumper) {
                left_slide.setTargetPosition(0);
                right_slide.setTargetPosition(0);
                left_slide.setPower(1);
                right_slide.setPower(1);
                left_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotate.setPosition(UNLIFTED);
            }

            prev = curr;
            telemetry.addData("Left Slide Height", left_slide.getCurrentPosition());
            telemetry.addData("Right Slide Height", right_slide.getCurrentPosition());
            telemetry.update();
        }
    }

    double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double angle = angles.firstAngle - imu_offset;

        if (angle < -180) {
            angle += 360;
        } else if (angle > 180) {
            angle -= 360;
        }

        return angle;
    }

    void moveMecanum(double forward, double strafe, double rotate) {
        double flp = (forward+strafe+rotate);
        double blp = (forward-strafe+rotate);
        double frp = (forward-strafe-rotate);
        double brp = (forward+strafe-rotate);

        double max = Math.max(Math.max(Math.abs((flp)), Math.abs(blp)), Math.max(Math.abs(frp), Math.abs(brp)));
        if (max > 1) {
            flp /= max;
            blp /= max;
            frp /= max;
            brp /= max;
        }

        fl.setPower(flp);
        bl.setPower(blp);
        fr.setPower(frp);
        br.setPower(brp);
    }

    void fieldRelative(double y, double x, double theta) {

        double magnitude = Math.hypot(y, x);
        double angle = Math.toDegrees(Math.atan2(y, x));

        if (angle < 0) {
            angle += 360;
        }

        double heading = getAngle() + 90;

        if (gamepad1.y) {
            shift = -(heading);
        }

        double modAngle = angle - (heading + shift);
        double forward = magnitude * Math.sin(Math.toRadians(modAngle));
        double strafe = magnitude * Math.cos(Math.toRadians(modAngle));

        moveMecanum(forward, strafe, theta);
    }
}
