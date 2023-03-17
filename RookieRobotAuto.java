package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Rookie Robot Auto")
public class RookieRobotAuto extends LinearOpMode {
    DcMotor fl, bl, fr, br, left_slide, right_slide;

    int fl_pos = 0;
    int fr_pos = 0;
    int bl_pos = 0;
    int br_pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotor.class, "front_left_motor");
        bl = hardwareMap.get(DcMotor.class, "back_left_motor");
        fr = hardwareMap.get(DcMotor.class, "front_right_motor");
        br = hardwareMap.get(DcMotor.class, "back_right_motor");
        left_slide = hardwareMap.get(DcMotor.class, "left_intake");
        right_slide = hardwareMap.get(DcMotor.class, "right_intake");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        left_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            moveMecanum(0, 6, 0, 0.5);
        }
    }

    void moveMecanum(double forward, double strafe, double rotate, double power) {
        double power_constant = Math.max(Math.abs(forward), Math.abs(strafe));
        double forwardp = (forward / power_constant) * power;
        double strafep = (strafe / power_constant) * power;

        double flp = (forwardp+strafep);
        double blp = (forwardp-strafep);
        double frp = (forwardp-strafep);
        double brp = (forwardp+strafep);

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

        while (Math.abs(fl_pos) <= (forward + strafe) * 45 || Math.abs(fr_pos) <= (forward + strafe) * 45 || Math.abs(bl_pos) <= (forward + strafe) * 45|| Math.abs(br_pos) <= (forward + strafe) * 45) {
            telemetry.addData("Motor", fl_pos);
            telemetry.update();

            fl_pos = fl.getCurrentPosition();
            fr_pos = fr.getCurrentPosition();
            bl_pos = bl.getCurrentPosition();
            br_pos = br.getCurrentPosition();
        }

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        fl_pos = 0;
        fr_pos = 0;
        bl_pos = 0;
        br_pos = 0;

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
