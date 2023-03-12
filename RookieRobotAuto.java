package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Rookie Robot Auto")
public class RookieRobotAuto extends LinearOpMode {
    DcMotor fl, bl, fr, br;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotor.class, "front_left_motor");
        bl = hardwareMap.get(DcMotor.class, "back_left_motor");
        fr = hardwareMap.get(DcMotor.class, "front_right_motor");
        br = hardwareMap.get(DcMotor.class, "back_right_motor");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            moveMecanum(36, 0, 0, 1);
        }
    }

    void moveMecanum(double forward, double strafe, double rotate, double power) {
        double power_constant = Math.max(Math.abs(forward), Math.abs(strafe));
        forward /= power_constant * power;
        strafe /= power_constant * power;

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
}
