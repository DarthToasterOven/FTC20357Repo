package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeV2 {
    private DcMotorEx intakeMotor;
    Gamepad gamepad1;
    boolean toggle = false;
    boolean toggle2 = false;
    private Servo gate;
    public IntakeV2(HardwareMap hardwareMap, Gamepad gamepad){
        gamepad1 = gamepad;
        intakeMotor = hardwareMap.get(DcMotorEx.class, " intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gate = hardwareMap.get(Servo.class,("gate"));
    }
    public void runIntake() {
        if (gamepad1.xWasPressed()) {
            gate.setPosition(0);
        }
        if (gamepad1.yWasPressed()) {
            gate.setPosition(0.2);
        }

        if (gamepad1.right_bumper) {
            intakeMotor.setPower(-0.5);
            gate.setPosition(0.2);
        }
        if (gamepad1.rightBumperWasReleased()) {
            intakeMotor.setPower(0);
        }
        if (gamepad1.left_bumper) {
            intakeMotor.setPower(0.5);
            gate.setPosition(0);
        }
        if (gamepad1.leftBumperWasReleased()) {
            intakeMotor.setPower(0);
        }
        if (gamepad1.b) {
            intakeMotor.setPower(0);
        }
    }

    }

