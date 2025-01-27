package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
@Config
@Autonomous(name = "AUTOV3")
public class AutoV3 extends LinearOpMode {
    public class Lift {
        private DcMotorEx slideLeft, slideRight;
        private int target1 = 0;

        public Lift(HardwareMap hardwareMap){
            slideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
            slideRight = hardwareMap.get(DcMotorEx.class,"slideRight");

            slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public class LiftUp implements Action{
            private boolean inited = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!inited){
                    slideLeft.setPower(-1);
                    slideRight.setPower(-1);
                    inited = true;
                }
                int pos = slideLeft.getCurrentPosition();
                telemetryPacket.put("lift pos",pos);
                if(slideLeft.getCurrentPosition() > -1000){
                    return true;
                }
                else {
                    slideLeft.setPower(0);
                    slideRight.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp(){
            return new LiftUp();
        }

       public class PIDLift implements Action{
            boolean runPID = true;

           @Override
           public boolean run(@NonNull TelemetryPacket telemetryPacket) {
               PIDController controller;
               double p1 = 0.006, i1 = 0.01, d1 = 0.00005;

               double f1 = 0.005;

               controller = new PIDController(p1,i1,d1);
               double ticks_in_degrees = 1440/180;

               controller.setPID(p1,i1,d1);
               int armPos = slideLeft.getCurrentPosition();
               double pid = controller.calculate(armPos, target1);
               double ff = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

               double power = pid + ff;


               slideLeft.setPower(power);
               slideRight.setPower(power);

               return true;
           }
       }
       public Action armLift(){
            return new PIDLift();
       }
       public class changeTarget implements Action{
            public changeTarget(int target){
                target1 = target;
            }

           @Override
           public boolean run(@NonNull TelemetryPacket telemetryPacket) {
               return false;
           }
       }

    }


    public class Claw{
        private Servo specClaw;
        private CRServo sampClaw;
        public Claw(HardwareMap hardwareMap){
            specClaw = hardwareMap.get(Servo.class,"specClaw");
            sampClaw = hardwareMap.get(CRServo.class,"sampClaw");


        }
        public class OpenFingers implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specClaw.setPosition(0.0);
                return false;
            }
        }
        public Action openFingers(){
            return new OpenFingers();
        }
        public class closeFingers implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {

                specClaw.setPosition(1.0);
                return false;
            }
        }
        public Action closeFingers(){
            return new closeFingers();
        }

        public class sampOut implements Action{

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sampClaw.setPower(0);
                return false;
            }
        }
        public Action openWrist(){
            return new sampOut();
        }
        public class sampIn implements Action{

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sampClaw.setPower(1);
                return false;
            }
        }
        public Action closeWrist(){
            return new sampIn();
        }

    }



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(9,-61,Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);

//        Actions.runBlocking(claw.closeFingers());
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                        .strafeTo(new Vector2d(0,1))
//                .stopAndAdd(lift.changeTarget(-1500))
//                        .waitSeconds(1);
//        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
//
//                .waitSeconds(3)
//                        .strafeTo(new Vector2d(0,0))
//                                .build();
//        Action tab1A = tab1.build();

        Action traj1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(9,-35))
                .build();
        Action traj2 = drive.actionBuilder(new Pose2d(9,-35,Math.toRadians(90)))
                .strafeTo(new Vector2d(30, -38))
                //.strafeTo(new Vector2d(40, -12))
                .splineToConstantHeading(new Vector2d(49,-18), Math.toRadians(0))
                .strafeTo(new Vector2d(49,-60))//push spec 3
                .build();
        //pick up spec 2
        Action traj3 = drive.actionBuilder(new Pose2d(49,-60,Math.toRadians(90)))
                .strafeTo(new Vector2d(3, -38))
                .build();
        //Hang spec 2
        Action traj4 = drive.actionBuilder(new Pose2d(3,-38,Math.toRadians(90)))
                .strafeTo(new Vector2d(30,-65))
                .splineToConstantHeading(new Vector2d(60,-35), Math.toRadians(270))
                .strafeTo(new Vector2d(55, -60))//push spec 4
                .build();
        //pick up spec 3
        Action traj5 = drive.actionBuilder(new Pose2d(55,-60,Math.toRadians(90)))
                .strafeTo(new Vector2d(0, -35))
                .build();
        //Hang spec 3
        Action traj6 = drive.actionBuilder(new Pose2d(0,-35,Math.toRadians(90)))
                .strafeTo(new Vector2d(20,-38))
                .splineToConstantHeading(new Vector2d(62,-15), Math.toRadians(270))
                .strafeTo(new Vector2d(62, -60))//push spec 5
                .build();
        Action traj7 = drive.actionBuilder(new Pose2d(62,-60,Math.toRadians(270)))
                .strafeTo(new Vector2d(-3, -35))
                .build();
        //Pick up spec 4

        waitForStart();
        Actions.runBlocking(
                //new ParallelAction(
                        //lift.pidAction(-425),
                        new SequentialAction(
                                claw.closeWrist()
                                //traj1,
                                //traj2,
                                //traj3,
                                //traj4,
                              //  traj5,
                            //    traj6,
                          //      traj7
//
//                       // tab1A,
//                        //lift.changeTarget(-250),
//                        //claw.openFingers(),
//                        //trajectoryActionCloseOut,
//                       // lift.changeTarget(0)
//                )
//
                        //)
                )
        );



        telemetry.update();
    }
}