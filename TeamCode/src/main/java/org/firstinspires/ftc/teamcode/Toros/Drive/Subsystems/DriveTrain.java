package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class DriveTrain {

    private DcMotor FrontLeftMotor,BackLeftMotor,FrontRightMotor,BackRightMotor; //Motors
    IMU imu;
    public double botHeading;
    private boolean Rtoggle, XYtoggle; // Toggles for turning down the speed of the robot
    Gamepad currentGamepad1 = new Gamepad(); //fragment of the toggles. needed just in case
    Gamepad gamepad1; // the gamepad which we intialize when we construct the class in the actual drive program
    public DriveTrain(HardwareMap hardwareMap,Gamepad gamepad){
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        BackRightMotor = hardwareMap.get(DcMotor.class, "br");

        // Sets the zero power behaviors which when it stops will resist force on it causing it to stop faster
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //These two motors need to be in reverse in order for correct movement
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Runs without encoders since they uneeded for the drivtrain during teleop
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Declares Imu based off of logo direction and USB direction
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));



        imu.initialize(parameters);
        imu.resetYaw();

        //creates our gamepad object in order to use controls
        this.gamepad1 = gamepad;


    }

    public void drive (){
        currentGamepad1.copy(gamepad1);

        //Our toggle to slow down either rotationally or just in the x or y directions
        if(currentGamepad1.dpadLeftWasPressed()){
            XYtoggle = !XYtoggle;
        }
        if(currentGamepad1.dpadRightWasPressed()){
            Rtoggle = !Rtoggle;
        }


        //Taking our gamepad inputs
        double x = gamepad1.left_stick_x; // the *1.1 counteracts imperfect strafing
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        //Toggles just slow down the speed to 1/2 of the robots speed/power
        //XYtoggle is for movement on the X and Y axis
        //Rtoggle is for rotational movement
        if(XYtoggle){
            x *= 0.5;
            y *= 0.5;
        }
        else{
            x*=1;
            y*=1;
        }
        if(Rtoggle){
            turn *= 0.5;
        }
        else{
            turn*=1;
        }

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        /**
         In basics this is taking the x and y of the left stick making them into an angle
         with the power being the hypot which is the square root of the sum of squares of the inputs
         more info here https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/hypot
         then takes the sin and cos of the angle making sure to convert to radians. It then creates a max
         using the absolute value of the sin and cos.

         The idea is that where you are going is angle theta with each wheel being a vector and when combined make the target direction when rotated 45 degrees

         Found on YT www.youtube.com/watch?v=gnSW2QpkGXQ which is a video about coding for mecanum drive wheels
         */


        //Calculations for our drive motors

        double fl = (power * cos / max + turn);
        double fr = (power * sin / max - turn);
        double bl = (power * sin / max + turn);
        double br = (power * cos / max - turn);

        /**
         In continuation the power is then calculated with the angles multiplied by the sin or cos divided the difference or sum of the max and turn
         */

        //If statement below is to make sure one motor does not exceed the power limit making it scale down

        if ((power + Math.abs(turn)) > 1) {
            fl /= power + Math.abs(turn);
            fr /= power + Math.abs(turn);
            bl /= power + Math.abs(turn);
            br /= power + Math.abs(turn);
        }



        //Motor Drive
        FrontLeftMotor.setPower(fl);
        FrontRightMotor.setPower(fr);
        BackLeftMotor.setPower(bl);
        BackRightMotor.setPower(br);

//        //Calculates our rotation based off the heading and joystick input
//        //This creates that absolute direction of field centric as it rotates the vectors of the joysticks based off of the angles
//        //These vectors then determine how much we move in a direction
//        //In summary based of the angle the powers will be different changing how the robot moves
//
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//        rotX = -rotX*1.1;
//
//        //The denominator variable ensures a max value so as to not allocate extra unusable power to the robot
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
//
//        //Now we combine all of our vectors to tell how the motors to turn
//        double frontLeftPower = (rotY + rotX + turn) / denominator;
//        double backLeftPower = (rotY - rotX + turn) / denominator;
//        double frontRightPower = (rotY - rotX - turn) / denominator;
//        double backRightPower = (rotY + rotX - turn) / denominator;
//
//
//        //reset imu button
//        if(gamepad1.start){
//            imu.resetYaw();
//        }
//
//        FrontLeftMotor.setPower(frontLeftPower);
//        BackLeftMotor.setPower(backLeftPower);
//        FrontRightMotor.setPower(frontRightPower);
//        BackRightMotor.setPower(backRightPower);



    }


    //Any getter methods that we need in order for telemetry or other use
    public boolean getXToggle(){
        return XYtoggle;
    }
    public boolean getRToggle(){
        return Rtoggle;
    }
    public double getHeading(){return botHeading; }
}
