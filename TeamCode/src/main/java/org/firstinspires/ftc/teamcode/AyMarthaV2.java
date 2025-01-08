package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
  *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 12/20/2024
 */
@TeleOp(name = "AyMartha RR TeleOP", group = "ITD Teleop")
public class AyMarthaV2 extends OpMode {

    private enum IntakeState{
        IN,
        OUT
    }
    private enum IntakeCurrState{
        IN,
        OUT
    }
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private DcMotorEx OuttakeSliderRight;
    private DcMotorEx OuttakeSliderLeft;
    private Servo IntakeSliderRight;
    private Servo IntakeSliderLeft;
    private Servo IntakeClaw;

    //private Servo OuttakeWrist;
    private Servo OuttakeElbowRight;
    private Servo OuttakeElbowLeft;
    private Servo OuttakeClaw;
    private Servo IntakeElbowRight;
    private Servo IntakeElbowLeft;
    //private Servo IntakeClaw;
    //private CRServo LeftWheel;
    //private CRServo RightWheel;
    private TouchSensor IntakeSensor;
    public static int HIGH_BASKET = 3600;
    public static int HIGH_CHAMBER = 1650;
    private int initialPositionLeft, initialPositionRight;

    private boolean IntakeElbowDown = false;
    private boolean OuttakeElbowDown = false;
    private boolean OuttakeClawOpen = false;
    private boolean IntakeClawOpen = true;
    private boolean IntakeSliderChanged = false;
    final double IntakeClawPositionClose = 1.0;
    final double IntakeClawPositionOpen = 0.00;
    public static double IntakeSliderPositionIN = 0.73;
    final double IntakeSliderPositionOut = 0.35;
    final double IntakeElbowPositionIn = 0.95;
    final double IntakeElbowPositionOut = 0.0;
    final double OuttakeElbowPositionOut = 0.14;
    final double OuttakeElbowPositionIn = 0.85;
    final double OuttakeElbowPositionMiddle = 0.48;
    final double OuttakeWristPositionOut = 0.80;
    final double OuttakeWristPositionIn = 0.00;
    final double OuttakeClawPositionClose = 1.0;
    final double OuttakeClawPositionOpen = 0.00;
    final boolean TurnOuttakeSlidersOff = true;

    private double f1x = 0.00;
    private double f1y = 0.00;
    private double f2x = 0.00;
    private double f2y = 0.00;
    private int intakeCurrentState = 0;

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        //Driving Motors Mapping and Setup
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //Intake
        //Servo Sliders Mapping and Setup
        IntakeSliderRight = hardwareMap.get(Servo.class, "IntakeSliderRight");
        IntakeSliderLeft = hardwareMap.get(Servo.class, "IntakeSliderLeft");

        IntakeSliderRight.setDirection(Servo.Direction.FORWARD);
        IntakeSliderLeft.setDirection(Servo.Direction.REVERSE);

        IntakeSliderLeft.scaleRange(0.0, 1.0);
        IntakeSliderRight.scaleRange(0.0, 1.0);



        //Servo Claw and Elbow Mapping and Setup
        IntakeClaw = hardwareMap.get(Servo.class, "IntakeClaw");
        //RightWheel = hardwareMap.get(CRServo.class, "RightWheel");
        // LeftWheel = hardwareMap.get(CRServo.class, "LeftWheel");
        IntakeElbowRight = hardwareMap.get(Servo.class, "IntakeElbowRight");
        IntakeElbowLeft = hardwareMap.get(Servo.class, "IntakeElbowLeft");
        IntakeSensor = hardwareMap.get(TouchSensor.class, "IntakeSensor");

        // RightWheel.setDirection(CRServo.Direction.FORWARD);
        // LeftWheel.setDirection(CRServo.Direction.REVERSE);
        IntakeElbowRight.setDirection(Servo.Direction.FORWARD);
        IntakeElbowLeft.setDirection(Servo.Direction.REVERSE);

        //IntakeClaw.scaleRange(0.0, 1.0);
        //IntakeElbowRight.scaleRange(0.0, 1.0);
        //IntakeElbowLeft.scaleRange(0.0, 1.0);

        //Outtake
        // Sliders Mapping and Setup
        OuttakeSliderRight = hardwareMap.get(DcMotorEx.class, "OuttakeSliderRight");
        OuttakeSliderLeft = hardwareMap.get(DcMotorEx.class, "OuttakeSliderLeft");

        OuttakeSliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OuttakeSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OuttakeSliderRight.setDirection(DcMotorSimple.Direction.FORWARD);
        OuttakeSliderLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        initialPositionLeft = OuttakeSliderLeft.getCurrentPosition();
        initialPositionRight = OuttakeSliderRight.getCurrentPosition();

        //Servo Claw, Elbow, and Wrist Mapping and Setup
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeElbowRight = hardwareMap.get(Servo.class, "OuttakeElbowRight");
        OuttakeElbowLeft = hardwareMap.get(Servo.class, "OuttakeElbowLeft");
        //OuttakeWrist = hardwareMap.get(Servo.class, "OuttakeWrist");

        OuttakeClaw.setDirection(Servo.Direction.FORWARD);
        OuttakeElbowRight.setDirection(Servo.Direction.FORWARD);
        OuttakeElbowLeft.setDirection(Servo.Direction.REVERSE);

    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        //follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        //follower.update();

        // Mecanum Drivetrain Manually Programmed
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);

        //All Intake Code
        //Intake Slider
        if(gamepad1.cross && !IntakeSliderChanged){
            if(intakeCurrentState==0){
                intakeSlidersElbow(IntakeState.OUT);
                intakeCurrentState=1;
            } else if (intakeCurrentState==1) {
                intakeSlidersElbow(IntakeState.IN);
                intakeCurrentState=0;
            }
            IntakeSliderChanged = true;
        } else if (!gamepad1.cross) {
            IntakeSliderChanged = false;
        }

        //Intake Claw
        /*if(gamepad1.right_trigger>0.25){
            RightWheel.setPower(1.0);
            LeftWheel.setPower(1.0);
        }else if(gamepad1.left_trigger>0.25){
            RightWheel.setPower(0.0);
            LeftWheel.setPower(0.0);
        }*/
        if(gamepad1.right_bumper && !IntakeClawOpen){
            if(IntakeClaw.getPosition() == IntakeClawPositionOpen){
                IntakeClaw.setPosition(IntakeClawPositionClose);
            }else{
                IntakeClaw.setPosition(IntakeClawPositionOpen);
            }
            IntakeClawOpen = true;
        }else if (!gamepad1.right_bumper) {
            IntakeClawOpen = false;
        }

        /*if(IntakeSensor.isPressed()){
            RightWheel.setPower(0.0);
            LeftWheel.setPower(0.0);
        }*/
        //Prepare Intake to get samples
        if(gamepad1.ps && !IntakeElbowDown){
            if(IntakeElbowRight.getPosition() == 1.0){
                IntakeElbowRight.setPosition(IntakeElbowPositionOut);
                IntakeElbowLeft.setPosition(IntakeElbowPositionOut);
                IntakeClaw.setPosition(IntakeClawPositionOpen);
            }else{
                IntakeElbowRight.setPosition(IntakeElbowPositionIn);
                IntakeElbowLeft.setPosition(IntakeElbowPositionIn);
            }
            IntakeElbowDown = true;
        }else if (!gamepad1.ps) {
            IntakeElbowDown = false;
        }

        //All Outtake Code
        //Outtake Sliders Programming
        if(gamepad1.triangle){
            outtakeSliders(HIGH_BASKET, 2000, 10);
            //OuttakeElbowMove(OuttakeElbowPositionOut);
        }else if(gamepad1.square){
            outtakeSliders(HIGH_CHAMBER, 2000, 10);
        }else if(gamepad1.circle){
            OuttakeElbowMove(OuttakeElbowPositionMiddle);
            outtakeSliders(5, 2000, 10);

        }else if(gamepad1.dpad_down){

            outtakeSliders(50, 2000, 10);

        }

        //Outtake elbow automatically goes out after reaching high basket position
        if((OuttakeSliderLeft.getCurrentPosition()>(HIGH_BASKET-20) && OuttakeSliderRight.getCurrentPosition()>(HIGH_BASKET-20))
                && (((OuttakeElbowLeft.getPosition()==OuttakeElbowPositionMiddle || OuttakeElbowRight.getPosition()==OuttakeElbowPositionMiddle))
                || (OuttakeElbowLeft.getPosition()==OuttakeElbowPositionIn || OuttakeElbowRight.getPosition()==OuttakeElbowPositionIn))){

            OuttakeElbowMove(OuttakeElbowPositionOut);
        }
        if(TurnOuttakeSlidersOff && OuttakeSliderRight.getCurrentPosition()<=initialPositionRight && OuttakeSliderLeft.getCurrentPosition()<=initialPositionLeft){
            OuttakeSliderLeft.setPower(0.0);
            OuttakeSliderRight.setPower(0.0);
        }

        //Outtake Elbow Middle
        if(gamepad1.dpad_up){
            OuttakeElbowMove(OuttakeElbowPositionMiddle);
        } else if (gamepad1.dpad_left) {
            //OuttakeElbowRight.setPosition(OuttakeElbowPositionOut);
            //OuttakeElbowLeft.setPosition(OuttakeElbowPositionOut);
            OuttakeElbowMove(OuttakeElbowPositionIn);
            OuttakeClaw.setPosition(OuttakeClawPositionOpen);
        } else if (gamepad1.dpad_right) {
            //OuttakeElbowRight.setPosition(OuttakeElbowPositionOut);
            //OuttakeElbowLeft.setPosition(OuttakeElbowPositionOut);
            OuttakeClaw.setPosition(OuttakeClawPositionClose);
            OuttakeElbowMove(OuttakeElbowPositionOut);
        }

        /*/Outtake Wrist Toggle
        if(gamepad1.dpad_left){
            OuttakeWrist.setPosition(OuttakeWristPositionIn);
        }else if(gamepad1.dpad_right){
            OuttakeWrist.setPosition(OuttakeWristPositionOut);
        }
        */

        //Outtake Claw Toggle
        if(gamepad1.left_bumper && !OuttakeClawOpen){
            if(OuttakeClaw.getPosition() == OuttakeClawPositionOpen){
                OuttakeClaw.setPosition(OuttakeClawPositionClose);
            }else{
                OuttakeClaw.setPosition(OuttakeClawPositionOpen);
            }
            OuttakeClawOpen = true;
        }else if (!gamepad1.left_bumper) {
            OuttakeClawOpen = false;
        }



        //All Telemetry goes here
        // Lets get the target positions.
        telemetry.addData("Intake Slider Right Pos: ", IntakeSliderRight.getPosition() );
        telemetry.addData("Intake Slider Left Pos: ", IntakeSliderLeft.getPosition() );
        telemetry.addData("OuttakeRight Pos: ", OuttakeSliderRight.getCurrentPosition() );
        telemetry.addData("OuttakeLeft Pos: ", OuttakeSliderLeft.getCurrentPosition() );
        telemetry.addData("Outtake Elbow L Pos: ", OuttakeElbowLeft.getPosition() );
        telemetry.addData("Outtake Elbow R Pos: ", OuttakeElbowRight.getPosition() );
        telemetry.addData("Outtake Elbow Down: ", OuttakeElbowDown );
        telemetry.addData("Intake Elbow Down: ", IntakeElbowDown );
        telemetry.addData("Intake Claw Open: ", IntakeClawOpen );

        //Telemetry to get the touchpad x / y
        /*if(gamepad1.touchpad_finger_1){
            if(gamepad1.touchpad_finger_1_x != f1x){
                f1x = gamepad1.touchpad_finger_1_x;
                gamepad1.setLedColor(120,200,240, 1500);
            }
            telemetry.addData("Finger 1 X: ", gamepad1.touchpad_finger_1_x );
            telemetry.addData("Finger 1 Y: ", gamepad1.touchpad_finger_1_y );
        }
        if(gamepad1.touchpad_finger_2){
            telemetry.addData("Finger 2 X: ", gamepad1.touchpad_finger_2_x );
            telemetry.addData("Finger 2 Y: ", gamepad1.touchpad_finger_2_y );
        }*/


    }
    public void outtakeSliders(int targetPosition, int velocity, int timeout){
        // Set run modes for both motors
        OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Synchronize motion
        OuttakeSliderRight.setTargetPosition(targetPosition + initialPositionRight);
        OuttakeSliderLeft.setTargetPosition(targetPosition + initialPositionLeft);

        //Run to target position
        OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set power
        OuttakeSliderRight.setPower(0.9);
        OuttakeSliderLeft.setPower(0.9);
        // while(OuttakeSliderRight.isBusy() && OuttakeSliderLeft.isBusy()){

        //}

        /*
        // Set the same PID coefficients for both motors
        OuttakeSliderRight.setVelocityPIDFCoefficients(2.0, 0.5, 0.1, 0.0);
        OuttakeSliderLeft.setVelocityPIDFCoefficients(2.0, 0.5, 0.1, 0.0);

        // Synchronize motion
        OuttakeSliderRight.setTargetPosition(targetPosition + initialPositionRight);
        OuttakeSliderLeft.setTargetPosition(targetPosition + initialPositionLeft);

        OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        OuttakeSliderRight.setVelocity(velocity);  // Set velocity in ticks per second
        OuttakeSliderLeft.setVelocity(velocity);
        */
    }
    public void intakeSlidersElbow(IntakeState os){
        // Set run modes for both motors

        switch (os){
            case IN:
                IntakeSliderRight.setPosition(IntakeSliderPositionIN);
                IntakeSliderLeft.setPosition(IntakeSliderPositionIN);
                IntakeElbowRight.setPosition(IntakeElbowPositionIn);
                IntakeElbowLeft.setPosition(IntakeElbowPositionIn);
                break;
            case OUT:
                IntakeSliderRight.setPosition(IntakeSliderPositionOut);
                IntakeSliderLeft.setPosition(IntakeSliderPositionOut);
                IntakeElbowRight.setPosition(IntakeElbowPositionOut);
                IntakeElbowLeft.setPosition(IntakeElbowPositionOut);
                break;

        }

    }
    private void OuttakeElbowMove(double OuttakeElbowTargetPosition){
        OuttakeElbowRight.setPosition(OuttakeElbowTargetPosition);
        OuttakeElbowLeft.setPosition(OuttakeElbowTargetPosition);
    }

}