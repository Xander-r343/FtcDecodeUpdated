package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configs.Config;
import org.firstinspires.ftc.teamcode.Sensors.OdoPods;
import org.firstinspires.ftc.teamcode.Subsystems.Aimbots;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
@TeleOp(name = "TELEOP")
public class LatestTeleOp extends OpMode {
    Servo rbgIndicator;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    MecanumDrivetrain drivetrain;
    OdoPods pods;
    ShooterSubsystem robotSubsystem;
    Config config;
    Aimbots aimbots;
    boolean engage;
    int dist;
    int vel;
    boolean flywheelActive;
    double targetH;
    double LLheadingError;
    double PodsIdealAngle;
    @Override
    public void init() {
        drivetrain = new MecanumDrivetrain(1, hardwareMap);
        //initialize OdoPods
        pods = new OdoPods(hardwareMap, drivetrain);
        //initialize Config
        config = new Config();
        rbgIndicator = hardwareMap.get(Servo.class, config.RBGName);
        pods.setPosition((double)blackboard.get(config.Xkey), (double)blackboard.get(config.Ykey) ,(double)blackboard.get(config.HeadingKey));
        //initialize the robotSubsystem class
        robotSubsystem = new ShooterSubsystem(hardwareMap);
        aimbots = new Aimbots((int)blackboard.get(config.AllianceKey), pods, hardwareMap);
        engage = false;
        dist = 0;
        vel = 3500;
        flywheelActive = false;
        aimbots.startLL();
        aimbots.switchPipeline(0);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        if(flywheelActive) {
            robotSubsystem.setFlywheelVelocity(vel);
        }
        else{
            robotSubsystem.setFlywheelVelocity(0);
        }
        aimbots.update();
        //intake & belt controls
        LLheadingError = aimbots.getHeadingErrorLL();
        PodsIdealAngle = aimbots.getIdealAngle();

        if(gamepad1.right_trigger > 0){
            robotSubsystem.spinIntake(gamepad1.right_trigger);
            robotSubsystem.spinBelt(gamepad1.right_trigger);
        }
        else if(gamepad1.right_bumper){
            robotSubsystem.spinIntake(-1);
            robotSubsystem.spinBelt(-1);
        }
        else{
            robotSubsystem.spinIntake(0.6);
        }
        // Set launch velocity light
        if((robotSubsystem.getRpm()) < vel+50 && robotSubsystem.getRpm() > vel-50){
            rbgIndicator.setPosition(config.Green);
        }
        else{
            rbgIndicator.setPosition(0.388);
        }

        // control launch feeder
        if(gamepad1.left_trigger > 0) {
            robotSubsystem.spinBelt(gamepad1.left_trigger);
            robotSubsystem.spinIntake(gamepad1.left_trigger);
            robotSubsystem.setServoPosition(1);
        }
        else if(gamepad1.left_bumper){
            robotSubsystem.spinBelt(-1);
            robotSubsystem.spinIntake(-1);
        }
        else{
            robotSubsystem.spinBelt(0);
            robotSubsystem.setServoPosition(0.45);
            robotSubsystem.spinIntake(0.6);
        }
        //flywheel controls
        if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
            vel -= 50;
        }
        else if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
            vel+= 50;
        }

        //auto aim
        if(gamepad1.a) {
            aimbots.update();
            if(aimbots.LLstatusIsValid()){
                targetH = pods.getHeading()-aimbots.getHeadingErrorLL();
                //pods.holdHeading(targetH,1);
            }
            else {
                targetH = aimbots.getIdealAngle();
                //pods.holdHeading(targetH, 1);
            }
            vel = robotSubsystem.flywheelGetRpmFromTable((int)aimbots.calculateSideLengthUsingPods());
            pods.holdHeading(targetH,1);
        }
        else if(!gamepad1.a){
            drivetrain.drive(gamepad1.left_stick_y*1, -1*gamepad1.left_stick_x, gamepad1.right_stick_x*0.8);
        }

        if(currentGamepad1.b && !previousGamepad1.b){
            if(flywheelActive){
                flywheelActive = false;
            }
            else{
                flywheelActive = true;
            }
        }
        if(currentGamepad1.x && !previousGamepad1.x){
            aimbots.stopLL();
            aimbots.startLL();
        }
        if(currentGamepad1.y && !previousGamepad1.y){
            vel = 3600;
        }


        // Control Intake wheel

        // write telemetry to Drive Hub
        telemetry.addData("x", pods.getX());
        telemetry.addData("y", pods.getY());
        telemetry.addData("h", pods.getHeading());
        telemetry.addData("LL", aimbots.LLstatusIsValid());
        telemetry.addData("rpm", robotSubsystem.getRpm());
        telemetry.addData("distance", aimbots.calculateSideLengthUsingPods());
        telemetry.update();
    }


}
