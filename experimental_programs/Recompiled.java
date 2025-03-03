package org.firstinspires.ftc.teamcode.TestingFiles;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;
import java.lang.Thread;
import java.util.*;

@Autonomous

public class Recompiled {

	// todo: write your code here
}
//classes

/**
 * Motor class with all motor information
 */
class Motor{
    protected DcMotorEx motor_=null;
    protected int velocity=0;
    protected int targetPosition=0;
    /**
     * Set motor
     * @param motor motor as DcMotorEx object
     */
    protected void setMotor(DcMotorEx motor){
        this.motor_=motor;
    }
    /**
     * Set motor target position
     * @param target double target position in encoder ticks
     */
    protected void setTargetPos(int target){
        this.targetPosition=target;
        this.motor_.setTargetPosition(this.targetPosition);
    }
    /**
     * Set motor velocity
     * @param velocity desired integer velocity for the motor in encoder ticks per second
     */
    protected void setMotorVelocity(int velocity){
        this.velocity=velocity;
        this.motor_.setVelocity(this.velocity);
    }
}
/**
 * IMU class
 */
class Imu{
    protected IMU imu_=null;
    /**
     * Set IMU
     * @param imu IMU object
     */
    protected void setIMU(IMU imu){
        this.imu_=imu;
    }
    /**
     * Reset IMU yaw angle
     */
    protected void resetYaw(){
        this.imu_.resetYaw();
    }
    /**
     * Get robot yaw angle
     * @return Yaw angle as a double in degrees
     */
    protected double getHeading(){
        YawPitchRollAngles orientation=this.imu_.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
/**
 * Drivetrain class
 */
//drivetrain
class Drivetrain{
    //motor information list
    protected Motor[] drivetrain=new Motor[4];
    /**
     * Get motor encoder values
     * @return Array of encoder values for each motor, [LF, LR, RR, RF]
     */
    protected int[] readMotorEncoders(){
        int[] encoder=new int[4];
        for (int i=0;i<4;i++){
            encoder[i]=drivetrain[i].motor_.getCurrentPosition();
        }
        return encoder;
    }
    /**
     * Set drivetrain motors, 0 is left front, 1 is left rear, 2 is right rear, 3 is right front
     * @param motors Array of DcMotorEx objects
     */
    protected void setDrivetrain(DcMotorEx[] motors){
        for (int i=0;i<4;i++){
            Motor motor=new Motor();
            motor.setMotor(motors[i]);
            drivetrain[i]=motor;
        }
    }
}
/**
 * Main robot drivetrain class
 */
class Robot_Drivetrain{
    //robot location information, relative to left-bottom corner of field(0,0).
    //0 degrees theta is facing positive y direction.
    protected double xPos=0;
    protected double yPos=0;
    protected double theta=0;
    //seconds to update robot location via IMU
    protected double secondsToUpdate=0.5;
    //Seconds to wait for verifying robot location data via external sensors, TO BE IMPLEMENTED
    protected double secondsToVerification=5;
    protected ElapsedTime timer=new ElapsedTime();
    //imu and drivetrain
    protected Imu imu=new Imu();
    protected Drivetrain drivetrain=new Drivetrain();
    protected Drivetrain drivetrain_save=drivetrain;
    /**
     * Initialize drivetrain and IMU objects, change IMU orientation here
     * @param motors Array of drivetrain DcMotorEx objects(0 is left front, 1 is left rear, 2 is right rear, 3 is right front)
     * @param imu IMU object
     */
    protected void initiateRobotParts(DcMotorEx[] motors, IMU imu){
        this.drivetrain.setDrivetrain(motors);
        this.drivetrain_save=this.drivetrain;
        this.imu.setIMU(imu);
        //set the orientation of the REV hub on the robot(the IMU location is on one of the corners of the robot)
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    }
    /**
     * Initialize robot position
     * @param xPos Robot x position relative to left-bottom of field
     * @param yPos Robot y position relative to left-bottom of field
     * @param theta Robot orientation relative to positive y direction
     */
    protected void initiateRobotPosition(double xPos, double yPos, double theta){
        this.xPos=xPos;
        this.yPos=yPos;
        this.theta=theta;
    }
    /**
     * Update robot position using IMU
     * @return Updated coordinates in an array [xPos, yPos, theta]
     */
    protected double[] updatePos(){
        //update theta
        this.theta=this.imu.getHeading();
        //read motor encoder changes
        int[] past_encoders=new int[4];
        int[] current_encoders=new int[4];
        double[] decomposed_motor_vectors=new double[4];
        double[] robot_vectors = {0, 0};
        past_encoders=this.drivetrain_save.readMotorEncoders();
        current_encoders=this.drivetrain.readMotorEncoders();
        for (int i=0;i<4;i++){
            decomposed_motor_vectors[i]=Math.sqrt(Math.pow(current_encoders[i]-past_encoders[i],2)/2);
            if (current_encoders[i]-past_encoders[i]<0){
                decomposed_motor_vectors[i]*=-1;
            }
        }
        //calculate robot vectors
        //positive x and y facing vectors(LF and RR)
        robot_vectors[0]=robot_vectors[0]+decomposed_motor_vectors[0]+decomposed_motor_vectors[2];
        robot_vectors[1]=robot_vectors[1]+decomposed_motor_vectors[0]+decomposed_motor_vectors[2];
        //negative x and positive y facing vectors(LR and RF)
        robot_vectors[0]=robot_vectors[0]-decomposed_motor_vectors[1]-decomposed_motor_vectors[3];
        robot_vectors[1]=robot_vectors[1]+decomposed_motor_vectors[1]+decomposed_motor_vectors[3];
        return new double[]{this.xPos,this.yPos,this.theta};
    }
}