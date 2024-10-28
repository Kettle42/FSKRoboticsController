package org.firstinspires.ftc.teamcode.DeepDive;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.function.Function;
import java.util.*;

public class PIDController 
{
    public double integralSum = 0;
    public double kp = 1;
    public double ki = 1;
    public double kd = 1;
    public double timeSave = 1;
    private ElapsedTime timer;
    private double lastError = 0;
    private ArrayList<Double> times; 
    private ArrayList<Double> errors;
    private boolean absoluteError = false;
    private static Function<Double, Double> func = a -> a;
    
    public PIDController(ElapsedTime timer)
    {
        timer.reset();
        this.timer = timer;
        this.times = new ArrayList<Double>();
        this.errors = new ArrayList<Double>();
    }
    
    
    /*
    reference -> where we want to be 
    state     -> where we are
    */
    public double update(double state, double reference)
    {
        double deltatime = this.timer.seconds();
        double error = reference - state;
        
        this.integralSum += error * deltatime;
        
        double derivative = (error - this.lastError) / (deltatime);
        this.lastError = error;
        
        //log("deltaTime: " + deltatime + " seconds");
        timer.reset();
        
        double output = (error * kp) + (derivative * kd) + (this.integralSum * ki);
        /*log("output: " + output);
        log("error: " + error);
        log("derivative: " + derivative);
        log("this.integralSum: " + this.integralSum);
        log("state, reference: " + state + " , " + reference);*/
        return output;
    }

    public double update(double error) {
        double deltatime = this.timer.seconds();
        this.integralSum += error * deltatime;

        double derivative = (error - this.lastError) / (deltatime);
        this.lastError = error;

        //log("deltaTime: " + deltatime + " seconds");
        timer.reset();

        double output = (error * kp) + (derivative * kd) + (this.integralSum * ki);
        /*log("output: " + output);
        log("error: " + error);
        log("derivative: " + derivative);
        log("this.integralSum: " + this.integralSum);
        log("state, reference: " + state + " , " + reference);*/
        return output;
    }
    
    public void setCoefficients(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    
    public double[] getCoefficients(){
        return new double[] {this.kp, this.ki, this.kd};
    }
    
    public double getError(){
        return this.lastError;
    }

    @Override
    public String toString(){
        return "Proportional: " + this.kp + " Integral: " + this.ki + " Derivative: " + this.kd;
    }
    
    public void log(String data)
    {
        // RobotLog.a(data);
    }
    
    public int sign(double n){
        return (int)(Math.abs(n)/n);
    }

}