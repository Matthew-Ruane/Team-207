/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.utility;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

 
/**
 *
 * @author joe
 */
public class MultiPIDController extends PIDController{
	private String name;
	
    public MultiPIDController(PIDGains gains, PIDSource source, PIDOutput output, double period, String name)
    {
        super(gains.getP(),gains.getI(),gains.getD(),gains.getF(),source,output,period);
        this.name = name;
    }
    
    public MultiPIDController(PIDGains gains, PIDSource source, PIDOutput output, String name)
    {
        super(gains.getP(),gains.getI(),gains.getD(),gains.getF(),source,output);
        this.name = name;
    }

    public void setPID(PIDGains gains) {
        setPID(gains.getP(), gains.getI(), gains.getD(), gains.getF());
        setMaxOutput(gains.getMaxOutput());     
    }
    
    public void setMaxOutput(double maxOutput) {
    	setOutputRange(-maxOutput,maxOutput);
    }

}
