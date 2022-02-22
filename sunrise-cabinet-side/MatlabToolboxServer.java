package lbrExampleApplications;
/* By Mohammad SAFEEA: Coimbra University-Portugal, 
 * Ensam University-France
 * Modified by Yihao Liu Johns Hopkins University
 * Sept 11 2021
 *
 * KST 1.7
 * 
 * First upload 07-May-2017
 * 
 * Final update 26th-06-2018 
 * 
 * This is a multi-threaded server program that is meant to be used with both
 *    KUKA iiwa 7 R 800
 * or KUKA iiwa 14 R 820.
 * The robot shall be provided with a pneumatic flange, 
 * the server of this application listens on the port 30001.
 * 
 * */

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Scanner;
import java.util.StringTokenizer;
import java.util.logging.Logger;

import com.kuka.connectivity.motionModel.directServo.DirectServo;
import com.kuka.connectivity.motionModel.directServo.IDirectServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
//import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;

 
public class MatlabToolboxServer extends RoboticsAPIApplication
{
    private LBR _lbr; 
	private Controller kuka_Sunrise_Cabinet_1;
	// Frame of the EEF
	/////////////////////////////////////
	// Tool Data
	public static Tool _toolAttachedToLBR;
	/////////////////////////////////////
	// Utility classes
	public static MediaFlangeFunctions mff; // utility functions to read Write mediaflange ports 
    public static StateVariablesOfRobot svr; // state variables publisher
    private BackgroundTask dabak; // server function.
    
    private PTPmotionClass ptpm;
    //-----------------------------
    
    private static double jDispMax[];
    private static double updateCycleJointPos[];
    
    //PTP variables
    public static double jRelVel;
    
	/* Public variables used by the ((BackgroundTask)) class
	 * to dumb the instructions into them
	 */
    public static String daCommand="";
    public static double jpos[];
    public static double EEFpos[];
    public static double EEFposCirc1[];
    public static double EEFposCirc2[];
    public static boolean terminateFlag=false;
    public static boolean directSmart_ServoMotionFlag=false;
    public static double EEfServoPos[];
    public static double jvel[];
    public static double jvelOld[];
    //---------------------------------------------------
    int _port;
    //private static final String stopCharacter="\n"+Character.toString((char)(10));
    private static final String stopCharacter=Character.toString((char)(10));
    private static final String ack="done"+stopCharacter;
    private static final String nak="nak"+stopCharacter;

    @Override
    public void initialize()
    {
        _lbr = getContext().getDeviceFromType(LBR.class);
        kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		// INitialize the variables
        jpos=new double[7];
        jDispMax=new double[7];
        EEFpos=new double[7];
        EEFposCirc1=new double[7];
        EEFposCirc2=new double[7];
        updateCycleJointPos=new double[7];
        EEfServoPos=new double[7];
        jvel=new double[7];
        jvelOld=new double[7];
        for(int i=0;i<7;i++)
        {
        	jpos[i]=0;
        	EEFpos[i]=0;
        	EEFposCirc1[i]=0;
            EEFposCirc2[i]=0;
        	jDispMax[i]=0;
        	updateCycleJointPos[i]=0;
        	EEfServoPos[i]=0;
        	jvel[i]=0;
        	jvelOld[i]=0;
        }
        terminateFlag=false;
        daCommand="";
        

        // Start the server
        _port=30001;
		int timeout=60*1000;  // milli seconds
		dabak=new BackgroundTask(_port,timeout,kuka_Sunrise_Cabinet_1,_lbr);
		// Call instructors of utility classes
		mff= new MediaFlangeFunctions(kuka_Sunrise_Cabinet_1,_lbr,  dabak);
		svr=new StateVariablesOfRobot( _lbr, dabak);
		ptpm=new PTPmotionClass(_lbr,dabak,kuka_Sunrise_Cabinet_1);
			
		
		
    }

    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is collision free.
     */
    private void moveToInitialPosition()
    {
        _lbr.move(
        		ptp(0., Math.PI / 180 * 20., 0., -Math.PI / 180 * 70., 0.,
                        Math.PI / 180 * 90., 0.).setJointVelocityRel(0.15));
        /* Note: The Validation itself justifies, that in this very time instance, the load parameter setting was
         * sufficient. This does not mean by far, that the parameter setting is valid in the sequel or lifetime of this
         * program */
        try
        {
        	
        }
        catch (IllegalStateException e)
        {
            getLogger().info("Omitting validation failure for this sample\n"
                    + e.getMessage());
        }
    }
    
    

    public void moveToSomePosition()
    {
        _lbr.move(
                ptp(0., Math.PI / 180 * 20., 0., -Math.PI / 180 * 60., 0.,
                        Math.PI / 180 * 90., 0.));
    }

    /**
     * Main Application Routine
     */
    @Override
    public void run()
    {
        //moveToInitialPosition();
        // You shall send an acknowledgment message to the server 
        // for each
        // instruction that do not sent something back.
        // This loop will be deactivated when the direct_servo function is being used
    	while(terminateFlag==false)
        {
    		try
    		{
    			////////////////////////////////
    			// Attach default tool equal to flange
    			double[] tempx1={0,0,0,0,0,0};
    			attachTheToolToFlange("defaultToolEqualFlange",tempx1);
    			///////////////////////////////////////////////
        while(terminateFlag==false)
        {
			/////////////////////////////////////
			// Attach tool
			if(daCommand.startsWith("TFtrans"))
			{
				String st=daCommand;
				double[] tempx2=getTransformDataFromCommand(st);
				if(tempx2.length==6)
				{
					attachTheToolToFlange("KST17Tool",tempx2);
			        dabak.sendCommand(ack);
					daCommand="";
					getLogger().info("The tool is attached successfully");
					String message;
					message="X tool "+Double.toString(tempx2[0]);
					getLogger().info(message) ;		
					message="Y tool "+Double.toString(tempx2[1]);
					getLogger().info(message) ;	
					message="Z tool "+Double.toString(tempx2[2]);
					getLogger().info(message) ;	
					message="A tool "+Double.toString(tempx2[3]);
					getLogger().info(message) ;		
					message="B tool "+Double.toString(tempx2[4]);
					getLogger().info(message) ;	
					message="C tool "+Double.toString(tempx2[5]);
					getLogger().info(message) ;	
				}
				else
				{
			        dabak.sendCommand(nak);
					daCommand="";
					getLogger().info("Could not attach tool to robot, App terminated");
					terminateFlag=true;
					break;
				}
			
			}
			/////////////////////////////////////
			else if(daCommand.startsWith("startDirectServoJoints"))
        	{
        		directSmart_ServoMotionFlag=true;
        		dabak.sendCommand(ack);
        		daCommand="";
        		getLogger().info("realtime control initiated");
        		directServoStartJoints();
        		getLogger().info("realtime control terminated");
        		
        	}
        	else if(daCommand.startsWith("stDcEEf_"))
        	{
        		directSmart_ServoMotionFlag=true;
        		dabak.sendCommand(ack);
        		daCommand="";
        		getLogger().info("realtime control in Cartesian space initiated");
        		directServoStartCartezian();
        		getLogger().info("realtime control terminated");
        		
        	}
        	else if(daCommand.startsWith("stSmtSEEf_"))
        	{
        		directSmart_ServoMotionFlag=true;
        		dabak.sendCommand(ack);
        		daCommand="";
        		getLogger().info("realtime control in Cartesian space initiated");
        		smartServoStartCartezian();
        		getLogger().info("realtime control terminated");
        		
        	}
        	// raltime velocity control in joint space
        	else if(daCommand.startsWith("stVelDcJoint_"))
        	{
        		directSmart_ServoMotionFlag=true;
        		dabak.sendCommand(ack);
        		daCommand="";
        		getLogger().info("Velocity-mode, realtime control in joint space initiated");
        		directServoStartVelMode();
        		getLogger().info("realtime control terminated");
        		
        	}

        	else if(daCommand.startsWith(
        			"startSmartImpedneceJoints"))
        	{
        		// pre-processing step
        		directSmart_ServoMotionFlag=true;
        		double[] variables=
        				SmartServoWithImpedence.
        				getControlParameters(daCommand);
        		if(variables.length>6)
        		{
	        		double massOfTool=variables[0];
	    			double[] toolsCOMCoordiantes=
	    				{variables[1],variables[2],variables[3]};
	    			double cStifness=variables[4];
	    			double rStifness=variables[5];
	    			double nStifness=variables[6];
	    			getLogger().info("Mass: "+Double.toString(massOfTool));
	    			getLogger().info("COM X : "+Double.toString(toolsCOMCoordiantes[0]));
	    			getLogger().info("COM Y : "+Double.toString(toolsCOMCoordiantes[1]));
	    			getLogger().info("COM Z : "+Double.toString(toolsCOMCoordiantes[2]));
	    			getLogger().info("Stiffness C: "+Double.toString(cStifness));
	    			getLogger().info("Stiffness R: "+Double.toString(rStifness));
	    			getLogger().info("Stiffness N: "+Double.toString(nStifness));
	    			dabak.sendCommand(ack);
	        		daCommand="";
	        		
	    			try
	        		{
	    				getLogger().info("realtime control initiated");
		    			SmartServoWithImpedence.
		    			startRealTimeWithImpedence
		    			( _lbr,kuka_Sunrise_Cabinet_1
		    				,  massOfTool,
		    				toolsCOMCoordiantes, 
		    				cStifness, rStifness,nStifness) ;
	        		}
	        		catch (Exception e) {
	        			// TODO: handle exception
	        			getLogger().error(e.toString());
	        		}
	    			getLogger().info("realtime control terminated");
	        		dabak.sendCommand(ack);
	        		daCommand="";
        		}
        		
        	}
        	// Start the hand guiding mode
        	else if(daCommand.startsWith("handGuiding"))
        	{
        		//FastHandGUiding.handGUiding(_lbr, kuka_Sunrise_Cabinet_1);
        		dabak.sendCommand(ack);
        		daCommand="";
        	}
        	// Start the precise hand guiding mode
        	else if(daCommand.startsWith("preciseHandGuiding"))
        	{
        		/*
    			double weightOfTool;
    			double[] toolsCOMCoordiantes=
    					new double[3];
        		try
        		{
        			// pre-processing step
        			String tempstring=daCommand;
        			double[] toolData=new double[4];
        			toolData=PreciseHandGuidingForUpload2.
        			getWightAndCoordiantesOfCOMofTool
        			(daCommand);
        			tempstring=PreciseHandGuidingForUpload2.LBRiiwa7R800;
        			// separate data of tool to weight+COM
        			weightOfTool=toolData[0];
        			for(int kj=0;kj<3;kj++)
        			{
        				toolsCOMCoordiantes[kj]=
        						toolData[kj+1];	
        			}
        			// start the precise hand guiding
        			PreciseHandGuidingForUpload2.
        			HandGuiding(_lbr, kuka_Sunrise_Cabinet_1
        			,tempstring,weightOfTool,toolsCOMCoordiantes);
        		
        		}
        		catch (Exception e) {
        			// TODO: handle exception
        			getLogger().error(e.toString());
        		}*/
        		dabak.sendCommand(ack);
        		daCommand="";
        	} 
        	
        	// PTP instructions
        	if(daCommand.startsWith("doPTPin"))
        	{
				if(daCommand.startsWith("doPTPinJS"))
				{
					dabak.sendCommand(ack);
					PTPmotionClass.PTPmotionJointSpace();
					daCommand="";
				}
				else if(daCommand.startsWith("doPTPinCSRelEEF"))
				{
					dabak.sendCommand(ack);
					PTPmotionClass.PTPmotionCartizianSpaceRelEEf();
					daCommand="";
				}
				else if(daCommand.startsWith("doPTPinCSRelBase"))
				{
					dabak.sendCommand(ack);
					//getLogger().info("move end effector relative, in base frame");
					PTPmotionClass.PTPmotionCartizianSpaceRelWorld();
					daCommand="";
				}
				else if(daCommand.startsWith("doPTPinCSCircle1"))
				{
					
					dabak.sendCommand(ack);
					PTPmotionClass.PTPmotionCartizianSpaceCircle();
					daCommand="";
				}
				else if(daCommand.startsWith("doPTPinCS"))
				{
					
					dabak.sendCommand(ack);
					PTPmotionClass.PTPmotionCartizianSpace();
					daCommand="";
				}
			}
        	// PTP motion with condition instructions
        	else if(daCommand.startsWith("doPTP!"))
        	{
        		if(daCommand.startsWith("doPTP!inJS"))
				{
        			double[] indices=new double[7];
        			double[] maxTorque=new double[7];
        			double[] minTorque=new double[7];
        			int n=StringManipulationFunctions.get_Indexes_ValBoundaries(daCommand,indices,minTorque,maxTorque);
        			dabak.sendCommand(ack);
    				daCommand="";
        			for(int i=0;i<n;i++)
        			{
        				String strInfo="[minTorque,maxTorque] for joint "+
        			Double.toString(indices[i])+" is "+Double.toString(minTorque[i])
        			+" , "+Double.toString(maxTorque[i]);
        	        	getLogger().info(strInfo);
        			}
        			PTPmotionClass.PTPmotionJointSpaceTorquesConditional( n, indices, minTorque, maxTorque);
				}
        		else if(daCommand.startsWith("doPTP!inCS"))
				{
        			double[] indices=new double[7];
        			double[] maxTorque=new double[7];
        			double[] minTorque=new double[7];
        			int n=StringManipulationFunctions.get_Indexes_ValBoundaries(daCommand,indices,minTorque,maxTorque);
        			dabak.sendCommand(ack);
    				daCommand="";
        			for(int i=0;i<n;i++)
        			{
        				String strInfo="[minTorque,maxTorque] for joint "+
        			Double.toString(indices[i])+" is "+Double.toString(minTorque[i])
        			+" , "+Double.toString(maxTorque[i]);
        	        	getLogger().info(strInfo);
        			}
        			PTPmotionClass.PTPmotionLineCartizianSpaceTorquesConditional(n, indices, minTorque, maxTorque);
				}
        		else if(daCommand.startsWith("doPTP!CSCircle1"))
				{
        			double[] indices=new double[7];
        			double[] maxTorque=new double[7];
        			double[] minTorque=new double[7];
        			int n=StringManipulationFunctions.get_Indexes_ValBoundaries(daCommand,indices,minTorque,maxTorque);
        			dabak.sendCommand(ack);
    				daCommand="";
        			for(int i=0;i<n;i++)
        			{
        				String strInfo="[minTorque,maxTorque] for joint "+
        			Double.toString(indices[i])+" is "+Double.toString(minTorque[i])
        			+" , "+Double.toString(maxTorque[i]);
        	        	getLogger().info(strInfo);
        			}
        			PTPmotionClass.PTPmotionJointSpaceTorquesConditional( n, indices, minTorque, maxTorque);
				}
        		
        	}
			else if(daCommand.startsWith("jRelVel"))
			{
				getLogger().info(daCommand);
				jRelVel=StringManipulationFunctions.jointRelativeVelocity(daCommand);
				dabak.sendCommand(ack);
				daCommand="";
			}
        	// Get torques of joints
			else if(daCommand.startsWith("Torques"))
			{
				if(daCommand.startsWith("Torques_ext_J"))
				{
					svr.sendJointsExternalTorquesToClient();
					daCommand="";
				}
				else if(daCommand.startsWith("Torques_m_J"))
				{
					svr.sendJointsMeasuredTorquesToClient();
					daCommand="";
				}		
			}
        	// Get parameters of end effector
			else if(daCommand.startsWith("Eef"))
			{			
				if(daCommand.equals("Eef_force"))
				{
					svr.sendEEFforcesToClient();
					daCommand="";
				}
				else if(daCommand.equals("Eef_moment"))
				{
					svr.sendEEFMomentsToClient();
					daCommand="";
				}	
				else if(daCommand.equals("Eef_pos"))
				{
					svr.sendEEfPositionToClient();
					daCommand="";
				}
			}
        	
			/*else if(daCommand.length()>0)
			{
				getLogger().warn("Unrecognized instruction: "+daCommand);
				try {
					Thread.sleep(100);
					// daCommand=""; // do not use
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
			}*/
        	
        	
        	}
    		}
    		catch(Exception e)
    		{
    			getLogger().error(e.toString());
    			daCommand="";
    			dabak.sendCommand(nak);
    		}
    		
        }
    }

	////////////////////////////////
	// Attach the tool to the flange functions
	private double[] getTransformDataFromCommand(String cmd)
	{
	
		double[] val=null;
		String thestring=cmd;
		int numericDataCount=0;
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			// First token is the instruction, should be deleted
			String temp=st.nextToken();
			// Following tokens are data of transform from TCP to frame of flange
			val=new double[6];
			int j=0;
			while(st.hasMoreTokens())
			{
				String stringData=st.nextToken();
				if(j<7)
				{
				//getLogger().warn(jointString);
				val[j]=
						Double.parseDouble(stringData);
				numericDataCount=numericDataCount+1;
				}
				
				j++;
			}				
		}
		
		if (numericDataCount==6)
		{
			return val;
		}
		else
		{
			double[] ret={1,2};
			return ret;
		}
	}
	
	private void attachTheToolToFlange(String string,double[] x) {
		// TODO Ap�ndice de m�todo gerado automaticamente
		String TOOL_FRAME = string;
		double[] TRANSFORM_OF_TOOL = x; 
		double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 0 }; 
		LoadData _loadData = new LoadData();
		double MASS=0;
		_loadData.setMass(MASS);
		_loadData.setCenterOfMass(
		CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
		CENTER_OF_MASS_IN_MILLIMETER[2]);
		
		_toolAttachedToLBR = new Tool(TOOL_FRAME, _loadData);
		XyzAbcTransformation trans = XyzAbcTransformation.ofRad(
		TRANSFORM_OF_TOOL[0], TRANSFORM_OF_TOOL[1],
		TRANSFORM_OF_TOOL[2], TRANSFORM_OF_TOOL[3],
		TRANSFORM_OF_TOOL[4], TRANSFORM_OF_TOOL[5]);
		
		ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
		+ "(TCP)", trans);
		_toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
		// Attach tool to the robot
		_toolAttachedToLBR.attachTo(_lbr.getFlange());
	}
	
	///////////////////////////////////
    // Velocity-mode, joint space
	public void directServoStartVelMode()
    {

        boolean doDebugPrints = false;
        
        
        JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());
        // Initiate joints velocities and positions
        double[] jPOS=new double[7];
        for(int i=0;i<7;i++)
        {
        	jvel[i]=0;
        	jvelOld[i]=0;
        	jPOS[i]=initialPosition.get(i);
        }
        
        DirectServo aDirectServoMotion = new DirectServo(initialPosition);

        aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

        getLogger().info("Starting realtime velocity control mode");
        _lbr.moveAsync(aDirectServoMotion);

        getLogger().info("Get the runtime of the DirectServo motion");
        IDirectServoRuntime theDirectServoRuntime = aDirectServoMotion
                .getRuntime();
        
        JointPosition destination = new JointPosition(
                _lbr.getJointCount());
        
        try
        {
            // do a cyclic loop
            // Do some timing...
            // in nanosec
        	double dt=0;
        	long t0=System.nanoTime();
        	long t1=t0;
			while(directSmart_ServoMotionFlag==true)
			{

                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // Synchronize with the realtime system
                //theDirectServoRuntime.updateWithRealtimeSystem();

                if (doDebugPrints)
                {
                	getLogger().info("Current fifth joint position " + jpos[5]);
                    getLogger().info("Current joint destination "
                            + theDirectServoRuntime.getCurrentJointDestination());
                }


                Thread.sleep(1);
                //getLogger().warn(Double.toString(jpos[0]));
                //getLogger().info(daCommand);
                JointPosition currentPos = new JointPosition(
                        _lbr.getCurrentJointPosition());
                
                  
                t1=System.nanoTime();
                dt=((double)(t1-t0))/(1e9);
                t0=t1;
                updatejointsPosFromVelAcc(jPOS,dt);
                for (int k = 0; k < destination.getAxisCount(); ++k)
                {            		                		                        
                	destination.set(k, jPOS[k]);
                }                                
	            theDirectServoRuntime.setDestination(destination);
	                
            }
        }
        catch (Exception e)
        {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
            //Print statistics and parameters of the motion
            getLogger().info("Simple Cartesian Test \n" + theDirectServoRuntime.toString());
            getLogger().info("Stop the realtime velocity control mode");

            
        }

        theDirectServoRuntime.stopMotion();
        getLogger().info("Stop realtime velocity control mode");
    }

	// updates joints positions by integration, Midpoint Rieman Sum is utilized
	private void updatejointsPosFromVelAcc(double[] jPOS,double dt)
	{
		for(int i=0;i<7;i++)
		{
			jPOS[i]=jPOS[i]+(jvel[i]+jvelOld[i])*dt/2;
			// update old joints velocities
			jvelOld[i]=jvel[i];
		}		
	}

	public void directServoStartJoints()
    {

        boolean doDebugPrints = false;
        
        
        JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());
        
        
        for(int i=0;i<7;i++)
        {
        	jpos[i]=initialPosition.get(i);
        }
        DirectServo aDirectServoMotion = new DirectServo(initialPosition);

        aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

        getLogger().info("Starting DirectServo motion in position control mode");
        _lbr.moveAsync(aDirectServoMotion);

        getLogger().info("Get the runtime of the DirectServo motion");
        IDirectServoRuntime theDirectServoRuntime = aDirectServoMotion
                .getRuntime();
        
        JointPosition destination = new JointPosition(
                _lbr.getJointCount());
        double disp;
        double temp;
        double absDisp;
        
        try
        {
            // do a cyclic loop
            // Do some timing...
            // in nanosec

			while(directSmart_ServoMotionFlag==true)
			{

                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // Synchronize with the realtime system
                //theDirectServoRuntime.updateWithRealtimeSystem();

                if (doDebugPrints)
                {
                	getLogger().info("Current fifth joint position " + jpos[5]);
                    getLogger().info("Current joint destination "
                            + theDirectServoRuntime.getCurrentJointDestination());
                }


                Thread.sleep(1);
                //getLogger().warn(Double.toString(jpos[0]));
                //getLogger().info(daCommand);
                JointPosition currentPos = new JointPosition(
                        _lbr.getCurrentJointPosition());
                
                
                
                for (int k = 0; k < destination.getAxisCount(); ++k)
                {
                		
                		
                		double dj=jpos[k]-currentPos.get(k);
                		disp= getTheDisplacment( dj);
                		temp=currentPos.get(k)+disp;
                		absDisp=Math.abs(disp);
                		if(absDisp>jDispMax[k])
                		{
                			jDispMax[k]=absDisp;
                		}
                        destination.set(k, temp);
                        updateCycleJointPos[k]=temp;
                        
                }
                	
                
	            theDirectServoRuntime.setDestination(destination);
	                
            }
        }
        catch (Exception e)
        {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
            //Print statistics and parameters of the motion
            getLogger().info("Simple Cartesian Test \n" + theDirectServoRuntime.toString());

            getLogger().info("Stop the DirectServo motion");

            
        }

        theDirectServoRuntime.stopMotion();
        getLogger().info("Stop the DirectServo motion for the stop instruction was sent");
    }

    private void smartServoStartCartezian(){
    	boolean doDebugPrints = false;

        JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());

        for(int i=1;i<7;i++)
        {
        	MatlabToolboxServer.jpos[i]=
        	initialPosition.get(i);
        }

        SmartServo aSmartServoMotion = new SmartServo(initialPosition);

        // Set the motion properties to 10% of the systems abilities
        aSmartServoMotion.setJointAccelerationRel(0.1);
        aSmartServoMotion.setJointVelocityRel(0.1);

        aSmartServoMotion.setMinimumTrajectoryExecutionTime(20e-3);
        getLogger().info("Starting SmartServo motion in position control mode");
        MatlabToolboxServer._toolAttachedToLBR.moveAsync(aSmartServoMotion);

        getLogger().info("Get the runtime of the SmartServo motion");
        ISmartServoRuntime theSmartServoRuntime = aSmartServoMotion
                .getRuntime();

        Frame aFrame = theSmartServoRuntime.getCurrentCartesianDestination(MatlabToolboxServer._toolAttachedToLBR.getDefaultMotionFrame());
        Frame destFrame = aFrame.copyWithRedundancy();
        // Initiate the initial position 
        EEfServoPos[0]=aFrame.getX();
        EEfServoPos[1]=aFrame.getY();
        EEfServoPos[2]=aFrame.getZ();
        EEfServoPos[3]=aFrame.getAlphaRad();
        EEfServoPos[4]=aFrame.getBetaRad();
        EEfServoPos[5]=aFrame.getGammaRad();

        try
        {
        	while(directSmart_ServoMotionFlag==true)
        	{
        		theSmartServoRuntime.updateWithRealtimeSystem();
        		Frame msrPose = theSmartServoRuntime
                        .getCurrentCartesianDestination(MatlabToolboxServer._toolAttachedToLBR.getDefaultMotionFrame());
                Thread.sleep(1);
                destFrame.setX(EEfServoPos[0]);
                destFrame.setY(EEfServoPos[1]);
                destFrame.setZ(EEfServoPos[2]);
                destFrame.setAlphaRad(EEfServoPos[3]);
                destFrame.setBetaRad(EEfServoPos[4]);
                destFrame.setGammaRad(EEfServoPos[5]);
                theSmartServoRuntime.setDestination(destFrame);
        	}
        }
        catch (Exception e)
        {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
            //Print statistics and parameters of the motion
            getLogger().info("Simple Cartesian Test \n" + theSmartServoRuntime.toString());

            getLogger().info("Stop the SmartServo motion");
            
        }
        theSmartServoRuntime.stopMotion();
        getLogger().info("Stop the SmartServo motion for the stop instruction was sent");
    	
    }

	private void directServoStartCartezian() {
		
		boolean doDebugPrints = false;

        DirectServo aDirectServoMotion = new DirectServo(
                _lbr.getCurrentJointPosition());

        aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

        getLogger().info("Starting DirectServo motion in position control mode");
        MatlabToolboxServer._toolAttachedToLBR.moveAsync(aDirectServoMotion);

        getLogger().info("Get the runtime of the DirectServo motion");
        IDirectServoRuntime theDirectServoRuntime = aDirectServoMotion
                .getRuntime();

        Frame aFrame = theDirectServoRuntime.getCurrentCartesianDestination(MatlabToolboxServer._toolAttachedToLBR.getDefaultMotionFrame());
        Frame destFrame = aFrame.copyWithRedundancy();
        // Initiate the initial position 
        EEfServoPos[0]=aFrame.getX();
        EEfServoPos[1]=aFrame.getY();
        EEfServoPos[2]=aFrame.getZ();
        EEfServoPos[3]=aFrame.getAlphaRad();
        EEfServoPos[4]=aFrame.getBetaRad();
        EEfServoPos[5]=aFrame.getGammaRad();
        
        try
        {
            // do a cyclic loop
            // Do some timing...
            // in nanosec

			while(directSmart_ServoMotionFlag==true)
			{

                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // Synchronize with the realtime system
                theDirectServoRuntime.updateWithRealtimeSystem();
                Frame msrPose = theDirectServoRuntime
                        .getCurrentCartesianDestination(MatlabToolboxServer._toolAttachedToLBR.getDefaultMotionFrame());

                if (doDebugPrints)
                {
                	getLogger().info("Current cartesian goal " + aFrame);
                    getLogger().info("Current joint destination "
                            + theDirectServoRuntime.getCurrentJointDestination());
                }

                Thread.sleep(1);

                // update Cartesian positions
                destFrame.setX(EEfServoPos[0]);
                destFrame.setY(EEfServoPos[1]);
                destFrame.setZ(EEfServoPos[2]);
                destFrame.setAlphaRad(EEfServoPos[3]);
                destFrame.setBetaRad(EEfServoPos[4]);
                destFrame.setGammaRad(EEfServoPos[5]);

                if (doDebugPrints)
                {
                    getLogger().info("New cartesian goal " + destFrame);
                    getLogger().info("LBR position "
	                            + _lbr.getCurrentCartesianPosition(_lbr
	                                    .getFlange()));
                    getLogger().info("Measured cartesian pose from runtime "
	                            + msrPose);
                }

	                theDirectServoRuntime.setDestination(destFrame);
                
            }
        }
        catch (Exception e)
        {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
            //Print statistics and parameters of the motion
            getLogger().info("Simple Cartesian Test \n" + theDirectServoRuntime.toString());

            getLogger().info("Stop the DirectServo motion");
            
        }
        theDirectServoRuntime.stopMotion();
        getLogger().info("Stop the DirectServo motion for the stop instruction was sent");
		
	}


	
	double getTheDisplacment(double dj)
    {
    	double   a=0.07; 
    	double b=a*0.75; 
		double exponenet=-Math.pow(dj/b, 2);
		return Math.signum(dj)*a*(1-Math.exp(exponenet));
		
    }


    /**
     * Main routine, which starts the application
     */
    public static void main(String[] args)
    {
        MatlabToolboxServer app = new MatlabToolboxServer();
        app.runApplication();
    }

}
