/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Vars;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

public class PixyCamSubsystem extends SubsystemBase {	// Creates the Pixy SPI bus in order to read off of
	SPI pixy = new SPI(Port.kOnboardCS0);
	public PixyCamSubsystem() {
		pixy.setMSBFirst();
		pixy.setChipSelectActiveLow();
		pixy.setClockRate(1000);
		pixy.setSampleDataOnFalling();
		pixy.setClockActiveLow();
	}

	
	public double PIXY_X;
	public double PIXY_Y;
	public double PIXY_WIDTH;
	public double PIXY_HEIGHT;
	public double PIXY_BALL_ANGLE;
	public double PIXY_BALL_DISTANCE;
	
	// The sync byte to get the pixy to talk
	byte PIXY_SYNC_BYTE = 0x5a;
	private int getWord() {
		int word = 0x00;
		int ret = -1;
		ByteBuffer writeBuf = ByteBuffer.allocateDirect(2);
		writeBuf.order(ByteOrder.BIG_ENDIAN);
		ByteBuffer readBuf = ByteBuffer.allocateDirect(2);
		readBuf.order(ByteOrder.BIG_ENDIAN);
		String readString = "";
		String writeString = "";
		  
		writeBuf.put(PIXY_SYNC_BYTE);
		  
		// Flip the writeBuf so it's ready to be read.
		writeBuf.flip();

		// Send the sync / data bit / 0 to get the Pixy to return data appropriately.
		ret = pixy.transaction(writeBuf, readBuf, 2);
		  
		// Set the position back to 0 in the buffer so we read it from the beginning next time.
		readBuf.rewind();
		  
		// Store the contents of the buffer in a int that will be returned to the caller.
		word = (int) (readBuf.getShort() & 0xffff);
		  
		// Clear the buffers, not needed, but nice to know they are cleaned out.
		writeBuf.clear();
		readBuf.clear();
		return(word);
	}
	
	/**
	 * The position the largest block in x
	 * @return position from 0-315 
	public int getXRaw() {
		if(words.get(0)!=0) {
			return words.get(2);
		}
		else {
			return -1;
		}
	}
	*/
	
	/**
	 * The position the largest block in y
	 * @return position from 0-207
	public int getYRaw() {
		if(words.get(0)!=0) {
			return words.get(3);
		}
		else {
			return -1;
		}
	}
	*/
	
	/**
	 * The width of the largest block
	 * @return width from 0-316
	public int getWidthRaw() {
		if(words.get(0)!=0) {
			return words.get(4);
		}
		else {
			return -1;
		}
	}
	*/

	/**
	 * The height of the largest block
	 * @return height from 0-208
	public int getHeightRaw() {
		if(words.get(0)!=0) {
			return words.get(5);
		}
		else {
			return -1;
		}
	}
	
	/**
	 * The distance the pixycam is away from the ball in inches
	 * @return distance in inches
	public double getDistance() {
		// a is created from a datatable that relates distance and width in a rational function
		// engineering notebook pg 36
		double a = 1521.25;
		// the relation of width and distance is described with a rational function
		return a / (double) getWidthRaw();
	}
	*/

	/**
	 * Angle the ball is relative to the camera
	 * @return angle in degrees
	public int getAngleX() {
		return 70 * (getXRaw() - 158) / 316;
	}
	*/

	// Names of the bytes the get word will return in order
	String[] byteNames = {"checksum","signature","x","y","width","height"};
	// Array for storing the bytes the Rio reads off of SPI
	ArrayList<Integer> words = new ArrayList<Integer>();
	int checksumError = 0;
	int i;
	
	// old get word debuging/test code
	// Boolean syncByte;
	// int count = 0;
	

	@Override
	public void periodic() {
		int word;
		int wordsToRead = 0;
		int checksum = 0;
		Boolean syncFound = false;

		// Every iteration of this periodic function will start with a clean ArrayList of words.
		// Then we know the largest object will be at the beginning of this ArrayList.
		words.clear();

		// Read no more than 100 words per periodic function.  100 is just
		// a guess, and the actual amount we can read without interfering 
		// with other periodic robot functions needs to be determined.
		for (int i = 0; i < 100; i++){

			word = getWord();

			// If we have found the start of a frame, read the remaining words of the block.
			// After the last word, break out of the loop to allow other robot functions to run.
			if (wordsToRead > 0){
				if (checksum == 0){
					checksum = word;
				}
				else {
					checksum -= word;
				}

				// Add the word to the array list of words
				words.add(word);

				// If we are done reading words of the block, we check the checksum; 
				// if the checksum is bad, dump the array list and count up the checksum error.
				// Either way, we break out of the loop.
				if (--wordsToRead <= 0) {
					if (checksum != 0)
						words.clear();
						checksumError+=1;
					break;
				}
			}
			else if (word == 0xaa55){
				if (syncFound) {
					// We have seen two sync words and we know we have 
					// found the start of a frame of many blocks.  We prepare
					// to read the remaining words on the next pass of the loop.
					wordsToRead = 6;
				}
				else {
					// We found the start of a block, but we need to wait for the
					// start of the frame to make sure we get the largest object.
					syncFound = true;
				}
			}
			else {
				// Clear out the sync flag since we did not read a sync word.
				syncFound = false;
			}

		}

		// If we did not find a valid block, push a null byte to identify the null block.
		if (words.size() == 0) {
			words.add(0);
		}

		// If the checksum of the block is valid 
		if (checksum == 0) {
			/*
			for(i=0; i<words.size(); i++){
				// String I = "" + i;
				// SmartDashboard.putNumber(I, words.get(i));
				SmartDashboard.putNumber(byteNames[i], words.get(i));
			}
			*/

		/*
		SmartDashboard.putNumber("command " + byteNames[2], getXRaw());
		SmartDashboard.putNumber("command " + byteNames[3], getYRaw());
		SmartDashboard.putNumber("command " + byteNames[4], getWidthRaw());
		SmartDashboard.putNumber("command " + byteNames[5], getHeightRaw());
		SmartDashboard.putNumber("Checksum Errors", checksumError);
		*/
		PIXY_X = words.get(2);
		PIXY_Y = words.get(3);
		PIXY_WIDTH = words.get(4);
		PIXY_HEIGHT = words.get(5);
		PIXY_BALL_ANGLE = 70 * (PIXY_X - 158) / 316;
		PIXY_BALL_DISTANCE = 1521.25 / PIXY_HEIGHT;
		}

		/*
		// old get word debuging/test code
		
		for(i=0; i<=8; i++){
			words.add(getWord());
		}
		
		for(i=0; i<=8; i++){
			String I = "" + i;
			SmartDashboard.putNumber(I, words.get(i));
		}
		SmartDashboard.putNumber("9", count++);
		*/

		/*
		int word = getWord();
		if((word==0xaa55) && (syncByte==true)){
			for(int i =0; i<6; i++){
				// SmartDashboard.putNumber(byteNames[i], getWord());
				SmartDashboard.putNumber("", getWord());
			}
		}
		else if(word==0xaa55){
			syncByte = true;
		}
		*/
	}
}