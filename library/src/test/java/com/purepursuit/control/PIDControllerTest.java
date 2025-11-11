package com.purepursuit.control;

import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Unit tests for PIDController class.
 */
public class PIDControllerTest {
    
    private static final double DELTA = 1e-6;
    private PIDController pid;
    
    @Before
    public void setUp() {
        pid = new PIDController(1.0, 0.0, 0.0);
    }
    
    @Test
    public void testProportionalOnly() {
        pid.setSetpoint(10.0);
        pid.setOutputLimits(-10.0, 10.0); // Set wider limits for this test
        double output = pid.calculate(5.0);
        // P-only: output = kP * error = 1.0 * (10 - 5) = 5.0
        assertEquals(5.0, output, DELTA);
    }
    
    @Test
    public void testSetpoint() {
        pid.setSetpoint(100.0);
        assertEquals(100.0, pid.getSetpoint(), DELTA);
    }
    
    @Test
    public void testOutputLimits() {
        pid.setSetpoint(100.0);
        pid.setOutputLimits(-1.0, 1.0);
        
        double output = pid.calculate(0.0); // Large error
        assertTrue(output <= 1.0);
        assertTrue(output >= -1.0);
    }
    
    @Test
    public void testReset() {
        pid.setSetpoint(10.0);
        pid.calculate(5.0);
        
        pid.reset();
        assertEquals(0.0, pid.getError(), DELTA);
        assertEquals(0.0, pid.getIntegral(), DELTA);
    }
    
    @Test
    public void testAtSetpoint() {
        pid.setSetpoint(10.0);
        pid.setTolerance(1.0);
        
        pid.calculate(9.5);
        assertTrue(pid.atSetpoint());
        
        pid.calculate(5.0);
        assertFalse(pid.atSetpoint());
    }
    
    @Test
    public void testGetters() {
        pid.setPID(1.5, 0.5, 0.25);
        assertEquals(1.5, pid.getkP(), DELTA);
        assertEquals(0.5, pid.getkI(), DELTA);
        assertEquals(0.25, pid.getkD(), DELTA);
    }
    
    @Test
    public void testIntegralLimit() {
        PIDController pidWithI = new PIDController(0.0, 1.0, 0.0);
        pidWithI.setSetpoint(10.0);
        pidWithI.setIntegralLimit(5.0);
        
        // Run many iterations to accumulate integral
        for (int i = 0; i < 100; i++) {
            pidWithI.calculate(0.0);
        }
        
        assertTrue(Math.abs(pidWithI.getIntegral()) <= 5.0);
    }
}
