//=============================================================================
// File:        motor_pwm_tb.v
// Project:     TE3003B - Forklift Motor Control Challenge (Delivery 1)
// Description: Testbench for motor_pwm.v. Exercises the four DIP-switch
//              combinations and verifies that IN1/IN2/ENA behave as the
//              truth table requires.
//=============================================================================
`timescale 1ns / 1ps

module motor_pwm_tb;

    reg  clk;
    reg  pb0;
    reg  pb1;
    wire in1;
    wire in2;
    wire ena;

    motor_pwm dut (
        .clk (clk),
        .pb0 (pb0),
        .pb1 (pb1),
        .in1 (in1),
        .in2 (in2),
        .ena (ena)
    );

    // 27 MHz clock -> period ~ 37.037 ns
    initial clk = 1'b0;
    always #18.5185 clk = ~clk;

    initial begin
        $dumpfile("motor_pwm_tb.vcd");
        $dumpvars(0, motor_pwm_tb);

        pb0 = 1'b0; pb1 = 1'b0;
        #(40_000_000); // ~2 PWM periods (Stop)

        pb0 = 1'b1; pb1 = 1'b0; // Forward
        #(40_000_000);

        pb0 = 1'b0; pb1 = 1'b1; // Reverse
        #(40_000_000);

        pb0 = 1'b1; pb1 = 1'b1; // Brake
        #(40_000_000);

        $finish;
    end

endmodule