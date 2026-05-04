

module top (
    input  wire clk_27mhz,
    input  wire rst_n,        // se queda como puerto pero no se usa internamente
    input  wire pb0,
    input  wire pb1,
    output wire l298_in1,
    output wire l298_in2,
    output wire l298_ena
);
    motor_pwm u_motor_pwm (
        .clk (clk_27mhz),
        .pb0 (pb0),
        .pb1 (pb1),
        .in1 (l298_in1),
        .in2 (l298_in2),
        .ena (l298_ena)
    );
endmodule

