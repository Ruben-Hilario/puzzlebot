module motor_pwm (
    input  wire clk,
    input  wire pb0,
    input  wire pb1,
    output wire in1,
    output wire in2,
    output reg  ena
);
    assign in1 = pb0;
    assign in2 = pb1;

    // Periodo de 2 ms a 27 MHz = 54,000 cuentas (PWM a 500 Hz, no audible)
    reg [15:0] counter = 0;
    localparam [15:0] PERIOD = 16'd54_000;
    reg [15:0] pulse;

    always @(*) begin
        case ({pb1, pb0})
            2'b01: pulse = 16'd51_300;  // 95% rapido
            2'b10: pulse = 16'd43_200;  // 80% medio
            default: pulse = 16'd0;     // 00 o 11 -> apagado
        endcase
    end

    always @(posedge clk) begin
        if (counter >= PERIOD - 1) counter <= 0;
        else counter <= counter + 1'b1;
    end

    always @(posedge clk) begin
        ena <= (counter < pulse);
    end
endmodule
