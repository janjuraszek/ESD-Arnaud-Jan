`timescale 1ns/1ps

module tb_profileCi;

    localparam CUSTOM_ID = 8'h03;

    reg         start = 0;
    reg         clock = 0;
    reg         reset = 1;
    reg         stall = 0;
    reg         busIdle = 0;
    reg  [31:0] valueA = 0;
    reg  [31:0] valueB = 0;
    reg  [7:0]  ciN = 0;

    wire        done;
    wire [31:0] result;

    profileCi #(.customId(CUSTOM_ID)) DUT (
        .start(start),
        .clock(clock),
        .reset(reset),
        .stall(stall),
        .busIdle(busIdle),
        .valueA(valueA),
        .valueB(valueB),
        .ciN(ciN),
        .done(done),
        .result(result)
    );
    
    
    initial
    begin
      $dumpfile("profileCiSignals.vcd"); /* define the name of the .vcd file that can be viewed by GTKWAVE */
      $dumpvars(1,DUT);             /* dump all signals inside the DUT-component in the .vcd file */
    end

    always #5 clock = ~clock;

    task pulse_control(input [31:0] ctrl);
    begin
        @(negedge clock);
        valueB = ctrl;
        @(negedge clock);
        valueB = 32'd0;
    end
    endtask

    task read_counter(input [1:0] idx);
    begin
        @(negedge clock);
        valueA = {30'd0, idx};
        ciN    = CUSTOM_ID;
        start  = 1'b1;
        @(posedge clock);
        #1;
        $display("[%0t] counter%0d = %0d, done=%b", $time, idx, result, done);
        @(negedge clock);
        start = 1'b0;
    end
    endtask

    initial begin
        repeat (3) @(posedge clock);
        reset = 0;

        // Counter0
        pulse_control(32'h0000_0001);   // enable0
        repeat (5) @(posedge clock);
        read_counter(2'd0);

        pulse_control(32'h0000_0010);   // disable0
        repeat (3) @(posedge clock);
        read_counter(2'd0);

        pulse_control(32'h0000_0100);   // reset0
        read_counter(2'd0);

        // Counter1
        stall = 1'b1;
        pulse_control(32'h0000_0002);   // enable1
        repeat (4) @(posedge clock);
        read_counter(2'd1);

        pulse_control(32'h0000_0020);   // disable1
        repeat (2) @(posedge clock);
        read_counter(2'd1);

        pulse_control(32'h0000_0200);   // reset1
        read_counter(2'd1);

        // Counter2
        busIdle = 1'b1;
        pulse_control(32'h0000_0004);   // enable2
        repeat (4) @(posedge clock);
        read_counter(2'd2);

        pulse_control(32'h0000_0040);   // disable2
        repeat (2) @(posedge clock);
        read_counter(2'd2);

        pulse_control(32'h0000_0400);   // reset2
        read_counter(2'd2);

        // Counter3
        pulse_control(32'h0000_0008);   // enable3
        repeat (6) @(posedge clock);
        read_counter(2'd3);

        pulse_control(32'h0000_0080);   // disable3
        repeat (2) @(posedge clock);
        read_counter(2'd3);

        pulse_control(32'h0000_0800);   // reset3
        read_counter(2'd3);

        // Wrong CI number
        @(negedge clock);
        valueA = 32'd0;
        ciN    = 8'h55;
        start  = 1'b1;
        @(posedge clock);
        #1;
        $display("[%0t] wrong ciN -> result=%0d, done=%b", $time, result, done);
        @(negedge clock);
        start  = 1'b0;

        $finish;
    end

endmodule
