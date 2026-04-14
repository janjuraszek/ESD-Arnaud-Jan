module rgb565GrayscaleIse #( parameter[7:0] customInstructionId = 8'h00)
                  ( input wire         start,
                    input wire [31:0]  valueA,
                    input wire [7:0]   iseId,
                    output wire        done,
                    output wire [31:0] result );
                    
wire [7:0] s_red, s_green, s_blue;
wire [4:0] s_red_5, s_blue_5;
wire [5:0] s_green_6;
wire [7:0] s_result;


assign s_red_5   = valueA[15:11];
assign s_green_6 = valueA[10:5];
assign s_blue_5  = valueA[4:0];

assign s_red   = {s_red_5,   3'b0};
assign s_green = {s_green_6, 2'b0};
assign s_blue  = {s_blue_5,  3'b0};

assign s_result = (54*s_red + 183*s_green + 19*s_blue)>>8;
assign result = (start && (iseId == customInstructionId)) ? {24'b0, s_result} : 32'd0;

assign done = start && (iseId == customInstructionId);
	
endmodule
