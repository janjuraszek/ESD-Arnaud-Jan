module rgb565GrayscaleIse #(parameter [7:0] customInstructionId = 8'd0 )
                           ( input wire         start,
                             input wire [31:0]  valueA,
                                                valueB,
                             input wire [7:0]   iseId,
                             output wire        done,
                             output wire [31:0] result );

  /* we compensate here for the big/little endian problem */

  wire s_isMyIse = (iseId == customInstructionId) ? start : 1'b0;
  wire [31:0] s_grayScaleValues;
  
  assign done   = s_isMyIse;
  assign result = (s_isMyIse == 1'b1) ? s_grayScaleValues : 32'd0;

  rgb565Grayscale pixel1 ( .rgb565(valueA[15:0]),
                         .grayscale(s_grayScaleValues[23:16]));
rgb565Grayscale pixel2 ( .rgb565(valueA[31:16]),
                         .grayscale(s_grayScaleValues[31:24]));
rgb565Grayscale pixel3 ( .rgb565(valueB[15:0]),
                         .grayscale(s_grayScaleValues[7:0]));
rgb565Grayscale pixel4 ( .rgb565(valueB[31:16]),
                         .grayscale(s_grayScaleValues[15:8]));
endmodule
