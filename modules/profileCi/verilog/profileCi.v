module profileCi #( 
       parameter [7:0] customId = 8'h00)
       (
       input  wire        start,
       input  wire        clock,
       input  wire        reset,
       input  wire        stall,
       input  wire        busIdle,

       input  wire [31:0] valueA,
       input  wire [31:0] valueB,
       input  wire [7:0]  ciN,

       output wire        done,
       output wire [31:0] result
       );
       localparam nrOfCounterBits = 32;
       
       wire [nrOfCounterBits-1:0] counter0, counter1, counter2, counter3;
       reg enable0, enable1, enable2, enable3;
       reg [31:0] s_result;


       wire reset0 = (reset == 1'b1 || (valueB[8] == 1'b1)) ? done : 1'b0;      
       wire reset1 = (reset == 1'b1 || (valueB[9] == 1'b1)) ? done : 1'b0;      
       wire reset2 = (reset == 1'b1 || (valueB[10] == 1'b1)) ? done : 1'b0;      
       wire reset3 = (reset == 1'b1 || (valueB[11] == 1'b1)) ? done : 1'b0;      
       




       always @(posedge clock) begin
              if (reset) begin
                     enable0 <= 1'b0;
                     enable1 <= 1'b0;
                     enable2 <= 1'b0;
                     enable3 <= 1'b0;
              end else if (done) begin
                     if (valueB[0] || valueB[4])
                     enable0 <= valueB[0] & ~valueB[4];
                     if (valueB[1] || valueB[5])
                     enable1 <= valueB[1] & ~valueB[5] & stall;
                     if (valueB[2] || valueB[6])
                     enable2 <= valueB[2] & ~valueB[6] & busIdle;
                     if (valueB[3] || valueB[7])
                     enable3 <= valueB[3] & ~valueB[7];
              end              
              

              

       end



       assign result = (done) ? s_result : 32'd0; 
  
       always @* begin
       //if (s_isMyCi == 1'b0) result <= 32'd0;
       //else case (valueA[1:0])
              case (valueA[1:0])
                     2'd0    : s_result <= counter0;
                     2'd1    : s_result <= counter1;
                     2'd2    : s_result <= counter2;
                     default : s_result <= counter3;
              endcase
       end

       assign done = start && (ciN == customId);

       counter #(.WIDTH(nrOfCounterBits)) Counter0
              (.reset(reset0),
              .clock(clock),
              .enable(enable0),
              .direction(1'b1), /* a 1 is counting up, a 0 is counting down */
              .counterValue(counter0));

       counter #(.WIDTH(nrOfCounterBits)) Counter1
              (.reset(reset1),
              .clock(clock),
              .enable(enable1),
              .direction(1'b1), /* a 1 is counting up, a 0 is counting down */
              .counterValue(counter1));

       counter #(.WIDTH(nrOfCounterBits)) Counter2
              (.reset(reset2),
              .clock(clock),
              .enable(enable2),
              .direction(1'b1), /* a 1 is counting up, a 0 is counting down */
              .counterValue(counter2));

       counter #(.WIDTH(nrOfCounterBits)) Counter3
              (.reset(reset3),
              .clock(clock),
              .enable(enable3),
              .direction(1'b1), /* a 1 is counting up, a 0 is counting down */
              .counterValue(counter3));


endmodule