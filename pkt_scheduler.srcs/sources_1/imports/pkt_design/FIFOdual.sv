// `define DEBUG

module FIFOdual #(
    parameter DWIDTH = 32,
    parameter QUEUE_SIZE = 16
) (
    // General I/O
    input   logic                                       clk,
    input   logic                                       rst,
    // Inqueue data and enable wire 
    output  logic                                       in_valid,
    input   logic                                       inA_enque_en,
    input   logic [DWIDTH-1:0]                          inA_data,
    input   logic                                       inB_enque_en,
    input   logic [DWIDTH-1:0]                          inB_data,
    // Head data and dequeue enable wire, out_deque_en==1 to switch to next data
    input   logic                                       out_deque_en,
    output  logic                                       out_valid,
    output  logic [DWIDTH-1:0]                          out_data
    
);
    logic [63:0] counter;
    logic [DWIDTH-1:0] OutBuff[QUEUE_SIZE-1:0];
    logic [$clog2(QUEUE_SIZE)-1:0] OutBuffHead;
    logic [$clog2(QUEUE_SIZE)-1:0] OutBuffTail;
    logic [$clog2(QUEUE_SIZE)-1:0] nextTail_p1 ;
    logic [$clog2(QUEUE_SIZE)-1:0] nextTail_p2 ;
    logic [$clog2(QUEUE_SIZE)-1:0] nextHead ;
    

    always_comb begin
        nextTail_p1 = (OutBuffTail+2'b01)%QUEUE_SIZE;
        nextTail_p2 = (OutBuffTail+2'b10)%QUEUE_SIZE;
        nextHead = (OutBuffHead+1'b1)%QUEUE_SIZE;
        out_valid = !((OutBuffHead == OutBuffTail) );
        in_valid  = ( (( nextTail_p1 == OutBuffHead )|( nextTail_p2 == OutBuffHead )) ?1'b0:1'b1);
        out_data = OutBuff[OutBuffHead];
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            counter <= 0;
            OutBuffHead <= 0;
            OutBuffTail <= 0;
            for (int i=0; i<QUEUE_SIZE; ++i) begin
                OutBuff[i] <= 0;
            end
        end else begin
            counter <= counter+1;
            if ( (inA_enque_en||inB_enque_en) && in_valid ) begin
                if(inA_enque_en&&inB_enque_en) begin
                    OutBuff[OutBuffTail] <= inA_data;
                    OutBuff[nextTail_p1] <= inB_data;
                    OutBuffTail <= nextTail_p2;
                end else begin
                    if (inA_enque_en) begin
                        OutBuff[OutBuffTail] <= inA_data;
                    end else if (inB_enque_en) begin
                        OutBuff[OutBuffTail] <= inB_data;
                    end else begin
                        $error("FAIL: unexcepted branch");
                    end
                    OutBuffTail <= nextTail_p1;
                end
            end
            
            if ( out_deque_en && out_valid ) begin
                OutBuff[OutBuffHead] <= 0;
                OutBuffHead <= nextHead;
            end
        end
    end
endmodule