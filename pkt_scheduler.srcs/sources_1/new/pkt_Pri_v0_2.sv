`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/08/17 22:33:17
// Design Name: 
// Module Name: pkt_Pri_v0_2
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
import pkt_h::*;


typedef struct packed {
    logic               valid;
    logic [15:0]        NoF;
    logic [3:0]         MatchFail;
    pkHeadInfo          Info;
} RecordSpot;

module pkt_Priorer_v0_2 #(
    parameter DWIDTH = 32,
    parameter SLOT_SIZE = 8,
    parameter SLOT_WIDTH = 8,
    parameter PRIOR_WIDTH = 6,
    parameter SET = 2
) (
    input   logic                                       clk,
    input   logic                                       rst,
    // Data & PktInfo for priority caculation
    input   logic                                       in_en,
    output  logic                                       in_valid,
    input   pkHeadInfo                                  in_pkt_info,
    input   logic [DWIDTH-1:0]                          in_data,
    // Outdata & priority, data and priority enabled when valid==1'b1, if 
    output  logic                                       out_valid,
    output  logic [DWIDTH-1:0]                          out_data,
    output  logic [PRIOR_WIDTH-1:0]                     out_prior
);

    // logic [5:0] KeyHash;
    RecordSpot          ppl_pkt [SLOT_SIZE:0];
    logic [DWIDTH-1:0]  ppl_data [SLOT_SIZE:0];
    RecordSpot          recordStack  [SLOT_SIZE-1:0][SLOT_WIDTH-1:0];
    
    logic match [SLOT_SIZE-1:0];
    int matchNum [SLOT_SIZE-1:0];

    logic empty [SLOT_SIZE-1:0];
    int emptyNum [SLOT_SIZE-1:0];
    
    int LRU_Num [SLOT_SIZE-1:0];

    always_comb begin
        for(int i=0;i<SLOT_SIZE;i++)begin
            empty[i] = 0;
            emptyNum[i] = 0;
            for(int j=0;j<SLOT_WIDTH;j++)begin
                if(recordStack[i][j].valid == 0 || recordStack[i][j].NoF >= 32'd64)begin
                    empty[i] = 1;
                    emptyNum[i] = j;
                end
            end
        end
    end

    always_comb begin
        for(int i=0;i<SLOT_SIZE;i++)begin
            match[i] = 0;
            matchNum[i] = 0;
            for(int j=0;j<SLOT_WIDTH;j++)begin
                if(recordStack[i][j].valid == 1 && recordStack[i][j].Info.key == ppl_pkt[i].Info.key && ppl_pkt[i].valid == 1)begin
                    match[i] = 1;
                    matchNum[i] = j;
                end
            end
        end
    end


    Ringslot    FIFO_pkt_in_set;
    Ringslot    FIFO_pkt_re_en;
    Ringslot    FIFO_out_set,ppl_end_set;
    logic       out_fail;
    logic [DWIDTH-1:0] FIFO_out_data;
    

    always_comb begin
        FIFO_pkt_in_set.valid               = 1'b1;
        FIFO_pkt_in_set.NoF                 = 0; 
        FIFO_pkt_in_set.MatchFail           = 0;
        FIFO_pkt_in_set.Info                = in_pkt_info;

        ppl_end_set                        = ppl_pkt[SLOT_SIZE];
        out_fail                            = (ppl_end_set.NoF==0)&&(ppl_end_set.valid==1);
        out_valid                           = (ppl_end_set.NoF!=0)&&(ppl_end_set.valid==1);
        
        out_data                            = ppl_data[SLOT_SIZE];
        out_prior                           = ppl_pkt[SLOT_SIZE].NoF;

        FIFO_pkt_re_en                      = ppl_pkt[SLOT_SIZE];
        FIFO_pkt_re_en.MatchFail            = ppl_pkt[SLOT_SIZE].MatchFail + 1;
        
    end

    FIFOdual #( .DWIDTH( $bits(FIFO_pkt_in_set)+$bits(in_data) ) ) EntryCollector (
        .clk(clk),
        .rst(rst),

        .in_valid(in_valid),
        .inA_enque_en(in_en),
        .inA_data({FIFO_pkt_in_set,in_data}),
        .inB_enque_en(out_fail),
        .inB_data({FIFO_pkt_re_en,ppl_data[SLOT_SIZE]}),

        .out_deque_en(1'b1),
        .out_valid(F_out_valid),
        .out_data({FIFO_out_set,FIFO_out_data})
    );
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for(int i=0;i<SLOT_SIZE;i++)begin
                for(int j=0;j<SLOT_WIDTH;j++)begin
                    recordStack[i][j] = 0;
                end
            end
            for(int i=0;i<SLOT_SIZE+1;i++)begin
                ppl_pkt[i] <= 0;
                ppl_data[i] <= 0;
            end
        end else begin
            if(F_out_valid)begin
                ppl_pkt[0]  <= FIFO_out_set;
                ppl_data[0] <= FIFO_out_data;
            end else begin
                ppl_pkt[0]  <= 0;
                ppl_data[0] <= 0;
            end
            
            for(int i=1;i<SLOT_SIZE+1;i++)begin
            
                ppl_data[i] <= ppl_data[i-1];
                ppl_pkt[i]  <= ppl_pkt[i-1];
                
                if( ppl_pkt[i-1].valid == 1 && ppl_pkt[i-1].NoF == 0 )begin
                    if(match[i-1] == 1)begin
                        ppl_pkt[i].NoF <= i*SLOT_WIDTH + matchNum[i-1] + 1;
//                        recordStack[i-1][matchNum[i-1]].NoF <= 0;

                        for(int j=0;j<SLOT_WIDTH;j++)begin
                            recordStack[i-1][j].NoF <= 0;
                        end
                        
                    end else if( empty[i-1] == 1 && ppl_pkt[i-1].MatchFail != 0 )begin
                        ppl_pkt[i].NoF <= i*SLOT_WIDTH + emptyNum[i-1] + 1;
                        recordStack[i-1][emptyNum[i-1]] <= ppl_pkt[i-1];
                        recordStack[i-1][emptyNum[i-1]].valid <= 1;
                        recordStack[i-1][emptyNum[i-1]].NoF <= 0;
                        recordStack[i-1][emptyNum[i-1]].MatchFail <= 0;
                    end else begin
                        for(int j=0;j<SLOT_WIDTH;j++)begin
                            recordStack[i-1][j].NoF <= (recordStack[i-1][j].NoF == 32'd64)?recordStack[i-1][j].NoF:recordStack[i-1][j].NoF+1;
                        end
                    end
                end else begin
                end
           
            end
        end
    end
endmodule
