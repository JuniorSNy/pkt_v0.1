import pkt_h::*;
`timescale 1 ns/10 ps

//typedef struct packed {
//    // logic out_valid;
//    logic               valid;
//    logic [15:0]        NoF;
//    logic [3:0]         MatchFail;
//    pkHeadInfo          Info;
//} RecordSpot;

module Pri_tb;
    parameter TEST_NR = 2048;
    
    logic [31:0]    info_n,addr_n;
    int gap_n;
    
    logic [31:0]    info   [TEST_NR-1:0];
    logic [31:0]    addr   [TEST_NR-1:0];
    int             gapn   [TEST_NR-1:0];
    int             nr_pkt;
    logic clk,rst;
    always #10 clk = ~clk;
    logic [31:0] pkt_in_info,pkt_in_data,pkt_out_data,pkt_out_prior;
    
    logic pkt_in_en;
    logic pkt_in_valid,pkt_out_valid;
    initial begin
        nr_pkt = 0;
        clk = 0;
        rst = 1;
        $readmemh("head_info.txt",info);
        $readmemh("buff_addr.txt",addr);
        $readmemh("buff_gapn.txt",gapn);
    end
    always_comb begin
        info_n = info[nr_pkt];
        addr_n = addr[nr_pkt];
        gap_n = gapn[nr_pkt];
        
        pkt_in_info = info_n;
        pkt_in_data = addr_n;
        
        pkt_in_en = ( nr_pkt<2048 );
    end
    
    always @(posedge clk)begin
        rst = 0;
        if(pkt_in_en)begin
            nr_pkt = nr_pkt+1;
        end
        if( nr_pkt == 2048 ) begin
            $stop();
        end
    end
    
    pkt_Priorer_v0_2 #(
    ) pkt_Priorer_inst (
        .clk(clk),
        .rst(rst),
        .in_en(pkt_in_en),
        .in_valid(pkt_in_valid),
        .in_pkt_info(pkt_in_info),
        .in_data(pkt_in_data),
        .out_valid(pkt_out_valid),
        .out_data(pkt_out_data),
        .out_prior(pkt_out_prior)
    );
    
endmodule