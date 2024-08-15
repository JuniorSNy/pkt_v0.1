`timescale 1 ns/10 ps
import heap_ops::*;
import pkt_h::*;

module pkt_tb;

// Simulation parameters. Some testcases implicitly depend
// on the values being set here, so they musn't be changed!
    parameter TEST_NR = 2048;
    logic clk,rst,pkt_ready,in_valid,in_enque_en,in_ugr_en,out_valid,out_deque_en;
    logic [pkt_sche_v0_1_inst.DWIDTH-1:0] in_data,out_data;
    int counter,nr_pkt;
    pkHeadInfo                                  in_pkt_info;
     
    initial rst = 1;
    initial clk = 0;
    initial counter = 0;
    initial out_deque_en = 1;
    initial nr_pkt = 0;
    
    logic [31:0]    info   [TEST_NR-1:0];
    logic [31:0]    addr   [TEST_NR-1:0];
    int             gapn   [TEST_NR-1:0];
    initial begin
        $readmemh("head_info.txt",info);
        $readmemh("buff_addr.txt",addr);
        $readmemh("buff_gapn.txt",gapn);
    end
    
    logic [31:0] info_n,addr_n;
    int gap_n;
    logic pkt_en;
    initial pkt_en = 0;
    
    always_comb begin
        info_n = info[nr_pkt];
        addr_n = addr[nr_pkt];
        gap_n = gapn[nr_pkt];
    end
    
    always_comb begin
            in_data = pkt_ready?counter+32'h114:0;
            in_enque_en = pkt_ready;
            in_pkt_info = pkt_ready?(counter%4 + 32'ha5a50000):0;
            in_ugr_en = ((counter%10==9||counter%10==0)&(pkt_ready))?1:0;
    end
    
    always #(10) clk = ~clk;
    always @( posedge clk ) begin
        rst <= 0;
        if (pkt_ready) begin
            pkt_en = 0;
            if(counter == 0 && nr_pkt == 0)begin
                pkt_en = 1;
            end
            counter = counter+1;
            if(nr_pkt == TEST_NR)begin
                $finish();
            end
            if(counter > gap_n)begin
                counter = 0;
                nr_pkt = nr_pkt + 1;
                pkt_en = 1;
            end
            if(counter>64)begin
                $stop;
            end
        end
    end
    
    // BBQ instance
    pkt_sche_v0_1 #() pkt_sche_v0_1_inst (
        .clk(clk),
        .rst(rst),
        .ready(pkt_ready),
        
        .in_valid(in_valid),
        .in_enque_en(in_enque_en),
        .in_ugr_en(in_ugr_en),
        .in_pkt_info(in_pkt_info),
        .in_data(in_data),
    
        .out_valid(out_valid),
        .out_deque_en(out_deque_en),
        .out_data(out_data)
    );
    
    

endmodule
