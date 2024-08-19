`timescale 1 ns/10 ps
import heap_ops::*;
import pkt_h::*;

module pkt_tb;

// Simulation parameters. Some testcases implicitly depend
// on the values being set here, so they musn't be changed!
    parameter TEST_NR = 2048;
    logic clk,rst,pkt_ready,in_valid,in_enque_en,in_ugr_en,out_valid,out_deque_en;
    logic [pkt_sche_v0_1_inst.DWIDTH-1:0] in_data,out_data;
    int counter,nr_pkt,glo_cnt;
    pkHeadInfo                                  in_pkt_info;
    logic [31:0] info_n,addr_n;
    int gap_n;
    logic pkt_en;
     
    initial rst = 1;
    initial clk = 0;
    initial glo_cnt = 0;
    initial counter = 0;
    initial out_deque_en = 1;
    initial nr_pkt = 0;
    
    initial pkt_en = 0;
    
    logic [31:0]    info   [TEST_NR-1:0];
    logic [31:0]    addr   [TEST_NR-1:0];
    int             gapn   [TEST_NR-1:0];
    initial begin
        $readmemh("head_info.txt",info);
        $readmemh("buff_addr.txt",addr);
        $readmemh("buff_gapn.txt",gapn);
    end
    always_comb begin
        info_n = info[nr_pkt];
        addr_n = addr[nr_pkt];
        gap_n = gapn[nr_pkt];
    end
    
    
    always_comb begin
//            in_data = pkt_ready?counter+32'h114:0;
//            in_enque_en = pkt_ready;
//            in_pkt_info = pkt_ready?(counter%4 + 32'ha5a50000):0;
//            in_ugr_en = ((counter%10==9||counter%10==0)&(pkt_ready))?1:0;
        in_data = pkt_ready?addr_n:0;
        in_pkt_info = pkt_ready?info_n:0;
        in_enque_en = pkt_en && (nr_pkt<32'd2048);
        in_ugr_en = ((nr_pkt%10==9||nr_pkt%10==0)&(pkt_ready))?1:0;
    end
    
    always #(10) clk = ~clk;
    
    
    
    
    always @( posedge clk ) begin
        glo_cnt = glo_cnt + 1;
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
//                info_n = 0;
//                addr_n = 0;
                //gap_n = 0;
            end
        end
    end
    integer pkt_o;
    initial pkt_o = $fopen("pkt_tb.txt");
    always @( posedge clk ) begin
        if(out_valid)begin
            $fdisplay(pkt_o,"out_data = %x; glo_cnt = %x",out_data,glo_cnt);
            ;
            ;
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
