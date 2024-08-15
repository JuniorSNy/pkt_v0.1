import pkt_h::*;
`timescale 1 ns/10 ps

module Pri_tb;

    logic clk;
    logic rst;
    int i;
    // logic init_done;
    // initial counter = 0;
    // initial init_done = 0;
    // initial test_timer = 0;
    logic pkt_in_en;
    logic pkt_in_valid;
    pkHeadInfo pkt_in_info;
    logic [pkt_Priorer_inst.DWIDTH-1:0] pkt_in_data,pkt_out_data;
    logic [pkt_Priorer_inst.PRIOR_WIDTH-1:0] pkt_out_prior;
    logic pkt_out_valid;
    logic [31:0] randomline;
    
        reg [31:0] dataArray [2048-1:0];
        initial begin
            $readmemh("rddata.hex",dataArray,0,2047);
        end
    
    logic [31:0] data_t;
    always_comb data_t = dataArray[i];
    initial clk = 0;
    initial rst = 1;
    initial i = 0;
    initial pkt_in_en = 0;
    always #(10) clk = ~clk;
    integer log_o;
    initial log_o = $fopen("tb_log.txt");
    
    always @(posedge clk) begin
        rst <= 0;
        pkt_in_en <= 1;
        i=i+1;
        randomline =  $random();
    
        pkt_in_info.key <= randomline%7 + 16;
        pkt_in_data <= randomline%7 + 1;
        $fdisplay(log_o,"%d",pkt_in_data);
        // o_fifo_data <= {$random(),$random(),$random(),$random(),$random(),$random(),$random(),$random()};
        // test_timer <= test_timer + 1;
        // in_data_addr <= $random();
    
        // if (test_timer) begin
        //     if (in_valid==1'b0) begin
        //         in_enque_en = 1'b0;
        //         out_deque_en = 1'b1;
        //     end
        //     if (test_timer > 30) begin
        //         $display("testbench init timed out");
        //         $finish;
        //     end
        // end
    end
    
    
    pkt_Priorer #(
        // .DWIDTH(64),
        // .QUEUE_SIZE(21)
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