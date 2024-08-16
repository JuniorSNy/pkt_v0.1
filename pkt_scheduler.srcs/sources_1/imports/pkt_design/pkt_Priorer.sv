import pkt_h::*;

module pkt_Priorer #(
    parameter DWIDTH = 32,
    parameter SLOT_SIZE = 8,
    parameter QUEUE_SIZE = 16,
    parameter PRIOR_WIDTH = 6
) (
    input   logic                                       clk,
    input   logic                                       rst,
    // Data & PktInfo for priority caculation
    input   logic                                       in_en,
    output  logic                                       in_valid,
    input   pkHeadInfo                                  in_pkt_info,
    input   logic [DWIDTH-1:0]                          in_data,
    // Outdata & priority, data and priority enabled when valid==1'b1 
    output  logic                                       out_valid,
    output  logic [DWIDTH-1:0]                          out_data,
    output  logic [PRIOR_WIDTH-1:0]                     out_prior
);

    integer  out_cnt;
    initial out_cnt = 0;
    logic       out_fail;
    logic       F_out_valid;
    Ringslot    FIFO_pkt_in_set;
    Ringslot    FIFO_out_set;
    Ringslot    Slot_in_set [SLOT_SIZE-1:0];
    Ringslot    Comp_in_set [SLOT_SIZE-1:0];

    logic [DWIDTH-1:0]      FIFO_out_data;
    logic [DWIDTH-1:0]      Comp_in_data [SLOT_SIZE-1:0];

    always_comb begin
        FIFO_pkt_in_set.valid               = 1'b1;
        FIFO_pkt_in_set.NoF                 = 0;
        FIFO_pkt_in_set.MatchFail           = 0;
        FIFO_pkt_in_set.Info                = in_pkt_info;

        out_prior   = Comp_in_set[SLOT_SIZE-1].NoF;
        out_data    = Comp_in_data[SLOT_SIZE-1];
        out_valid   = (Comp_in_set[SLOT_SIZE-1].NoF!=0)&&(Comp_in_set[SLOT_SIZE-1].valid==1);
        out_fail    = (Comp_in_set[SLOT_SIZE-1].NoF==0)&&(Comp_in_set[SLOT_SIZE-1].valid==1);
        
    end
    
    always @(posedge clk)begin
        if(out_valid)begin
            out_cnt = out_cnt+1;
        end
    end


    FIFOdual #( .DWIDTH( $bits(FIFO_pkt_in_set)+$bits(in_data) ) ) EntryCollector (
        .clk(clk),
        .rst(rst),

        .in_valid(in_valid),
        .inA_enque_en(in_en),
        .inA_data({FIFO_pkt_in_set,in_data}),
        .inB_enque_en(out_fail),
        .inB_data({Comp_in_set[SLOT_SIZE-1],Comp_in_data[SLOT_SIZE-1]}),

        .out_deque_en(1'b1),
        .out_valid(F_out_valid),
        .out_data({FIFO_out_set,FIFO_out_data})
    );

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for(int i=0;i<SLOT_SIZE;i++) begin
                Slot_in_set[i]  <= 0;
                Comp_in_set[i]  <= 0;
                Comp_in_data[i] <= 0;
            end
        end else begin
            if (F_out_valid) begin
                Comp_in_set[0]  <= FIFO_out_set;
                Comp_in_data[0] <= FIFO_out_data;
            end else begin
                Comp_in_set[0]  <= 0;
                Comp_in_data[0] <= 0;
            end
            for(int i=1;i<SLOT_SIZE;i++) begin
                if((Comp_in_set[i-1].valid==1)&&(Comp_in_set[i-1].NoF==0)&&(Slot_in_set[i-1].valid==0))begin
                    Slot_in_set[i-1]    <= Comp_in_set[i-1];
                    Comp_in_set[i]      <= Comp_in_set[i-1];
                    Comp_in_set[i].NoF  <= i;
                    Comp_in_data[i]     <= Comp_in_data[i-1];
                end else if( (Comp_in_set[i-1].valid==1)&&(Comp_in_set[i-1].NoF==0)&&(Slot_in_set[i-1].valid==1) ) begin
                    if(Comp_in_set[i-1].Info.key==Slot_in_set[i-1].Info.key) begin
                        Slot_in_set[i-1].NoF    <= 0;
                        Comp_in_set[i]          <= Comp_in_set[i-1];
                        Comp_in_set[i].NoF      <= i;   
                        Comp_in_data[i]         <= Comp_in_data[i-1];
                    end else if (Slot_in_set[i-1].NoF < 16'd10 )begin
                        Slot_in_set[i-1].NoF        <= Slot_in_set[i-1].NoF + 1'b1;
                        Comp_in_set[i].MatchFail    <= Comp_in_set[i-1].MatchFail+1'b1;
                        Comp_in_set[i]              <= Comp_in_set[i-1];
                        Comp_in_data[i]             <= Comp_in_data[i-1];
                    end else begin
                        Slot_in_set[i-1]    <= Comp_in_set[i-1];
                        Comp_in_set[i]      <= Comp_in_set[i-1];
                        Comp_in_set[i].NoF  <= i;
                        Comp_in_data[i]     <= Comp_in_data[i-1];
                    end
                end else begin
                    Comp_in_set[i]  <= Comp_in_set[i-1];
                    Comp_in_data[i] <= Comp_in_data[i-1];
                end
            end
        end
    end
endmodule