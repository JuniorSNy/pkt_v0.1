import heap_ops::*;

module BBQ_router #(
    parameter DWIDTH = 32,
    parameter PRIOR_WIDTH = 6
) (
    // General I/O
    input   logic                                       clk,
    input   logic                                       rst,

    // Routing the bbq_op to one of bbq_insts according to out_ctrl, and dequeue another bbq_inst
    input   logic                                       bbq_rdy,
    input   logic                                       out_ctrl,
    input   heap_op_t                                   out_op,
    input   logic                                       in_enque_en,
    input   logic [DWIDTH-1:0]                          in_data,
    input   logic [PRIOR_WIDTH-1:0]                     in_prior,
    
    
    output  logic                                       out_0_valid,
    output  heap_op_t                                   out_0_op_type,
    output  logic [DWIDTH-1:0]                          out_0_he_data,
    output  logic [PRIOR_WIDTH-1:0]                     out_0_he_priority,

    output  logic                                       out_1_valid,
    output  heap_op_t                                   out_1_op_type,
    output  logic [DWIDTH-1:0]                          out_1_he_data,
    output  logic [PRIOR_WIDTH-1:0]                     out_1_he_priority
);
    logic clk_div;
    initial clk_div = 0;

    always_comb begin
        if(bbq_rdy)begin
            if (out_ctrl) begin

                out_0_valid         = clk_div;
                out_0_op_type       = clk_div?out_op:HEAP_OP_ENQUE;
                out_0_he_data       = 0;
                out_0_he_priority   = 1;

                out_1_valid         = in_enque_en;
                out_1_op_type       = HEAP_OP_ENQUE;
                out_1_he_data       = in_data;
                out_1_he_priority   = in_prior;

            end else begin

                out_0_valid         = in_enque_en;
                out_0_op_type       = HEAP_OP_ENQUE;
                out_0_he_data       = in_data;
                out_0_he_priority   = in_prior;

                out_1_valid         = clk_div;
                out_1_op_type       = clk_div?out_op:HEAP_OP_ENQUE;
                out_1_he_data       = 0;
                out_1_he_priority   = 1;
                
            end
        end else begin
                out_0_valid         = 0;
                out_0_op_type       = out_op;
                out_0_he_data       = 0;
                out_0_he_priority   = 0;

                out_1_valid         = 0;
                out_1_op_type       = out_op;
                out_1_he_data       = 0;
                out_1_he_priority   = 0;
        end
    end
    integer out_1_nr,out_0_nr;
    initial out_0_nr = 0;
    initial out_1_nr = 0;
    always @(posedge clk or posedge rst) begin
        if(out_0_op_type==HEAP_OP_ENQUE && out_0_valid==1)begin
            out_0_nr = out_0_nr+1;
        end
        if(out_1_op_type==HEAP_OP_ENQUE && out_1_valid==1)begin
            out_1_nr = out_1_nr+1;
        end
        clk_div = ~clk_div;
    end

endmodule