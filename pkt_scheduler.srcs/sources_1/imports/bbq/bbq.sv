import heap_ops::*;
// `define DEBUG

/**
 * Implements an integer priority queue in hardware using a configurable
 * Hierarchical Find First Set (HFFS) Queue. The implementation is fully
 * pipelined, capable of performing one operation (enqueue, dequeue-*,
 * or peek) every cycle.
 */
module bbq #(
    parameter HEAP_ENTRY_DWIDTH = 17,
    parameter HEAP_MAX_NUM_ENTRIES = ((1 << 17) - 1),
    localparam HEAP_BITMAP_WIDTH = 2, // Bitmap bit-width
    localparam HEAP_NUM_LPS = 2, // Number of logical BBQs
    localparam HEAP_LOGICAL_BBQ_AWIDTH = ($clog2(HEAP_NUM_LPS)),
    localparam HEAP_ENTRY_AWIDTH = ($clog2(HEAP_MAX_NUM_ENTRIES)),
    localparam HEAP_NUM_LEVELS = 3, // Number of bitmap tree levels
    localparam HEAP_NUM_PRIORITIES = (HEAP_BITMAP_WIDTH ** HEAP_NUM_LEVELS),
    localparam HEAP_PRIORITY_BUCKETS_AWIDTH = ($clog2(HEAP_NUM_PRIORITIES)),
    localparam HEAP_NUM_PRIORITIES_PER_LP = (HEAP_NUM_PRIORITIES / HEAP_NUM_LPS),
    localparam HEAP_PRIORITY_BUCKETS_LP_AWIDTH = ($clog2(HEAP_NUM_PRIORITIES_PER_LP))
) (
    // General I/O
    input   logic                                       clk,
    input   logic                                       rst,
    output  logic                                       ready,

    // Operation input
    input   logic                                       in_valid,
    input   heap_op_t                                   in_op_type,
    input   logic [HEAP_ENTRY_DWIDTH-1:0]               in_he_data,
    input   logic [HEAP_PRIORITY_BUCKETS_AWIDTH-1:0]    in_he_priority,

    // Operation output
    output  logic                                       out_valid,
    output  heap_op_t                                   out_op_type,
    output  logic [HEAP_ENTRY_DWIDTH-1:0]               out_he_data,
    output  logic [HEAP_PRIORITY_BUCKETS_AWIDTH-1:0]    out_he_priority
);

// Optimization: Subtree occupancy counters (StOCs) must represent
// values in the range [0, HEAP_MAX_NUM_ENTRIES]. Consequently, to
// support 2^k entries, every StOC must be (k + 1)-bits wide; this
// is wasteful because the MSb is only ever used to encode maximum
// occupancy (2^k). Instead, by supporting one less entry (2^k - 1)
// we can reduce memory usage by using 1 fewer bit per StOC.
localparam ROUNDED_MAX_NUM_ENTRIES = (1 << HEAP_ENTRY_AWIDTH);
if (HEAP_MAX_NUM_ENTRIES != (ROUNDED_MAX_NUM_ENTRIES - 1)) begin
    $error("HEAP_MAX_NUM_ENTRIES must be of the form (2^k - 1)");
end

integer i;
integer j;

/**
 * Derived parameters.
 */
localparam NUM_PIPELINE_STAGES          = 14;

localparam NUM_BITMAPS_L1               = 1;
localparam NUM_BITMAPS_L2               = (HEAP_BITMAP_WIDTH ** 1);
localparam NUM_BITMAPS_L3               = (HEAP_BITMAP_WIDTH ** 2);
localparam BITMAP_L2_AWIDTH             = ($clog2(NUM_BITMAPS_L2));
localparam BITMAP_L3_AWIDTH             = ($clog2(NUM_BITMAPS_L3));

localparam NUM_COUNTERS_L1              = (NUM_BITMAPS_L2);
localparam NUM_COUNTERS_L2              = (NUM_BITMAPS_L3);
localparam NUM_COUNTERS_L3              = (HEAP_NUM_PRIORITIES);
localparam COUNTER_T_WIDTH              = (HEAP_ENTRY_AWIDTH + 1);
localparam COUNTER_L1_AWIDTH            = ($clog2(NUM_COUNTERS_L1));
localparam COUNTER_L2_AWIDTH            = ($clog2(NUM_COUNTERS_L2));
localparam COUNTER_L3_AWIDTH            = ($clog2(NUM_COUNTERS_L3));

localparam WATERLEVEL_IDX               = (COUNTER_T_WIDTH - 1);
localparam LIST_T_WIDTH                 = (HEAP_ENTRY_AWIDTH * 2);
localparam BITMAP_IDX_MASK              = (HEAP_BITMAP_WIDTH - 1);
localparam HEAP_LOG_BITMAP_WIDTH        = ($clog2(HEAP_BITMAP_WIDTH));

/**
 * Local typedefs.
 */
typedef logic [COUNTER_T_WIDTH-1:0] counter_t;
typedef logic [HEAP_BITMAP_WIDTH-1:0] bitmap_t;
typedef logic [HEAP_LOGICAL_BBQ_AWIDTH-1:0] bbq_id_t;
typedef logic [HEAP_ENTRY_AWIDTH-1:0] heap_entry_ptr_t;
typedef logic [HEAP_ENTRY_DWIDTH-1:0] heap_entry_data_t;
typedef logic [HEAP_PRIORITY_BUCKETS_AWIDTH-1:0] heap_priority_t;
typedef struct packed { heap_entry_ptr_t head; heap_entry_ptr_t tail; } list_t;

typedef enum logic [1:0] {
    FSM_STATE_IDLE,
    FSM_STATE_INIT,
    FSM_STATE_READY
} fsm_state_t;

typedef enum logic {
    OP_COLOR_BLUE,
    OP_COLOR_RED
} op_color_t;

typedef enum logic [1:0] {
    READ_CARRY_RIGHT,
    READ_CARRY_DOWN,
    READ_CARRY_UP
} read_carry_direction_t;

// Heap state
bitmap_t l2_bitmaps[NUM_BITMAPS_L2-1:0]; // L2 bitmaps

// Free list
logic fl_empty;
logic fl_rdreq;
logic fl_wrreq;
logic [HEAP_ENTRY_AWIDTH-1:0] fl_q;
logic [HEAP_ENTRY_AWIDTH-1:0] fl_data;
logic [HEAP_ENTRY_AWIDTH-1:0] fl_q_r[10:0];
logic [HEAP_ENTRY_AWIDTH-1:0] fl_wraddress_counter_r;

// Heap entries
logic he_rden;
logic he_wren;
logic he_rden_r;
logic he_wren_r;
logic [HEAP_ENTRY_DWIDTH-1:0] he_q;
logic [HEAP_ENTRY_DWIDTH-1:0] he_data;
logic [HEAP_ENTRY_AWIDTH-1:0] he_rdaddress;
logic [HEAP_ENTRY_AWIDTH-1:0] he_wraddress;
logic [HEAP_ENTRY_AWIDTH-1:0] he_rdaddress_r;
logic [HEAP_ENTRY_AWIDTH-1:0] he_wraddress_r;

// Next pointers
logic np_rden;
logic np_wren;
logic np_rden_r;
logic np_wren_r;
logic [HEAP_ENTRY_AWIDTH-1:0] np_q;
logic [HEAP_ENTRY_AWIDTH-1:0] np_data;
logic [HEAP_ENTRY_AWIDTH-1:0] np_rdaddress;
logic [HEAP_ENTRY_AWIDTH-1:0] np_wraddress;
logic [HEAP_ENTRY_AWIDTH-1:0] np_rdaddress_r;
logic [HEAP_ENTRY_AWIDTH-1:0] np_wraddress_r;

// Previous pointers
logic pp_rden;
logic pp_wren;
logic pp_rden_r;
logic pp_wren_r;
logic [HEAP_ENTRY_AWIDTH-1:0] pp_q;
logic [HEAP_ENTRY_AWIDTH-1:0] pp_data;
logic [HEAP_ENTRY_AWIDTH-1:0] pp_rdaddress;
logic [HEAP_ENTRY_AWIDTH-1:0] pp_wraddress;
logic [HEAP_ENTRY_AWIDTH-1:0] pp_rdaddress_r;
logic [HEAP_ENTRY_AWIDTH-1:0] pp_wraddress_r;

// Priority buckets
logic pb_rden;
logic pb_wren;
logic pb_rdwr_conflict;
logic reg_pb_rdwr_conflict_r1;
logic reg_pb_rdwr_conflict_r2;
logic [LIST_T_WIDTH-1:0] pb_q;
logic [LIST_T_WIDTH-1:0] pb_q_r;
logic [LIST_T_WIDTH-1:0] pb_data;
logic [HEAP_PRIORITY_BUCKETS_AWIDTH-1:0] pb_rdaddress;
logic [HEAP_PRIORITY_BUCKETS_AWIDTH-1:0] pb_wraddress;

// L2 counters
logic counter_l2_rden;
logic counter_l2_wren;
logic [COUNTER_T_WIDTH-1:0] counter_l2_q;
logic [COUNTER_T_WIDTH-1:0] counter_l2_data;
logic [COUNTER_L2_AWIDTH-1:0] counter_l2_rdaddress;
logic [COUNTER_L2_AWIDTH-1:0] counter_l2_wraddress;
logic [COUNTER_L2_AWIDTH-1:0] counter_l2_wraddress_counter_r;

// L3 bitmaps
logic bm_l3_rden;
logic bm_l3_wren;
logic [HEAP_BITMAP_WIDTH-1:0] bm_l3_q;
logic [HEAP_BITMAP_WIDTH-1:0] bm_l3_data;
logic [HEAP_BITMAP_WIDTH-1:0] bm_l3_data_r;
logic [BITMAP_L3_AWIDTH-1:0] bm_l3_rdaddress;
logic [BITMAP_L3_AWIDTH-1:0] bm_l3_wraddress;
logic [BITMAP_L3_AWIDTH-1:0] bm_l3_wraddress_counter_r;

// L3 counters
logic counter_l3_rden;
logic counter_l3_wren;
logic [COUNTER_T_WIDTH-1:0] counter_l3_q;
logic [COUNTER_T_WIDTH-1:0] counter_l3_data;
logic [COUNTER_L3_AWIDTH-1:0] counter_l3_rdaddress;
logic [COUNTER_L3_AWIDTH-1:0] counter_l3_wraddress;
logic [COUNTER_L3_AWIDTH-1:0] counter_l3_wraddress_counter_r;

// Heap occupancy per logical BBQ
counter_t occupancy[HEAP_NUM_LPS-1:0];

/**
 * Housekeeping.
 */
// Common pipeline metadata
logic                                   reg_valid_s[NUM_PIPELINE_STAGES:0];
bbq_id_t                                reg_bbq_id_s[NUM_PIPELINE_STAGES:0];
heap_op_t                               reg_op_type_s[NUM_PIPELINE_STAGES:0];
heap_entry_data_t                       reg_he_data_s[NUM_PIPELINE_STAGES:0];
logic [BITMAP_L2_AWIDTH-1:0]            reg_l2_addr_s[NUM_PIPELINE_STAGES:0];
logic [BITMAP_L3_AWIDTH-1:0]            reg_l3_addr_s[NUM_PIPELINE_STAGES:0];
op_color_t                              reg_op_color_s[NUM_PIPELINE_STAGES:0];
logic                                   reg_is_enque_s[NUM_PIPELINE_STAGES:0];
heap_priority_t                         reg_priority_s[NUM_PIPELINE_STAGES:0];
bitmap_t                                reg_l2_bitmap_s[NUM_PIPELINE_STAGES:0];
bitmap_t                                reg_l3_bitmap_s[NUM_PIPELINE_STAGES:0];
logic                                   reg_is_deque_min_s[NUM_PIPELINE_STAGES:0];
logic                                   reg_is_deque_max_s[NUM_PIPELINE_STAGES:0];

// Stage 0 metadata
bbq_id_t                                bbq_id_s0;

// Stage 1 metadata
logic                                   valid_s1;
counter_t                               old_occupancy_s1;
counter_t                               new_occupancy_s1;
counter_t                               reg_old_occupancy_s1;
counter_t                               reg_new_occupancy_s1;

// Stage 2 metadata
logic                                   l2_addr_conflict_s3_s2;
logic                                   l2_addr_conflict_s4_s2;
logic                                   l2_addr_conflict_s5_s2;
logic                                   l2_addr_conflict_s6_s2;
logic                                   reg_l2_addr_conflict_s3_s2;
logic                                   reg_l2_addr_conflict_s4_s2;
logic                                   reg_l2_addr_conflict_s5_s2;
logic                                   reg_l2_addr_conflict_s6_s2;

// Stage 3 metadata
read_carry_direction_t                  rcd_s3;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       l2_bitmap_idx_s3;
logic                                   l2_bitmap_empty_s3;
bitmap_t                                l2_bitmap_postop_s3;
bitmap_t                                l2_bitmap_idx_onehot_s3;
logic                                   l2_bitmap_changes_s5_s3;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l2_bitmap_idx_s3;
logic                                   reg_l2_bitmap_empty_s3;
bitmap_t                                reg_l2_bitmap_postop_s3;
bitmap_t                                reg_l2_bitmap_idx_onehot_s3;
logic                                   reg_l2_counter_rdvalid_r1_s3;
logic                                   reg_l2_addr_conflict_s4_s3;
logic                                   reg_l2_addr_conflict_s5_s3;
logic                                   reg_l2_addr_conflict_s6_s3;
logic                                   reg_l2_addr_conflict_s7_s3;

// Stage 4 metadata
read_carry_direction_t                  rcd_s4;
counter_t                               l2_counter_s4;
counter_t                               l2_counter_q_s4;
counter_t                               reg_l2_counter_s4;
counter_t                               reg_l2_counter_rc_s4;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l2_bitmap_idx_s4;
bitmap_t                                reg_l2_bitmap_postop_s4;
bitmap_t                                reg_l2_bitmap_idx_onehot_s4;
logic                                   reg_l2_addr_conflict_s5_s4;
logic                                   reg_l2_addr_conflict_s6_s4;
logic                                   reg_l2_addr_conflict_s7_s4;
logic                                   reg_l2_addr_conflict_s8_s4;

// Stage 5 metadata
bitmap_t                                l2_bitmap_s5;
counter_t                               l2_counter_s5;
logic                                   l2_counter_non_zero_s5;
logic                                   l3_addr_conflict_s6_s5;
logic                                   l3_addr_conflict_s7_s5;
logic                                   l3_addr_conflict_s8_s5;
logic                                   l3_addr_conflict_s9_s5;
counter_t                               reg_l2_counter_s5;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l2_bitmap_idx_s5;
logic                                   reg_l3_addr_conflict_s6_s5;
logic                                   reg_l3_addr_conflict_s7_s5;
logic                                   reg_l3_addr_conflict_s8_s5;
logic                                   reg_l3_addr_conflict_s9_s5;

// Stage 6 metadata
bitmap_t                                l3_bitmap_s6;
logic                                   reg_l3_addr_conflict_s7_s6;
logic                                   reg_l3_addr_conflict_s8_s6;
logic                                   reg_l3_addr_conflict_s9_s6;
logic                                   reg_l3_addr_conflict_s10_s6;

// Stage 7 metadata
read_carry_direction_t                  rcd_s7;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       l3_bitmap_idx_s7;
logic                                   l3_bitmap_empty_s7;
bitmap_t                                l3_bitmap_postop_s7;
bitmap_t                                l3_bitmap_idx_onehot_s7;
logic                                   l3_bitmap_changes_s9_s7;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l3_bitmap_idx_s7;
logic                                   reg_l3_bitmap_empty_s7;
bitmap_t                                reg_l3_bitmap_postop_s7;
bitmap_t                                reg_l3_bitmap_idx_onehot_s7;
logic                                   reg_l3_counter_rdvalid_r1_s7;
logic                                   reg_l3_addr_conflict_s8_s7;
logic                                   reg_l3_addr_conflict_s9_s7;
logic                                   reg_l3_addr_conflict_s10_s7;
logic                                   reg_l3_addr_conflict_s11_s7;

// Stage 8 metadata
read_carry_direction_t                  rcd_s8;
counter_t                               l3_counter_s8;
counter_t                               l3_counter_q_s8;
counter_t                               reg_l3_counter_s8;
counter_t                               reg_l3_counter_rc_s8;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l3_bitmap_idx_s8;
bitmap_t                                reg_l3_bitmap_postop_s8;
bitmap_t                                reg_l3_bitmap_idx_onehot_s8;
logic                                   reg_l3_addr_conflict_s9_s8;
logic                                   reg_l3_addr_conflict_s10_s8;
logic                                   reg_l3_addr_conflict_s11_s8;
logic                                   reg_l3_addr_conflict_s12_s8;

// Stage 9 metadata
heap_priority_t                         priority_s9;
counter_t                               l3_counter_s9;
logic                                   l3_counter_non_zero_s9;
logic                                   pb_addr_conflict_s10_s9;
logic                                   pb_addr_conflict_s11_s9;
logic                                   pb_addr_conflict_s12_s9;
logic                                   pb_addr_conflict_s13_s9;
counter_t                               reg_l3_counter_s9;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l3_bitmap_idx_s9;
counter_t                               reg_old_l3_counter_s9;
logic                                   reg_l3_counter_non_zero_s9;
logic                                   reg_pb_addr_conflict_s10_s9;
logic                                   reg_pb_addr_conflict_s11_s9;
logic                                   reg_pb_addr_conflict_s12_s9;
logic                                   reg_pb_addr_conflict_s13_s9;

// Stage 10 metadata
op_color_t                              op_color_s10;
logic                                   reg_pb_update_s10;
logic                                   reg_pb_data_conflict_s10;
logic                                   reg_pb_state_changes_s10;
logic                                   reg_pb_tail_pp_changes_s10;
logic                                   reg_pb_addr_conflict_s11_s10;
logic                                   reg_pb_addr_conflict_s12_s10;

// Stage 11 metadata
logic                                   pp_changes_s12_s11;
logic                                   pp_changes_s13_s11;
list_t                                  reg_pb_q_s11;
heap_entry_ptr_t                        reg_pp_data_s11;
logic                                   reg_pp_data_valid_s11;
logic                                   reg_pb_data_conflict_s11;
logic                                   reg_pb_state_changes_s11;
logic                                   reg_pb_tail_pp_changes_s11;
logic                                   reg_pb_addr_conflict_s12_s11;
logic                                   reg_pb_addr_conflict_s13_s11;

// Stage 12 metadata
heap_entry_data_t                       he_q_s12;
heap_entry_ptr_t                        np_q_s12;
heap_entry_ptr_t                        pp_q_s12;
heap_entry_data_t                       reg_he_q_s12;
heap_entry_ptr_t                        reg_np_q_s12;
heap_entry_ptr_t                        reg_pp_q_s12;
list_t                                  reg_pb_q_s12;
list_t                                  reg_pb_new_s12;
logic                                   reg_pb_data_conflict_s12;
logic                                   reg_pb_state_changes_s12;
logic                                   reg_pb_tail_pp_changes_s12;

// Stage 13 metadata
heap_entry_data_t                       he_data_s13;
heap_entry_data_t                       reg_he_data_s13;
heap_entry_ptr_t                        reg_np_data_s13;
heap_entry_ptr_t                        reg_pp_data_s13;
list_t                                  reg_pb_data_s13;

// Stage 14 metadata
list_t                                  reg_pb_data_s14;

// Init signals
fsm_state_t                             state = FSM_STATE_IDLE;
logic                                   counter_l2_init_done_r;
logic                                   counter_l3_init_done_r;
logic                                   bm_l3_init_done_r;
logic                                   fl_init_done_r;
logic                                   counter_l2_init_done;
logic                                   counter_l3_init_done;
logic                                   bm_l3_init_done;
logic                                   fl_init_done;
fsm_state_t                             state_next;

// Intermediate signals
list_t                                  int_pb_data;
list_t                                  int_pb_q;

// Miscellaneous signals
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       ffs_l2_inst_msb[2:0];
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       ffs_l2_inst_lsb[2:0];
logic                                   ffs_l2_inst_zero[2:0];
bitmap_t                                ffs_l2_inst_msb_onehot[2:0];
bitmap_t                                ffs_l2_inst_lsb_onehot[2:0];

logic [HEAP_LOG_BITMAP_WIDTH-1:0]       ffs_l3_inst_msb[2:0];
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       ffs_l3_inst_lsb[2:0];
logic                                   ffs_l3_inst_zero[2:0];
bitmap_t                                ffs_l3_inst_msb_onehot[2:0];
bitmap_t                                ffs_l3_inst_lsb_onehot[2:0];

`ifdef DEBUG
logic                                   debug_newline;
`endif

assign pb_data = int_pb_data;

// Output assignments
assign ready = !rst & (state == FSM_STATE_READY);
assign out_valid = reg_valid_s[NUM_PIPELINE_STAGES-1];
assign out_op_type = reg_op_type_s[NUM_PIPELINE_STAGES-1];
assign out_he_data = reg_he_data_s[NUM_PIPELINE_STAGES-1];
assign out_he_priority = reg_priority_s[NUM_PIPELINE_STAGES-1];

/**
 * State-dependent signals (data, wraddress, and wren) for the
 * FL, priority buckets and SRAM-based LX bitmaps and counters.
 */
always_comb begin
    state_next = state;
    fl_init_done = fl_init_done_r;
    bm_l3_init_done = bm_l3_init_done_r;
    counter_l2_init_done = counter_l2_init_done_r;
    counter_l3_init_done = counter_l3_init_done_r;

    fl_wrreq = 0;
    bm_l3_wren = 0;
    counter_l2_wren = 0;
    counter_l3_wren = 0;

    // Initialization state
    if (state == FSM_STATE_INIT) begin
        // Free list
        fl_data = fl_wraddress_counter_r;
        if (!fl_init_done_r) begin
            fl_wrreq = 1;
            fl_init_done = (fl_wraddress_counter_r ==
                            (HEAP_MAX_NUM_ENTRIES - 1));
        end
        // L2 counters
        counter_l2_data = 0;
        counter_l2_wraddress = counter_l2_wraddress_counter_r;
        if (!counter_l2_init_done_r) begin
            counter_l2_wren = 1;
            counter_l2_init_done = (counter_l2_wraddress_counter_r ==
                                    (NUM_COUNTERS_L2 - 1));
        end
        // L3 bitmaps
        bm_l3_data = 0;
        bm_l3_wraddress = bm_l3_wraddress_counter_r;
        if (!bm_l3_init_done_r) begin
            bm_l3_wren = 1;
            bm_l3_init_done = (bm_l3_wraddress_counter_r ==
                               (NUM_BITMAPS_L3 - 1));
        end
        // L3 counters
        counter_l3_data = 0;
        counter_l3_wraddress = counter_l3_wraddress_counter_r;
        if (!counter_l3_init_done_r) begin
            counter_l3_wren = 1;
            counter_l3_init_done = (counter_l3_wraddress_counter_r ==
                                    (NUM_COUNTERS_L3 - 1));
        end
        // Finished initializing the queue (including priority buckets,
        // free list, and the LX bitmaps). Proceed to the ready state.
        if (fl_init_done_r & counter_l2_init_done_r & bm_l3_init_done_r &
            counter_l3_init_done_r) begin
            state_next = FSM_STATE_READY;
        end
    end
    else begin
        /**
         * Stage 13: Perform writes: update the priority bucket,
         * the free list, heap entries, next and prev pointers.
         */
        fl_data = (
            (reg_op_color_s[12] == OP_COLOR_BLUE) ?
            reg_pb_q_s12.head : reg_pb_q_s12.tail);

        // Perform deque
        if (!reg_is_enque_s[12]) begin
            // Update the free list
            fl_wrreq = reg_valid_s[12];
        end
        /**
         * Stage 9: Write-back the L3 counter and bitmap,
         * and read the corresponding PB (head and tail).
         */
        // Write L3 counter
        counter_l3_wren = reg_valid_s[8];
        counter_l3_data = l3_counter_s9;
        counter_l3_wraddress = {reg_l3_addr_s[8],
                                reg_l3_bitmap_idx_s8};
        // Write L3 bitmap
        bm_l3_wren = reg_valid_s[8];
        bm_l3_wraddress = reg_l3_addr_s[8];
        if (reg_is_enque_s[8]) begin
            bm_l3_data = (reg_l3_bitmap_s[8] |
                          reg_l3_bitmap_idx_onehot_s8);
        end
        else begin
            bm_l3_data = (
                l3_counter_non_zero_s9 ? reg_l3_bitmap_s[8] :
                (reg_l3_bitmap_s[8] & ~reg_l3_bitmap_idx_onehot_s8));
        end
        /**
         * Stage 5: Write-back the L2 counter and bitmap,
         * and read the corresponding L3 bitmap.
         */
        // Write L2 counter
        counter_l2_wren = reg_valid_s[4];
        counter_l2_data = l2_counter_s5;
        counter_l2_wraddress = {reg_l2_addr_s[4],
                                reg_l2_bitmap_idx_s4};
    end
end

/**
 * State-independent logic.
 */
always_comb begin
    bbq_id_s0 = in_he_priority[HEAP_PRIORITY_BUCKETS_AWIDTH-1:
                               HEAP_PRIORITY_BUCKETS_LP_AWIDTH];
    valid_s1 = 0;
    old_occupancy_s1 = occupancy[reg_bbq_id_s[0]];
    rcd_s3 = READ_CARRY_DOWN;
    rcd_s4 = READ_CARRY_DOWN;
    l2_counter_s4 = reg_l2_counter_s4;
    l2_counter_q_s4 = counter_l2_q;
    l3_bitmap_s6 = bm_l3_q;
    rcd_s7 = READ_CARRY_DOWN;
    rcd_s8 = READ_CARRY_DOWN;
    l3_counter_s8 = reg_l3_counter_s8;
    l3_counter_q_s8 = counter_l3_q;
    priority_s9 = {reg_l3_addr_s[8], reg_l3_bitmap_idx_s8};
    op_color_s10 = reg_is_enque_s[9] ? OP_COLOR_BLUE : OP_COLOR_RED;
    he_q_s12 = he_q;
    np_q_s12 = np_q;
    pp_q_s12 = pp_q;

    int_pb_q = pb_q_r;

    fl_rdreq = 0;

    he_rden = 0;
    he_wren = 0;
    he_data = reg_he_data_s[12];
    he_wraddress = fl_q_r[10];

    np_rden = 0;
    np_wren = 0;
    np_data = reg_pb_q_s12.head;
    np_wraddress = fl_q_r[10];

    pp_rden = 0;
    pp_wren = 0;
    pp_data = fl_q_r[10];
    pp_wraddress = reg_pb_q_s12.head;

    pb_rdwr_conflict = 0;
    pb_rdaddress = priority_s9;
    int_pb_data = reg_pb_new_s12;
    pb_wraddress = reg_priority_s[12];

    bm_l3_rden = 0;
    counter_l2_rden = 0;
    counter_l3_rden = 0;

    /**
     * Stage 13: Perform writes: update the priority bucket,
     * the free list, heap entries, next and prev pointers.
     */
    pb_wren = reg_valid_s[12];

    // Perform enque
    if (reg_is_enque_s[12]) begin
        if (reg_valid_s[12]) begin
            he_wren = 1; // Update the heap entry
            np_wren = 1; // Update the next pointer

            // Update the entry's previous pointer. The
            // pointer address is only valid if the PB
            // was not previously empty, so write must
            // be predicated on no change of state.
            if (!reg_pb_state_changes_s12) begin
                pp_wren = 1;
            end
        end

        // Update the data
        he_data_s13 = reg_he_data_s[12];
    end
    // Perform deque
    else begin
        if (reg_op_color_s[12] == OP_COLOR_BLUE) begin
            // BLUE-colored dequeue (from HEAD)
            int_pb_data.head = reg_np_q_s12;
        end
        else begin
            // RED-colored dequeue (from TAIL)
            int_pb_data.tail = reg_pp_q_s12;
        end

        // Update the data
        he_data_s13 = (
            reg_pb_data_conflict_s12 ?
            reg_he_data_s[13] : reg_he_q_s12);
    end
    /**
     * Stage 12: Read delay for HE and pointers.
     */
    // This HE was updated on the last cycle, so the output is stale
    if (he_wren_r && (he_wraddress_r == he_rdaddress_r)) begin
        he_q_s12 = reg_he_data_s13;
    end
    // Fallthrough: default to he_q

    // This NP was updated on the last cycle, so the output is stale
    if (np_wren_r && (np_wraddress_r == np_rdaddress_r)) begin
        np_q_s12 = reg_np_data_s13;
    end
    // Fallthrough: default to np_q

    // This PP was updated in the last 2 cycles
    if (reg_pp_data_valid_s11) begin
        pp_q_s12 = reg_pp_data_s11;
    end
    // Fallthrough: default to pp_q

    /**
     * Stage 11: Read the heap entry and prev/next pointer
     * corresponding to the priority bucket to deque.
     */
    // The PB is being updated on this cycle
    if (reg_pb_addr_conflict_s12_s10) begin
        int_pb_q = int_pb_data;
    end
    // The PB was updated last cycle, so output is stale
    else if (reg_pb_update_s10) begin
        int_pb_q = reg_pb_data_s13;
    end
    // The PB was updated 2 cycles ago (and thus never read)
    else if (reg_pb_rdwr_conflict_r2) begin
        int_pb_q = reg_pb_data_s14;
    end
    // Fallthrough: default to pb_q_r

    // Read next and prev pointers
    np_rdaddress = int_pb_q.head;
    pp_rdaddress = int_pb_q.tail;

    // Compute tail PP updates
    pp_changes_s12_s11 = (reg_pb_tail_pp_changes_s11 &&
                          reg_pb_addr_conflict_s11_s10);

    pp_changes_s13_s11 = (reg_pb_tail_pp_changes_s12 &&
                          reg_pb_addr_conflict_s12_s10);

    // Read HE data
    he_rdaddress = (
        (reg_op_color_s[10] == OP_COLOR_BLUE) ?
        int_pb_q.head : int_pb_q.tail);

    if (reg_valid_s[10]) begin
        if (!reg_is_enque_s[10]) begin
            he_rden = 1; // Dequeing, read HE and PP/NP
            if (reg_op_color_s[10] == OP_COLOR_BLUE) begin
                np_rden = 1; // BLUE-colored dequeue (from HEAD)
            end
            else begin
                pp_rden = 1; // RED-colored dequeue (from TAIL)
            end
        end
    end
    /**
     * Stage 10: Compute op color, read delay for PB.
     */
    if (!reg_is_enque_s[9]) begin
        // Dequeing, recolor this op if required
        if (reg_pb_addr_conflict_s10_s9) begin
            op_color_s10 = (
                (reg_op_color_s[10] == OP_COLOR_BLUE)
                    ? OP_COLOR_RED : OP_COLOR_BLUE);
        end
    end
    /**
     * Stage 9: Write-back the L3 counter and bitmap,
     * and read the corresponding PB (head and tail).
     */
    l3_counter_s9[WATERLEVEL_IDX-1:0] = (
        reg_is_enque_s[8] ? (reg_l3_counter_rc_s8[WATERLEVEL_IDX-1:0] + 1) :
                            (reg_l3_counter_rc_s8[WATERLEVEL_IDX-1:0] - 1));

    l3_counter_s9[WATERLEVEL_IDX] = (reg_is_enque_s[8] ?
        (reg_l3_counter_rc_s8[WATERLEVEL_IDX] | reg_l3_counter_rc_s8[0]) :
        ((|reg_l3_counter_rc_s8[WATERLEVEL_IDX-1:2]) | (&reg_l3_counter_rc_s8[1:0])));

    l3_counter_non_zero_s9 = (reg_is_enque_s[8] |
                              reg_l3_counter_rc_s8[WATERLEVEL_IDX]);
    // Read PB contents
    pb_rden = reg_valid_s[8];

    // Compute conflicts
    pb_addr_conflict_s10_s9 = (
        reg_l3_addr_conflict_s9_s8
            && (reg_l3_bitmap_idx_s8 ==
                reg_priority_s[9][HEAP_LOG_BITMAP_WIDTH-1:0]));

    pb_addr_conflict_s11_s9 = (
        reg_l3_addr_conflict_s10_s8
            && (reg_l3_bitmap_idx_s8 ==
                reg_priority_s[10][HEAP_LOG_BITMAP_WIDTH-1:0]));

    pb_addr_conflict_s12_s9 = (
        reg_l3_addr_conflict_s11_s8
            && (reg_l3_bitmap_idx_s8 ==
                reg_priority_s[11][HEAP_LOG_BITMAP_WIDTH-1:0]));

    pb_addr_conflict_s13_s9 = (
        reg_l3_addr_conflict_s12_s8
            && (reg_l3_bitmap_idx_s8 ==
                reg_priority_s[12][HEAP_LOG_BITMAP_WIDTH-1:0]));

    // Disable conflicting reads during writes
    if (pb_addr_conflict_s13_s9) begin
        pb_rdwr_conflict = 1;
        pb_rden = 0;
    end
    /**
     * Stage 8: NOOP, read delay for L3 counter.
     */
    // Compute the read carry direction. If the
    // active op in Stage 9 is of the same type
    // or the bitmap is empty, carry right.
    if (!reg_is_enque_s[7] &&
        l3_counter_non_zero_s9 &&
        reg_l3_addr_conflict_s8_s7 &&
        (reg_l3_bitmap_empty_s7 || (reg_op_type_s[7] ==
                                    reg_op_type_s[8]))) begin
        rcd_s8 = READ_CARRY_RIGHT;
    end
    // Fallthrough: default to carry down

    // Counter is updating this cycle, so output is stale
    if ((reg_l3_bitmap_idx_s7 == reg_l3_bitmap_idx_s8)
        && reg_l3_addr_conflict_s8_s7) begin
        l3_counter_q_s8 = l3_counter_s9;
        l3_counter_s8 = l3_counter_s9;
    end
    // Counter was updated last cycle (there was R/W conflict)
    else if ((reg_l3_bitmap_idx_s7 == reg_l3_bitmap_idx_s9)
             && reg_l3_addr_conflict_s9_s7) begin
        l3_counter_q_s8 = reg_l3_counter_s9;
        l3_counter_s8 = reg_l3_counter_s9;
    end
    // Fallthrough, defaults to:
    // counter_l3_q for l3_counter_q_s8
    // reg_l3_counter_s8 for l3_counter_s8

    /**
     * Stage 7: Compute the L3 bitmap index and postop
     * bitmap, and read the corresponding L3 counter.
     */
    // L3 bitmap changes?
    l3_bitmap_changes_s9_s7 = (
        reg_l3_addr_conflict_s8_s6 &&
        (reg_is_enque_s[8] || !l3_counter_non_zero_s9));

    // Compute L3 bitmap idx and postop
    case (reg_op_type_s[6])
    HEAP_OP_DEQUE_MAX: begin
        l3_bitmap_idx_s7 = (
            reg_l3_addr_conflict_s7_s6 ? ffs_l3_inst_msb[1] :
               l3_bitmap_changes_s9_s7 ? ffs_l3_inst_msb[2] :
                                         ffs_l3_inst_msb[0]);

        l3_bitmap_empty_s7 = (
            reg_l3_addr_conflict_s7_s6 ? ffs_l3_inst_zero[1] :
               l3_bitmap_changes_s9_s7 ? ffs_l3_inst_zero[2] :
                                         ffs_l3_inst_zero[0]);

        l3_bitmap_idx_onehot_s7 = (
            reg_l3_addr_conflict_s7_s6 ? ffs_l3_inst_msb_onehot[1] :
               l3_bitmap_changes_s9_s7 ? ffs_l3_inst_msb_onehot[2] :
                                         ffs_l3_inst_msb_onehot[0]);

        l3_bitmap_postop_s7 = (
            reg_l3_addr_conflict_s7_s6 ? (l3_bitmap_idx_onehot_s7 ^
                                          reg_l3_bitmap_postop_s7) :
               l3_bitmap_changes_s9_s7 ? (l3_bitmap_idx_onehot_s7 ^
                                          reg_l3_bitmap_postop_s8) :
                                         (l3_bitmap_idx_onehot_s7 ^
                                          reg_l3_bitmap_s[6]));
    end
    HEAP_OP_DEQUE_MIN: begin
        l3_bitmap_idx_s7 = (
            reg_l3_addr_conflict_s7_s6 ? ffs_l3_inst_lsb[1] :
               l3_bitmap_changes_s9_s7 ? ffs_l3_inst_lsb[2] :
                                         ffs_l3_inst_lsb[0]);

        l3_bitmap_empty_s7 = (
            reg_l3_addr_conflict_s7_s6 ? ffs_l3_inst_zero[1] :
               l3_bitmap_changes_s9_s7 ? ffs_l3_inst_zero[2] :
                                         ffs_l3_inst_zero[0]);

        l3_bitmap_idx_onehot_s7 = (
            reg_l3_addr_conflict_s7_s6 ? ffs_l3_inst_lsb_onehot[1] :
               l3_bitmap_changes_s9_s7 ? ffs_l3_inst_lsb_onehot[2] :
                                         ffs_l3_inst_lsb_onehot[0]);

        l3_bitmap_postop_s7 = (
            reg_l3_addr_conflict_s7_s6 ? (l3_bitmap_idx_onehot_s7 ^
                                          reg_l3_bitmap_postop_s7) :
               l3_bitmap_changes_s9_s7 ? (l3_bitmap_idx_onehot_s7 ^
                                          reg_l3_bitmap_postop_s8) :
                                         (l3_bitmap_idx_onehot_s7 ^
                                          reg_l3_bitmap_s[6]));
    end
    // HEAP_OP_ENQUE
    default: begin
        l3_bitmap_empty_s7 = 0;
        l3_bitmap_idx_s7 = (reg_priority_s[6][(
                (1 * HEAP_LOG_BITMAP_WIDTH) - 1)
                : 0]);

        l3_bitmap_idx_onehot_s7 = (1 << l3_bitmap_idx_s7);
        l3_bitmap_postop_s7 = (
            reg_l3_addr_conflict_s7_s6 ? (l3_bitmap_idx_onehot_s7 |
                                          reg_l3_bitmap_postop_s7) :
               l3_bitmap_changes_s9_s7 ? (l3_bitmap_idx_onehot_s7 |
                                          reg_l3_bitmap_postop_s8) :
                                         (l3_bitmap_idx_onehot_s7 |
                                          reg_l3_bitmap_s[6]));
    end
    endcase
    // Compute the read carry direction. If the active
    // op in Stage 9 is of the same type, carry up.
    if (!reg_is_enque_s[6] &&
        l3_counter_non_zero_s9 &&
        reg_l3_addr_conflict_s8_s6 &&
        (reg_op_type_s[6] == reg_op_type_s[8])) begin
        rcd_s7 = READ_CARRY_UP;

        // Special case: The active op in Stage 8 is also
        // of the same type, which means that it's bound
        // to carry right; here, we do the same.
        if ((reg_op_type_s[6] == reg_op_type_s[7]) &&
            reg_l3_addr_conflict_s7_s6) begin
            rcd_s7 = READ_CARRY_RIGHT;
        end
    end
    // Fallthrough: default to carry down

    // Read the L3 counter
    counter_l3_rden = reg_valid_s[6];
    counter_l3_rdaddress = {reg_l3_addr_s[6],
                            l3_bitmap_idx_s7};
    /**
     * Stage 6: NOOP, read delay for L3 bitmap.
     */
    // L3 bitmap updated this cycle, so output is stale
    if (reg_l3_addr_conflict_s8_s5) begin
        l3_bitmap_s6 = bm_l3_data;
    end
    // L3 bitmap was updated last cycle (R/W conflict)
    else if (reg_l3_addr_conflict_s9_s5) begin
        l3_bitmap_s6 = bm_l3_data_r;
    end
    // Fallthrough: default to bm_l3_q

    /**
     * Stage 5: Write-back the L2 counter and bitmap,
     * and read the corresponding L3 bitmap.
     */
    l2_counter_s5[WATERLEVEL_IDX-1:0] = (
        reg_is_enque_s[4] ? (reg_l2_counter_rc_s4[WATERLEVEL_IDX-1:0] + 1) :
                            (reg_l2_counter_rc_s4[WATERLEVEL_IDX-1:0] - 1));

    l2_counter_s5[WATERLEVEL_IDX] = (reg_is_enque_s[4] ?
        (reg_l2_counter_rc_s4[WATERLEVEL_IDX] | reg_l2_counter_rc_s4[0]) :
        ((|reg_l2_counter_rc_s4[WATERLEVEL_IDX-1:2]) | (&reg_l2_counter_rc_s4[1:0])));

    l2_counter_non_zero_s5 = (reg_is_enque_s[4] |
                              reg_l2_counter_rc_s4[WATERLEVEL_IDX]);
    // Write L2 bitmap
    if (reg_is_enque_s[4]) begin
        l2_bitmap_s5 = (reg_l2_bitmap_s[4] |
                        reg_l2_bitmap_idx_onehot_s4);
    end
    else begin
        l2_bitmap_s5 = (
            l2_counter_non_zero_s5 ? reg_l2_bitmap_s[4] :
            (reg_l2_bitmap_s[4] & ~reg_l2_bitmap_idx_onehot_s4));
    end
    // Read L3 bitmap
    bm_l3_rden = reg_valid_s[4];
    bm_l3_rdaddress = {reg_l2_addr_s[4],
                       reg_l2_bitmap_idx_s4};

    // Compute conflicts
    l3_addr_conflict_s6_s5 = (
        reg_l2_addr_conflict_s5_s4
            && (reg_l2_bitmap_idx_s4 ==
                reg_l3_addr_s[5][HEAP_LOG_BITMAP_WIDTH-1:0]));

    l3_addr_conflict_s7_s5 = (
        reg_l2_addr_conflict_s6_s4
            && (reg_l2_bitmap_idx_s4 ==
                reg_l3_addr_s[6][HEAP_LOG_BITMAP_WIDTH-1:0]));

    l3_addr_conflict_s8_s5 = (
        reg_l2_addr_conflict_s7_s4
            && (reg_l2_bitmap_idx_s4 ==
                reg_l3_addr_s[7][HEAP_LOG_BITMAP_WIDTH-1:0]));

    l3_addr_conflict_s9_s5 = (
        reg_l2_addr_conflict_s8_s4
            && (reg_l2_bitmap_idx_s4 ==
                reg_l3_addr_s[8][HEAP_LOG_BITMAP_WIDTH-1:0]));

    /**
     * Stage 4: NOOP, read delay for L2 counter.
     */
    // Compute the read carry direction. If the
    // active op in Stage 5 is of the same type
    // or the bitmap is empty, carry right.
    if (!reg_is_enque_s[3] &&
        l2_counter_non_zero_s5 &&
        reg_l2_addr_conflict_s4_s3 &&
        (reg_l2_bitmap_empty_s3 || (reg_op_type_s[3] ==
                                    reg_op_type_s[4]))) begin
        rcd_s4 = READ_CARRY_RIGHT;
    end
    // Fallthrough: default to carry down

    // Counter is updating this cycle, so output is stale
    if ((reg_l2_bitmap_idx_s3 == reg_l2_bitmap_idx_s4)
        && reg_l2_addr_conflict_s4_s3) begin
        l2_counter_q_s4 = l2_counter_s5;
        l2_counter_s4 = l2_counter_s5;
    end
    // Counter was updated last cycle (there was R/W conflict)
    else if ((reg_l2_bitmap_idx_s3 == reg_l2_bitmap_idx_s5)
             && reg_l2_addr_conflict_s5_s3) begin
        l2_counter_q_s4 = reg_l2_counter_s5;
        l2_counter_s4 = reg_l2_counter_s5;
    end
    // Fallthrough, defaults to:
    // counter_l2_q for l2_counter_q_s4
    // reg_l2_counter_s4 for l2_counter_s4

    /**
     * Stage 3: Compute the L2 bitmap index and postop
     * bitmap, and read the corresponding L2 counter.
     */
    // L2 bitmap changes?
    l2_bitmap_changes_s5_s3 = (
        reg_l2_addr_conflict_s4_s2 &&
        (reg_is_enque_s[4] || !l2_counter_non_zero_s5));

    // Compute L2 bitmap idx and postop
    case (reg_op_type_s[2])
    HEAP_OP_DEQUE_MAX: begin
        l2_bitmap_idx_s3 = (
            reg_l2_addr_conflict_s3_s2 ? ffs_l2_inst_msb[1] :
               l2_bitmap_changes_s5_s3 ? ffs_l2_inst_msb[2] :
                                         ffs_l2_inst_msb[0]);

        l2_bitmap_empty_s3 = (
            reg_l2_addr_conflict_s3_s2 ? ffs_l2_inst_zero[1] :
               l2_bitmap_changes_s5_s3 ? ffs_l2_inst_zero[2] :
                                         ffs_l2_inst_zero[0]);

        l2_bitmap_idx_onehot_s3 = (
            reg_l2_addr_conflict_s3_s2 ? ffs_l2_inst_msb_onehot[1] :
               l2_bitmap_changes_s5_s3 ? ffs_l2_inst_msb_onehot[2] :
                                         ffs_l2_inst_msb_onehot[0]);

        l2_bitmap_postop_s3 = (
            reg_l2_addr_conflict_s3_s2 ? (l2_bitmap_idx_onehot_s3 ^
                                          reg_l2_bitmap_postop_s3) :
               l2_bitmap_changes_s5_s3 ? (l2_bitmap_idx_onehot_s3 ^
                                          reg_l2_bitmap_postop_s4) :
                                         (l2_bitmap_idx_onehot_s3 ^
                                          reg_l2_bitmap_s[2]));
    end
    HEAP_OP_DEQUE_MIN: begin
        l2_bitmap_idx_s3 = (
            reg_l2_addr_conflict_s3_s2 ? ffs_l2_inst_lsb[1] :
               l2_bitmap_changes_s5_s3 ? ffs_l2_inst_lsb[2] :
                                         ffs_l2_inst_lsb[0]);

        l2_bitmap_empty_s3 = (
            reg_l2_addr_conflict_s3_s2 ? ffs_l2_inst_zero[1] :
               l2_bitmap_changes_s5_s3 ? ffs_l2_inst_zero[2] :
                                         ffs_l2_inst_zero[0]);

        l2_bitmap_idx_onehot_s3 = (
            reg_l2_addr_conflict_s3_s2 ? ffs_l2_inst_lsb_onehot[1] :
               l2_bitmap_changes_s5_s3 ? ffs_l2_inst_lsb_onehot[2] :
                                         ffs_l2_inst_lsb_onehot[0]);

        l2_bitmap_postop_s3 = (
            reg_l2_addr_conflict_s3_s2 ? (l2_bitmap_idx_onehot_s3 ^
                                          reg_l2_bitmap_postop_s3) :
               l2_bitmap_changes_s5_s3 ? (l2_bitmap_idx_onehot_s3 ^
                                          reg_l2_bitmap_postop_s4) :
                                         (l2_bitmap_idx_onehot_s3 ^
                                          reg_l2_bitmap_s[2]));
    end
    // HEAP_OP_ENQUE
    default: begin
        l2_bitmap_empty_s3 = 0;
        l2_bitmap_idx_s3 = (reg_priority_s[2][(
                (2 * HEAP_LOG_BITMAP_WIDTH) - 1)
                : (1 * HEAP_LOG_BITMAP_WIDTH)]);

        l2_bitmap_idx_onehot_s3 = (1 << l2_bitmap_idx_s3);
        l2_bitmap_postop_s3 = (
            reg_l2_addr_conflict_s3_s2 ? (l2_bitmap_idx_onehot_s3 |
                                          reg_l2_bitmap_postop_s3) :
               l2_bitmap_changes_s5_s3 ? (l2_bitmap_idx_onehot_s3 |
                                          reg_l2_bitmap_postop_s4) :
                                         (l2_bitmap_idx_onehot_s3 |
                                          reg_l2_bitmap_s[2]));
    end
    endcase
    // Compute the read carry direction. If the active
    // op in Stage 5 is of the same type, carry up.
    if (!reg_is_enque_s[2] &&
        l2_counter_non_zero_s5 &&
        reg_l2_addr_conflict_s4_s2 &&
        (reg_op_type_s[2] == reg_op_type_s[4])) begin
        rcd_s3 = READ_CARRY_UP;

        // Special case: The active op in Stage 4 is also
        // of the same type, which means that it's bound
        // to carry right; here, we do the same.
        if ((reg_op_type_s[2] == reg_op_type_s[3]) &&
            reg_l2_addr_conflict_s3_s2) begin
            rcd_s3 = READ_CARRY_RIGHT;
        end
    end
    // Fallthrough: default to carry down

    // Read the L2 counter
    counter_l2_rden = reg_valid_s[2];
    counter_l2_rdaddress = {reg_l2_addr_s[2],
                            l2_bitmap_idx_s3};
    /**
     * Stage 2: Steer op to the appropriate logical BBQ.
     */
    // Compute conflicts
    l2_addr_conflict_s3_s2 = (
        reg_valid_s[1] && reg_valid_s[2] &&
        (reg_bbq_id_s[1] == reg_bbq_id_s[2]));

    l2_addr_conflict_s4_s2 = (
        reg_valid_s[1] && reg_valid_s[3] &&
        (reg_bbq_id_s[1] == reg_bbq_id_s[3]));

    l2_addr_conflict_s5_s2 = (
        reg_valid_s[1] && reg_valid_s[4] &&
        (reg_bbq_id_s[1] == reg_bbq_id_s[4]));

    l2_addr_conflict_s6_s2 = (
        reg_valid_s[1] && reg_valid_s[5] &&
        (reg_bbq_id_s[1] == reg_bbq_id_s[5]));

    /**
     * Stage 1: Determine operation validity. Disables the pipeline
     * stage if the BBQ is empty (deques), or FL is empty (enques).
     */
    if (reg_valid_s[0]) begin
        valid_s1 = (
            (reg_is_enque_s[0] && !fl_empty) ||
            (!reg_is_enque_s[0] && (old_occupancy_s1[0] |
                                    old_occupancy_s1[WATERLEVEL_IDX])));
    end
    // Update the occupancy counter
    new_occupancy_s1[WATERLEVEL_IDX-1:0] = (
        reg_is_enque_s[0] ? (old_occupancy_s1[WATERLEVEL_IDX-1:0] + 1) :
                            (old_occupancy_s1[WATERLEVEL_IDX-1:0] - 1));

    new_occupancy_s1[WATERLEVEL_IDX] = (reg_is_enque_s[0] ?
        (old_occupancy_s1[WATERLEVEL_IDX] | old_occupancy_s1[0]) :
        ((|old_occupancy_s1[WATERLEVEL_IDX-1:2]) | (&old_occupancy_s1[1:0])));

    // If enqueing, also deque the free list
    if (valid_s1 && reg_is_enque_s[0]) begin
        fl_rdreq = 1;
    end

    `ifdef DEBUG
    /**
     * Print a newline between pipeline output across timesteps.
     */
    debug_newline = in_valid;
    for (j = 0; j < (NUM_PIPELINE_STAGES - 1); j = j + 1) begin
        debug_newline |= reg_valid_s[j];
    end
    `endif
end

always @(posedge clk) begin
    if (rst) begin
        // Reset occupancy
        for (i = 0; i < HEAP_NUM_LPS; i = i + 1) begin
            occupancy[i] <= 0;
        end

        // Reset bitmaps
        for (i = 0; i < NUM_BITMAPS_L2; i = i + 1) begin
            l2_bitmaps[i] <= 0;
        end

        // Reset pipeline stages
        for (i = 0; i <= NUM_PIPELINE_STAGES; i = i + 1) begin
            reg_valid_s[i] <= 0;
        end

        // Reset init signals
        fl_init_done_r <= 0;
        bm_l3_init_done_r <= 0;
        fl_wraddress_counter_r <= 0;
        counter_l2_init_done_r <= 0;
        counter_l3_init_done_r <= 0;
        bm_l3_wraddress_counter_r <= 0;
        counter_l2_wraddress_counter_r <= 0;
        counter_l3_wraddress_counter_r <= 0;

        // Reset FSM state
        state <= FSM_STATE_INIT;
    end
    else begin
        /**
         * Stage 14: Spillover stage.
         */
        reg_valid_s[14] <= reg_valid_s[13];
        reg_bbq_id_s[14] <= reg_bbq_id_s[13];
        reg_he_data_s[14] <= reg_he_data_s[13];
        reg_op_type_s[14] <= reg_op_type_s[13];
        reg_is_enque_s[14] <= reg_is_enque_s[13];
        reg_priority_s[14] <= reg_priority_s[13];
        reg_is_deque_max_s[14] <= reg_is_deque_max_s[13];
        reg_is_deque_min_s[14] <= reg_is_deque_min_s[13];

        reg_pb_data_s14 <= reg_pb_data_s13;
        reg_l2_addr_s[14] <= reg_l2_addr_s[13];
        reg_l3_addr_s[14] <= reg_l3_addr_s[13];
        reg_op_color_s[14] <= reg_op_color_s[13];
        reg_l2_bitmap_s[14] <= reg_l2_bitmap_s[13];
        reg_l3_bitmap_s[14] <= reg_l3_bitmap_s[13];

        /**
         * Stage 13: Perform writes: update the priority bucket,
         * the free list, heap entries, next and prev pointers.
         */
        reg_valid_s[13] <= reg_valid_s[12];
        reg_bbq_id_s[13] <= reg_bbq_id_s[12];
        reg_he_data_s[13] <= he_data_s13;
        reg_op_type_s[13] <= reg_op_type_s[12];
        reg_is_enque_s[13] <= reg_is_enque_s[12];
        reg_priority_s[13] <= reg_priority_s[12];
        reg_is_deque_max_s[13] <= reg_is_deque_max_s[12];
        reg_is_deque_min_s[13] <= reg_is_deque_min_s[12];

        reg_he_data_s13 <= he_data;
        reg_np_data_s13 <= np_data;
        reg_pp_data_s13 <= pp_data;
        reg_pb_data_s13 <= int_pb_data;
        reg_l2_addr_s[13] <= reg_l2_addr_s[12];
        reg_l3_addr_s[13] <= reg_l3_addr_s[12];
        reg_op_color_s[13] <= reg_op_color_s[12];
        reg_l2_bitmap_s[13] <= reg_l2_bitmap_s[12];
        reg_l3_bitmap_s[13] <= reg_l3_bitmap_s[12];

        `ifdef DEBUG
        if (reg_valid_s[12]) begin
            if (!reg_pb_state_changes_s12) begin
                $display(
                    "[BBQ] At S13 (logical ID: %0d, op: %s, color: %s),",
                    reg_bbq_id_s[12], reg_op_type_s[12].name, reg_op_color_s[12].name,
                    " updating (relative priority = %0d),",
                    reg_priority_s[12] & (HEAP_NUM_PRIORITIES_PER_LP - 1),
                    " pb (head, tail) changes from ",
                    "(%b, %b) to (%b, %b)", reg_pb_q_s12.head,
                    reg_pb_q_s12.tail, int_pb_data.head, int_pb_data.tail);
            end
            else if (reg_is_enque_s[12]) begin
                $display(
                    "[BBQ] At S13 (logical ID: %0d, op: %s, color: %s),",
                    reg_bbq_id_s[12], reg_op_type_s[12].name, reg_op_color_s[12].name,
                    " updating (relative priority = %0d),",
                    reg_priority_s[12] & (HEAP_NUM_PRIORITIES_PER_LP - 1),
                    " pb (head, tail) changes from ",
                    "(INVALID_PTR, INVALID_PTR) to (%b, %b)",
                    int_pb_data.head, int_pb_data.tail);
            end
            else begin
                $display(
                    "[BBQ] At S13 (logical ID: %0d, op: %s, color: %s),",
                    reg_bbq_id_s[12], reg_op_type_s[12].name, reg_op_color_s[12].name,
                    " updating (relative priority = %0d),",
                    reg_priority_s[12] & (HEAP_NUM_PRIORITIES_PER_LP - 1),
                    " pb (head, tail) changes from ",
                    "(%b, %b) to (INVALID_PTR, INVALID_PTR)",
                    reg_pb_q_s12.head, reg_pb_q_s12.tail);
            end
        end
        `endif

        /**
         * Stage 12: Read delay for HE and pointers.
         */
        reg_valid_s[12] <= reg_valid_s[11];
        reg_bbq_id_s[12] <= reg_bbq_id_s[11];
        reg_he_data_s[12] <= reg_he_data_s[11];
        reg_op_type_s[12] <= reg_op_type_s[11];
        reg_is_enque_s[12] <= reg_is_enque_s[11];
        reg_priority_s[12] <= reg_priority_s[11];
        reg_is_deque_max_s[12] <= reg_is_deque_max_s[11];
        reg_is_deque_min_s[12] <= reg_is_deque_min_s[11];

        reg_l2_addr_s[12] <= reg_l2_addr_s[11];
        reg_l3_addr_s[12] <= reg_l3_addr_s[11];
        reg_op_color_s[12] <= reg_op_color_s[11];
        reg_l2_bitmap_s[12] <= reg_l2_bitmap_s[11];
        reg_l3_bitmap_s[12] <= reg_l3_bitmap_s[11];
        reg_pb_data_conflict_s12 <= reg_pb_data_conflict_s11;
        reg_pb_state_changes_s12 <= reg_pb_state_changes_s11;
        reg_pb_tail_pp_changes_s12 <= reg_pb_tail_pp_changes_s11;

        reg_he_q_s12 <= he_q_s12;
        reg_np_q_s12 <= np_q_s12;
        reg_pp_q_s12 <= pp_q_s12;

        reg_pb_q_s12 <= (
            reg_pb_addr_conflict_s12_s11 ?
               int_pb_data : reg_pb_q_s11);

        reg_pb_new_s12 <= (
            reg_pb_addr_conflict_s12_s11 ?
               int_pb_data : reg_pb_q_s11);

        if (reg_is_enque_s[11]) begin
            // PB becomes non-empty, update tail
            if (reg_pb_state_changes_s11) begin
                reg_pb_new_s12.tail <= fl_q_r[9];
            end
            reg_pb_new_s12.head <= fl_q_r[9];
        end

        `ifdef SIM
        if (reg_valid_s[11]) begin
            if ((he_wren && he_rden_r && (he_wraddress == he_rdaddress_r)) ||
                (np_wren && np_rden_r && (np_wraddress == np_rdaddress_r))) begin
                $display("[BBQ] Error: Unexpected conflict in R/W access");
                $finish;
            end
        end
        `endif
        `ifdef DEBUG
        if (reg_valid_s[11]) begin
            $display(
                "[BBQ] At S12 (logical ID: %0d, op: %s)",
                reg_bbq_id_s[11], reg_op_type_s[11].name,
                " for PB (relative priority = %0d)",
                reg_priority_s[11] & (HEAP_NUM_PRIORITIES_PER_LP - 1));
        end
        `endif

        /**
         * Stage 11: Read the heap entry and prev/next pointer
         * corresponding to the priority bucket to deque.
         */
        reg_valid_s[11] <= reg_valid_s[10];
        reg_bbq_id_s[11] <= reg_bbq_id_s[10];
        reg_he_data_s[11] <= reg_he_data_s[10];
        reg_op_type_s[11] <= reg_op_type_s[10];
        reg_is_enque_s[11] <= reg_is_enque_s[10];
        reg_priority_s[11] <= reg_priority_s[10];
        reg_is_deque_max_s[11] <= reg_is_deque_max_s[10];
        reg_is_deque_min_s[11] <= reg_is_deque_min_s[10];

        reg_pb_q_s11 <= int_pb_q;
        reg_l2_addr_s[11] <= reg_l2_addr_s[10];
        reg_l3_addr_s[11] <= reg_l3_addr_s[10];
        reg_op_color_s[11] <= reg_op_color_s[10];
        reg_l2_bitmap_s[11] <= reg_l2_bitmap_s[10];
        reg_l3_bitmap_s[11] <= reg_l3_bitmap_s[10];
        reg_pb_data_conflict_s11 <= reg_pb_data_conflict_s10;
        reg_pb_state_changes_s11 <= reg_pb_state_changes_s10;
        reg_pb_tail_pp_changes_s11 <= reg_pb_tail_pp_changes_s10;
        reg_pb_addr_conflict_s12_s11 <= reg_pb_addr_conflict_s11_s10;
        reg_pb_addr_conflict_s13_s11 <= reg_pb_addr_conflict_s12_s10;

        reg_pp_data_s11 <= pp_changes_s12_s11 ? fl_q_r[8] : fl_q_r[9];
        reg_pp_data_valid_s11 <= (pp_changes_s12_s11 || pp_changes_s13_s11);

        `ifdef DEBUG
        if (reg_valid_s[10]) begin
            $display(
                "[BBQ] At S11 (logical ID: %0d, op: %s)",
                reg_bbq_id_s[10], reg_op_type_s[10].name,
                " for PB (relative priority = %0d)",
                reg_priority_s[10] & (HEAP_NUM_PRIORITIES_PER_LP - 1));
        end
        `endif

        /**
         * Stage 10: Compute op color, read delay for PB.
         */
        reg_valid_s[10] <= reg_valid_s[9];
        reg_bbq_id_s[10] <= reg_bbq_id_s[9];
        reg_he_data_s[10] <= reg_he_data_s[9];
        reg_op_type_s[10] <= reg_op_type_s[9];
        reg_is_enque_s[10] <= reg_is_enque_s[9];
        reg_priority_s[10] <= reg_priority_s[9];
        reg_is_deque_max_s[10] <= reg_is_deque_max_s[9];
        reg_is_deque_min_s[10] <= reg_is_deque_min_s[9];

        reg_op_color_s[10] <= op_color_s10;
        reg_l2_addr_s[10] <= reg_l2_addr_s[9];
        reg_l3_addr_s[10] <= reg_l3_addr_s[9];
        reg_l2_bitmap_s[10] <= reg_l2_bitmap_s[9];
        reg_l3_bitmap_s[10] <= reg_l3_bitmap_s[9];
        reg_pb_update_s10 <= reg_pb_addr_conflict_s12_s9;
        reg_pb_addr_conflict_s11_s10 <= reg_pb_addr_conflict_s10_s9;
        reg_pb_addr_conflict_s12_s10 <= reg_pb_addr_conflict_s11_s9;

        // Determine if this op is going to result in PB data
        // conflict (dequeing a PB immediately after an enque
        // operation that causes it to become non-empty).
        reg_pb_data_conflict_s10 <= (reg_is_enque_s[10] &&
            !reg_l3_counter_non_zero_s9 && reg_pb_addr_conflict_s10_s9);

        // Determine if this op causes the PB state to change.
        // Change of state is defined differently based on op:
        // for enques, corresponds to a PB becoming non-empty,
        // and for deques, corresponds to a PB becoming empty.
        reg_pb_state_changes_s10 <= (reg_is_enque_s[9] ?
            (!reg_l3_counter_s9[WATERLEVEL_IDX] && reg_l3_counter_s9[0]) :
            (!reg_l3_counter_s9[WATERLEVEL_IDX] && !reg_l3_counter_s9[0]));

        // Determine if this op causes the previous pointer
        // corresponding to the PB tail to change. High iff
        // enqueing into a PB containing a single element.
        reg_pb_tail_pp_changes_s10 <= (reg_is_enque_s[9] &&
            !reg_old_l3_counter_s9[WATERLEVEL_IDX]
            && reg_old_l3_counter_s9[0]);

        `ifdef DEBUG
        if (reg_valid_s[9]) begin
            $display(
                "[BBQ] At S10 (logical ID: %0d, op: %s)",
                reg_bbq_id_s[9], reg_op_type_s[9].name,
                " for PB (relative priority = %0d)",
                reg_priority_s[9] & (HEAP_NUM_PRIORITIES_PER_LP - 1),
                " assigned color %s", op_color_s10.name);
        end
        `endif

        /**
         * Stage 9: Write-back the L3 counter and bitmap,
         * and read the corresponding PB (head and tail).
         */
        reg_valid_s[9] <= reg_valid_s[8];
        reg_bbq_id_s[9] <= reg_bbq_id_s[8];
        reg_he_data_s[9] <= reg_he_data_s[8];
        reg_op_type_s[9] <= reg_op_type_s[8];
        reg_is_enque_s[9] <= reg_is_enque_s[8];
        reg_priority_s[9] <= priority_s9;
        reg_is_deque_max_s[9] <= reg_is_deque_max_s[8];
        reg_is_deque_min_s[9] <= reg_is_deque_min_s[8];

        reg_l3_bitmap_s[9] <= bm_l3_data;
        reg_l2_addr_s[9] <= reg_l2_addr_s[8];
        reg_l3_addr_s[9] <= reg_l3_addr_s[8];
        reg_l2_bitmap_s[9] <= reg_l2_bitmap_s[8];

        reg_l3_counter_s9 <= l3_counter_s9;
        reg_l3_bitmap_idx_s9 <= reg_l3_bitmap_idx_s8;
        reg_old_l3_counter_s9 <= reg_l3_counter_rc_s8;
        reg_l3_counter_non_zero_s9 <= l3_counter_non_zero_s9;

        reg_pb_addr_conflict_s10_s9 <= pb_addr_conflict_s10_s9;
        reg_pb_addr_conflict_s11_s9 <= pb_addr_conflict_s11_s9;
        reg_pb_addr_conflict_s12_s9 <= pb_addr_conflict_s12_s9;
        reg_pb_addr_conflict_s13_s9 <= pb_addr_conflict_s13_s9;

        `ifdef DEBUG
        if (reg_valid_s[8]) begin
            $display(
                "[BBQ] At S9 (logical ID: %0d, op: %s), updating L3 counter (L3_addr, L3_idx) ",
                reg_bbq_id_s[8], reg_op_type_s[8].name, "= (%0d, %0d) to %0d", reg_l3_addr_s[8],
                reg_l3_bitmap_idx_s8, l3_counter_s9[WATERLEVEL_IDX-1:0]);
        end
        `endif

        /**
         * Stage 8: NOOP, read delay for L3 counter.
         */
        reg_valid_s[8] <= reg_valid_s[7];
        reg_bbq_id_s[8] <= reg_bbq_id_s[7];
        reg_he_data_s[8] <= reg_he_data_s[7];
        reg_op_type_s[8] <= reg_op_type_s[7];
        reg_is_enque_s[8] <= reg_is_enque_s[7];
        reg_priority_s[8] <= reg_priority_s[7];
        reg_is_deque_max_s[8] <= reg_is_deque_max_s[7];
        reg_is_deque_min_s[8] <= reg_is_deque_min_s[7];

        reg_l2_addr_s[8] <= reg_l2_addr_s[7];
        reg_l3_addr_s[8] <= reg_l3_addr_s[7];
        reg_l2_bitmap_s[8] <= reg_l2_bitmap_s[7];
        reg_l3_addr_conflict_s9_s8 <= reg_l3_addr_conflict_s8_s7;
        reg_l3_addr_conflict_s10_s8 <= reg_l3_addr_conflict_s9_s7;
        reg_l3_addr_conflict_s11_s8 <= reg_l3_addr_conflict_s10_s7;
        reg_l3_addr_conflict_s12_s8 <= reg_l3_addr_conflict_s11_s7;

        reg_l3_counter_s8 <= (reg_l3_counter_rdvalid_r1_s7 ?
                              l3_counter_q_s8 : l3_counter_s8);
        case (rcd_s8)
        READ_CARRY_DOWN: begin
            reg_l3_bitmap_idx_s8 <= reg_l3_bitmap_idx_s7;
            reg_l3_bitmap_postop_s8 <= reg_l3_bitmap_postop_s7;
            reg_l3_bitmap_idx_onehot_s8 <= reg_l3_bitmap_idx_onehot_s7;

            reg_l3_counter_rc_s8 <= (reg_l3_counter_rdvalid_r1_s7 ?
                                     l3_counter_q_s8 : l3_counter_s8);
        end
        READ_CARRY_RIGHT: begin
            reg_l3_counter_rc_s8 <= l3_counter_s9;
        end
        default: ;
        endcase

        // Forward L3 bitmap updates
        reg_l3_bitmap_s[8] <= (
            reg_l3_addr_conflict_s8_s7 ?
            bm_l3_data : reg_l3_bitmap_s[7]);

        `ifdef DEBUG
        if (reg_valid_s[7]) begin
            $display(
                "[BBQ] At S8 (logical ID: %0d, op: %s) for (L3 addr = %0d),",
                reg_bbq_id_s[7], reg_op_type_s[7].name, reg_l3_addr_s[7],
                " RCD is %s", rcd_s8.name);
        end
        `endif

        /**
         * Stage 7: Compute the L3 bitmap index and postop
         * bitmap, and read the corresponding L3 counter.
         */
        reg_valid_s[7] <= reg_valid_s[6];
        reg_bbq_id_s[7] <= reg_bbq_id_s[6];
        reg_he_data_s[7] <= reg_he_data_s[6];
        reg_op_type_s[7] <= reg_op_type_s[6];
        reg_is_enque_s[7] <= reg_is_enque_s[6];
        reg_priority_s[7] <= reg_priority_s[6];
        reg_is_deque_max_s[7] <= reg_is_deque_max_s[6];
        reg_is_deque_min_s[7] <= reg_is_deque_min_s[6];

        reg_l2_addr_s[7] <= reg_l2_addr_s[6];
        reg_l3_addr_s[7] <= reg_l3_addr_s[6];
        reg_l2_bitmap_s[7] <= reg_l2_bitmap_s[6];
        reg_l3_addr_conflict_s8_s7 <= reg_l3_addr_conflict_s7_s6;
        reg_l3_addr_conflict_s9_s7 <= reg_l3_addr_conflict_s8_s6;
        reg_l3_addr_conflict_s10_s7 <= reg_l3_addr_conflict_s9_s6;
        reg_l3_addr_conflict_s11_s7 <= reg_l3_addr_conflict_s10_s6;

        reg_l3_counter_rdvalid_r1_s7 <= 0;

        case (rcd_s7)
        READ_CARRY_DOWN: begin
            reg_l3_bitmap_idx_s7 <= l3_bitmap_idx_s7;
            reg_l3_bitmap_empty_s7 <= l3_bitmap_empty_s7;
            reg_l3_bitmap_postop_s7 <= l3_bitmap_postop_s7;
            reg_l3_bitmap_idx_onehot_s7 <= l3_bitmap_idx_onehot_s7;

            reg_l3_counter_rdvalid_r1_s7 <= (!l3_bitmap_empty_s7);
        end
        READ_CARRY_UP: begin
            reg_l3_bitmap_empty_s7 <= 0;
            reg_l3_bitmap_idx_s7 <= reg_l3_bitmap_idx_s8;
            reg_l3_bitmap_idx_onehot_s7 <= reg_l3_bitmap_idx_onehot_s8;

            if (!reg_l3_addr_conflict_s7_s6) begin
                reg_l3_bitmap_postop_s7 <= (
                    reg_l3_bitmap_postop_s8);
            end
        end
        default: ;
        endcase

        // Forward L3 bitmap updates
        reg_l3_bitmap_s[7] <= (
            reg_l3_addr_conflict_s8_s6 ?
            bm_l3_data : reg_l3_bitmap_s[6]);

        `ifdef DEBUG
        if (reg_valid_s[6]) begin
            $display(
                "[BBQ] At S7 (logical ID: %0d, op: %s) for (L3 addr = %0d),",
                reg_bbq_id_s[6], reg_op_type_s[6].name, reg_l3_addr_s[6],
                " RCD is %s", rcd_s7.name);
        end
        `endif

        /**
         * Stage 6: NOOP, read delay for L3 bitmap.
         */
        reg_valid_s[6] <= reg_valid_s[5];
        reg_bbq_id_s[6] <= reg_bbq_id_s[5];
        reg_he_data_s[6] <= reg_he_data_s[5];
        reg_op_type_s[6] <= reg_op_type_s[5];
        reg_is_enque_s[6] <= reg_is_enque_s[5];
        reg_priority_s[6] <= reg_priority_s[5];
        reg_is_deque_max_s[6] <= reg_is_deque_max_s[5];
        reg_is_deque_min_s[6] <= reg_is_deque_min_s[5];

        reg_l3_bitmap_s[6] <= l3_bitmap_s6;
        reg_l2_addr_s[6] <= reg_l2_addr_s[5];
        reg_l3_addr_s[6] <= reg_l3_addr_s[5];
        reg_l2_bitmap_s[6] <= reg_l2_bitmap_s[5];
        reg_l3_addr_conflict_s7_s6 <= reg_l3_addr_conflict_s6_s5;
        reg_l3_addr_conflict_s8_s6 <= reg_l3_addr_conflict_s7_s5;
        reg_l3_addr_conflict_s9_s6 <= reg_l3_addr_conflict_s8_s5;
        reg_l3_addr_conflict_s10_s6 <= reg_l3_addr_conflict_s9_s5;

        `ifdef DEBUG
        if (reg_valid_s[5]) begin
            $display(
                "[BBQ] At S6 (logical ID: %0d, op: %s) for (L3 addr = %0d)",
                reg_bbq_id_s[5], reg_op_type_s[5].name, reg_l3_addr_s[5]);
        end
        `endif

        /**
         * Stage 5: Write-back the L2 counter and bitmap,
         * and read the corresponding L3 bitmap.
         */
        reg_valid_s[5] <= reg_valid_s[4];
        reg_bbq_id_s[5] <= reg_bbq_id_s[4];
        reg_he_data_s[5] <= reg_he_data_s[4];
        reg_op_type_s[5] <= reg_op_type_s[4];
        reg_is_enque_s[5] <= reg_is_enque_s[4];
        reg_priority_s[5] <= reg_priority_s[4];
        reg_is_deque_max_s[5] <= reg_is_deque_max_s[4];
        reg_is_deque_min_s[5] <= reg_is_deque_min_s[4];

        reg_l2_bitmap_s[5] <= l2_bitmap_s5;
        reg_l2_addr_s[5] <= reg_l2_addr_s[4];
        reg_l3_addr_s[5] <= {reg_l2_addr_s[4], reg_l2_bitmap_idx_s4};

        reg_l2_counter_s5 <= l2_counter_s5;
        reg_l2_bitmap_idx_s5 <= reg_l2_bitmap_idx_s4;

        reg_l3_addr_conflict_s6_s5 <= l3_addr_conflict_s6_s5;
        reg_l3_addr_conflict_s7_s5 <= l3_addr_conflict_s7_s5;
        reg_l3_addr_conflict_s8_s5 <= l3_addr_conflict_s8_s5;
        reg_l3_addr_conflict_s9_s5 <= l3_addr_conflict_s9_s5;

        // Write-back L2 bitmap
        if (reg_valid_s[4]) begin
            l2_bitmaps[reg_l2_addr_s[4]] <= l2_bitmap_s5;
        end

        `ifdef DEBUG
        if (reg_valid_s[4]) begin
            $display(
                "[BBQ] At S5 (logical ID: %0d, op: %s), updating L2 counter (L2_addr, L2_idx) ",
                reg_bbq_id_s[4], reg_op_type_s[4].name, "= (%0d, %0d) to %0d", reg_l2_addr_s[4],
                reg_l2_bitmap_idx_s4, l2_counter_s5[WATERLEVEL_IDX-1:0]);
        end
        `endif

        /**
         * Stage 4: NOOP, read delay for L2 counter.
         */
        reg_valid_s[4] <= reg_valid_s[3];
        reg_bbq_id_s[4] <= reg_bbq_id_s[3];
        reg_he_data_s[4] <= reg_he_data_s[3];
        reg_op_type_s[4] <= reg_op_type_s[3];
        reg_is_enque_s[4] <= reg_is_enque_s[3];
        reg_priority_s[4] <= reg_priority_s[3];
        reg_is_deque_max_s[4] <= reg_is_deque_max_s[3];
        reg_is_deque_min_s[4] <= reg_is_deque_min_s[3];

        reg_l2_addr_s[4] <= reg_l2_addr_s[3];
        reg_l2_addr_conflict_s5_s4 <= reg_l2_addr_conflict_s4_s3;
        reg_l2_addr_conflict_s6_s4 <= reg_l2_addr_conflict_s5_s3;
        reg_l2_addr_conflict_s7_s4 <= reg_l2_addr_conflict_s6_s3;
        reg_l2_addr_conflict_s8_s4 <= reg_l2_addr_conflict_s7_s3;

        reg_l2_counter_s4 <= (reg_l2_counter_rdvalid_r1_s3 ?
                              l2_counter_q_s4 : l2_counter_s4);
        case (rcd_s4)
        READ_CARRY_DOWN: begin
            reg_l2_bitmap_idx_s4 <= reg_l2_bitmap_idx_s3;
            reg_l2_bitmap_postop_s4 <= reg_l2_bitmap_postop_s3;
            reg_l2_bitmap_idx_onehot_s4 <= reg_l2_bitmap_idx_onehot_s3;

            reg_l2_counter_rc_s4 <= (reg_l2_counter_rdvalid_r1_s3 ?
                                     l2_counter_q_s4 : l2_counter_s4);
        end
        READ_CARRY_RIGHT: begin
            reg_l2_counter_rc_s4 <= l2_counter_s5;
        end
        default: ;
        endcase

        // Forward L2 bitmap updates
        reg_l2_bitmap_s[4] <= (
            reg_l2_addr_conflict_s4_s3 ?
            l2_bitmap_s5 : reg_l2_bitmap_s[3]);

        `ifdef DEBUG
        if (reg_valid_s[3]) begin
            $display(
                "[BBQ] At S4 (logical ID: %0d, op: %s) for (L2 addr = %0d),",
                reg_bbq_id_s[3], reg_op_type_s[3].name, reg_l2_addr_s[3],
                " RCD is %s", rcd_s4.name);
        end
        `endif

        /**
         * Stage 3: Compute the L2 bitmap index and postop
         * bitmap, and read the corresponding L2 counter.
         */
        reg_valid_s[3] <= reg_valid_s[2];
        reg_bbq_id_s[3] <= reg_bbq_id_s[2];
        reg_he_data_s[3] <= reg_he_data_s[2];
        reg_op_type_s[3] <= reg_op_type_s[2];
        reg_is_enque_s[3] <= reg_is_enque_s[2];
        reg_priority_s[3] <= reg_priority_s[2];
        reg_is_deque_max_s[3] <= reg_is_deque_max_s[2];
        reg_is_deque_min_s[3] <= reg_is_deque_min_s[2];

        reg_l2_addr_s[3] <= reg_l2_addr_s[2];
        reg_l2_addr_conflict_s4_s3 <= reg_l2_addr_conflict_s3_s2;
        reg_l2_addr_conflict_s5_s3 <= reg_l2_addr_conflict_s4_s2;
        reg_l2_addr_conflict_s6_s3 <= reg_l2_addr_conflict_s5_s2;
        reg_l2_addr_conflict_s7_s3 <= reg_l2_addr_conflict_s6_s2;

        reg_l2_counter_rdvalid_r1_s3 <= 0;

        case (rcd_s3)
        READ_CARRY_DOWN: begin
            reg_l2_bitmap_idx_s3 <= l2_bitmap_idx_s3;
            reg_l2_bitmap_empty_s3 <= l2_bitmap_empty_s3;
            reg_l2_bitmap_postop_s3 <= l2_bitmap_postop_s3;
            reg_l2_bitmap_idx_onehot_s3 <= l2_bitmap_idx_onehot_s3;

            reg_l2_counter_rdvalid_r1_s3 <= (!l2_bitmap_empty_s3);
        end
        READ_CARRY_UP: begin
            reg_l2_bitmap_empty_s3 <= 0;
            reg_l2_bitmap_idx_s3 <= reg_l2_bitmap_idx_s4;
            reg_l2_bitmap_idx_onehot_s3 <= reg_l2_bitmap_idx_onehot_s4;

            if (!reg_l2_addr_conflict_s3_s2) begin
                reg_l2_bitmap_postop_s3 <= (
                    reg_l2_bitmap_postop_s4);
            end
        end
        default: ;
        endcase

        // Forward L2 bitmap updates
        reg_l2_bitmap_s[3] <= (
            reg_l2_addr_conflict_s4_s2 ?
            l2_bitmap_s5 : reg_l2_bitmap_s[2]);

        `ifdef DEBUG
        if (reg_valid_s[2]) begin
            $display(
                "[BBQ] At S3 (logical ID: %0d, op: %s) for (L2 addr = %0d),",
                reg_bbq_id_s[2], reg_op_type_s[2].name, reg_l2_addr_s[2],
                " RCD is %s", rcd_s3.name);
        end
        `endif

        /**
         * Stage 2: Steer op to the appropriate logical BBQ.
         */
        reg_valid_s[2] <= reg_valid_s[1];
        reg_bbq_id_s[2] <= reg_bbq_id_s[1];
        reg_he_data_s[2] <= reg_he_data_s[1];
        reg_op_type_s[2] <= reg_op_type_s[1];
        reg_is_enque_s[2] <= reg_is_enque_s[1];
        reg_priority_s[2] <= reg_priority_s[1];
        reg_is_deque_max_s[2] <= reg_is_deque_max_s[1];
        reg_is_deque_min_s[2] <= reg_is_deque_min_s[1];

        reg_l2_addr_s[2] <= reg_bbq_id_s[1];

        reg_l2_addr_conflict_s3_s2 <= l2_addr_conflict_s3_s2;
        reg_l2_addr_conflict_s4_s2 <= l2_addr_conflict_s4_s2;
        reg_l2_addr_conflict_s5_s2 <= l2_addr_conflict_s5_s2;
        reg_l2_addr_conflict_s6_s2 <= l2_addr_conflict_s6_s2;

        // Forward L2 bitmap updates
        reg_l2_bitmap_s[2] <= (
            l2_addr_conflict_s5_s2 ?
            l2_bitmap_s5 : l2_bitmaps[reg_bbq_id_s[1]]);

        `ifdef DEBUG
        if (reg_valid_s[1]) begin
            $display(
                "[BBQ] At S2 (logical ID: %0d, op: %s),",
                reg_bbq_id_s[1], reg_op_type_s[1].name,
                " steering op to the corresponding L2 bitmap");
        end
        `endif
        /**
         * Stage 1: Determine operation validity. Disables the pipeline
         * stage if the BBQ is empty (deques) or FL is empty (enqueues).
         */
        reg_valid_s[1] <= valid_s1;
        reg_bbq_id_s[1] <= reg_bbq_id_s[0];
        reg_he_data_s[1] <= reg_he_data_s[0];
        reg_op_type_s[1] <= reg_op_type_s[0];
        reg_is_enque_s[1] <= reg_is_enque_s[0];
        reg_priority_s[1] <= reg_priority_s[0];
        reg_is_deque_max_s[1] <= reg_is_deque_max_s[0];
        reg_is_deque_min_s[1] <= reg_is_deque_min_s[0];

        reg_old_occupancy_s1 <= old_occupancy_s1;
        reg_new_occupancy_s1 <= new_occupancy_s1;

        if (valid_s1) begin
            occupancy[reg_bbq_id_s[0]] <= new_occupancy_s1;
        end

        `ifdef DEBUG
        if (reg_valid_s[0] && !valid_s1) begin
            $display(
                "[BBQ] At S1 (logical ID: %0d, op: %s), rejected at Stage 0->1",
                reg_bbq_id_s[0], reg_op_type_s[0].name);
        end
        if (valid_s1) begin
            $display(
                "[BBQ] At S1 (logical ID: %0d, op: %s), updating occupancy",
                reg_bbq_id_s[0], reg_op_type_s[0].name, " from %0d to %0d",
                old_occupancy_s1[WATERLEVEL_IDX-1:0],
                new_occupancy_s1[WATERLEVEL_IDX-1:0]);
        end
        `endif

        /**
         * Stage 0: Register inputs.
         */
        reg_bbq_id_s[0] <= bbq_id_s0;
        reg_op_type_s[0] <= in_op_type;
        reg_he_data_s[0] <= in_he_data;
        reg_priority_s[0] <= in_he_priority;
        reg_valid_s[0] <= (ready & in_valid);
        reg_is_enque_s[0] <= (in_op_type == HEAP_OP_ENQUE);
        reg_is_deque_max_s[0] <= (in_op_type == HEAP_OP_DEQUE_MAX);
        reg_is_deque_min_s[0] <= (in_op_type == HEAP_OP_DEQUE_MIN);

        `ifdef DEBUG
        if (in_valid) begin
            if (in_op_type == HEAP_OP_ENQUE) begin
                $display("[BBQ] At S0 (logical ID: %0d), enqueing %0d with relative priority %0d",
                         bbq_id_s0, in_he_data, in_he_priority & (HEAP_NUM_PRIORITIES_PER_LP - 1));
            end
            else begin
                $display("[BBQ] At S0 (logical ID: %0d), performing %s",
                         bbq_id_s0, in_op_type.name);
            end
        end

        if (debug_newline) begin
            $display("");
        end
        if ((state == FSM_STATE_INIT) &&
            (state_next == FSM_STATE_READY)) begin
            $display("[BBQ] Heap initialization complete!");
        end
        `endif

        // Register init signals
        fl_init_done_r <= fl_init_done;
        bm_l3_init_done_r <= bm_l3_init_done;
        counter_l2_init_done_r <= counter_l2_init_done;
        counter_l3_init_done_r <= counter_l3_init_done;

        fl_wraddress_counter_r <= fl_wraddress_counter_r + 1;
        bm_l3_wraddress_counter_r <= bm_l3_wraddress_counter_r + 1;
        counter_l2_wraddress_counter_r <= counter_l2_wraddress_counter_r + 1;
        counter_l3_wraddress_counter_r <= counter_l3_wraddress_counter_r + 1;

        // Register read signals
        pb_q_r <= pb_q;
        he_rden_r <= he_rden;
        np_rden_r <= np_rden;
        pp_rden_r <= pp_rden;
        he_rdaddress_r <= he_rdaddress;
        np_rdaddress_r <= np_rdaddress;
        pp_rdaddress_r <= pp_rdaddress;

        // Register write signals
        he_wren_r <= he_wren;
        np_wren_r <= np_wren;
        pp_wren_r <= pp_wren;
        bm_l3_data_r <= bm_l3_data;
        he_wraddress_r <= he_wraddress;
        np_wraddress_r <= np_wraddress;
        pp_wraddress_r <= pp_wraddress;

        fl_q_r[0] <= fl_q;
        for (i = 0; i < 10; i = i + 1) begin
            fl_q_r[i + 1] <= fl_q_r[i];
        end

        // Register R/W conflict signals
        reg_pb_rdwr_conflict_r1 <= pb_rdwr_conflict;
        reg_pb_rdwr_conflict_r2 <= reg_pb_rdwr_conflict_r1;

        // Update FSM state
        state <= state_next;
    end
end

// Free list
sc_fifo #(
    .DWIDTH(HEAP_ENTRY_AWIDTH),
    .DEPTH(HEAP_MAX_NUM_ENTRIES),
    .IS_SHOWAHEAD(0),
    .IS_OUTDATA_REG(1)
)
free_list (
    .clock(clk),
    .data(fl_data),
    .rdreq(fl_rdreq),
    .wrreq(fl_wrreq),
    .empty(fl_empty),
    .full(),
    .q(fl_q),
    .usedw()
);

// Heap entries
bram_simple2port #(
    .DWIDTH(HEAP_ENTRY_DWIDTH),
    .AWIDTH(HEAP_ENTRY_AWIDTH),
    .DEPTH(HEAP_MAX_NUM_ENTRIES),
    .IS_OUTDATA_REG(0)
)
heap_entries (
    .clock(clk),
    .data(he_data),
    .rden(he_rden),
    .wren(he_wren),
    .rdaddress(he_rdaddress),
    .wraddress(he_wraddress),
    .q(he_q)
);

// Next pointers
bram_simple2port #(
    .DWIDTH(HEAP_ENTRY_AWIDTH),
    .AWIDTH(HEAP_ENTRY_AWIDTH),
    .DEPTH(HEAP_MAX_NUM_ENTRIES),
    .IS_OUTDATA_REG(0)
)
next_pointers (
    .clock(clk),
    .data(np_data),
    .rden(np_rden),
    .wren(np_wren),
    .rdaddress(np_rdaddress),
    .wraddress(np_wraddress),
    .q(np_q)
);

// Previous pointers
bram_simple2port #(
    .DWIDTH(HEAP_ENTRY_AWIDTH),
    .AWIDTH(HEAP_ENTRY_AWIDTH),
    .DEPTH(HEAP_MAX_NUM_ENTRIES),
    .IS_OUTDATA_REG(0)
)
previous_pointers (
    .clock(clk),
    .data(pp_data),
    .rden(pp_rden),
    .wren(pp_wren),
    .rdaddress(pp_rdaddress),
    .wraddress(pp_wraddress),
    .q(pp_q)
);

// Priority buckets
bram_simple2port #(
    .DWIDTH(LIST_T_WIDTH),
    .AWIDTH(HEAP_PRIORITY_BUCKETS_AWIDTH),
    .DEPTH(HEAP_NUM_PRIORITIES),
    .IS_OUTDATA_REG(0)
)
priority_buckets (
    .clock(clk),
    .data(pb_data),
    .rden(pb_rden),
    .wren(pb_wren),
    .rdaddress(pb_rdaddress),
    .wraddress(pb_wraddress),
    .q(pb_q)
);

// L3 bitmaps
bram_simple2port #(
    .DWIDTH(HEAP_BITMAP_WIDTH),
    .AWIDTH(BITMAP_L3_AWIDTH),
    .DEPTH(NUM_BITMAPS_L3),
    .IS_OUTDATA_REG(0)
)
bm_l3 (
    .clock(clk),
    .data(bm_l3_data),
    .rden(bm_l3_rden),
    .wren(bm_l3_wren),
    .rdaddress(bm_l3_rdaddress),
    .wraddress(bm_l3_wraddress),
    .q(bm_l3_q)
);

// L2 counters
bram_simple2port #(
    .DWIDTH(COUNTER_T_WIDTH),
    .AWIDTH(COUNTER_L2_AWIDTH),
    .DEPTH(NUM_COUNTERS_L2),
    .IS_OUTDATA_REG(0)
)
counters_l2 (
    .clock(clk),
    .data(counter_l2_data),
    .rden(counter_l2_rden),
    .wren(counter_l2_wren),
    .rdaddress(counter_l2_rdaddress),
    .wraddress(counter_l2_wraddress),
    .q(counter_l2_q)
);

// L3 counters
bram_simple2port #(
    .DWIDTH(COUNTER_T_WIDTH),
    .AWIDTH(COUNTER_L3_AWIDTH),
    .DEPTH(NUM_COUNTERS_L3),
    .IS_OUTDATA_REG(0)
)
counters_l3 (
    .clock(clk),
    .data(counter_l3_data),
    .rden(counter_l3_rden),
    .wren(counter_l3_wren),
    .rdaddress(counter_l3_rdaddress),
    .wraddress(counter_l3_wraddress),
    .q(counter_l3_q)
);

// L2 FFSs
ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l2_inst0 (
    .x(reg_l2_bitmap_s[2]),
    .msb(ffs_l2_inst_msb[0]),
    .lsb(ffs_l2_inst_lsb[0]),
    .msb_onehot(ffs_l2_inst_msb_onehot[0]),
    .lsb_onehot(ffs_l2_inst_lsb_onehot[0]),
    .zero(ffs_l2_inst_zero[0])
);

ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l2_inst1 (
    .x(reg_l2_bitmap_postop_s3),
    .msb(ffs_l2_inst_msb[1]),
    .lsb(ffs_l2_inst_lsb[1]),
    .msb_onehot(ffs_l2_inst_msb_onehot[1]),
    .lsb_onehot(ffs_l2_inst_lsb_onehot[1]),
    .zero(ffs_l2_inst_zero[1])
);

ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l2_inst2 (
    .x(reg_l2_bitmap_postop_s4),
    .msb(ffs_l2_inst_msb[2]),
    .lsb(ffs_l2_inst_lsb[2]),
    .msb_onehot(ffs_l2_inst_msb_onehot[2]),
    .lsb_onehot(ffs_l2_inst_lsb_onehot[2]),
    .zero(ffs_l2_inst_zero[2])
);

// L3 FFSs
ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l3_inst0 (
    .x(reg_l3_bitmap_s[6]),
    .msb(ffs_l3_inst_msb[0]),
    .lsb(ffs_l3_inst_lsb[0]),
    .msb_onehot(ffs_l3_inst_msb_onehot[0]),
    .lsb_onehot(ffs_l3_inst_lsb_onehot[0]),
    .zero(ffs_l3_inst_zero[0])
);

ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l3_inst1 (
    .x(reg_l3_bitmap_postop_s7),
    .msb(ffs_l3_inst_msb[1]),
    .lsb(ffs_l3_inst_lsb[1]),
    .msb_onehot(ffs_l3_inst_msb_onehot[1]),
    .lsb_onehot(ffs_l3_inst_lsb_onehot[1]),
    .zero(ffs_l3_inst_zero[1])
);

ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l3_inst2 (
    .x(reg_l3_bitmap_postop_s8),
    .msb(ffs_l3_inst_msb[2]),
    .lsb(ffs_l3_inst_lsb[2]),
    .msb_onehot(ffs_l3_inst_msb_onehot[2]),
    .lsb_onehot(ffs_l3_inst_lsb_onehot[2]),
    .zero(ffs_l3_inst_zero[2])
);

endmodule
