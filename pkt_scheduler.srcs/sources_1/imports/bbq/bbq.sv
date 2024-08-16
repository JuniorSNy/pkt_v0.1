import heap_ops::*;
// `define DEBUG

/**
 * Implements an integer priority queue in hardware using a configurable
 * Hierarchical Find First Set (HFFS) Queue. The implementation is fully
 * pipelined, capable of performing one operation (enqueue, dequeue-*,
 * or peek) every cycle.
 */
module bbq #(
    parameter HEAP_ENTRY_DWIDTH = 32,
    parameter HEAP_MAX_NUM_ENTRIES = ((1 << 17) - 1),
    localparam HEAP_BITMAP_WIDTH = 2, // Bitmap bit-width
    localparam HEAP_NUM_LPS = 2, // Number of logical BBQs
    localparam HEAP_LOGICAL_BBQ_AWIDTH = ($clog2(HEAP_NUM_LPS)),
    localparam HEAP_ENTRY_AWIDTH = ($clog2(HEAP_MAX_NUM_ENTRIES)),
    localparam HEAP_NUM_LEVELS = 6, // Number of bitmap tree levels
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
localparam NUM_PIPELINE_STAGES          = 26;

localparam NUM_BITMAPS_L1               = 1;
localparam NUM_BITMAPS_L2               = (HEAP_BITMAP_WIDTH ** 1);
localparam NUM_BITMAPS_L3               = (HEAP_BITMAP_WIDTH ** 2);
localparam NUM_BITMAPS_L4               = (HEAP_BITMAP_WIDTH ** 3);
localparam NUM_BITMAPS_L5               = (HEAP_BITMAP_WIDTH ** 4);
localparam NUM_BITMAPS_L6               = (HEAP_BITMAP_WIDTH ** 5);
localparam BITMAP_L2_AWIDTH             = ($clog2(NUM_BITMAPS_L2));
localparam BITMAP_L3_AWIDTH             = ($clog2(NUM_BITMAPS_L3));
localparam BITMAP_L4_AWIDTH             = ($clog2(NUM_BITMAPS_L4));
localparam BITMAP_L5_AWIDTH             = ($clog2(NUM_BITMAPS_L5));
localparam BITMAP_L6_AWIDTH             = ($clog2(NUM_BITMAPS_L6));

localparam NUM_COUNTERS_L1              = (NUM_BITMAPS_L2);
localparam NUM_COUNTERS_L2              = (NUM_BITMAPS_L3);
localparam NUM_COUNTERS_L3              = (NUM_BITMAPS_L4);
localparam NUM_COUNTERS_L4              = (NUM_BITMAPS_L5);
localparam NUM_COUNTERS_L5              = (NUM_BITMAPS_L6);
localparam NUM_COUNTERS_L6              = (HEAP_NUM_PRIORITIES);
localparam COUNTER_T_WIDTH              = (HEAP_ENTRY_AWIDTH + 1);
localparam COUNTER_L1_AWIDTH            = ($clog2(NUM_COUNTERS_L1));
localparam COUNTER_L2_AWIDTH            = ($clog2(NUM_COUNTERS_L2));
localparam COUNTER_L3_AWIDTH            = ($clog2(NUM_COUNTERS_L3));
localparam COUNTER_L4_AWIDTH            = ($clog2(NUM_COUNTERS_L4));
localparam COUNTER_L5_AWIDTH            = ($clog2(NUM_COUNTERS_L5));
localparam COUNTER_L6_AWIDTH            = ($clog2(NUM_COUNTERS_L6));

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
logic [HEAP_ENTRY_AWIDTH-1:0] fl_q_r[22:0];
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

// L4 bitmaps
logic bm_l4_rden;
logic bm_l4_wren;
logic [HEAP_BITMAP_WIDTH-1:0] bm_l4_q;
logic [HEAP_BITMAP_WIDTH-1:0] bm_l4_data;
logic [HEAP_BITMAP_WIDTH-1:0] bm_l4_data_r;
logic [BITMAP_L4_AWIDTH-1:0] bm_l4_rdaddress;
logic [BITMAP_L4_AWIDTH-1:0] bm_l4_wraddress;
logic [BITMAP_L4_AWIDTH-1:0] bm_l4_wraddress_counter_r;

// L4 counters
logic counter_l4_rden;
logic counter_l4_wren;
logic [COUNTER_T_WIDTH-1:0] counter_l4_q;
logic [COUNTER_T_WIDTH-1:0] counter_l4_data;
logic [COUNTER_L4_AWIDTH-1:0] counter_l4_rdaddress;
logic [COUNTER_L4_AWIDTH-1:0] counter_l4_wraddress;
logic [COUNTER_L4_AWIDTH-1:0] counter_l4_wraddress_counter_r;

// L5 bitmaps
logic bm_l5_rden;
logic bm_l5_wren;
logic [HEAP_BITMAP_WIDTH-1:0] bm_l5_q;
logic [HEAP_BITMAP_WIDTH-1:0] bm_l5_data;
logic [HEAP_BITMAP_WIDTH-1:0] bm_l5_data_r;
logic [BITMAP_L5_AWIDTH-1:0] bm_l5_rdaddress;
logic [BITMAP_L5_AWIDTH-1:0] bm_l5_wraddress;
logic [BITMAP_L5_AWIDTH-1:0] bm_l5_wraddress_counter_r;

// L5 counters
logic counter_l5_rden;
logic counter_l5_wren;
logic [COUNTER_T_WIDTH-1:0] counter_l5_q;
logic [COUNTER_T_WIDTH-1:0] counter_l5_data;
logic [COUNTER_L5_AWIDTH-1:0] counter_l5_rdaddress;
logic [COUNTER_L5_AWIDTH-1:0] counter_l5_wraddress;
logic [COUNTER_L5_AWIDTH-1:0] counter_l5_wraddress_counter_r;

// L6 bitmaps
logic bm_l6_rden;
logic bm_l6_wren;
logic [HEAP_BITMAP_WIDTH-1:0] bm_l6_q;
logic [HEAP_BITMAP_WIDTH-1:0] bm_l6_data;
logic [HEAP_BITMAP_WIDTH-1:0] bm_l6_data_r;
logic [BITMAP_L6_AWIDTH-1:0] bm_l6_rdaddress;
logic [BITMAP_L6_AWIDTH-1:0] bm_l6_wraddress;
logic [BITMAP_L6_AWIDTH-1:0] bm_l6_wraddress_counter_r;

// L6 counters
logic counter_l6_rden;
logic counter_l6_wren;
logic [COUNTER_T_WIDTH-1:0] counter_l6_q;
logic [COUNTER_T_WIDTH-1:0] counter_l6_data;
logic [COUNTER_L6_AWIDTH-1:0] counter_l6_rdaddress;
logic [COUNTER_L6_AWIDTH-1:0] counter_l6_wraddress;
logic [COUNTER_L6_AWIDTH-1:0] counter_l6_wraddress_counter_r;

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
logic [BITMAP_L4_AWIDTH-1:0]            reg_l4_addr_s[NUM_PIPELINE_STAGES:0];
logic [BITMAP_L5_AWIDTH-1:0]            reg_l5_addr_s[NUM_PIPELINE_STAGES:0];
logic [BITMAP_L6_AWIDTH-1:0]            reg_l6_addr_s[NUM_PIPELINE_STAGES:0];
op_color_t                              reg_op_color_s[NUM_PIPELINE_STAGES:0];
logic                                   reg_is_enque_s[NUM_PIPELINE_STAGES:0];
heap_priority_t                         reg_priority_s[NUM_PIPELINE_STAGES:0];
bitmap_t                                reg_l2_bitmap_s[NUM_PIPELINE_STAGES:0];
bitmap_t                                reg_l3_bitmap_s[NUM_PIPELINE_STAGES:0];
bitmap_t                                reg_l4_bitmap_s[NUM_PIPELINE_STAGES:0];
bitmap_t                                reg_l5_bitmap_s[NUM_PIPELINE_STAGES:0];
bitmap_t                                reg_l6_bitmap_s[NUM_PIPELINE_STAGES:0];
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
counter_t                               l3_counter_s9;
logic                                   l3_counter_non_zero_s9;
logic                                   l4_addr_conflict_s10_s9;
logic                                   l4_addr_conflict_s11_s9;
logic                                   l4_addr_conflict_s12_s9;
logic                                   l4_addr_conflict_s13_s9;
counter_t                               reg_l3_counter_s9;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l3_bitmap_idx_s9;
logic                                   reg_l4_addr_conflict_s10_s9;
logic                                   reg_l4_addr_conflict_s11_s9;
logic                                   reg_l4_addr_conflict_s12_s9;
logic                                   reg_l4_addr_conflict_s13_s9;

// Stage 10 metadata
bitmap_t                                l4_bitmap_s10;
logic                                   reg_l4_addr_conflict_s11_s10;
logic                                   reg_l4_addr_conflict_s12_s10;
logic                                   reg_l4_addr_conflict_s13_s10;
logic                                   reg_l4_addr_conflict_s14_s10;

// Stage 11 metadata
read_carry_direction_t                  rcd_s11;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       l4_bitmap_idx_s11;
logic                                   l4_bitmap_empty_s11;
bitmap_t                                l4_bitmap_postop_s11;
bitmap_t                                l4_bitmap_idx_onehot_s11;
logic                                   l4_bitmap_changes_s13_s11;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l4_bitmap_idx_s11;
logic                                   reg_l4_bitmap_empty_s11;
bitmap_t                                reg_l4_bitmap_postop_s11;
bitmap_t                                reg_l4_bitmap_idx_onehot_s11;
logic                                   reg_l4_counter_rdvalid_r1_s11;
logic                                   reg_l4_addr_conflict_s12_s11;
logic                                   reg_l4_addr_conflict_s13_s11;
logic                                   reg_l4_addr_conflict_s14_s11;
logic                                   reg_l4_addr_conflict_s15_s11;

// Stage 12 metadata
read_carry_direction_t                  rcd_s12;
counter_t                               l4_counter_s12;
counter_t                               l4_counter_q_s12;
counter_t                               reg_l4_counter_s12;
counter_t                               reg_l4_counter_rc_s12;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l4_bitmap_idx_s12;
bitmap_t                                reg_l4_bitmap_postop_s12;
bitmap_t                                reg_l4_bitmap_idx_onehot_s12;
logic                                   reg_l4_addr_conflict_s13_s12;
logic                                   reg_l4_addr_conflict_s14_s12;
logic                                   reg_l4_addr_conflict_s15_s12;
logic                                   reg_l4_addr_conflict_s16_s12;

// Stage 13 metadata
counter_t                               l4_counter_s13;
logic                                   l4_counter_non_zero_s13;
logic                                   l5_addr_conflict_s14_s13;
logic                                   l5_addr_conflict_s15_s13;
logic                                   l5_addr_conflict_s16_s13;
logic                                   l5_addr_conflict_s17_s13;
counter_t                               reg_l4_counter_s13;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l4_bitmap_idx_s13;
logic                                   reg_l5_addr_conflict_s14_s13;
logic                                   reg_l5_addr_conflict_s15_s13;
logic                                   reg_l5_addr_conflict_s16_s13;
logic                                   reg_l5_addr_conflict_s17_s13;

// Stage 14 metadata
bitmap_t                                l5_bitmap_s14;
logic                                   reg_l5_addr_conflict_s15_s14;
logic                                   reg_l5_addr_conflict_s16_s14;
logic                                   reg_l5_addr_conflict_s17_s14;
logic                                   reg_l5_addr_conflict_s18_s14;

// Stage 15 metadata
read_carry_direction_t                  rcd_s15;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       l5_bitmap_idx_s15;
logic                                   l5_bitmap_empty_s15;
bitmap_t                                l5_bitmap_postop_s15;
bitmap_t                                l5_bitmap_idx_onehot_s15;
logic                                   l5_bitmap_changes_s17_s15;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l5_bitmap_idx_s15;
logic                                   reg_l5_bitmap_empty_s15;
bitmap_t                                reg_l5_bitmap_postop_s15;
bitmap_t                                reg_l5_bitmap_idx_onehot_s15;
logic                                   reg_l5_counter_rdvalid_r1_s15;
logic                                   reg_l5_addr_conflict_s16_s15;
logic                                   reg_l5_addr_conflict_s17_s15;
logic                                   reg_l5_addr_conflict_s18_s15;
logic                                   reg_l5_addr_conflict_s19_s15;

// Stage 16 metadata
read_carry_direction_t                  rcd_s16;
counter_t                               l5_counter_s16;
counter_t                               l5_counter_q_s16;
counter_t                               reg_l5_counter_s16;
counter_t                               reg_l5_counter_rc_s16;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l5_bitmap_idx_s16;
bitmap_t                                reg_l5_bitmap_postop_s16;
bitmap_t                                reg_l5_bitmap_idx_onehot_s16;
logic                                   reg_l5_addr_conflict_s17_s16;
logic                                   reg_l5_addr_conflict_s18_s16;
logic                                   reg_l5_addr_conflict_s19_s16;
logic                                   reg_l5_addr_conflict_s20_s16;

// Stage 17 metadata
counter_t                               l5_counter_s17;
logic                                   l5_counter_non_zero_s17;
logic                                   l6_addr_conflict_s18_s17;
logic                                   l6_addr_conflict_s19_s17;
logic                                   l6_addr_conflict_s20_s17;
logic                                   l6_addr_conflict_s21_s17;
counter_t                               reg_l5_counter_s17;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l5_bitmap_idx_s17;
logic                                   reg_l6_addr_conflict_s18_s17;
logic                                   reg_l6_addr_conflict_s19_s17;
logic                                   reg_l6_addr_conflict_s20_s17;
logic                                   reg_l6_addr_conflict_s21_s17;

// Stage 18 metadata
bitmap_t                                l6_bitmap_s18;
logic                                   reg_l6_addr_conflict_s19_s18;
logic                                   reg_l6_addr_conflict_s20_s18;
logic                                   reg_l6_addr_conflict_s21_s18;
logic                                   reg_l6_addr_conflict_s22_s18;

// Stage 19 metadata
read_carry_direction_t                  rcd_s19;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       l6_bitmap_idx_s19;
logic                                   l6_bitmap_empty_s19;
bitmap_t                                l6_bitmap_postop_s19;
bitmap_t                                l6_bitmap_idx_onehot_s19;
logic                                   l6_bitmap_changes_s21_s19;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l6_bitmap_idx_s19;
logic                                   reg_l6_bitmap_empty_s19;
bitmap_t                                reg_l6_bitmap_postop_s19;
bitmap_t                                reg_l6_bitmap_idx_onehot_s19;
logic                                   reg_l6_counter_rdvalid_r1_s19;
logic                                   reg_l6_addr_conflict_s20_s19;
logic                                   reg_l6_addr_conflict_s21_s19;
logic                                   reg_l6_addr_conflict_s22_s19;
logic                                   reg_l6_addr_conflict_s23_s19;

// Stage 20 metadata
read_carry_direction_t                  rcd_s20;
counter_t                               l6_counter_s20;
counter_t                               l6_counter_q_s20;
counter_t                               reg_l6_counter_s20;
counter_t                               reg_l6_counter_rc_s20;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l6_bitmap_idx_s20;
bitmap_t                                reg_l6_bitmap_postop_s20;
bitmap_t                                reg_l6_bitmap_idx_onehot_s20;
logic                                   reg_l6_addr_conflict_s21_s20;
logic                                   reg_l6_addr_conflict_s22_s20;
logic                                   reg_l6_addr_conflict_s23_s20;
logic                                   reg_l6_addr_conflict_s24_s20;

// Stage 21 metadata
heap_priority_t                         priority_s21;
counter_t                               l6_counter_s21;
logic                                   l6_counter_non_zero_s21;
logic                                   pb_addr_conflict_s22_s21;
logic                                   pb_addr_conflict_s23_s21;
logic                                   pb_addr_conflict_s24_s21;
logic                                   pb_addr_conflict_s25_s21;
counter_t                               reg_l6_counter_s21;
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       reg_l6_bitmap_idx_s21;
counter_t                               reg_old_l6_counter_s21;
logic                                   reg_l6_counter_non_zero_s21;
logic                                   reg_pb_addr_conflict_s22_s21;
logic                                   reg_pb_addr_conflict_s23_s21;
logic                                   reg_pb_addr_conflict_s24_s21;
logic                                   reg_pb_addr_conflict_s25_s21;

// Stage 22 metadata
op_color_t                              op_color_s22;
logic                                   reg_pb_update_s22;
logic                                   reg_pb_data_conflict_s22;
logic                                   reg_pb_state_changes_s22;
logic                                   reg_pb_tail_pp_changes_s22;
logic                                   reg_pb_addr_conflict_s23_s22;
logic                                   reg_pb_addr_conflict_s24_s22;

// Stage 23 metadata
logic                                   pp_changes_s24_s23;
logic                                   pp_changes_s25_s23;
list_t                                  reg_pb_q_s23;
heap_entry_ptr_t                        reg_pp_data_s23;
logic                                   reg_pp_data_valid_s23;
logic                                   reg_pb_data_conflict_s23;
logic                                   reg_pb_state_changes_s23;
logic                                   reg_pb_tail_pp_changes_s23;
logic                                   reg_pb_addr_conflict_s24_s23;
logic                                   reg_pb_addr_conflict_s25_s23;

// Stage 24 metadata
heap_entry_data_t                       he_q_s24;
heap_entry_ptr_t                        np_q_s24;
heap_entry_ptr_t                        pp_q_s24;
heap_entry_data_t                       reg_he_q_s24;
heap_entry_ptr_t                        reg_np_q_s24;
heap_entry_ptr_t                        reg_pp_q_s24;
list_t                                  reg_pb_q_s24;
list_t                                  reg_pb_new_s24;
logic                                   reg_pb_data_conflict_s24;
logic                                   reg_pb_state_changes_s24;
logic                                   reg_pb_tail_pp_changes_s24;

// Stage 25 metadata
heap_entry_data_t                       he_data_s25;
heap_entry_data_t                       reg_he_data_s25;
heap_entry_ptr_t                        reg_np_data_s25;
heap_entry_ptr_t                        reg_pp_data_s25;
list_t                                  reg_pb_data_s25;

// Stage 26 metadata
list_t                                  reg_pb_data_s26;

// Init signals
fsm_state_t                             state = FSM_STATE_IDLE;
logic                                   counter_l2_init_done_r;
logic                                   counter_l3_init_done_r;
logic                                   counter_l4_init_done_r;
logic                                   counter_l5_init_done_r;
logic                                   counter_l6_init_done_r;
logic                                   bm_l3_init_done_r;
logic                                   bm_l4_init_done_r;
logic                                   bm_l5_init_done_r;
logic                                   bm_l6_init_done_r;
logic                                   fl_init_done_r;
logic                                   counter_l2_init_done;
logic                                   counter_l3_init_done;
logic                                   counter_l4_init_done;
logic                                   counter_l5_init_done;
logic                                   counter_l6_init_done;
logic                                   bm_l3_init_done;
logic                                   bm_l4_init_done;
logic                                   bm_l5_init_done;
logic                                   bm_l6_init_done;
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

logic [HEAP_LOG_BITMAP_WIDTH-1:0]       ffs_l4_inst_msb[2:0];
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       ffs_l4_inst_lsb[2:0];
logic                                   ffs_l4_inst_zero[2:0];
bitmap_t                                ffs_l4_inst_msb_onehot[2:0];
bitmap_t                                ffs_l4_inst_lsb_onehot[2:0];

logic [HEAP_LOG_BITMAP_WIDTH-1:0]       ffs_l5_inst_msb[2:0];
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       ffs_l5_inst_lsb[2:0];
logic                                   ffs_l5_inst_zero[2:0];
bitmap_t                                ffs_l5_inst_msb_onehot[2:0];
bitmap_t                                ffs_l5_inst_lsb_onehot[2:0];

logic [HEAP_LOG_BITMAP_WIDTH-1:0]       ffs_l6_inst_msb[2:0];
logic [HEAP_LOG_BITMAP_WIDTH-1:0]       ffs_l6_inst_lsb[2:0];
logic                                   ffs_l6_inst_zero[2:0];
bitmap_t                                ffs_l6_inst_msb_onehot[2:0];
bitmap_t                                ffs_l6_inst_lsb_onehot[2:0];

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
    bm_l4_init_done = bm_l4_init_done_r;
    bm_l5_init_done = bm_l5_init_done_r;
    bm_l6_init_done = bm_l6_init_done_r;
    counter_l2_init_done = counter_l2_init_done_r;
    counter_l3_init_done = counter_l3_init_done_r;
    counter_l4_init_done = counter_l4_init_done_r;
    counter_l5_init_done = counter_l5_init_done_r;
    counter_l6_init_done = counter_l6_init_done_r;

    fl_wrreq = 0;
    bm_l3_wren = 0;
    bm_l4_wren = 0;
    bm_l5_wren = 0;
    bm_l6_wren = 0;
    counter_l2_wren = 0;
    counter_l3_wren = 0;
    counter_l4_wren = 0;
    counter_l5_wren = 0;
    counter_l6_wren = 0;

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
        // L4 bitmaps
        bm_l4_data = 0;
        bm_l4_wraddress = bm_l4_wraddress_counter_r;
        if (!bm_l4_init_done_r) begin
            bm_l4_wren = 1;
            bm_l4_init_done = (bm_l4_wraddress_counter_r ==
                               (NUM_BITMAPS_L4 - 1));
        end
        // L4 counters
        counter_l4_data = 0;
        counter_l4_wraddress = counter_l4_wraddress_counter_r;
        if (!counter_l4_init_done_r) begin
            counter_l4_wren = 1;
            counter_l4_init_done = (counter_l4_wraddress_counter_r ==
                                    (NUM_COUNTERS_L4 - 1));
        end
        // L5 bitmaps
        bm_l5_data = 0;
        bm_l5_wraddress = bm_l5_wraddress_counter_r;
        if (!bm_l5_init_done_r) begin
            bm_l5_wren = 1;
            bm_l5_init_done = (bm_l5_wraddress_counter_r ==
                               (NUM_BITMAPS_L5 - 1));
        end
        // L5 counters
        counter_l5_data = 0;
        counter_l5_wraddress = counter_l5_wraddress_counter_r;
        if (!counter_l5_init_done_r) begin
            counter_l5_wren = 1;
            counter_l5_init_done = (counter_l5_wraddress_counter_r ==
                                    (NUM_COUNTERS_L5 - 1));
        end
        // L6 bitmaps
        bm_l6_data = 0;
        bm_l6_wraddress = bm_l6_wraddress_counter_r;
        if (!bm_l6_init_done_r) begin
            bm_l6_wren = 1;
            bm_l6_init_done = (bm_l6_wraddress_counter_r ==
                               (NUM_BITMAPS_L6 - 1));
        end
        // L6 counters
        counter_l6_data = 0;
        counter_l6_wraddress = counter_l6_wraddress_counter_r;
        if (!counter_l6_init_done_r) begin
            counter_l6_wren = 1;
            counter_l6_init_done = (counter_l6_wraddress_counter_r ==
                                    (NUM_COUNTERS_L6 - 1));
        end
        // Finished initializing the queue (including priority buckets,
        // free list, and the LX bitmaps). Proceed to the ready state.
        if (fl_init_done_r & counter_l2_init_done_r & bm_l3_init_done_r &
            counter_l3_init_done_r & bm_l4_init_done_r & counter_l4_init_done_r &
            bm_l5_init_done_r & counter_l5_init_done_r & bm_l6_init_done_r &
            counter_l6_init_done_r) begin
            state_next = FSM_STATE_READY;
        end
    end
    else begin
        /**
         * Stage 25: Perform writes: update the priority bucket,
         * the free list, heap entries, next and prev pointers.
         */
        fl_data = (
            (reg_op_color_s[24] == OP_COLOR_BLUE) ?
            reg_pb_q_s24.head : reg_pb_q_s24.tail);

        // Perform deque
        if (!reg_is_enque_s[24]) begin
            // Update the free list
            fl_wrreq = reg_valid_s[24];
        end
        /**
         * Stage 21: Write-back the L6 counter and bitmap,
         * and read the corresponding PB (head and tail).
         */
        // Write L6 counter
        counter_l6_wren = reg_valid_s[20];
        counter_l6_data = l6_counter_s21;
        counter_l6_wraddress = {reg_l6_addr_s[20],
                                reg_l6_bitmap_idx_s20};
        // Write L6 bitmap
        bm_l6_wren = reg_valid_s[20];
        bm_l6_wraddress = reg_l6_addr_s[20];
        if (reg_is_enque_s[20]) begin
            bm_l6_data = (reg_l6_bitmap_s[20] |
                          reg_l6_bitmap_idx_onehot_s20);
        end
        else begin
            bm_l6_data = (
                l6_counter_non_zero_s21 ? reg_l6_bitmap_s[20] :
                (reg_l6_bitmap_s[20] & ~reg_l6_bitmap_idx_onehot_s20));
        end
        /**
         * Stage 17: Write-back the L5 counter and bitmap,
         * and read the corresponding L6 bitmap.
         */
        // Write L5 counter
        counter_l5_wren = reg_valid_s[16];
        counter_l5_data = l5_counter_s17;
        counter_l5_wraddress = {reg_l5_addr_s[16],
                                reg_l5_bitmap_idx_s16};
        // Write L5 bitmap
        bm_l5_wren = reg_valid_s[16];
        bm_l5_wraddress = reg_l5_addr_s[16];
        if (reg_is_enque_s[16]) begin
            bm_l5_data = (reg_l5_bitmap_s[16] |
                          reg_l5_bitmap_idx_onehot_s16);
        end
        else begin
            bm_l5_data = (
                l5_counter_non_zero_s17 ? reg_l5_bitmap_s[16] :
                (reg_l5_bitmap_s[16] & ~reg_l5_bitmap_idx_onehot_s16));
        end
        /**
         * Stage 13: Write-back the L4 counter and bitmap,
         * and read the corresponding L5 bitmap.
         */
        // Write L4 counter
        counter_l4_wren = reg_valid_s[12];
        counter_l4_data = l4_counter_s13;
        counter_l4_wraddress = {reg_l4_addr_s[12],
                                reg_l4_bitmap_idx_s12};
        // Write L4 bitmap
        bm_l4_wren = reg_valid_s[12];
        bm_l4_wraddress = reg_l4_addr_s[12];
        if (reg_is_enque_s[12]) begin
            bm_l4_data = (reg_l4_bitmap_s[12] |
                          reg_l4_bitmap_idx_onehot_s12);
        end
        else begin
            bm_l4_data = (
                l4_counter_non_zero_s13 ? reg_l4_bitmap_s[12] :
                (reg_l4_bitmap_s[12] & ~reg_l4_bitmap_idx_onehot_s12));
        end
        /**
         * Stage 9: Write-back the L3 counter and bitmap,
         * and read the corresponding L4 bitmap.
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
    l4_bitmap_s10 = bm_l4_q;
    rcd_s11 = READ_CARRY_DOWN;
    rcd_s12 = READ_CARRY_DOWN;
    l4_counter_s12 = reg_l4_counter_s12;
    l4_counter_q_s12 = counter_l4_q;
    l5_bitmap_s14 = bm_l5_q;
    rcd_s15 = READ_CARRY_DOWN;
    rcd_s16 = READ_CARRY_DOWN;
    l5_counter_s16 = reg_l5_counter_s16;
    l5_counter_q_s16 = counter_l5_q;
    l6_bitmap_s18 = bm_l6_q;
    rcd_s19 = READ_CARRY_DOWN;
    rcd_s20 = READ_CARRY_DOWN;
    l6_counter_s20 = reg_l6_counter_s20;
    l6_counter_q_s20 = counter_l6_q;
    priority_s21 = {reg_l6_addr_s[20], reg_l6_bitmap_idx_s20};
    op_color_s22 = reg_is_enque_s[21] ? OP_COLOR_BLUE : OP_COLOR_RED;
    he_q_s24 = he_q;
    np_q_s24 = np_q;
    pp_q_s24 = pp_q;

    int_pb_q = pb_q_r;

    fl_rdreq = 0;

    he_rden = 0;
    he_wren = 0;
    he_data = reg_he_data_s[24];
    he_wraddress = fl_q_r[22];

    np_rden = 0;
    np_wren = 0;
    np_data = reg_pb_q_s24.head;
    np_wraddress = fl_q_r[22];

    pp_rden = 0;
    pp_wren = 0;
    pp_data = fl_q_r[22];
    pp_wraddress = reg_pb_q_s24.head;

    pb_rdwr_conflict = 0;
    pb_rdaddress = priority_s21;
    int_pb_data = reg_pb_new_s24;
    pb_wraddress = reg_priority_s[24];

    bm_l3_rden = 0;
    bm_l4_rden = 0;
    bm_l5_rden = 0;
    bm_l6_rden = 0;
    counter_l2_rden = 0;
    counter_l3_rden = 0;
    counter_l4_rden = 0;
    counter_l5_rden = 0;
    counter_l6_rden = 0;

    /**
     * Stage 25: Perform writes: update the priority bucket,
     * the free list, heap entries, next and prev pointers.
     */
    pb_wren = reg_valid_s[24];

    // Perform enque
    if (reg_is_enque_s[24]) begin
        if (reg_valid_s[24]) begin
            he_wren = 1; // Update the heap entry
            np_wren = 1; // Update the next pointer

            // Update the entry's previous pointer. The
            // pointer address is only valid if the PB
            // was not previously empty, so write must
            // be predicated on no change of state.
            if (!reg_pb_state_changes_s24) begin
                pp_wren = 1;
            end
        end

        // Update the data
        he_data_s25 = reg_he_data_s[24];
    end
    // Perform deque
    else begin
        if (reg_op_color_s[24] == OP_COLOR_BLUE) begin
            // BLUE-colored dequeue (from HEAD)
            int_pb_data.head = reg_np_q_s24;
        end
        else begin
            // RED-colored dequeue (from TAIL)
            int_pb_data.tail = reg_pp_q_s24;
        end

        // Update the data
        he_data_s25 = (
            reg_pb_data_conflict_s24 ?
            reg_he_data_s[25] : reg_he_q_s24);
    end
    /**
     * Stage 24: Read delay for HE and pointers.
     */
    // This HE was updated on the last cycle, so the output is stale
    if (he_wren_r && (he_wraddress_r == he_rdaddress_r)) begin
        he_q_s24 = reg_he_data_s25;
    end
    // Fallthrough: default to he_q

    // This NP was updated on the last cycle, so the output is stale
    if (np_wren_r && (np_wraddress_r == np_rdaddress_r)) begin
        np_q_s24 = reg_np_data_s25;
    end
    // Fallthrough: default to np_q

    // This PP was updated in the last 2 cycles
    if (reg_pp_data_valid_s23) begin
        pp_q_s24 = reg_pp_data_s23;
    end
    // Fallthrough: default to pp_q

    /**
     * Stage 23: Read the heap entry and prev/next pointer
     * corresponding to the priority bucket to deque.
     */
    // The PB is being updated on this cycle
    if (reg_pb_addr_conflict_s24_s22) begin
        int_pb_q = int_pb_data;
    end
    // The PB was updated last cycle, so output is stale
    else if (reg_pb_update_s22) begin
        int_pb_q = reg_pb_data_s25;
    end
    // The PB was updated 2 cycles ago (and thus never read)
    else if (reg_pb_rdwr_conflict_r2) begin
        int_pb_q = reg_pb_data_s26;
    end
    // Fallthrough: default to pb_q_r

    // Read next and prev pointers
    np_rdaddress = int_pb_q.head;
    pp_rdaddress = int_pb_q.tail;

    // Compute tail PP updates
    pp_changes_s24_s23 = (reg_pb_tail_pp_changes_s23 &&
                          reg_pb_addr_conflict_s23_s22);

    pp_changes_s25_s23 = (reg_pb_tail_pp_changes_s24 &&
                          reg_pb_addr_conflict_s24_s22);

    // Read HE data
    he_rdaddress = (
        (reg_op_color_s[22] == OP_COLOR_BLUE) ?
        int_pb_q.head : int_pb_q.tail);

    if (reg_valid_s[22]) begin
        if (!reg_is_enque_s[22]) begin
            he_rden = 1; // Dequeing, read HE and PP/NP
            if (reg_op_color_s[22] == OP_COLOR_BLUE) begin
                np_rden = 1; // BLUE-colored dequeue (from HEAD)
            end
            else begin
                pp_rden = 1; // RED-colored dequeue (from TAIL)
            end
        end
    end
    /**
     * Stage 22: Compute op color, read delay for PB.
     */
    if (!reg_is_enque_s[21]) begin
        // Dequeing, recolor this op if required
        if (reg_pb_addr_conflict_s22_s21) begin
            op_color_s22 = (
                (reg_op_color_s[22] == OP_COLOR_BLUE)
                    ? OP_COLOR_RED : OP_COLOR_BLUE);
        end
    end
    /**
     * Stage 21: Write-back the L6 counter and bitmap,
     * and read the corresponding PB (head and tail).
     */
    l6_counter_s21[WATERLEVEL_IDX-1:0] = (
        reg_is_enque_s[20] ? (reg_l6_counter_rc_s20[WATERLEVEL_IDX-1:0] + 1) :
                             (reg_l6_counter_rc_s20[WATERLEVEL_IDX-1:0] - 1));

    l6_counter_s21[WATERLEVEL_IDX] = (reg_is_enque_s[20] ?
        (reg_l6_counter_rc_s20[WATERLEVEL_IDX] | reg_l6_counter_rc_s20[0]) :
        ((|reg_l6_counter_rc_s20[WATERLEVEL_IDX-1:2]) | (&reg_l6_counter_rc_s20[1:0])));

    l6_counter_non_zero_s21 = (reg_is_enque_s[20] |
                               reg_l6_counter_rc_s20[WATERLEVEL_IDX]);
    // Read PB contents
    pb_rden = reg_valid_s[20];

    // Compute conflicts
    pb_addr_conflict_s22_s21 = (
        reg_l6_addr_conflict_s21_s20
            && (reg_l6_bitmap_idx_s20 ==
                reg_priority_s[21][HEAP_LOG_BITMAP_WIDTH-1:0]));

    pb_addr_conflict_s23_s21 = (
        reg_l6_addr_conflict_s22_s20
            && (reg_l6_bitmap_idx_s20 ==
                reg_priority_s[22][HEAP_LOG_BITMAP_WIDTH-1:0]));

    pb_addr_conflict_s24_s21 = (
        reg_l6_addr_conflict_s23_s20
            && (reg_l6_bitmap_idx_s20 ==
                reg_priority_s[23][HEAP_LOG_BITMAP_WIDTH-1:0]));

    pb_addr_conflict_s25_s21 = (
        reg_l6_addr_conflict_s24_s20
            && (reg_l6_bitmap_idx_s20 ==
                reg_priority_s[24][HEAP_LOG_BITMAP_WIDTH-1:0]));

    // Disable conflicting reads during writes
    if (pb_addr_conflict_s25_s21) begin
        pb_rdwr_conflict = 1;
        pb_rden = 0;
    end
    /**
     * Stage 20: NOOP, read delay for L6 counter.
     */
    // Compute the read carry direction. If the
    // active op in Stage 21 is of the same type
    // or the bitmap is empty, carry right.
    if (!reg_is_enque_s[19] &&
        l6_counter_non_zero_s21 &&
        reg_l6_addr_conflict_s20_s19 &&
        (reg_l6_bitmap_empty_s19 || (reg_op_type_s[19] ==
                                     reg_op_type_s[20]))) begin
        rcd_s20 = READ_CARRY_RIGHT;
    end
    // Fallthrough: default to carry down

    // Counter is updating this cycle, so output is stale
    if ((reg_l6_bitmap_idx_s19 == reg_l6_bitmap_idx_s20)
        && reg_l6_addr_conflict_s20_s19) begin
        l6_counter_q_s20 = l6_counter_s21;
        l6_counter_s20 = l6_counter_s21;
    end
    // Counter was updated last cycle (there was R/W conflict)
    else if ((reg_l6_bitmap_idx_s19 == reg_l6_bitmap_idx_s21)
             && reg_l6_addr_conflict_s21_s19) begin
        l6_counter_q_s20 = reg_l6_counter_s21;
        l6_counter_s20 = reg_l6_counter_s21;
    end
    // Fallthrough, defaults to:
    // counter_l6_q for l6_counter_q_s20
    // reg_l6_counter_s20 for l6_counter_s20

    /**
     * Stage 19: Compute the L6 bitmap index and postop
     * bitmap, and read the corresponding L6 counter.
     */
    // L6 bitmap changes?
    l6_bitmap_changes_s21_s19 = (
        reg_l6_addr_conflict_s20_s18 &&
        (reg_is_enque_s[20] || !l6_counter_non_zero_s21));

    // Compute L6 bitmap idx and postop
    case (reg_op_type_s[18])
    HEAP_OP_DEQUE_MAX: begin
        l6_bitmap_idx_s19 = (
            reg_l6_addr_conflict_s19_s18 ? ffs_l6_inst_msb[1] :
               l6_bitmap_changes_s21_s19 ? ffs_l6_inst_msb[2] :
                                           ffs_l6_inst_msb[0]);

        l6_bitmap_empty_s19 = (
            reg_l6_addr_conflict_s19_s18 ? ffs_l6_inst_zero[1] :
               l6_bitmap_changes_s21_s19 ? ffs_l6_inst_zero[2] :
                                           ffs_l6_inst_zero[0]);

        l6_bitmap_idx_onehot_s19 = (
            reg_l6_addr_conflict_s19_s18 ? ffs_l6_inst_msb_onehot[1] :
               l6_bitmap_changes_s21_s19 ? ffs_l6_inst_msb_onehot[2] :
                                           ffs_l6_inst_msb_onehot[0]);

        l6_bitmap_postop_s19 = (
            reg_l6_addr_conflict_s19_s18 ? (l6_bitmap_idx_onehot_s19 ^
                                            reg_l6_bitmap_postop_s19) :
               l6_bitmap_changes_s21_s19 ? (l6_bitmap_idx_onehot_s19 ^
                                            reg_l6_bitmap_postop_s20) :
                                           (l6_bitmap_idx_onehot_s19 ^
                                            reg_l6_bitmap_s[18]));
    end
    HEAP_OP_DEQUE_MIN: begin
        l6_bitmap_idx_s19 = (
            reg_l6_addr_conflict_s19_s18 ? ffs_l6_inst_lsb[1] :
               l6_bitmap_changes_s21_s19 ? ffs_l6_inst_lsb[2] :
                                           ffs_l6_inst_lsb[0]);

        l6_bitmap_empty_s19 = (
            reg_l6_addr_conflict_s19_s18 ? ffs_l6_inst_zero[1] :
               l6_bitmap_changes_s21_s19 ? ffs_l6_inst_zero[2] :
                                           ffs_l6_inst_zero[0]);

        l6_bitmap_idx_onehot_s19 = (
            reg_l6_addr_conflict_s19_s18 ? ffs_l6_inst_lsb_onehot[1] :
               l6_bitmap_changes_s21_s19 ? ffs_l6_inst_lsb_onehot[2] :
                                           ffs_l6_inst_lsb_onehot[0]);

        l6_bitmap_postop_s19 = (
            reg_l6_addr_conflict_s19_s18 ? (l6_bitmap_idx_onehot_s19 ^
                                            reg_l6_bitmap_postop_s19) :
               l6_bitmap_changes_s21_s19 ? (l6_bitmap_idx_onehot_s19 ^
                                            reg_l6_bitmap_postop_s20) :
                                           (l6_bitmap_idx_onehot_s19 ^
                                            reg_l6_bitmap_s[18]));
    end
    // HEAP_OP_ENQUE
    default: begin
        l6_bitmap_empty_s19 = 0;
        l6_bitmap_idx_s19 = (reg_priority_s[18][(
                (1 * HEAP_LOG_BITMAP_WIDTH) - 1)
                : 0]);

        l6_bitmap_idx_onehot_s19 = (1 << l6_bitmap_idx_s19);
        l6_bitmap_postop_s19 = (
            reg_l6_addr_conflict_s19_s18 ? (l6_bitmap_idx_onehot_s19 |
                                            reg_l6_bitmap_postop_s19) :
               l6_bitmap_changes_s21_s19 ? (l6_bitmap_idx_onehot_s19 |
                                            reg_l6_bitmap_postop_s20) :
                                           (l6_bitmap_idx_onehot_s19 |
                                            reg_l6_bitmap_s[18]));
    end
    endcase
    // Compute the read carry direction. If the active
    // op in Stage 21 is of the same type, carry up.
    if (!reg_is_enque_s[18] &&
        l6_counter_non_zero_s21 &&
        reg_l6_addr_conflict_s20_s18 &&
        (reg_op_type_s[18] == reg_op_type_s[20])) begin
        rcd_s19 = READ_CARRY_UP;

        // Special case: The active op in Stage 20 is also
        // of the same type, which means that it's bound
        // to carry right; here, we do the same.
        if ((reg_op_type_s[18] == reg_op_type_s[19]) &&
            reg_l6_addr_conflict_s19_s18) begin
            rcd_s19 = READ_CARRY_RIGHT;
        end
    end
    // Fallthrough: default to carry down

    // Read the L6 counter
    counter_l6_rden = reg_valid_s[18];
    counter_l6_rdaddress = {reg_l6_addr_s[18],
                            l6_bitmap_idx_s19};
    /**
     * Stage 18: NOOP, read delay for L6 bitmap.
     */
    // L6 bitmap updated this cycle, so output is stale
    if (reg_l6_addr_conflict_s20_s17) begin
        l6_bitmap_s18 = bm_l6_data;
    end
    // L6 bitmap was updated last cycle (R/W conflict)
    else if (reg_l6_addr_conflict_s21_s17) begin
        l6_bitmap_s18 = bm_l6_data_r;
    end
    // Fallthrough: default to bm_l6_q

    /**
     * Stage 17: Write-back the L5 counter and bitmap,
     * and read the corresponding L6 bitmap.
     */
    l5_counter_s17[WATERLEVEL_IDX-1:0] = (
        reg_is_enque_s[16] ? (reg_l5_counter_rc_s16[WATERLEVEL_IDX-1:0] + 1) :
                             (reg_l5_counter_rc_s16[WATERLEVEL_IDX-1:0] - 1));

    l5_counter_s17[WATERLEVEL_IDX] = (reg_is_enque_s[16] ?
        (reg_l5_counter_rc_s16[WATERLEVEL_IDX] | reg_l5_counter_rc_s16[0]) :
        ((|reg_l5_counter_rc_s16[WATERLEVEL_IDX-1:2]) | (&reg_l5_counter_rc_s16[1:0])));

    l5_counter_non_zero_s17 = (reg_is_enque_s[16] |
                               reg_l5_counter_rc_s16[WATERLEVEL_IDX]);
    // Read L6 bitmap
    bm_l6_rden = reg_valid_s[16];
    bm_l6_rdaddress = {reg_l5_addr_s[16],
                       reg_l5_bitmap_idx_s16};

    // Compute conflicts
    l6_addr_conflict_s18_s17 = (
        reg_l5_addr_conflict_s17_s16
            && (reg_l5_bitmap_idx_s16 ==
                reg_l6_addr_s[17][HEAP_LOG_BITMAP_WIDTH-1:0]));

    l6_addr_conflict_s19_s17 = (
        reg_l5_addr_conflict_s18_s16
            && (reg_l5_bitmap_idx_s16 ==
                reg_l6_addr_s[18][HEAP_LOG_BITMAP_WIDTH-1:0]));

    l6_addr_conflict_s20_s17 = (
        reg_l5_addr_conflict_s19_s16
            && (reg_l5_bitmap_idx_s16 ==
                reg_l6_addr_s[19][HEAP_LOG_BITMAP_WIDTH-1:0]));

    l6_addr_conflict_s21_s17 = (
        reg_l5_addr_conflict_s20_s16
            && (reg_l5_bitmap_idx_s16 ==
                reg_l6_addr_s[20][HEAP_LOG_BITMAP_WIDTH-1:0]));

    /**
     * Stage 16: NOOP, read delay for L5 counter.
     */
    // Compute the read carry direction. If the
    // active op in Stage 17 is of the same type
    // or the bitmap is empty, carry right.
    if (!reg_is_enque_s[15] &&
        l5_counter_non_zero_s17 &&
        reg_l5_addr_conflict_s16_s15 &&
        (reg_l5_bitmap_empty_s15 || (reg_op_type_s[15] ==
                                     reg_op_type_s[16]))) begin
        rcd_s16 = READ_CARRY_RIGHT;
    end
    // Fallthrough: default to carry down

    // Counter is updating this cycle, so output is stale
    if ((reg_l5_bitmap_idx_s15 == reg_l5_bitmap_idx_s16)
        && reg_l5_addr_conflict_s16_s15) begin
        l5_counter_q_s16 = l5_counter_s17;
        l5_counter_s16 = l5_counter_s17;
    end
    // Counter was updated last cycle (there was R/W conflict)
    else if ((reg_l5_bitmap_idx_s15 == reg_l5_bitmap_idx_s17)
             && reg_l5_addr_conflict_s17_s15) begin
        l5_counter_q_s16 = reg_l5_counter_s17;
        l5_counter_s16 = reg_l5_counter_s17;
    end
    // Fallthrough, defaults to:
    // counter_l5_q for l5_counter_q_s16
    // reg_l5_counter_s16 for l5_counter_s16

    /**
     * Stage 15: Compute the L5 bitmap index and postop
     * bitmap, and read the corresponding L5 counter.
     */
    // L5 bitmap changes?
    l5_bitmap_changes_s17_s15 = (
        reg_l5_addr_conflict_s16_s14 &&
        (reg_is_enque_s[16] || !l5_counter_non_zero_s17));

    // Compute L5 bitmap idx and postop
    case (reg_op_type_s[14])
    HEAP_OP_DEQUE_MAX: begin
        l5_bitmap_idx_s15 = (
            reg_l5_addr_conflict_s15_s14 ? ffs_l5_inst_msb[1] :
               l5_bitmap_changes_s17_s15 ? ffs_l5_inst_msb[2] :
                                           ffs_l5_inst_msb[0]);

        l5_bitmap_empty_s15 = (
            reg_l5_addr_conflict_s15_s14 ? ffs_l5_inst_zero[1] :
               l5_bitmap_changes_s17_s15 ? ffs_l5_inst_zero[2] :
                                           ffs_l5_inst_zero[0]);

        l5_bitmap_idx_onehot_s15 = (
            reg_l5_addr_conflict_s15_s14 ? ffs_l5_inst_msb_onehot[1] :
               l5_bitmap_changes_s17_s15 ? ffs_l5_inst_msb_onehot[2] :
                                           ffs_l5_inst_msb_onehot[0]);

        l5_bitmap_postop_s15 = (
            reg_l5_addr_conflict_s15_s14 ? (l5_bitmap_idx_onehot_s15 ^
                                            reg_l5_bitmap_postop_s15) :
               l5_bitmap_changes_s17_s15 ? (l5_bitmap_idx_onehot_s15 ^
                                            reg_l5_bitmap_postop_s16) :
                                           (l5_bitmap_idx_onehot_s15 ^
                                            reg_l5_bitmap_s[14]));
    end
    HEAP_OP_DEQUE_MIN: begin
        l5_bitmap_idx_s15 = (
            reg_l5_addr_conflict_s15_s14 ? ffs_l5_inst_lsb[1] :
               l5_bitmap_changes_s17_s15 ? ffs_l5_inst_lsb[2] :
                                           ffs_l5_inst_lsb[0]);

        l5_bitmap_empty_s15 = (
            reg_l5_addr_conflict_s15_s14 ? ffs_l5_inst_zero[1] :
               l5_bitmap_changes_s17_s15 ? ffs_l5_inst_zero[2] :
                                           ffs_l5_inst_zero[0]);

        l5_bitmap_idx_onehot_s15 = (
            reg_l5_addr_conflict_s15_s14 ? ffs_l5_inst_lsb_onehot[1] :
               l5_bitmap_changes_s17_s15 ? ffs_l5_inst_lsb_onehot[2] :
                                           ffs_l5_inst_lsb_onehot[0]);

        l5_bitmap_postop_s15 = (
            reg_l5_addr_conflict_s15_s14 ? (l5_bitmap_idx_onehot_s15 ^
                                            reg_l5_bitmap_postop_s15) :
               l5_bitmap_changes_s17_s15 ? (l5_bitmap_idx_onehot_s15 ^
                                            reg_l5_bitmap_postop_s16) :
                                           (l5_bitmap_idx_onehot_s15 ^
                                            reg_l5_bitmap_s[14]));
    end
    // HEAP_OP_ENQUE
    default: begin
        l5_bitmap_empty_s15 = 0;
        l5_bitmap_idx_s15 = (reg_priority_s[14][(
                (2 * HEAP_LOG_BITMAP_WIDTH) - 1)
                : (1 * HEAP_LOG_BITMAP_WIDTH)]);

        l5_bitmap_idx_onehot_s15 = (1 << l5_bitmap_idx_s15);
        l5_bitmap_postop_s15 = (
            reg_l5_addr_conflict_s15_s14 ? (l5_bitmap_idx_onehot_s15 |
                                            reg_l5_bitmap_postop_s15) :
               l5_bitmap_changes_s17_s15 ? (l5_bitmap_idx_onehot_s15 |
                                            reg_l5_bitmap_postop_s16) :
                                           (l5_bitmap_idx_onehot_s15 |
                                            reg_l5_bitmap_s[14]));
    end
    endcase
    // Compute the read carry direction. If the active
    // op in Stage 17 is of the same type, carry up.
    if (!reg_is_enque_s[14] &&
        l5_counter_non_zero_s17 &&
        reg_l5_addr_conflict_s16_s14 &&
        (reg_op_type_s[14] == reg_op_type_s[16])) begin
        rcd_s15 = READ_CARRY_UP;

        // Special case: The active op in Stage 16 is also
        // of the same type, which means that it's bound
        // to carry right; here, we do the same.
        if ((reg_op_type_s[14] == reg_op_type_s[15]) &&
            reg_l5_addr_conflict_s15_s14) begin
            rcd_s15 = READ_CARRY_RIGHT;
        end
    end
    // Fallthrough: default to carry down

    // Read the L5 counter
    counter_l5_rden = reg_valid_s[14];
    counter_l5_rdaddress = {reg_l5_addr_s[14],
                            l5_bitmap_idx_s15};
    /**
     * Stage 14: NOOP, read delay for L5 bitmap.
     */
    // L5 bitmap updated this cycle, so output is stale
    if (reg_l5_addr_conflict_s16_s13) begin
        l5_bitmap_s14 = bm_l5_data;
    end
    // L5 bitmap was updated last cycle (R/W conflict)
    else if (reg_l5_addr_conflict_s17_s13) begin
        l5_bitmap_s14 = bm_l5_data_r;
    end
    // Fallthrough: default to bm_l5_q

    /**
     * Stage 13: Write-back the L4 counter and bitmap,
     * and read the corresponding L5 bitmap.
     */
    l4_counter_s13[WATERLEVEL_IDX-1:0] = (
        reg_is_enque_s[12] ? (reg_l4_counter_rc_s12[WATERLEVEL_IDX-1:0] + 1) :
                             (reg_l4_counter_rc_s12[WATERLEVEL_IDX-1:0] - 1));

    l4_counter_s13[WATERLEVEL_IDX] = (reg_is_enque_s[12] ?
        (reg_l4_counter_rc_s12[WATERLEVEL_IDX] | reg_l4_counter_rc_s12[0]) :
        ((|reg_l4_counter_rc_s12[WATERLEVEL_IDX-1:2]) | (&reg_l4_counter_rc_s12[1:0])));

    l4_counter_non_zero_s13 = (reg_is_enque_s[12] |
                               reg_l4_counter_rc_s12[WATERLEVEL_IDX]);
    // Read L5 bitmap
    bm_l5_rden = reg_valid_s[12];
    bm_l5_rdaddress = {reg_l4_addr_s[12],
                       reg_l4_bitmap_idx_s12};

    // Compute conflicts
    l5_addr_conflict_s14_s13 = (
        reg_l4_addr_conflict_s13_s12
            && (reg_l4_bitmap_idx_s12 ==
                reg_l5_addr_s[13][HEAP_LOG_BITMAP_WIDTH-1:0]));

    l5_addr_conflict_s15_s13 = (
        reg_l4_addr_conflict_s14_s12
            && (reg_l4_bitmap_idx_s12 ==
                reg_l5_addr_s[14][HEAP_LOG_BITMAP_WIDTH-1:0]));

    l5_addr_conflict_s16_s13 = (
        reg_l4_addr_conflict_s15_s12
            && (reg_l4_bitmap_idx_s12 ==
                reg_l5_addr_s[15][HEAP_LOG_BITMAP_WIDTH-1:0]));

    l5_addr_conflict_s17_s13 = (
        reg_l4_addr_conflict_s16_s12
            && (reg_l4_bitmap_idx_s12 ==
                reg_l5_addr_s[16][HEAP_LOG_BITMAP_WIDTH-1:0]));

    /**
     * Stage 12: NOOP, read delay for L4 counter.
     */
    // Compute the read carry direction. If the
    // active op in Stage 13 is of the same type
    // or the bitmap is empty, carry right.
    if (!reg_is_enque_s[11] &&
        l4_counter_non_zero_s13 &&
        reg_l4_addr_conflict_s12_s11 &&
        (reg_l4_bitmap_empty_s11 || (reg_op_type_s[11] ==
                                     reg_op_type_s[12]))) begin
        rcd_s12 = READ_CARRY_RIGHT;
    end
    // Fallthrough: default to carry down

    // Counter is updating this cycle, so output is stale
    if ((reg_l4_bitmap_idx_s11 == reg_l4_bitmap_idx_s12)
        && reg_l4_addr_conflict_s12_s11) begin
        l4_counter_q_s12 = l4_counter_s13;
        l4_counter_s12 = l4_counter_s13;
    end
    // Counter was updated last cycle (there was R/W conflict)
    else if ((reg_l4_bitmap_idx_s11 == reg_l4_bitmap_idx_s13)
             && reg_l4_addr_conflict_s13_s11) begin
        l4_counter_q_s12 = reg_l4_counter_s13;
        l4_counter_s12 = reg_l4_counter_s13;
    end
    // Fallthrough, defaults to:
    // counter_l4_q for l4_counter_q_s12
    // reg_l4_counter_s12 for l4_counter_s12

    /**
     * Stage 11: Compute the L4 bitmap index and postop
     * bitmap, and read the corresponding L4 counter.
     */
    // L4 bitmap changes?
    l4_bitmap_changes_s13_s11 = (
        reg_l4_addr_conflict_s12_s10 &&
        (reg_is_enque_s[12] || !l4_counter_non_zero_s13));

    // Compute L4 bitmap idx and postop
    case (reg_op_type_s[10])
    HEAP_OP_DEQUE_MAX: begin
        l4_bitmap_idx_s11 = (
            reg_l4_addr_conflict_s11_s10 ? ffs_l4_inst_msb[1] :
               l4_bitmap_changes_s13_s11 ? ffs_l4_inst_msb[2] :
                                           ffs_l4_inst_msb[0]);

        l4_bitmap_empty_s11 = (
            reg_l4_addr_conflict_s11_s10 ? ffs_l4_inst_zero[1] :
               l4_bitmap_changes_s13_s11 ? ffs_l4_inst_zero[2] :
                                           ffs_l4_inst_zero[0]);

        l4_bitmap_idx_onehot_s11 = (
            reg_l4_addr_conflict_s11_s10 ? ffs_l4_inst_msb_onehot[1] :
               l4_bitmap_changes_s13_s11 ? ffs_l4_inst_msb_onehot[2] :
                                           ffs_l4_inst_msb_onehot[0]);

        l4_bitmap_postop_s11 = (
            reg_l4_addr_conflict_s11_s10 ? (l4_bitmap_idx_onehot_s11 ^
                                            reg_l4_bitmap_postop_s11) :
               l4_bitmap_changes_s13_s11 ? (l4_bitmap_idx_onehot_s11 ^
                                            reg_l4_bitmap_postop_s12) :
                                           (l4_bitmap_idx_onehot_s11 ^
                                            reg_l4_bitmap_s[10]));
    end
    HEAP_OP_DEQUE_MIN: begin
        l4_bitmap_idx_s11 = (
            reg_l4_addr_conflict_s11_s10 ? ffs_l4_inst_lsb[1] :
               l4_bitmap_changes_s13_s11 ? ffs_l4_inst_lsb[2] :
                                           ffs_l4_inst_lsb[0]);

        l4_bitmap_empty_s11 = (
            reg_l4_addr_conflict_s11_s10 ? ffs_l4_inst_zero[1] :
               l4_bitmap_changes_s13_s11 ? ffs_l4_inst_zero[2] :
                                           ffs_l4_inst_zero[0]);

        l4_bitmap_idx_onehot_s11 = (
            reg_l4_addr_conflict_s11_s10 ? ffs_l4_inst_lsb_onehot[1] :
               l4_bitmap_changes_s13_s11 ? ffs_l4_inst_lsb_onehot[2] :
                                           ffs_l4_inst_lsb_onehot[0]);

        l4_bitmap_postop_s11 = (
            reg_l4_addr_conflict_s11_s10 ? (l4_bitmap_idx_onehot_s11 ^
                                            reg_l4_bitmap_postop_s11) :
               l4_bitmap_changes_s13_s11 ? (l4_bitmap_idx_onehot_s11 ^
                                            reg_l4_bitmap_postop_s12) :
                                           (l4_bitmap_idx_onehot_s11 ^
                                            reg_l4_bitmap_s[10]));
    end
    // HEAP_OP_ENQUE
    default: begin
        l4_bitmap_empty_s11 = 0;
        l4_bitmap_idx_s11 = (reg_priority_s[10][(
                (3 * HEAP_LOG_BITMAP_WIDTH) - 1)
                : (2 * HEAP_LOG_BITMAP_WIDTH)]);

        l4_bitmap_idx_onehot_s11 = (1 << l4_bitmap_idx_s11);
        l4_bitmap_postop_s11 = (
            reg_l4_addr_conflict_s11_s10 ? (l4_bitmap_idx_onehot_s11 |
                                            reg_l4_bitmap_postop_s11) :
               l4_bitmap_changes_s13_s11 ? (l4_bitmap_idx_onehot_s11 |
                                            reg_l4_bitmap_postop_s12) :
                                           (l4_bitmap_idx_onehot_s11 |
                                            reg_l4_bitmap_s[10]));
    end
    endcase
    // Compute the read carry direction. If the active
    // op in Stage 13 is of the same type, carry up.
    if (!reg_is_enque_s[10] &&
        l4_counter_non_zero_s13 &&
        reg_l4_addr_conflict_s12_s10 &&
        (reg_op_type_s[10] == reg_op_type_s[12])) begin
        rcd_s11 = READ_CARRY_UP;

        // Special case: The active op in Stage 12 is also
        // of the same type, which means that it's bound
        // to carry right; here, we do the same.
        if ((reg_op_type_s[10] == reg_op_type_s[11]) &&
            reg_l4_addr_conflict_s11_s10) begin
            rcd_s11 = READ_CARRY_RIGHT;
        end
    end
    // Fallthrough: default to carry down

    // Read the L4 counter
    counter_l4_rden = reg_valid_s[10];
    counter_l4_rdaddress = {reg_l4_addr_s[10],
                            l4_bitmap_idx_s11};
    /**
     * Stage 10: NOOP, read delay for L4 bitmap.
     */
    // L4 bitmap updated this cycle, so output is stale
    if (reg_l4_addr_conflict_s12_s9) begin
        l4_bitmap_s10 = bm_l4_data;
    end
    // L4 bitmap was updated last cycle (R/W conflict)
    else if (reg_l4_addr_conflict_s13_s9) begin
        l4_bitmap_s10 = bm_l4_data_r;
    end
    // Fallthrough: default to bm_l4_q

    /**
     * Stage 9: Write-back the L3 counter and bitmap,
     * and read the corresponding L4 bitmap.
     */
    l3_counter_s9[WATERLEVEL_IDX-1:0] = (
        reg_is_enque_s[8] ? (reg_l3_counter_rc_s8[WATERLEVEL_IDX-1:0] + 1) :
                            (reg_l3_counter_rc_s8[WATERLEVEL_IDX-1:0] - 1));

    l3_counter_s9[WATERLEVEL_IDX] = (reg_is_enque_s[8] ?
        (reg_l3_counter_rc_s8[WATERLEVEL_IDX] | reg_l3_counter_rc_s8[0]) :
        ((|reg_l3_counter_rc_s8[WATERLEVEL_IDX-1:2]) | (&reg_l3_counter_rc_s8[1:0])));

    l3_counter_non_zero_s9 = (reg_is_enque_s[8] |
                              reg_l3_counter_rc_s8[WATERLEVEL_IDX]);
    // Read L4 bitmap
    bm_l4_rden = reg_valid_s[8];
    bm_l4_rdaddress = {reg_l3_addr_s[8],
                       reg_l3_bitmap_idx_s8};

    // Compute conflicts
    l4_addr_conflict_s10_s9 = (
        reg_l3_addr_conflict_s9_s8
            && (reg_l3_bitmap_idx_s8 ==
                reg_l4_addr_s[9][HEAP_LOG_BITMAP_WIDTH-1:0]));

    l4_addr_conflict_s11_s9 = (
        reg_l3_addr_conflict_s10_s8
            && (reg_l3_bitmap_idx_s8 ==
                reg_l4_addr_s[10][HEAP_LOG_BITMAP_WIDTH-1:0]));

    l4_addr_conflict_s12_s9 = (
        reg_l3_addr_conflict_s11_s8
            && (reg_l3_bitmap_idx_s8 ==
                reg_l4_addr_s[11][HEAP_LOG_BITMAP_WIDTH-1:0]));

    l4_addr_conflict_s13_s9 = (
        reg_l3_addr_conflict_s12_s8
            && (reg_l3_bitmap_idx_s8 ==
                reg_l4_addr_s[12][HEAP_LOG_BITMAP_WIDTH-1:0]));

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
                (4 * HEAP_LOG_BITMAP_WIDTH) - 1)
                : (3 * HEAP_LOG_BITMAP_WIDTH)]);

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
                (5 * HEAP_LOG_BITMAP_WIDTH) - 1)
                : (4 * HEAP_LOG_BITMAP_WIDTH)]);

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
        bm_l4_init_done_r <= 0;
        bm_l5_init_done_r <= 0;
        bm_l6_init_done_r <= 0;
        fl_wraddress_counter_r <= 0;
        counter_l2_init_done_r <= 0;
        counter_l3_init_done_r <= 0;
        counter_l4_init_done_r <= 0;
        counter_l5_init_done_r <= 0;
        counter_l6_init_done_r <= 0;
        bm_l3_wraddress_counter_r <= 0;
        bm_l4_wraddress_counter_r <= 0;
        bm_l5_wraddress_counter_r <= 0;
        bm_l6_wraddress_counter_r <= 0;
        counter_l2_wraddress_counter_r <= 0;
        counter_l3_wraddress_counter_r <= 0;
        counter_l4_wraddress_counter_r <= 0;
        counter_l5_wraddress_counter_r <= 0;
        counter_l6_wraddress_counter_r <= 0;

        // Reset FSM state
        state <= FSM_STATE_INIT;
    end
    else begin
        /**
         * Stage 26: Spillover stage.
         */
        reg_valid_s[26] <= reg_valid_s[25];
        reg_bbq_id_s[26] <= reg_bbq_id_s[25];
        reg_he_data_s[26] <= reg_he_data_s[25];
        reg_op_type_s[26] <= reg_op_type_s[25];
        reg_is_enque_s[26] <= reg_is_enque_s[25];
        reg_priority_s[26] <= reg_priority_s[25];
        reg_is_deque_max_s[26] <= reg_is_deque_max_s[25];
        reg_is_deque_min_s[26] <= reg_is_deque_min_s[25];

        reg_pb_data_s26 <= reg_pb_data_s25;
        reg_l2_addr_s[26] <= reg_l2_addr_s[25];
        reg_l3_addr_s[26] <= reg_l3_addr_s[25];
        reg_l4_addr_s[26] <= reg_l4_addr_s[25];
        reg_l5_addr_s[26] <= reg_l5_addr_s[25];
        reg_l6_addr_s[26] <= reg_l6_addr_s[25];
        reg_op_color_s[26] <= reg_op_color_s[25];
        reg_l2_bitmap_s[26] <= reg_l2_bitmap_s[25];
        reg_l3_bitmap_s[26] <= reg_l3_bitmap_s[25];
        reg_l4_bitmap_s[26] <= reg_l4_bitmap_s[25];
        reg_l5_bitmap_s[26] <= reg_l5_bitmap_s[25];
        reg_l6_bitmap_s[26] <= reg_l6_bitmap_s[25];

        /**
         * Stage 25: Perform writes: update the priority bucket,
         * the free list, heap entries, next and prev pointers.
         */
        reg_valid_s[25] <= reg_valid_s[24];
        reg_bbq_id_s[25] <= reg_bbq_id_s[24];
        reg_he_data_s[25] <= he_data_s25;
        reg_op_type_s[25] <= reg_op_type_s[24];
        reg_is_enque_s[25] <= reg_is_enque_s[24];
        reg_priority_s[25] <= reg_priority_s[24];
        reg_is_deque_max_s[25] <= reg_is_deque_max_s[24];
        reg_is_deque_min_s[25] <= reg_is_deque_min_s[24];

        reg_he_data_s25 <= he_data;
        reg_np_data_s25 <= np_data;
        reg_pp_data_s25 <= pp_data;
        reg_pb_data_s25 <= int_pb_data;
        reg_l2_addr_s[25] <= reg_l2_addr_s[24];
        reg_l3_addr_s[25] <= reg_l3_addr_s[24];
        reg_l4_addr_s[25] <= reg_l4_addr_s[24];
        reg_l5_addr_s[25] <= reg_l5_addr_s[24];
        reg_l6_addr_s[25] <= reg_l6_addr_s[24];
        reg_op_color_s[25] <= reg_op_color_s[24];
        reg_l2_bitmap_s[25] <= reg_l2_bitmap_s[24];
        reg_l3_bitmap_s[25] <= reg_l3_bitmap_s[24];
        reg_l4_bitmap_s[25] <= reg_l4_bitmap_s[24];
        reg_l5_bitmap_s[25] <= reg_l5_bitmap_s[24];
        reg_l6_bitmap_s[25] <= reg_l6_bitmap_s[24];

        `ifdef DEBUG
        if (reg_valid_s[24]) begin
            if (!reg_pb_state_changes_s24) begin
                $display(
                    "[BBQ] At S25 (logical ID: %0d, op: %s, color: %s),",
                    reg_bbq_id_s[24], reg_op_type_s[24].name, reg_op_color_s[24].name,
                    " updating (relative priority = %0d),",
                    reg_priority_s[24] & (HEAP_NUM_PRIORITIES_PER_LP - 1),
                    " pb (head, tail) changes from ",
                    "(%b, %b) to (%b, %b)", reg_pb_q_s24.head,
                    reg_pb_q_s24.tail, int_pb_data.head, int_pb_data.tail);
            end
            else if (reg_is_enque_s[24]) begin
                $display(
                    "[BBQ] At S25 (logical ID: %0d, op: %s, color: %s),",
                    reg_bbq_id_s[24], reg_op_type_s[24].name, reg_op_color_s[24].name,
                    " updating (relative priority = %0d),",
                    reg_priority_s[24] & (HEAP_NUM_PRIORITIES_PER_LP - 1),
                    " pb (head, tail) changes from ",
                    "(INVALID_PTR, INVALID_PTR) to (%b, %b)",
                    int_pb_data.head, int_pb_data.tail);
            end
            else begin
                $display(
                    "[BBQ] At S25 (logical ID: %0d, op: %s, color: %s),",
                    reg_bbq_id_s[24], reg_op_type_s[24].name, reg_op_color_s[24].name,
                    " updating (relative priority = %0d),",
                    reg_priority_s[24] & (HEAP_NUM_PRIORITIES_PER_LP - 1),
                    " pb (head, tail) changes from ",
                    "(%b, %b) to (INVALID_PTR, INVALID_PTR)",
                    reg_pb_q_s24.head, reg_pb_q_s24.tail);
            end
        end
        `endif

        /**
         * Stage 24: Read delay for HE and pointers.
         */
        reg_valid_s[24] <= reg_valid_s[23];
        reg_bbq_id_s[24] <= reg_bbq_id_s[23];
        reg_he_data_s[24] <= reg_he_data_s[23];
        reg_op_type_s[24] <= reg_op_type_s[23];
        reg_is_enque_s[24] <= reg_is_enque_s[23];
        reg_priority_s[24] <= reg_priority_s[23];
        reg_is_deque_max_s[24] <= reg_is_deque_max_s[23];
        reg_is_deque_min_s[24] <= reg_is_deque_min_s[23];

        reg_l2_addr_s[24] <= reg_l2_addr_s[23];
        reg_l3_addr_s[24] <= reg_l3_addr_s[23];
        reg_l4_addr_s[24] <= reg_l4_addr_s[23];
        reg_l5_addr_s[24] <= reg_l5_addr_s[23];
        reg_l6_addr_s[24] <= reg_l6_addr_s[23];
        reg_op_color_s[24] <= reg_op_color_s[23];
        reg_l2_bitmap_s[24] <= reg_l2_bitmap_s[23];
        reg_l3_bitmap_s[24] <= reg_l3_bitmap_s[23];
        reg_l4_bitmap_s[24] <= reg_l4_bitmap_s[23];
        reg_l5_bitmap_s[24] <= reg_l5_bitmap_s[23];
        reg_l6_bitmap_s[24] <= reg_l6_bitmap_s[23];
        reg_pb_data_conflict_s24 <= reg_pb_data_conflict_s23;
        reg_pb_state_changes_s24 <= reg_pb_state_changes_s23;
        reg_pb_tail_pp_changes_s24 <= reg_pb_tail_pp_changes_s23;

        reg_he_q_s24 <= he_q_s24;
        reg_np_q_s24 <= np_q_s24;
        reg_pp_q_s24 <= pp_q_s24;

        reg_pb_q_s24 <= (
            reg_pb_addr_conflict_s24_s23 ?
               int_pb_data : reg_pb_q_s23);

        reg_pb_new_s24 <= (
            reg_pb_addr_conflict_s24_s23 ?
               int_pb_data : reg_pb_q_s23);

        if (reg_is_enque_s[23]) begin
            // PB becomes non-empty, update tail
            if (reg_pb_state_changes_s23) begin
                reg_pb_new_s24.tail <= fl_q_r[21];
            end
            reg_pb_new_s24.head <= fl_q_r[21];
        end

        `ifdef SIM
        if (reg_valid_s[23]) begin
            if ((he_wren && he_rden_r && (he_wraddress == he_rdaddress_r)) ||
                (np_wren && np_rden_r && (np_wraddress == np_rdaddress_r))) begin
                $display("[BBQ] Error: Unexpected conflict in R/W access");
                $finish;
            end
        end
        `endif
        `ifdef DEBUG
        if (reg_valid_s[23]) begin
            $display(
                "[BBQ] At S24 (logical ID: %0d, op: %s)",
                reg_bbq_id_s[23], reg_op_type_s[23].name,
                " for PB (relative priority = %0d)",
                reg_priority_s[23] & (HEAP_NUM_PRIORITIES_PER_LP - 1));
        end
        `endif

        /**
         * Stage 23: Read the heap entry and prev/next pointer
         * corresponding to the priority bucket to deque.
         */
        reg_valid_s[23] <= reg_valid_s[22];
        reg_bbq_id_s[23] <= reg_bbq_id_s[22];
        reg_he_data_s[23] <= reg_he_data_s[22];
        reg_op_type_s[23] <= reg_op_type_s[22];
        reg_is_enque_s[23] <= reg_is_enque_s[22];
        reg_priority_s[23] <= reg_priority_s[22];
        reg_is_deque_max_s[23] <= reg_is_deque_max_s[22];
        reg_is_deque_min_s[23] <= reg_is_deque_min_s[22];

        reg_pb_q_s23 <= int_pb_q;
        reg_l2_addr_s[23] <= reg_l2_addr_s[22];
        reg_l3_addr_s[23] <= reg_l3_addr_s[22];
        reg_l4_addr_s[23] <= reg_l4_addr_s[22];
        reg_l5_addr_s[23] <= reg_l5_addr_s[22];
        reg_l6_addr_s[23] <= reg_l6_addr_s[22];
        reg_op_color_s[23] <= reg_op_color_s[22];
        reg_l2_bitmap_s[23] <= reg_l2_bitmap_s[22];
        reg_l3_bitmap_s[23] <= reg_l3_bitmap_s[22];
        reg_l4_bitmap_s[23] <= reg_l4_bitmap_s[22];
        reg_l5_bitmap_s[23] <= reg_l5_bitmap_s[22];
        reg_l6_bitmap_s[23] <= reg_l6_bitmap_s[22];
        reg_pb_data_conflict_s23 <= reg_pb_data_conflict_s22;
        reg_pb_state_changes_s23 <= reg_pb_state_changes_s22;
        reg_pb_tail_pp_changes_s23 <= reg_pb_tail_pp_changes_s22;
        reg_pb_addr_conflict_s24_s23 <= reg_pb_addr_conflict_s23_s22;
        reg_pb_addr_conflict_s25_s23 <= reg_pb_addr_conflict_s24_s22;

        reg_pp_data_s23 <= pp_changes_s24_s23 ? fl_q_r[20] : fl_q_r[21];
        reg_pp_data_valid_s23 <= (pp_changes_s24_s23 || pp_changes_s25_s23);

        `ifdef DEBUG
        if (reg_valid_s[22]) begin
            $display(
                "[BBQ] At S23 (logical ID: %0d, op: %s)",
                reg_bbq_id_s[22], reg_op_type_s[22].name,
                " for PB (relative priority = %0d)",
                reg_priority_s[22] & (HEAP_NUM_PRIORITIES_PER_LP - 1));
        end
        `endif

        /**
         * Stage 22: Compute op color, read delay for PB.
         */
        reg_valid_s[22] <= reg_valid_s[21];
        reg_bbq_id_s[22] <= reg_bbq_id_s[21];
        reg_he_data_s[22] <= reg_he_data_s[21];
        reg_op_type_s[22] <= reg_op_type_s[21];
        reg_is_enque_s[22] <= reg_is_enque_s[21];
        reg_priority_s[22] <= reg_priority_s[21];
        reg_is_deque_max_s[22] <= reg_is_deque_max_s[21];
        reg_is_deque_min_s[22] <= reg_is_deque_min_s[21];

        reg_op_color_s[22] <= op_color_s22;
        reg_l2_addr_s[22] <= reg_l2_addr_s[21];
        reg_l3_addr_s[22] <= reg_l3_addr_s[21];
        reg_l4_addr_s[22] <= reg_l4_addr_s[21];
        reg_l5_addr_s[22] <= reg_l5_addr_s[21];
        reg_l6_addr_s[22] <= reg_l6_addr_s[21];
        reg_l2_bitmap_s[22] <= reg_l2_bitmap_s[21];
        reg_l3_bitmap_s[22] <= reg_l3_bitmap_s[21];
        reg_l4_bitmap_s[22] <= reg_l4_bitmap_s[21];
        reg_l5_bitmap_s[22] <= reg_l5_bitmap_s[21];
        reg_l6_bitmap_s[22] <= reg_l6_bitmap_s[21];
        reg_pb_update_s22 <= reg_pb_addr_conflict_s24_s21;
        reg_pb_addr_conflict_s23_s22 <= reg_pb_addr_conflict_s22_s21;
        reg_pb_addr_conflict_s24_s22 <= reg_pb_addr_conflict_s23_s21;

        // Determine if this op is going to result in PB data
        // conflict (dequeing a PB immediately after an enque
        // operation that causes it to become non-empty).
        reg_pb_data_conflict_s22 <= (reg_is_enque_s[22] &&
            !reg_l6_counter_non_zero_s21 && reg_pb_addr_conflict_s22_s21);

        // Determine if this op causes the PB state to change.
        // Change of state is defined differently based on op:
        // for enques, corresponds to a PB becoming non-empty,
        // and for deques, corresponds to a PB becoming empty.
        reg_pb_state_changes_s22 <= (reg_is_enque_s[21] ?
            (!reg_l6_counter_s21[WATERLEVEL_IDX] && reg_l6_counter_s21[0]) :
            (!reg_l6_counter_s21[WATERLEVEL_IDX] && !reg_l6_counter_s21[0]));

        // Determine if this op causes the previous pointer
        // corresponding to the PB tail to change. High iff
        // enqueing into a PB containing a single element.
        reg_pb_tail_pp_changes_s22 <= (reg_is_enque_s[21] &&
            !reg_old_l6_counter_s21[WATERLEVEL_IDX]
            && reg_old_l6_counter_s21[0]);

        `ifdef DEBUG
        if (reg_valid_s[21]) begin
            $display(
                "[BBQ] At S22 (logical ID: %0d, op: %s)",
                reg_bbq_id_s[21], reg_op_type_s[21].name,
                " for PB (relative priority = %0d)",
                reg_priority_s[21] & (HEAP_NUM_PRIORITIES_PER_LP - 1),
                " assigned color %s", op_color_s22.name);
        end
        `endif

        /**
         * Stage 21: Write-back the L6 counter and bitmap,
         * and read the corresponding PB (head and tail).
         */
        reg_valid_s[21] <= reg_valid_s[20];
        reg_bbq_id_s[21] <= reg_bbq_id_s[20];
        reg_he_data_s[21] <= reg_he_data_s[20];
        reg_op_type_s[21] <= reg_op_type_s[20];
        reg_is_enque_s[21] <= reg_is_enque_s[20];
        reg_priority_s[21] <= priority_s21;
        reg_is_deque_max_s[21] <= reg_is_deque_max_s[20];
        reg_is_deque_min_s[21] <= reg_is_deque_min_s[20];

        reg_l6_bitmap_s[21] <= bm_l6_data;
        reg_l2_addr_s[21] <= reg_l2_addr_s[20];
        reg_l3_addr_s[21] <= reg_l3_addr_s[20];
        reg_l4_addr_s[21] <= reg_l4_addr_s[20];
        reg_l5_addr_s[21] <= reg_l5_addr_s[20];
        reg_l6_addr_s[21] <= reg_l6_addr_s[20];
        reg_l2_bitmap_s[21] <= reg_l2_bitmap_s[20];
        reg_l3_bitmap_s[21] <= reg_l3_bitmap_s[20];
        reg_l4_bitmap_s[21] <= reg_l4_bitmap_s[20];
        reg_l5_bitmap_s[21] <= reg_l5_bitmap_s[20];

        reg_l6_counter_s21 <= l6_counter_s21;
        reg_l6_bitmap_idx_s21 <= reg_l6_bitmap_idx_s20;
        reg_old_l6_counter_s21 <= reg_l6_counter_rc_s20;
        reg_l6_counter_non_zero_s21 <= l6_counter_non_zero_s21;

        reg_pb_addr_conflict_s22_s21 <= pb_addr_conflict_s22_s21;
        reg_pb_addr_conflict_s23_s21 <= pb_addr_conflict_s23_s21;
        reg_pb_addr_conflict_s24_s21 <= pb_addr_conflict_s24_s21;
        reg_pb_addr_conflict_s25_s21 <= pb_addr_conflict_s25_s21;

        `ifdef DEBUG
        if (reg_valid_s[20]) begin
            $display(
                "[BBQ] At S21 (logical ID: %0d, op: %s), updating L6 counter (L6_addr, L6_idx) ",
                reg_bbq_id_s[20], reg_op_type_s[20].name, "= (%0d, %0d) to %0d", reg_l6_addr_s[20],
                reg_l6_bitmap_idx_s20, l6_counter_s21[WATERLEVEL_IDX-1:0]);
        end
        `endif

        /**
         * Stage 20: NOOP, read delay for L6 counter.
         */
        reg_valid_s[20] <= reg_valid_s[19];
        reg_bbq_id_s[20] <= reg_bbq_id_s[19];
        reg_he_data_s[20] <= reg_he_data_s[19];
        reg_op_type_s[20] <= reg_op_type_s[19];
        reg_is_enque_s[20] <= reg_is_enque_s[19];
        reg_priority_s[20] <= reg_priority_s[19];
        reg_is_deque_max_s[20] <= reg_is_deque_max_s[19];
        reg_is_deque_min_s[20] <= reg_is_deque_min_s[19];

        reg_l2_addr_s[20] <= reg_l2_addr_s[19];
        reg_l3_addr_s[20] <= reg_l3_addr_s[19];
        reg_l4_addr_s[20] <= reg_l4_addr_s[19];
        reg_l5_addr_s[20] <= reg_l5_addr_s[19];
        reg_l6_addr_s[20] <= reg_l6_addr_s[19];
        reg_l2_bitmap_s[20] <= reg_l2_bitmap_s[19];
        reg_l3_bitmap_s[20] <= reg_l3_bitmap_s[19];
        reg_l4_bitmap_s[20] <= reg_l4_bitmap_s[19];
        reg_l5_bitmap_s[20] <= reg_l5_bitmap_s[19];
        reg_l6_addr_conflict_s21_s20 <= reg_l6_addr_conflict_s20_s19;
        reg_l6_addr_conflict_s22_s20 <= reg_l6_addr_conflict_s21_s19;
        reg_l6_addr_conflict_s23_s20 <= reg_l6_addr_conflict_s22_s19;
        reg_l6_addr_conflict_s24_s20 <= reg_l6_addr_conflict_s23_s19;

        reg_l6_counter_s20 <= (reg_l6_counter_rdvalid_r1_s19 ?
                               l6_counter_q_s20 : l6_counter_s20);
        case (rcd_s20)
        READ_CARRY_DOWN: begin
            reg_l6_bitmap_idx_s20 <= reg_l6_bitmap_idx_s19;
            reg_l6_bitmap_postop_s20 <= reg_l6_bitmap_postop_s19;
            reg_l6_bitmap_idx_onehot_s20 <= reg_l6_bitmap_idx_onehot_s19;

            reg_l6_counter_rc_s20 <= (reg_l6_counter_rdvalid_r1_s19 ?
                                      l6_counter_q_s20 : l6_counter_s20);
        end
        READ_CARRY_RIGHT: begin
            reg_l6_counter_rc_s20 <= l6_counter_s21;
        end
        default: ;
        endcase

        // Forward L6 bitmap updates
        reg_l6_bitmap_s[20] <= (
            reg_l6_addr_conflict_s20_s19 ?
            bm_l6_data : reg_l6_bitmap_s[19]);

        `ifdef DEBUG
        if (reg_valid_s[19]) begin
            $display(
                "[BBQ] At S20 (logical ID: %0d, op: %s) for (L6 addr = %0d),",
                reg_bbq_id_s[19], reg_op_type_s[19].name, reg_l6_addr_s[19],
                " RCD is %s", rcd_s20.name);
        end
        `endif

        /**
         * Stage 19: Compute the L6 bitmap index and postop
         * bitmap, and read the corresponding L6 counter.
         */
        reg_valid_s[19] <= reg_valid_s[18];
        reg_bbq_id_s[19] <= reg_bbq_id_s[18];
        reg_he_data_s[19] <= reg_he_data_s[18];
        reg_op_type_s[19] <= reg_op_type_s[18];
        reg_is_enque_s[19] <= reg_is_enque_s[18];
        reg_priority_s[19] <= reg_priority_s[18];
        reg_is_deque_max_s[19] <= reg_is_deque_max_s[18];
        reg_is_deque_min_s[19] <= reg_is_deque_min_s[18];

        reg_l2_addr_s[19] <= reg_l2_addr_s[18];
        reg_l3_addr_s[19] <= reg_l3_addr_s[18];
        reg_l4_addr_s[19] <= reg_l4_addr_s[18];
        reg_l5_addr_s[19] <= reg_l5_addr_s[18];
        reg_l6_addr_s[19] <= reg_l6_addr_s[18];
        reg_l2_bitmap_s[19] <= reg_l2_bitmap_s[18];
        reg_l3_bitmap_s[19] <= reg_l3_bitmap_s[18];
        reg_l4_bitmap_s[19] <= reg_l4_bitmap_s[18];
        reg_l5_bitmap_s[19] <= reg_l5_bitmap_s[18];
        reg_l6_addr_conflict_s20_s19 <= reg_l6_addr_conflict_s19_s18;
        reg_l6_addr_conflict_s21_s19 <= reg_l6_addr_conflict_s20_s18;
        reg_l6_addr_conflict_s22_s19 <= reg_l6_addr_conflict_s21_s18;
        reg_l6_addr_conflict_s23_s19 <= reg_l6_addr_conflict_s22_s18;

        reg_l6_counter_rdvalid_r1_s19 <= 0;

        case (rcd_s19)
        READ_CARRY_DOWN: begin
            reg_l6_bitmap_idx_s19 <= l6_bitmap_idx_s19;
            reg_l6_bitmap_empty_s19 <= l6_bitmap_empty_s19;
            reg_l6_bitmap_postop_s19 <= l6_bitmap_postop_s19;
            reg_l6_bitmap_idx_onehot_s19 <= l6_bitmap_idx_onehot_s19;

            reg_l6_counter_rdvalid_r1_s19 <= (!l6_bitmap_empty_s19);
        end
        READ_CARRY_UP: begin
            reg_l6_bitmap_empty_s19 <= 0;
            reg_l6_bitmap_idx_s19 <= reg_l6_bitmap_idx_s20;
            reg_l6_bitmap_idx_onehot_s19 <= reg_l6_bitmap_idx_onehot_s20;

            if (!reg_l6_addr_conflict_s19_s18) begin
                reg_l6_bitmap_postop_s19 <= (
                    reg_l6_bitmap_postop_s20);
            end
        end
        default: ;
        endcase

        // Forward L6 bitmap updates
        reg_l6_bitmap_s[19] <= (
            reg_l6_addr_conflict_s20_s18 ?
            bm_l6_data : reg_l6_bitmap_s[18]);

        `ifdef DEBUG
        if (reg_valid_s[18]) begin
            $display(
                "[BBQ] At S19 (logical ID: %0d, op: %s) for (L6 addr = %0d),",
                reg_bbq_id_s[18], reg_op_type_s[18].name, reg_l6_addr_s[18],
                " RCD is %s", rcd_s19.name);
        end
        `endif

        /**
         * Stage 18: NOOP, read delay for L6 bitmap.
         */
        reg_valid_s[18] <= reg_valid_s[17];
        reg_bbq_id_s[18] <= reg_bbq_id_s[17];
        reg_he_data_s[18] <= reg_he_data_s[17];
        reg_op_type_s[18] <= reg_op_type_s[17];
        reg_is_enque_s[18] <= reg_is_enque_s[17];
        reg_priority_s[18] <= reg_priority_s[17];
        reg_is_deque_max_s[18] <= reg_is_deque_max_s[17];
        reg_is_deque_min_s[18] <= reg_is_deque_min_s[17];

        reg_l6_bitmap_s[18] <= l6_bitmap_s18;
        reg_l2_addr_s[18] <= reg_l2_addr_s[17];
        reg_l3_addr_s[18] <= reg_l3_addr_s[17];
        reg_l4_addr_s[18] <= reg_l4_addr_s[17];
        reg_l5_addr_s[18] <= reg_l5_addr_s[17];
        reg_l6_addr_s[18] <= reg_l6_addr_s[17];
        reg_l2_bitmap_s[18] <= reg_l2_bitmap_s[17];
        reg_l3_bitmap_s[18] <= reg_l3_bitmap_s[17];
        reg_l4_bitmap_s[18] <= reg_l4_bitmap_s[17];
        reg_l5_bitmap_s[18] <= reg_l5_bitmap_s[17];
        reg_l6_addr_conflict_s19_s18 <= reg_l6_addr_conflict_s18_s17;
        reg_l6_addr_conflict_s20_s18 <= reg_l6_addr_conflict_s19_s17;
        reg_l6_addr_conflict_s21_s18 <= reg_l6_addr_conflict_s20_s17;
        reg_l6_addr_conflict_s22_s18 <= reg_l6_addr_conflict_s21_s17;

        `ifdef DEBUG
        if (reg_valid_s[17]) begin
            $display(
                "[BBQ] At S18 (logical ID: %0d, op: %s) for (L6 addr = %0d)",
                reg_bbq_id_s[17], reg_op_type_s[17].name, reg_l6_addr_s[17]);
        end
        `endif

        /**
         * Stage 17: Write-back the L5 counter and bitmap,
         * and read the corresponding L6 bitmap.
         */
        reg_valid_s[17] <= reg_valid_s[16];
        reg_bbq_id_s[17] <= reg_bbq_id_s[16];
        reg_he_data_s[17] <= reg_he_data_s[16];
        reg_op_type_s[17] <= reg_op_type_s[16];
        reg_is_enque_s[17] <= reg_is_enque_s[16];
        reg_priority_s[17] <= reg_priority_s[16];
        reg_is_deque_max_s[17] <= reg_is_deque_max_s[16];
        reg_is_deque_min_s[17] <= reg_is_deque_min_s[16];

        reg_l5_bitmap_s[17] <= bm_l5_data;
        reg_l2_addr_s[17] <= reg_l2_addr_s[16];
        reg_l3_addr_s[17] <= reg_l3_addr_s[16];
        reg_l4_addr_s[17] <= reg_l4_addr_s[16];
        reg_l5_addr_s[17] <= reg_l5_addr_s[16];
        reg_l2_bitmap_s[17] <= reg_l2_bitmap_s[16];
        reg_l3_bitmap_s[17] <= reg_l3_bitmap_s[16];
        reg_l4_bitmap_s[17] <= reg_l4_bitmap_s[16];
        reg_l6_addr_s[17] <= {reg_l5_addr_s[16], reg_l5_bitmap_idx_s16};

        reg_l5_counter_s17 <= l5_counter_s17;
        reg_l5_bitmap_idx_s17 <= reg_l5_bitmap_idx_s16;

        reg_l6_addr_conflict_s18_s17 <= l6_addr_conflict_s18_s17;
        reg_l6_addr_conflict_s19_s17 <= l6_addr_conflict_s19_s17;
        reg_l6_addr_conflict_s20_s17 <= l6_addr_conflict_s20_s17;
        reg_l6_addr_conflict_s21_s17 <= l6_addr_conflict_s21_s17;

        `ifdef DEBUG
        if (reg_valid_s[16]) begin
            $display(
                "[BBQ] At S17 (logical ID: %0d, op: %s), updating L5 counter (L5_addr, L5_idx) ",
                reg_bbq_id_s[16], reg_op_type_s[16].name, "= (%0d, %0d) to %0d", reg_l5_addr_s[16],
                reg_l5_bitmap_idx_s16, l5_counter_s17[WATERLEVEL_IDX-1:0]);
        end
        `endif

        /**
         * Stage 16: NOOP, read delay for L5 counter.
         */
        reg_valid_s[16] <= reg_valid_s[15];
        reg_bbq_id_s[16] <= reg_bbq_id_s[15];
        reg_he_data_s[16] <= reg_he_data_s[15];
        reg_op_type_s[16] <= reg_op_type_s[15];
        reg_is_enque_s[16] <= reg_is_enque_s[15];
        reg_priority_s[16] <= reg_priority_s[15];
        reg_is_deque_max_s[16] <= reg_is_deque_max_s[15];
        reg_is_deque_min_s[16] <= reg_is_deque_min_s[15];

        reg_l2_addr_s[16] <= reg_l2_addr_s[15];
        reg_l3_addr_s[16] <= reg_l3_addr_s[15];
        reg_l4_addr_s[16] <= reg_l4_addr_s[15];
        reg_l5_addr_s[16] <= reg_l5_addr_s[15];
        reg_l2_bitmap_s[16] <= reg_l2_bitmap_s[15];
        reg_l3_bitmap_s[16] <= reg_l3_bitmap_s[15];
        reg_l4_bitmap_s[16] <= reg_l4_bitmap_s[15];
        reg_l5_addr_conflict_s17_s16 <= reg_l5_addr_conflict_s16_s15;
        reg_l5_addr_conflict_s18_s16 <= reg_l5_addr_conflict_s17_s15;
        reg_l5_addr_conflict_s19_s16 <= reg_l5_addr_conflict_s18_s15;
        reg_l5_addr_conflict_s20_s16 <= reg_l5_addr_conflict_s19_s15;

        reg_l5_counter_s16 <= (reg_l5_counter_rdvalid_r1_s15 ?
                               l5_counter_q_s16 : l5_counter_s16);
        case (rcd_s16)
        READ_CARRY_DOWN: begin
            reg_l5_bitmap_idx_s16 <= reg_l5_bitmap_idx_s15;
            reg_l5_bitmap_postop_s16 <= reg_l5_bitmap_postop_s15;
            reg_l5_bitmap_idx_onehot_s16 <= reg_l5_bitmap_idx_onehot_s15;

            reg_l5_counter_rc_s16 <= (reg_l5_counter_rdvalid_r1_s15 ?
                                      l5_counter_q_s16 : l5_counter_s16);
        end
        READ_CARRY_RIGHT: begin
            reg_l5_counter_rc_s16 <= l5_counter_s17;
        end
        default: ;
        endcase

        // Forward L5 bitmap updates
        reg_l5_bitmap_s[16] <= (
            reg_l5_addr_conflict_s16_s15 ?
            bm_l5_data : reg_l5_bitmap_s[15]);

        `ifdef DEBUG
        if (reg_valid_s[15]) begin
            $display(
                "[BBQ] At S16 (logical ID: %0d, op: %s) for (L5 addr = %0d),",
                reg_bbq_id_s[15], reg_op_type_s[15].name, reg_l5_addr_s[15],
                " RCD is %s", rcd_s16.name);
        end
        `endif

        /**
         * Stage 15: Compute the L5 bitmap index and postop
         * bitmap, and read the corresponding L5 counter.
         */
        reg_valid_s[15] <= reg_valid_s[14];
        reg_bbq_id_s[15] <= reg_bbq_id_s[14];
        reg_he_data_s[15] <= reg_he_data_s[14];
        reg_op_type_s[15] <= reg_op_type_s[14];
        reg_is_enque_s[15] <= reg_is_enque_s[14];
        reg_priority_s[15] <= reg_priority_s[14];
        reg_is_deque_max_s[15] <= reg_is_deque_max_s[14];
        reg_is_deque_min_s[15] <= reg_is_deque_min_s[14];

        reg_l2_addr_s[15] <= reg_l2_addr_s[14];
        reg_l3_addr_s[15] <= reg_l3_addr_s[14];
        reg_l4_addr_s[15] <= reg_l4_addr_s[14];
        reg_l5_addr_s[15] <= reg_l5_addr_s[14];
        reg_l2_bitmap_s[15] <= reg_l2_bitmap_s[14];
        reg_l3_bitmap_s[15] <= reg_l3_bitmap_s[14];
        reg_l4_bitmap_s[15] <= reg_l4_bitmap_s[14];
        reg_l5_addr_conflict_s16_s15 <= reg_l5_addr_conflict_s15_s14;
        reg_l5_addr_conflict_s17_s15 <= reg_l5_addr_conflict_s16_s14;
        reg_l5_addr_conflict_s18_s15 <= reg_l5_addr_conflict_s17_s14;
        reg_l5_addr_conflict_s19_s15 <= reg_l5_addr_conflict_s18_s14;

        reg_l5_counter_rdvalid_r1_s15 <= 0;

        case (rcd_s15)
        READ_CARRY_DOWN: begin
            reg_l5_bitmap_idx_s15 <= l5_bitmap_idx_s15;
            reg_l5_bitmap_empty_s15 <= l5_bitmap_empty_s15;
            reg_l5_bitmap_postop_s15 <= l5_bitmap_postop_s15;
            reg_l5_bitmap_idx_onehot_s15 <= l5_bitmap_idx_onehot_s15;

            reg_l5_counter_rdvalid_r1_s15 <= (!l5_bitmap_empty_s15);
        end
        READ_CARRY_UP: begin
            reg_l5_bitmap_empty_s15 <= 0;
            reg_l5_bitmap_idx_s15 <= reg_l5_bitmap_idx_s16;
            reg_l5_bitmap_idx_onehot_s15 <= reg_l5_bitmap_idx_onehot_s16;

            if (!reg_l5_addr_conflict_s15_s14) begin
                reg_l5_bitmap_postop_s15 <= (
                    reg_l5_bitmap_postop_s16);
            end
        end
        default: ;
        endcase

        // Forward L5 bitmap updates
        reg_l5_bitmap_s[15] <= (
            reg_l5_addr_conflict_s16_s14 ?
            bm_l5_data : reg_l5_bitmap_s[14]);

        `ifdef DEBUG
        if (reg_valid_s[14]) begin
            $display(
                "[BBQ] At S15 (logical ID: %0d, op: %s) for (L5 addr = %0d),",
                reg_bbq_id_s[14], reg_op_type_s[14].name, reg_l5_addr_s[14],
                " RCD is %s", rcd_s15.name);
        end
        `endif

        /**
         * Stage 14: NOOP, read delay for L5 bitmap.
         */
        reg_valid_s[14] <= reg_valid_s[13];
        reg_bbq_id_s[14] <= reg_bbq_id_s[13];
        reg_he_data_s[14] <= reg_he_data_s[13];
        reg_op_type_s[14] <= reg_op_type_s[13];
        reg_is_enque_s[14] <= reg_is_enque_s[13];
        reg_priority_s[14] <= reg_priority_s[13];
        reg_is_deque_max_s[14] <= reg_is_deque_max_s[13];
        reg_is_deque_min_s[14] <= reg_is_deque_min_s[13];

        reg_l5_bitmap_s[14] <= l5_bitmap_s14;
        reg_l2_addr_s[14] <= reg_l2_addr_s[13];
        reg_l3_addr_s[14] <= reg_l3_addr_s[13];
        reg_l4_addr_s[14] <= reg_l4_addr_s[13];
        reg_l5_addr_s[14] <= reg_l5_addr_s[13];
        reg_l2_bitmap_s[14] <= reg_l2_bitmap_s[13];
        reg_l3_bitmap_s[14] <= reg_l3_bitmap_s[13];
        reg_l4_bitmap_s[14] <= reg_l4_bitmap_s[13];
        reg_l5_addr_conflict_s15_s14 <= reg_l5_addr_conflict_s14_s13;
        reg_l5_addr_conflict_s16_s14 <= reg_l5_addr_conflict_s15_s13;
        reg_l5_addr_conflict_s17_s14 <= reg_l5_addr_conflict_s16_s13;
        reg_l5_addr_conflict_s18_s14 <= reg_l5_addr_conflict_s17_s13;

        `ifdef DEBUG
        if (reg_valid_s[13]) begin
            $display(
                "[BBQ] At S14 (logical ID: %0d, op: %s) for (L5 addr = %0d)",
                reg_bbq_id_s[13], reg_op_type_s[13].name, reg_l5_addr_s[13]);
        end
        `endif

        /**
         * Stage 13: Write-back the L4 counter and bitmap,
         * and read the corresponding L5 bitmap.
         */
        reg_valid_s[13] <= reg_valid_s[12];
        reg_bbq_id_s[13] <= reg_bbq_id_s[12];
        reg_he_data_s[13] <= reg_he_data_s[12];
        reg_op_type_s[13] <= reg_op_type_s[12];
        reg_is_enque_s[13] <= reg_is_enque_s[12];
        reg_priority_s[13] <= reg_priority_s[12];
        reg_is_deque_max_s[13] <= reg_is_deque_max_s[12];
        reg_is_deque_min_s[13] <= reg_is_deque_min_s[12];

        reg_l4_bitmap_s[13] <= bm_l4_data;
        reg_l2_addr_s[13] <= reg_l2_addr_s[12];
        reg_l3_addr_s[13] <= reg_l3_addr_s[12];
        reg_l4_addr_s[13] <= reg_l4_addr_s[12];
        reg_l2_bitmap_s[13] <= reg_l2_bitmap_s[12];
        reg_l3_bitmap_s[13] <= reg_l3_bitmap_s[12];
        reg_l5_addr_s[13] <= {reg_l4_addr_s[12], reg_l4_bitmap_idx_s12};

        reg_l4_counter_s13 <= l4_counter_s13;
        reg_l4_bitmap_idx_s13 <= reg_l4_bitmap_idx_s12;

        reg_l5_addr_conflict_s14_s13 <= l5_addr_conflict_s14_s13;
        reg_l5_addr_conflict_s15_s13 <= l5_addr_conflict_s15_s13;
        reg_l5_addr_conflict_s16_s13 <= l5_addr_conflict_s16_s13;
        reg_l5_addr_conflict_s17_s13 <= l5_addr_conflict_s17_s13;

        `ifdef DEBUG
        if (reg_valid_s[12]) begin
            $display(
                "[BBQ] At S13 (logical ID: %0d, op: %s), updating L4 counter (L4_addr, L4_idx) ",
                reg_bbq_id_s[12], reg_op_type_s[12].name, "= (%0d, %0d) to %0d", reg_l4_addr_s[12],
                reg_l4_bitmap_idx_s12, l4_counter_s13[WATERLEVEL_IDX-1:0]);
        end
        `endif

        /**
         * Stage 12: NOOP, read delay for L4 counter.
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
        reg_l4_addr_s[12] <= reg_l4_addr_s[11];
        reg_l2_bitmap_s[12] <= reg_l2_bitmap_s[11];
        reg_l3_bitmap_s[12] <= reg_l3_bitmap_s[11];
        reg_l4_addr_conflict_s13_s12 <= reg_l4_addr_conflict_s12_s11;
        reg_l4_addr_conflict_s14_s12 <= reg_l4_addr_conflict_s13_s11;
        reg_l4_addr_conflict_s15_s12 <= reg_l4_addr_conflict_s14_s11;
        reg_l4_addr_conflict_s16_s12 <= reg_l4_addr_conflict_s15_s11;

        reg_l4_counter_s12 <= (reg_l4_counter_rdvalid_r1_s11 ?
                               l4_counter_q_s12 : l4_counter_s12);
        case (rcd_s12)
        READ_CARRY_DOWN: begin
            reg_l4_bitmap_idx_s12 <= reg_l4_bitmap_idx_s11;
            reg_l4_bitmap_postop_s12 <= reg_l4_bitmap_postop_s11;
            reg_l4_bitmap_idx_onehot_s12 <= reg_l4_bitmap_idx_onehot_s11;

            reg_l4_counter_rc_s12 <= (reg_l4_counter_rdvalid_r1_s11 ?
                                      l4_counter_q_s12 : l4_counter_s12);
        end
        READ_CARRY_RIGHT: begin
            reg_l4_counter_rc_s12 <= l4_counter_s13;
        end
        default: ;
        endcase

        // Forward L4 bitmap updates
        reg_l4_bitmap_s[12] <= (
            reg_l4_addr_conflict_s12_s11 ?
            bm_l4_data : reg_l4_bitmap_s[11]);

        `ifdef DEBUG
        if (reg_valid_s[11]) begin
            $display(
                "[BBQ] At S12 (logical ID: %0d, op: %s) for (L4 addr = %0d),",
                reg_bbq_id_s[11], reg_op_type_s[11].name, reg_l4_addr_s[11],
                " RCD is %s", rcd_s12.name);
        end
        `endif

        /**
         * Stage 11: Compute the L4 bitmap index and postop
         * bitmap, and read the corresponding L4 counter.
         */
        reg_valid_s[11] <= reg_valid_s[10];
        reg_bbq_id_s[11] <= reg_bbq_id_s[10];
        reg_he_data_s[11] <= reg_he_data_s[10];
        reg_op_type_s[11] <= reg_op_type_s[10];
        reg_is_enque_s[11] <= reg_is_enque_s[10];
        reg_priority_s[11] <= reg_priority_s[10];
        reg_is_deque_max_s[11] <= reg_is_deque_max_s[10];
        reg_is_deque_min_s[11] <= reg_is_deque_min_s[10];

        reg_l2_addr_s[11] <= reg_l2_addr_s[10];
        reg_l3_addr_s[11] <= reg_l3_addr_s[10];
        reg_l4_addr_s[11] <= reg_l4_addr_s[10];
        reg_l2_bitmap_s[11] <= reg_l2_bitmap_s[10];
        reg_l3_bitmap_s[11] <= reg_l3_bitmap_s[10];
        reg_l4_addr_conflict_s12_s11 <= reg_l4_addr_conflict_s11_s10;
        reg_l4_addr_conflict_s13_s11 <= reg_l4_addr_conflict_s12_s10;
        reg_l4_addr_conflict_s14_s11 <= reg_l4_addr_conflict_s13_s10;
        reg_l4_addr_conflict_s15_s11 <= reg_l4_addr_conflict_s14_s10;

        reg_l4_counter_rdvalid_r1_s11 <= 0;

        case (rcd_s11)
        READ_CARRY_DOWN: begin
            reg_l4_bitmap_idx_s11 <= l4_bitmap_idx_s11;
            reg_l4_bitmap_empty_s11 <= l4_bitmap_empty_s11;
            reg_l4_bitmap_postop_s11 <= l4_bitmap_postop_s11;
            reg_l4_bitmap_idx_onehot_s11 <= l4_bitmap_idx_onehot_s11;

            reg_l4_counter_rdvalid_r1_s11 <= (!l4_bitmap_empty_s11);
        end
        READ_CARRY_UP: begin
            reg_l4_bitmap_empty_s11 <= 0;
            reg_l4_bitmap_idx_s11 <= reg_l4_bitmap_idx_s12;
            reg_l4_bitmap_idx_onehot_s11 <= reg_l4_bitmap_idx_onehot_s12;

            if (!reg_l4_addr_conflict_s11_s10) begin
                reg_l4_bitmap_postop_s11 <= (
                    reg_l4_bitmap_postop_s12);
            end
        end
        default: ;
        endcase

        // Forward L4 bitmap updates
        reg_l4_bitmap_s[11] <= (
            reg_l4_addr_conflict_s12_s10 ?
            bm_l4_data : reg_l4_bitmap_s[10]);

        `ifdef DEBUG
        if (reg_valid_s[10]) begin
            $display(
                "[BBQ] At S11 (logical ID: %0d, op: %s) for (L4 addr = %0d),",
                reg_bbq_id_s[10], reg_op_type_s[10].name, reg_l4_addr_s[10],
                " RCD is %s", rcd_s11.name);
        end
        `endif

        /**
         * Stage 10: NOOP, read delay for L4 bitmap.
         */
        reg_valid_s[10] <= reg_valid_s[9];
        reg_bbq_id_s[10] <= reg_bbq_id_s[9];
        reg_he_data_s[10] <= reg_he_data_s[9];
        reg_op_type_s[10] <= reg_op_type_s[9];
        reg_is_enque_s[10] <= reg_is_enque_s[9];
        reg_priority_s[10] <= reg_priority_s[9];
        reg_is_deque_max_s[10] <= reg_is_deque_max_s[9];
        reg_is_deque_min_s[10] <= reg_is_deque_min_s[9];

        reg_l4_bitmap_s[10] <= l4_bitmap_s10;
        reg_l2_addr_s[10] <= reg_l2_addr_s[9];
        reg_l3_addr_s[10] <= reg_l3_addr_s[9];
        reg_l4_addr_s[10] <= reg_l4_addr_s[9];
        reg_l2_bitmap_s[10] <= reg_l2_bitmap_s[9];
        reg_l3_bitmap_s[10] <= reg_l3_bitmap_s[9];
        reg_l4_addr_conflict_s11_s10 <= reg_l4_addr_conflict_s10_s9;
        reg_l4_addr_conflict_s12_s10 <= reg_l4_addr_conflict_s11_s9;
        reg_l4_addr_conflict_s13_s10 <= reg_l4_addr_conflict_s12_s9;
        reg_l4_addr_conflict_s14_s10 <= reg_l4_addr_conflict_s13_s9;

        `ifdef DEBUG
        if (reg_valid_s[9]) begin
            $display(
                "[BBQ] At S10 (logical ID: %0d, op: %s) for (L4 addr = %0d)",
                reg_bbq_id_s[9], reg_op_type_s[9].name, reg_l4_addr_s[9]);
        end
        `endif

        /**
         * Stage 9: Write-back the L3 counter and bitmap,
         * and read the corresponding L4 bitmap.
         */
        reg_valid_s[9] <= reg_valid_s[8];
        reg_bbq_id_s[9] <= reg_bbq_id_s[8];
        reg_he_data_s[9] <= reg_he_data_s[8];
        reg_op_type_s[9] <= reg_op_type_s[8];
        reg_is_enque_s[9] <= reg_is_enque_s[8];
        reg_priority_s[9] <= reg_priority_s[8];
        reg_is_deque_max_s[9] <= reg_is_deque_max_s[8];
        reg_is_deque_min_s[9] <= reg_is_deque_min_s[8];

        reg_l3_bitmap_s[9] <= bm_l3_data;
        reg_l2_addr_s[9] <= reg_l2_addr_s[8];
        reg_l3_addr_s[9] <= reg_l3_addr_s[8];
        reg_l2_bitmap_s[9] <= reg_l2_bitmap_s[8];
        reg_l4_addr_s[9] <= {reg_l3_addr_s[8], reg_l3_bitmap_idx_s8};

        reg_l3_counter_s9 <= l3_counter_s9;
        reg_l3_bitmap_idx_s9 <= reg_l3_bitmap_idx_s8;

        reg_l4_addr_conflict_s10_s9 <= l4_addr_conflict_s10_s9;
        reg_l4_addr_conflict_s11_s9 <= l4_addr_conflict_s11_s9;
        reg_l4_addr_conflict_s12_s9 <= l4_addr_conflict_s12_s9;
        reg_l4_addr_conflict_s13_s9 <= l4_addr_conflict_s13_s9;

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
        bm_l4_init_done_r <= bm_l4_init_done;
        bm_l5_init_done_r <= bm_l5_init_done;
        bm_l6_init_done_r <= bm_l6_init_done;
        counter_l2_init_done_r <= counter_l2_init_done;
        counter_l3_init_done_r <= counter_l3_init_done;
        counter_l4_init_done_r <= counter_l4_init_done;
        counter_l5_init_done_r <= counter_l5_init_done;
        counter_l6_init_done_r <= counter_l6_init_done;

        fl_wraddress_counter_r <= fl_wraddress_counter_r + 1;
        bm_l3_wraddress_counter_r <= bm_l3_wraddress_counter_r + 1;
        bm_l4_wraddress_counter_r <= bm_l4_wraddress_counter_r + 1;
        bm_l5_wraddress_counter_r <= bm_l5_wraddress_counter_r + 1;
        bm_l6_wraddress_counter_r <= bm_l6_wraddress_counter_r + 1;
        counter_l2_wraddress_counter_r <= counter_l2_wraddress_counter_r + 1;
        counter_l3_wraddress_counter_r <= counter_l3_wraddress_counter_r + 1;
        counter_l4_wraddress_counter_r <= counter_l4_wraddress_counter_r + 1;
        counter_l5_wraddress_counter_r <= counter_l5_wraddress_counter_r + 1;
        counter_l6_wraddress_counter_r <= counter_l6_wraddress_counter_r + 1;

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
        bm_l4_data_r <= bm_l4_data;
        bm_l5_data_r <= bm_l5_data;
        bm_l6_data_r <= bm_l6_data;
        he_wraddress_r <= he_wraddress;
        np_wraddress_r <= np_wraddress;
        pp_wraddress_r <= pp_wraddress;

        fl_q_r[0] <= fl_q;
        for (i = 0; i < 22; i = i + 1) begin
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

// L4 bitmaps
bram_simple2port #(
    .DWIDTH(HEAP_BITMAP_WIDTH),
    .AWIDTH(BITMAP_L4_AWIDTH),
    .DEPTH(NUM_BITMAPS_L4),
    .IS_OUTDATA_REG(0)
)
bm_l4 (
    .clock(clk),
    .data(bm_l4_data),
    .rden(bm_l4_rden),
    .wren(bm_l4_wren),
    .rdaddress(bm_l4_rdaddress),
    .wraddress(bm_l4_wraddress),
    .q(bm_l4_q)
);

// L5 bitmaps
bram_simple2port #(
    .DWIDTH(HEAP_BITMAP_WIDTH),
    .AWIDTH(BITMAP_L5_AWIDTH),
    .DEPTH(NUM_BITMAPS_L5),
    .IS_OUTDATA_REG(0)
)
bm_l5 (
    .clock(clk),
    .data(bm_l5_data),
    .rden(bm_l5_rden),
    .wren(bm_l5_wren),
    .rdaddress(bm_l5_rdaddress),
    .wraddress(bm_l5_wraddress),
    .q(bm_l5_q)
);

// L6 bitmaps
bram_simple2port #(
    .DWIDTH(HEAP_BITMAP_WIDTH),
    .AWIDTH(BITMAP_L6_AWIDTH),
    .DEPTH(NUM_BITMAPS_L6),
    .IS_OUTDATA_REG(0)
)
bm_l6 (
    .clock(clk),
    .data(bm_l6_data),
    .rden(bm_l6_rden),
    .wren(bm_l6_wren),
    .rdaddress(bm_l6_rdaddress),
    .wraddress(bm_l6_wraddress),
    .q(bm_l6_q)
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

// L4 counters
bram_simple2port #(
    .DWIDTH(COUNTER_T_WIDTH),
    .AWIDTH(COUNTER_L4_AWIDTH),
    .DEPTH(NUM_COUNTERS_L4),
    .IS_OUTDATA_REG(0)
)
counters_l4 (
    .clock(clk),
    .data(counter_l4_data),
    .rden(counter_l4_rden),
    .wren(counter_l4_wren),
    .rdaddress(counter_l4_rdaddress),
    .wraddress(counter_l4_wraddress),
    .q(counter_l4_q)
);

// L5 counters
bram_simple2port #(
    .DWIDTH(COUNTER_T_WIDTH),
    .AWIDTH(COUNTER_L5_AWIDTH),
    .DEPTH(NUM_COUNTERS_L5),
    .IS_OUTDATA_REG(0)
)
counters_l5 (
    .clock(clk),
    .data(counter_l5_data),
    .rden(counter_l5_rden),
    .wren(counter_l5_wren),
    .rdaddress(counter_l5_rdaddress),
    .wraddress(counter_l5_wraddress),
    .q(counter_l5_q)
);

// L6 counters
bram_simple2port #(
    .DWIDTH(COUNTER_T_WIDTH),
    .AWIDTH(COUNTER_L6_AWIDTH),
    .DEPTH(NUM_COUNTERS_L6),
    .IS_OUTDATA_REG(0)
)
counters_l6 (
    .clock(clk),
    .data(counter_l6_data),
    .rden(counter_l6_rden),
    .wren(counter_l6_wren),
    .rdaddress(counter_l6_rdaddress),
    .wraddress(counter_l6_wraddress),
    .q(counter_l6_q)
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

// L4 FFSs
ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l4_inst0 (
    .x(reg_l4_bitmap_s[10]),
    .msb(ffs_l4_inst_msb[0]),
    .lsb(ffs_l4_inst_lsb[0]),
    .msb_onehot(ffs_l4_inst_msb_onehot[0]),
    .lsb_onehot(ffs_l4_inst_lsb_onehot[0]),
    .zero(ffs_l4_inst_zero[0])
);

ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l4_inst1 (
    .x(reg_l4_bitmap_postop_s11),
    .msb(ffs_l4_inst_msb[1]),
    .lsb(ffs_l4_inst_lsb[1]),
    .msb_onehot(ffs_l4_inst_msb_onehot[1]),
    .lsb_onehot(ffs_l4_inst_lsb_onehot[1]),
    .zero(ffs_l4_inst_zero[1])
);

ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l4_inst2 (
    .x(reg_l4_bitmap_postop_s12),
    .msb(ffs_l4_inst_msb[2]),
    .lsb(ffs_l4_inst_lsb[2]),
    .msb_onehot(ffs_l4_inst_msb_onehot[2]),
    .lsb_onehot(ffs_l4_inst_lsb_onehot[2]),
    .zero(ffs_l4_inst_zero[2])
);

// L5 FFSs
ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l5_inst0 (
    .x(reg_l5_bitmap_s[14]),
    .msb(ffs_l5_inst_msb[0]),
    .lsb(ffs_l5_inst_lsb[0]),
    .msb_onehot(ffs_l5_inst_msb_onehot[0]),
    .lsb_onehot(ffs_l5_inst_lsb_onehot[0]),
    .zero(ffs_l5_inst_zero[0])
);

ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l5_inst1 (
    .x(reg_l5_bitmap_postop_s15),
    .msb(ffs_l5_inst_msb[1]),
    .lsb(ffs_l5_inst_lsb[1]),
    .msb_onehot(ffs_l5_inst_msb_onehot[1]),
    .lsb_onehot(ffs_l5_inst_lsb_onehot[1]),
    .zero(ffs_l5_inst_zero[1])
);

ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l5_inst2 (
    .x(reg_l5_bitmap_postop_s16),
    .msb(ffs_l5_inst_msb[2]),
    .lsb(ffs_l5_inst_lsb[2]),
    .msb_onehot(ffs_l5_inst_msb_onehot[2]),
    .lsb_onehot(ffs_l5_inst_lsb_onehot[2]),
    .zero(ffs_l5_inst_zero[2])
);

// L6 FFSs
ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l6_inst0 (
    .x(reg_l6_bitmap_s[18]),
    .msb(ffs_l6_inst_msb[0]),
    .lsb(ffs_l6_inst_lsb[0]),
    .msb_onehot(ffs_l6_inst_msb_onehot[0]),
    .lsb_onehot(ffs_l6_inst_lsb_onehot[0]),
    .zero(ffs_l6_inst_zero[0])
);

ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l6_inst1 (
    .x(reg_l6_bitmap_postop_s19),
    .msb(ffs_l6_inst_msb[1]),
    .lsb(ffs_l6_inst_lsb[1]),
    .msb_onehot(ffs_l6_inst_msb_onehot[1]),
    .lsb_onehot(ffs_l6_inst_lsb_onehot[1]),
    .zero(ffs_l6_inst_zero[1])
);

ffs #(
    .WIDTH_LOG(HEAP_LOG_BITMAP_WIDTH)
)
ffs_l6_inst2 (
    .x(reg_l6_bitmap_postop_s20),
    .msb(ffs_l6_inst_msb[2]),
    .lsb(ffs_l6_inst_lsb[2]),
    .msb_onehot(ffs_l6_inst_msb_onehot[2]),
    .lsb_onehot(ffs_l6_inst_lsb_onehot[2]),
    .zero(ffs_l6_inst_zero[2])
);

endmodule
